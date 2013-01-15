// -----------------------------------------------------------------------------
// Headers
// ------------------------------------------------------------------------------

#include <board.h>
#include <string.h>
#include <utility/trace.h>
#include <pio/pio.h>
#include <pio/pio_it.h>
#include <aic/aic.h>
#include <usb/device/cdc-serial/CDCDSerialDriver.h>
#include <usb/device/cdc-serial/CDCDSerialDriverDescriptors.h>

#include "communication.h"
#include "registerFPGA.h"
#include "controlVelocity.h"
#include "controlPWM.h"

unsigned char send_buf[1024];
unsigned char receive_buf[2048];
int w_receive_buf = 0;
int r_receive_buf = 0;
unsigned long send_buf_pos = 0;
extern const Pin pinPWMEnable;

inline void send( char *buf )
{
	for( ; *buf; buf++ )
	{
		send_buf[send_buf_pos] = ( unsigned char )( *buf );
		send_buf_pos++;
	}
}

inline void flush( void )
{
	static int itry = 0;
	if( send_buf_pos == 0 )
		return;
	while( 1 )
	{
		if( CDCDSerialDriver_Write( send_buf, send_buf_pos, 0, 0 ) != USBD_STATUS_SUCCESS )
		{
			itry++;
			if( itry > 0 )
			{
				send_buf[send_buf_pos] = 0;
				TRACE_ERROR( "Send Failed\n\r%s\n\r", send_buf );
				break;
			}
		}
		else
		{
			break;
		}
	}
	// CDCDSerialDriver_Write(send_buf, send_buf_pos, 0, 0);
	// printf("%u\n\r",send_buf_pos);
	send_buf_pos = 0;
}

/**
 * @brief エンコード
 */
inline int encode( unsigned char *src, int len, unsigned char *dst, int buf_max )
{
	static int pos, s_pos, w_pos;
	static unsigned short b;
	pos = 0;									// read position
	w_pos = 0;									// write_position
	s_pos = 0;
	b = 0;

	while( pos < len || s_pos >= 6 )
	{
		if( s_pos >= 6 )
		{
			dst[w_pos] = ( ( b >> 10 ) & 0x3f ) + 0x40;
			w_pos++;
			if( w_pos >= buf_max )
				return ( -1 );
			b = b << 6;
			s_pos -= 6;
		}
		else
		{
			b |= src[pos] << ( 8 - s_pos );
			s_pos += 8;
			pos++;
			if( pos >= len )
				s_pos += 4;						// 最後
		}
	}

	if( w_pos >= buf_max )
		return ( -1 );

	return w_pos;
}

/**
 * @brief デコード
 * @param src[in] デコードする文字列
 * @param len[in] デコードする文字列の長さ
 * @param dst[out] デコード後のデータ
 * @param buf_max[in] デコード後のデータバッファのサイズ
 * @return デコード後のバイト数
 */
inline int decord( unsigned char *src, int len, unsigned char *dst, int buf_max )
{
	static unsigned short dat, b;
	// static int pos;
	static int s_pos, w_pos;
	static int rerr;
	// pos = 0; // read position
	w_pos = 0;									// write_position
	s_pos = 0;									// shift position
	rerr = 0;
	dat = 0;
	b = 0;
	while(  /* pos < */ len )
	{
		// if( src[pos] >= 0x40 )
		// b = src[pos] - 0x40;
		if( *src >= 0x40 )
			b = *src - 0x40;
		else
			rerr++;

		dat |= ( b << ( 10 - s_pos ) );
		s_pos += 6;
		if( s_pos >= 8 )
		{
			dst[w_pos] = ( dat >> 8 );
			w_pos++;
			if( w_pos >= buf_max )
				return 0;
			s_pos -= 8;
			dat = dat << 8;
		}
		// pos++;
		src++;
		len--;
	}

	if( rerr )
		return -rerr;
	return w_pos;
}

/* オドメトリデータの送信 */
inline int data_send( short cnt1, short cnt2, short pwm1, short pwm2, short *analog, unsigned short analog_mask )
{
	static unsigned char data[34];

	static int len, i, encode_len;

	data[0] = ( ( Integer2 ) cnt1 ).byte[1];
	data[1] = ( ( Integer2 ) cnt1 ).byte[0];
	data[2] = ( ( Integer2 ) cnt2 ).byte[1];
	data[3] = ( ( Integer2 ) cnt2 ).byte[0];
	data[4] = ( ( Integer2 ) pwm1 ).byte[1];
	data[5] = ( ( Integer2 ) pwm1 ).byte[0];
	data[6] = ( ( Integer2 ) pwm2 ).byte[1];
	data[7] = ( ( Integer2 ) pwm2 ).byte[0];

	len = 8;
	for( i = 0; analog_mask != 0; analog_mask = analog_mask >> 1, i++ )
	{
		if( analog_mask & 1 )
		{
			data[len] = ( ( Integer2 ) analog[i] ).byte[1];
			data[len + 1] = ( ( Integer2 ) analog[i] ).byte[0];
			len += 2;
		}
	}

	// 変換
	send_buf_pos = 0;
	send_buf[0] = COMMUNICATION_START_BYTE;
	encode_len = encode( ( unsigned char * )data, len, send_buf + 1, 1024 - 2 );
	if( encode_len < 0 )
		return encode_len;
	send_buf[encode_len + 1] = COMMUNICATION_END_BYTE;
	send_buf_pos = encode_len + 2;

	flush(  );

	return encode_len;
}

inline int data_fetch( unsigned char *data, int len )
{
	unsigned char *data_begin;

	data_begin = data;
	for( ; len; len-- )
	{
		receive_buf[w_receive_buf] = *data;
		w_receive_buf++;
		data++;
		if( w_receive_buf >= 2048 )
			w_receive_buf = 0;
		if( w_receive_buf == r_receive_buf )
		{
			break;
		}
	}
	if( len )
	{
		int i;
		for( i = 0; i < len; i++ )
		{
			data_begin[i] = data[i];
		}
	}
	return len;
}

inline int data_analyze(  )
{
	unsigned char line[64];
	unsigned char *data;
	int r_buf, len;
	enum
	{
		STATE_IDLE,
		STATE_RECIEVING
	} state = STATE_IDLE;

	r_buf = r_receive_buf;
	data = &receive_buf[r_receive_buf];
	len = 0;
	for( ;; )
	{
		if( r_buf == w_receive_buf )
			break;
		line[len] = *data;
		len++;

		switch ( state )
		{
		case STATE_IDLE:
			if( *data == COMMUNICATION_START_BYTE )
			{
				len = 0;
				state = STATE_RECIEVING;
			}

			if( *data == COMMUNICATION_END_BYTE )
			{
				line[len - 1] = 0;
				extended_command_analyze( ( char * )line );
				len = 0;
				r_receive_buf = r_buf;
				state = STATE_IDLE;
			}
			break;
		case STATE_RECIEVING:
			if( *data == COMMUNICATION_END_BYTE )
			{
				static unsigned char rawdata[16];
				int data_len;

				data_len = decord( line, len - 1, rawdata, 16 );
				command_analyze( rawdata, data_len );
				len = 0;
				r_receive_buf = r_buf;
				state = STATE_IDLE;
			}
			break;
		}
		data++;
		r_buf++;
		if( r_buf >= 2048 )
		{
			r_buf = 0;
			data = receive_buf;
		}
	}
	return 0;
}

// //////////////////////////////////////////////////
/* 受信したYPSpur拡張コマンドの解析 */
inline int extended_command_analyze( char *data )
{
	// char line[64];
	static int i;

	if( driver_param.servo_level != SERVO_LEVEL_STOP )
		return 0;

	send_buf_pos = 0;
	if( strstr( data, "VV" ) == data )
	{
		send( data );
		send( "\n00P\nVEND:" );
		send( YP_VENDOR_NAME );
		send( "; \nPROD:" );
		send( YP_PRODUCT_NAME );
		send( "; \nFIRM:" );
		send( YP_FIRMWARE_NAME );
		send( "; \nPROT:" );
		send( YP_PROTOCOL_NAME );
		send( "; \nSERI:Reserved; \n\n" );

	}
	else if( strstr( data, "ADMASK" ) == data )
	{
		unsigned char tmp;

		tmp = 0;
		for( i = 6; data[i] != 0 && data[i] != '\n' && data[i] != '\r'; i++ )
		{
			tmp = tmp << 1;
			if( data[i] == '1' )
				tmp |= 1;
		}
		// analog_mask = tmp;
		driver_param.admask = tmp;
		send( data );
		send( "\n00P\n\n" );
	}
	else if( strstr( data, "SETIODIR" ) == data )
	{
		unsigned char tmp;
		// PE0-3(0-3), PB2-5(4-7)

		tmp = 0;
		for( i = 8; data[i] != 0 && data[i] != '\n' && data[i] != '\r'; i++ )
		{
			tmp = tmp << 1;
			if( data[i] == '1' )
				tmp |= 1;
		}
		// PFC.PEIOR.WORD = ( PFC.PEIOR.WORD & 0xFFF0 ) | ( ( tmp & 0x0F ) << 0 );
		// PFC.PBIOR.WORD = ( PFC.PBIOR.WORD & 0xFFC3 ) | ( ( tmp & 0xF0 ) >> 2 );
		driver_param.io_dir = tmp;
		send( data );
		send( "\n00P\n\n" );
	}
	else if( strstr( data, "GETIOVAL" ) == data )
	{
		unsigned short tmp;
		char num[3];
		// tmp = ( PE.DR.WORD & 0x0F ) | ( ( PB.DR.WORD & 0x3C ) << 2 );
		tmp = 0;
		send( data );
		send( "\n" );
		if( ( tmp >> 4 ) > 9 )
		{
			num[0] = ( tmp >> 4 ) - 10 + 'A';
		}
		else
		{
			num[0] = ( tmp >> 4 ) + '0';
		}
		if( ( tmp & 0xF ) > 9 )
		{
			num[1] = ( tmp & 0xF ) - 10 + 'A';
		}
		else
		{
			num[1] = ( tmp & 0xF ) + '0';
		}
		num[2] = 0;
		send( num );
		send( " \n\n" );
	}
	else if( strstr( data, "GETIO" ) == data )
	{
		if( data[5] == '1' )
		{
			// dio_enable = 1;
			driver_param.io_mask = 0xFF;
		}
		else
		{
			// dio_enable = 0;
			driver_param.io_mask = 0;
		}
		send( data );
		send( "\n00P\n\n" );
	}
	else if( strstr( data, "OUTPUT" ) == data )
	{
		unsigned char tmp;
		// PA18-21(0-3), PB2-5(4-7)

		tmp = 0;
		for( i = 6; data[i] != 0 && data[i] != '\n' && data[i] != '\r'; i++ )
		{
			tmp = tmp << 1;
			if( data[i] == '1' )
				tmp |= 1;
		}
		// PE.DR.WORD = ( PE.DR.WORD & 0xFFF0 ) | ( ( tmp & 0x0F ) << 0 );
		// PB.DR.WORD = ( PB.DR.WORD & 0xFFC3 ) | ( ( tmp & 0xF0 ) >> 2 );
		send( data );
		send( "\n00P\n\n" );
	}
	else if( strstr( data, "SS" ) == data )
	{
		int tmp;
		// volatile int lo;

		// cnt_updated = 0;
		tmp = 0;
		for( i = 2; data[i] != 0 && data[i] != '\n' && data[i] != '\r'; i++ )
		{
			tmp *= 10;
			tmp += data[i] - '0';
		}
		send( data );
		send( "\n00P\n\n" );
		// 送信終了まで待機
		// while( SCI_send_rp[channel] != SCI_send_wp[channel] );
		// for ( lo = 0; lo < 10000; lo++ ); /* wait more than 1bit time */
		// sci_init( tmp );
		// sci_start( ); // start SCI
		// cnt_updated = 0;
	}
	else if( strstr( data, "STORE" ) == data )
	{
		int chk = 0xAACC;
		AT91C_BASE_EFC1->EFC_FMR = AT91C_MC_FWS_1FWS;
		memcpy( ( int * )0x0017FF00, &driver_param, sizeof ( driver_param ) );
		memcpy( ( int * )( 0x0017FF00 + sizeof ( driver_param ) ), motor_param, sizeof ( motor_param ) );
		memcpy( ( int * )( 0x0017FF00 + sizeof ( driver_param ) + sizeof ( motor_param ) ), &chk, sizeof ( chk ) );
		AT91C_BASE_EFC1->EFC_FCR = AT91C_MC_FCMD_START_PROG | ( 0x3FF << 8 );
		send( data );
		send( "\n00P\n\n" );
	}
	else
	{
		if( data[0] == 0 || data[0] == '\n' || data[0] == '\r' )
			return 0;
		send( data );
		send( "\n0Ee\n\n" );
	}
	flush(  );

	return 1;
}

// //////////////////////////////////////////////////
/* 受信したコマンドの解析 */
inline int command_analyze( unsigned char *data, int len )
{
	static int imotor;

	static Integer4 i;

	i.byte[3] = data[2];
	i.byte[2] = data[3];
	i.byte[1] = data[4];
	i.byte[0] = data[5];

	imotor = data[1];
	if( imotor < 0 || imotor >= 2 )
		return 0;

	// if(data[0] != PARAM_w_ref)
	// printf("get %d %d %d\n\r",data[0],data[1],i.integer);
	switch ( data[0] )
	{
	case PARAM_w_ref:
		motor[imotor].ref.vel = i.integer * 16;
		motor[imotor].ref.vel_changed = 1;
		break;
	case PARAM_w_ref_highprec:
		motor[imotor].ref.vel = i.integer;
		motor[imotor].ref.vel_changed = 1;
		break;
	case PARAM_p_ki:
		motor_param[imotor].Kcurrent = i.integer;
		break;
	case PARAM_p_kv:
		motor_param[imotor].Kvolt = i.integer;
		break;
	case PARAM_p_fr_plus:
		motor_param[imotor].fr_plus = i.integer;
		break;
	case PARAM_p_fr_wplus:
		motor_param[imotor].fr_wplus = i.integer;
		break;
	case PARAM_p_fr_minus:
		motor_param[imotor].fr_minus = i.integer;
		break;
	case PARAM_p_fr_wminus:
		motor_param[imotor].fr_wminus = i.integer;
		break;
	case PARAM_p_A:
		driver_param.Kdynamics[0] = i.integer;
		break;
	case PARAM_p_B:
		driver_param.Kdynamics[1] = i.integer;
		break;
	case PARAM_p_C:
		driver_param.Kdynamics[2] = i.integer;
		break;
	case PARAM_p_D:
		driver_param.Kdynamics[3] = i.integer;
		break;
	case PARAM_p_E:
		driver_param.Kdynamics[4] = i.integer;
		break;
	case PARAM_p_F:
		driver_param.Kdynamics[5] = i.integer;
		break;
	case PARAM_p_pi_kp:
		motor_param[imotor].Kp = i.integer;
		break;
	case PARAM_p_pi_ki:
		motor_param[imotor].Ki = i.integer;
		break;
	case PARAM_pwm_max:
		driver_param.PWM_max = i.integer;
		break;
	case PARAM_pwm_min:
		driver_param.PWM_min = i.integer;
		break;
	case PARAM_toq_max:
		motor_param[imotor].torque_max = i.integer;
		break;
	case PARAM_toq_min:
		motor_param[imotor].torque_min = i.integer;
		break;
	case PARAM_toq_limit:
		motor_param[imotor].torque_limit = i.integer;
		break;
	case PARAM_p_toq_offset:
		motor_param[imotor].torque_offset = i.integer;
		driver_param.watchdog = 0;
		break;
	case PARAM_int_max:
		driver_param.integ_max = i.integer * 16;
		break;
	case PARAM_int_min:
		driver_param.integ_min = i.integer * 16;
		break;
	case PARAM_servo:
		if( driver_param.servo_level < SERVO_LEVEL_TORQUE && i.integer >= SERVO_LEVEL_TORQUE )
		{
			if( THEVA.GENERAL.ID != 0xA0 )
			{
				TRACE_ERROR( "Invalid FPGA %u !\n\r", THEVA.GENERAL.ID );
				while( 1 );
			}

			// printf("initialized\n\r" );
			controlPWM_config(  );

			THEVA.GENERAL.PWM.COUNT_ENABLE = 1;
			THEVA.GENERAL.OUTPUT_ENABLE = 1;

			PIO_Clear( &pinPWMEnable );
			// printf("PWM Period: %d\n\r", THEVA.GENERAL.PWM.HALF_PERIOD);
			// printf("PWM Deadtime: %d\n\r", THEVA.GENERAL.PWM.DEADTIME);

			// AIC_EnableIT(AT91C_ID_TC0);
		}
		if( driver_param.servo_level < SERVO_LEVEL_VELOCITY && i.integer >= SERVO_LEVEL_VELOCITY )
		{										// servo levelが速度制御に推移した
			motor[0].error_integ = 0;
			motor[1].error_integ = 0;
		}
		driver_param.servo_level = i.integer;
		break;
	case PARAM_watch_dog_limit:
		driver_param.watchdog_limit = i.integer;
		driver_param.watchdog = 0;
		driver_param.enable_watchdog = 1;
		break;
	case PARAM_io_dir:
		driver_param.io_dir = i.integer;
		break;
	case PARAM_io_data:
		break;
	case PARAM_ad_mask:
		driver_param.admask = i.integer;
		break;
	case PARAM_enc_rev:
		motor_param[imotor].enc_rev = i.integer;
	default:
		return 0;
	}
	driver_param.watchdog = 0;
	return 0;
}

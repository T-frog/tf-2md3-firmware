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
#include <flash/flashd.h>

#include "communication.h"
#include "registerFPGA.h"
#include "controlVelocity.h"
#include "controlPWM.h"
#include "eeprom.h"
#include "io.h"

#define SEND_BUF_LEN  1024
#define RECV_BUF_LEN  1024

unsigned char send_buf[SEND_BUF_LEN];
unsigned char receive_buf[RECV_BUF_LEN];
volatile int w_receive_buf = 0;
volatile int r_receive_buf = 0;
volatile unsigned long send_buf_pos = 0;
extern const Pin pinPWMEnable;
extern Tfrog_EEPROM_data saved_param;

int hextoi( char *buf )
{
	int ret;
	ret = 0;
	for( ; *buf; )
	{
		if( '0' <= *buf && *buf <= '9' )
		{
			ret *= 16;
			ret += *buf - '0';
		}
		else if( 'A' <= *buf && *buf <= 'F' )
		{
			ret *= 16;
			ret += *buf - 'A' + 0xA;
		}
		buf++;
	}
	return ret;
}

int atoi( char *buf )
{
	int ret;
	ret = 0;
	for( ; *buf; )
	{
		if( '0' <= *buf && *buf <= '9' )
		{
			ret *= 10;
			ret += *buf - '0';
		}
		buf++;
	}
	return ret;
}

int nhex( char *buf, int data, int len )
{
	int i;
	for( i = 0; i < len; i++ )
	{
		*buf = ( ( ( unsigned int )data >> ( ( len - i - 1 ) * 4 ) ) & 0xF ) + '0';
		if( *buf > '9' )
		{
			*buf = *buf - '9' - 1 + 'A';
		}
		buf++;
	}
	*buf = 0;
	return len;
}

int itoa10( char *buf, int data )
{
	int i;
	int len;
	char txt[16];
	int sign = 0;

	if( data < 0 )
	{
		sign = 1;
		data = -data;
		buf[ 0 ] = '-';
	}
	for( i = 0; data; i++ )
	{
		txt[i] = data % 10 + '0';
		data = data / 10;
	}
	if( i == 0 )
	{
		txt[i] = '0';
		i ++;
	}
	len = i;
	for( i = 0; i < len; i ++ )
	{
		buf[ sign + len - i - 1 ] = txt[ i ];
	}
	buf[ sign + len ] = 0;
	return len;
}

int send( char *buf )
{
	int i = 0;
	for( ; *buf; buf++ )
	{
		send_buf[send_buf_pos] = ( unsigned char )( *buf );
		send_buf_pos++;
		if( send_buf_pos >= SEND_BUF_LEN || send_buf[send_buf_pos] == '\n' ) flush();
		i ++;
	}
	return i;
}

int nsend( char *buf, int len )
{
	int i;
	for( i = 0; i < len && *buf; i ++, buf ++ )
	{
		send_buf[send_buf_pos] = ( unsigned char )( *buf );
		send_buf_pos++;
		if( send_buf_pos >= SEND_BUF_LEN || send_buf[send_buf_pos] == '\n' ) flush();
	}
	return i;
}

void flush( void )
{
	int len;
	
	len = send_buf_pos;
	send_buf[len] = 0;
	if( len == 0 )
		return;
	while( 1 )
	{
		char ret;
		if( driver_param.watchdog >= driver_param.watchdog_limit )
		{
			TRACE_ERROR( "Send timeout\n\r" );
			send_buf_pos = 0;
			break;
		}
		ret = CDCDSerialDriver_Write( send_buf, len, 0, 0 );
		if( ret == USBD_STATUS_LOCKED )
		{
			continue;
		}
		else if( ret != USBD_STATUS_SUCCESS )
		{
			TRACE_ERROR( "Send failed\n\r  buf: %s\n\r", send_buf );
			break;
		}
		else
		{
			send_buf_pos -= len;
			break;
		}
	}
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
	static int s_pos, w_pos;
	static int rerr;
	
	w_pos = 0;									// write_position
	s_pos = 0;									// shift position
	rerr = 0;
	dat = 0;
	b = 0;
	while(  len )
	{
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
	encode_len = encode( ( unsigned char * )data, len, send_buf + 1, RECV_BUF_LEN - 2 );
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
		if( w_receive_buf >= RECV_BUF_LEN )
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
	unsigned char line[256];
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
				r_receive_buf = r_buf + 1;
				state = STATE_IDLE;
			}
			break;
		case STATE_RECIEVING:
			if( *data == COMMUNICATION_START_BYTE )
			{
				len = 0;
				state = STATE_RECIEVING;
			}
			if( *data == COMMUNICATION_END_BYTE )
			{
				static unsigned char rawdata[16];
				int data_len;

				data_len = decord( line, len - 1, rawdata, 16 );
				if( data_len < 6 )
				{
					int i;
					printf( "Data err%d[%d]: ", data_len, len );
					for( i = 0; i < len; i ++ )
					{
						printf( "%02x ", line[i] );
					}
					printf( "\n\r" );
				}
				else
				{
					command_analyze( rawdata, data_len );
				}
				len = 0;
				r_receive_buf = r_buf + 1;
				state = STATE_IDLE;
			}
			break;
		}
		data++;
		r_buf++;
		if( r_buf >= RECV_BUF_LEN )
		{
			r_buf = 0;
			data = receive_buf;
		}
	}
	return 0;
}

int ext_continue = -1;
// //////////////////////////////////////////////////
/* 受信したYPSpur拡張コマンドの解析 */
inline int extended_command_analyze( char *data )
{
	static int i;

	if( driver_param.servo_level != SERVO_LEVEL_STOP )
		return 0;

	if( ext_continue >= 0 )
	{
		char val[10];
		int len;
		int wrote;

		if( data[0] == 0 )
		{
			char zero = 0;
			msleep( 5 );
			EEPROM_Write( TFROG_EEPROM_ROBOTPARAM_ADDR + ext_continue,
						&zero, 1 );
			send( data );
			send( "\n00P\n" );
			itoa10( val, ext_continue );
			send( val );
			send( " bytes saved\n\n" );
			flush(  );
			ext_continue = -1;
			return 1;
		}

		len = strlen( data );
		data[len] = '\n';
		msleep( 5 );
		wrote = EEPROM_Write( TFROG_EEPROM_ROBOTPARAM_ADDR + ext_continue, 
					data, len + 1 );
		data[len] = 0;
		if( wrote < 0 )
		{
			char zero = 0;
			msleep( 5 );
			EEPROM_Write( TFROG_EEPROM_ROBOTPARAM_ADDR + ext_continue,
						&zero, 1 );
			send( data );
			send( "\n01Q\nFailed (" );
			itoa10( val, ext_continue );
			send( val );
			send( " bytes saved)\n\n" );
			flush(  );
			ext_continue = -1;
			return 0;
		}

		ext_continue += len + 1;
		return 1;
	}


	if( strstr( data, "VV" ) == data )
	{
		char val[10];
		send( data );
		send( "\n00P\nVEND:" );
		send( YP_VENDOR_NAME );
		send( "; \nPROD:" );
		send( BOARD_NAME );
		send( "; \nFIRM:" );
		send( YP_FIRMWARE_NAME );
		send( "; \nPROT:" );
		send( YP_PROTOCOL_NAME );
		send( "; \nSERI:" );
		nhex( val, saved_param.serial_no, 8 );
		send( val );
		send( "; \n\n" );

	}
	else if( strstr( data, "PP" ) == data )
	{
		char val[10];
		send( data );
		send( "\n00P\nNAME:" );
		send( saved_param.robot_name );
		send( "; \nPWMRES:" );
		itoa10( val, saved_param.PWM_resolution );
		send( val );
		send( "; \nMOTORNUM:" );
		send( YP_DRIVERPARAM_MOTORNUM );
		send( "; \nDEADTIME:" );
		itoa10( val, saved_param.PWM_deadtime );
		send( val );
		send( "; \n\n" );
	}
	else if( strstr( data, "GETEMBEDDEDPARAM" ) == data )
	{
		char epval[256];
		int len;
		int i;

		send( data );
		send( "\n" );
		len = EEPROM_Read( TFROG_EEPROM_ROBOTPARAM_ADDR, epval, 256 );
		if( len < 0 )
		{
			send( "01Q\n\n" );
		}
		else
		{
			send( "00P\n" );
			flush( );
			for( i = 0; i < 1792; i += 256 )
			{
				len = EEPROM_Read( TFROG_EEPROM_ROBOTPARAM_ADDR + i, epval, 256 );
				if( nsend( epval, 256 ) < 256 ) break;
				AT91C_BASE_WDTC->WDTC_WDCR = 1 | 0xA5000000;
				flush( );
			}
			send( "\n\n" );
		}
	}
	else if( strstr( data, "SETEMBEDDEDPARAM" ) == data )
	{
		ext_continue = 0;
		send( data );
		send( "\n00P\n\n" );
	}
	else if( strstr( data, "$EEPROMDUMP" ) == data )
	{
		char val[3];
		char epval[256];
		int i, j;

		send( data );
		send( "\n00P\n" );

		for( j = 0; j < 8; j ++ )
		{
			EEPROM_Read( 256 * j, epval, 256 );
			for( i = 0; i < 256; i ++ )
			{
				nhex( val, epval[i], 2 );
				send( val );
			}
			send( "\n" );
			if( j == 7 ) break;
			flush();
			AT91C_BASE_WDTC->WDTC_WDCR = 1 | 0xA5000000;
		}
		send( "\n" );
	}
	else if( strstr( data, "$FLASHERACE" ) == data )
	{
		static int erace_flag = 0;
		if( erace_flag == 0 && data[11] == 'A' )
		{
			erace_flag = 1;
			send( data );
			send( "\n00P\n\n" );
		}
		else if( erace_flag == 1 && data[11] == 'B' )
		{
			erace_flag = 0;
			send( data );
			send( "\n00P\n\n" );
			flush( );
			FLASHD_ClearGPNVM( 2 );
		}
		else
		{
			erace_flag = 0;
			send( data );
			send( "\n01Q\n\n" );
		}
	}
	else if( strstr( data, "$EEPROMERACE" ) == data )
	{
		int i;
		char clear[16];

		for( i = 0; i < 16; i ++ ) clear[i] = 0xFF;

		for( i = 0; i < 2048; i += 16 )
		{
			msleep( 5 );
			EEPROM_Write( i, &clear, 16 );
			AT91C_BASE_WDTC->WDTC_WDCR = 1 | 0xA5000000;
		}

		send( data );
		send( "\n00P\n\n" );
	}
	else if( strstr( data, "$SETSERIALNO" ) == data )
	{
		saved_param.serial_no = hextoi( data + 12 );

		send( data );
		send( "\n00P\n\n" );
	}
	else if( strstr( data, "$SETNAME" ) == data )
	{
		strcpy( saved_param.robot_name, data + 8 );

		send( data );
		send( "\n00P\n\n" );
	}
	else if( strstr( data, "$SETPWMRESOLUTION" ) == data )
	{
		saved_param.PWM_resolution = atoi( data + 17 );

		send( data );
		send( "\n00P\n\n" );
	}
	else if( strstr( data, "$SETPWMDEADTIME" ) == data )
	{
		saved_param.PWM_deadtime = atoi( data + 15 );

		send( data );
		send( "\n00P\n\n" );
	}
	else if( strstr( data, "$EEPROMSAVE" ) == data )
	{
		if( EEPROM_Write( 0, &saved_param, sizeof(saved_param) ) < 0 )
		{
			send( data );
			send( "\n01Q\n\n" );
		}
		else
		{
			send( data );
			send( "\n00P\n\n" );
		}
	}
	else if( strstr( data, "$ENC0" ) == data )
	{
		char num[16];

		send( data );
		send( "\nANG0  " );
		nhex( num, motor_param[0].enc0tran, 4 );
		num[4] = 0;
		send( num );
		send( "," );
		nhex( num, motor_param[1].enc0tran, 4 );
		num[4] = 0;
		send( num );
		send( "\n" );
		send( "ANG0T " );
		nhex( num, motor_param[0].enc0, 4 );
		num[4] = 0;
		send( num );
		send( "," );
		nhex( num, motor_param[1].enc0, 4 );
		num[4] = 0;
		send( num );
		send( "\n\n" );
	}
	else if( strstr( data, "$TESTENC" ) == data )
	{
		unsigned short tmp;
		char num[16];

		send( data );
		send( "\nHALL" );
		tmp = THEVA.MOTOR[0].ROT_DETECTER.HALL;
		if( tmp & HALL_U ) num[0] = 'U'; else num[0] = '-';
		if( tmp & HALL_V ) num[1] = 'V'; else num[1] = '-';
		if( tmp & HALL_W ) num[2] = 'W'; else num[2] = '-';
		num[3] = 0;
		send( num );
		send( "," );
		tmp = THEVA.MOTOR[1].ROT_DETECTER.HALL;
		if( tmp & HALL_U ) num[0] = 'U'; else num[0] = '-';
		if( tmp & HALL_V ) num[1] = 'V'; else num[1] = '-';
		if( tmp & HALL_W ) num[2] = 'W'; else num[2] = '-';
		num[3] = 0;
		send( num );
		send( "\n" );

		send( "ANG " );
		nhex( num, THEVA.MOTOR[0].ENCODER, 4 );
		num[4] = 0;
		send( num );
		send( "," );
		nhex( num, THEVA.MOTOR[1].ENCODER, 4 );
		num[4] = 0;
		send( num );
		send( "\n" );

		send( "SPD " );
		nhex( num, THEVA.MOTOR[0].SPEED, 4 );
		num[4] = 0;
		send( num );
		send( "," );
		nhex( num, THEVA.MOTOR[1].SPEED, 4 );
		num[4] = 0;
		send( num );

		send( "\n\n" );
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
		driver_param.admask = tmp;
		send( data );
		send( "\n00P\n\n" );
	}
	else if( strstr( data, "SETIODIR" ) == data )
	{
		unsigned char tmp;

		tmp = 0;
		for( i = 8; data[i] != 0 && data[i] != '\n' && data[i] != '\r'; i++ )
		{
			tmp = tmp << 1;
			if( data[i] == '1' )
				tmp |= 1;
		}
		set_io_dir( tmp );
		driver_param.io_dir = tmp;
		send( data );
		send( "\n00P\n\n" );
	}
	else if( strstr( data, "GETIOVAL" ) == data )
	{
		unsigned short tmp;
		char num[3];
		tmp = get_io_data();
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
		switch( data[5] )
		{
		case '1':
			driver_param.io_mask[0] = 0xFF;
			driver_param.io_mask[1] = 0;
			break;
		case '2':
			driver_param.io_mask[1] = 0xFF;
			driver_param.io_mask[0] = 0;
			break;
		case '3':
			driver_param.io_mask[0] = 0xFF;
			driver_param.io_mask[1] = 0xFF;
			break;
		default:
			driver_param.io_mask[0] = 0;
			driver_param.io_mask[1] = 0;
			break;
		}
		send( data );
		send( "\n00P\n\n" );
	}
	else if( strstr( data, "OUTPUT" ) == data )
	{
		unsigned char tmp;

		tmp = 0;
		for( i = 6; data[i] != 0 && data[i] != '\n' && data[i] != '\r'; i++ )
		{
			tmp = tmp << 1;
			if( data[i] == '1' )
				tmp |= 1;
		}
		set_io_data( tmp );
		send( data );
		send( "\n00P\n\n" );
	}
	else if( strstr( data, "SS" ) == data )
	{
		int tmp;

		tmp = 0;
		for( i = 2; data[i] != 0 && data[i] != '\n' && data[i] != '\r'; i++ )
		{
			tmp *= 10;
			tmp += data[i] - '0';
		}
		send( data );
		send( "\n04T\n\n" );
	}
	else
	{
		if( data[0] == 0 || data[0] == '\n' || data[0] == '\r' )
		{
			flush(  );
		}
		else
		{
			send( data );
			send( "\n0Ee\n\n" );
		}
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
		i.integer *= 16;
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
		break;
	case PARAM_int_max:
		driver_param.integ_max = i.integer * 16;
		break;
	case PARAM_int_min:
		driver_param.integ_min = i.integer * 16;
		break;
	case PARAM_motor_phase:
		switch( i.integer )
		{
		case 0:
			motor_param[imotor].motor_type = MOTOR_TYPE_DC;
			break;
		case 3:
			motor_param[imotor].motor_type = MOTOR_TYPE_AC3;
			break;
		}
		break;
	case PARAM_servo:
		if( driver_param.servo_level < SERVO_LEVEL_TORQUE && i.integer >= SERVO_LEVEL_TORQUE )
		{
			controlPWM_config(  );

			THEVA.GENERAL.PWM.COUNT_ENABLE = 1;
			THEVA.GENERAL.OUTPUT_ENABLE = 1;

			PIO_Clear( &pinPWMEnable );
		}
		if( ( driver_param.servo_level < SERVO_LEVEL_VELOCITY || driver_param.servo_level == SERVO_LEVEL_OPENFREE ) && 
			( i.integer >= SERVO_LEVEL_VELOCITY && i.integer != SERVO_LEVEL_OPENFREE ) )
		{
			// servo levelが速度制御に推移した
			motor[0].control_init = 1;
			motor[1].control_init = 1;
		}
		if( driver_param.servo_level != SERVO_LEVEL_OPENFREE && i.integer == SERVO_LEVEL_OPENFREE )
		{
			THEVA.GENERAL.OUTPUT_ENABLE = 0;
			PIO_Set( &pinPWMEnable );
		}
		if( driver_param.servo_level == SERVO_LEVEL_OPENFREE && i.integer != SERVO_LEVEL_OPENFREE )
		{
			THEVA.GENERAL.OUTPUT_ENABLE = 1;
			PIO_Clear( &pinPWMEnable );
		}
		driver_param.servo_level = i.integer;
		break;
	case PARAM_watch_dog_limit:
		driver_param.watchdog_limit = i.integer;
		break;
	case PARAM_io_dir:
		driver_param.io_dir = i.integer;
		set_io_dir( driver_param.io_dir );
		break;
	case PARAM_io_data:
		set_io_data( i.integer );
		break;
	case PARAM_ad_mask:
		driver_param.admask = i.integer;
		break;
	case PARAM_enc_rev:
		motor_param[imotor].enc_rev = i.integer;
		break;
	case PARAM_vsrc:
		// ad = 1024 * ( vsrc * VSRC_DIV ) / 3.3
		driver_param.vsrc_rated = 310 * ( (int)i.integer * VSRC_DIV ) / 256;
		if( driver_param.vsrc_rated > 1024 )
		{
			driver_param.vsrc_rated = 0;
		}
		break;
	default:
		return 0;
	}
	driver_param.watchdog = 0;
	return 0;
}


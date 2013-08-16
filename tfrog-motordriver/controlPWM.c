// -----------------------------------------------------------------------------
// Headers
// ------------------------------------------------------------------------------

#include <board.h>
#include <pio/pio.h>
#include <pio/pio_it.h>
#include <aic/aic.h>
#include <tc/tc.h>
#include <utility/trace.h>
#include <utility/led.h>
#include <usb/device/cdc-serial/CDCDSerialDriver.h>
#include <usb/device/cdc-serial/CDCDSerialDriverDescriptors.h>

#define FP4_POINTBIT 17

#include "registerFPGA.h"
#include "fixpawd.h"
#include "fixpawd_math.h"
#include "controlPWM.h"
#include "controlVelocity.h"
#include "power.h"
#include "eeprom.h"

static const Pin pinsLeds[] = { PINS_LEDS };

static const unsigned int numLeds = PIO_LISTSIZE( pinsLeds );

static const Pin pinPWMCycle = PIN_PWM_CYCLE;
static const Pin pinPWMCycle2 = PIN_PWM_CYCLE2;

// / PWM Enable pin instance.
static const Pin pinPWMEnable = PIN_PWM_ENABLE;

// static long enc2phase[2];
short SinTB[2][4096];
int phase_offset[2][2];
int phase90[2];
int PWM_abs_max = 0;
int PWM_abs_min = 0;
int PWM_center = 0;
int PWM_init = 0;
int PWM_resolution = 0;
int PWM_thinning = 0;
int PWM_deadtime;

extern Tfrog_EEPROM_data saved_param;


void normalize( int *val, int min, int max, int resolution )
{
	while( *val < min )
		*val += resolution;
	while( *val >= max )
		*val -= resolution;
}

int _abs( int x )
{
	if( x < 0 ) return -x;
	return x;
}

void controlPWM_config(  )
{
	int i, j;

	PWM_resolution = saved_param.PWM_resolution;
	PWM_thinning = 2000 / PWM_resolution;
	PWM_deadtime = saved_param.PWM_deadtime;

	THEVA.GENERAL.PWM.HALF_PERIOD = PWM_resolution;
	THEVA.GENERAL.PWM.DEADTIME = PWM_deadtime;

	driver_param.PWM_resolution = PWM_resolution;

	PWM_abs_max = PWM_resolution - PWM_deadtime - 1;
	PWM_abs_min = PWM_deadtime + 1;
	PWM_center = PWM_resolution / 2;

	for( i = 0; i < 2; i ++ )
	{
		motor[i].pos = 0;
		motor[i].vel = 0;
		motor[i].error_integ = 0;
		motor_param[i].enc0 = 0;
		motor_param[i].enc0tran = 0;
	}
	// PIO_Clear( &pinsLeds[USBD_LEDPOWER] );
	for( i = 0; i < 2; i++ )
	{
		// enc2phase[i] = 2000 / motor_param[i].enc_rev;

		switch( motor_param[i].motor_type )
		{
		case MOTOR_TYPE_DC:
			THEVA.MOTOR[i].PWM[0].H = PWM_resolution;
			THEVA.MOTOR[i].PWM[1].H = 0;
			THEVA.MOTOR[i].PWM[2].H = PWM_resolution;
			THEVA.MOTOR[i].PWM[0].L = PWM_resolution;
			THEVA.MOTOR[i].PWM[1].L = 0;
			THEVA.MOTOR[i].PWM[2].L = PWM_resolution;
			break;
		case MOTOR_TYPE_AC3:
			THEVA.MOTOR[i].PWM[0].H = PWM_resolution;
			THEVA.MOTOR[i].PWM[1].H = PWM_resolution;
			THEVA.MOTOR[i].PWM[2].H = PWM_resolution;
			THEVA.MOTOR[i].PWM[0].L = PWM_resolution;
			THEVA.MOTOR[i].PWM[1].L = PWM_resolution;
			THEVA.MOTOR[i].PWM[2].L = PWM_resolution;
			break;
		}

		motor[i].ref.rate = 0;

		motor_param[i].enc_drev[0] = motor_param[i].enc_rev / 6;
		motor_param[i].enc_drev[1] = motor_param[i].enc_rev * 2 / 6;
		motor_param[i].enc_drev[2] = motor_param[i].enc_rev * 3 / 6;
		motor_param[i].enc_drev[3] = motor_param[i].enc_rev * 4 / 6;
		motor_param[i].enc_drev[4] = motor_param[i].enc_rev * 5 / 6;
		motor_param[i].enc_drev[5] = motor_param[i].enc_rev;

		motor_param[i].enc_10hz = motor_param[i].enc_rev * 10 * 16 / 1000;

		phase90[i] = motor_param[i].enc_rev - motor_param[i].enc_rev / 4;
		phase_offset[i][0] = motor_param[i].enc_rev / 3;
		phase_offset[i][1] = motor_param[i].enc_rev * 2 / 3;
		if( i == 1 && motor_param[1].enc_rev == motor_param[0].enc_rev )
		{
			for( j = 0; j < motor_param[i].enc_rev; j++ )
			{
				SinTB[1][j] = SinTB[0][j];
			}
			break;
		}
		for( j = 0; j < motor_param[i].enc_rev / 2; j++ )
		{
			fixp4 val;
			int ival;
			int ang;
			
			ang = j;
			val = ( fp4sin( FP4_PI2 * ang / motor_param[i].enc_rev )
					+ fp4mul( fp4sin( FP4_PI2 * 3 * ang / motor_param[i].enc_rev ), DOUBLE2FP4( 0.1547 ) ) );
			// = ( 2.0 / sqrt( 3.0 ) - 1.0 )

			ival = val * /* 4730 */ 3547 / FP4_ONE;

			if( ival > 4096 )
				ival = 4096;
			else if( ival < -4096 )
				ival = -4096;

			SinTB[i][j] = ival;
			driver_param.watchdog = 0;
		}
		for( j = 0; j < motor_param[i].enc_rev / 2; j++ )
		{
			SinTB[i][j + motor_param[i].enc_rev / 2] = -SinTB[i][j];
		}
	}
	for( i = 0; i < 2; i++ )
	{
		motor[i].ref.vel_diff = 0;
		motor[i].ref.vel_interval = 0;
		motor[i].ref.vel_changed = 0;
		motor[i].error_integ = 0;
		motor[i].vel = 0;
		motor[i].dir = 0;
	}
	PWM_init = 0;
	
	driver_param.watchdog = 0;
}

// ------------------------------------------------------------------------------
// / PWM control interrupt (every PWM period) 20us/50us
// ------------------------------------------------------------------------------
void FIQ_PWMPeriod(  )
{
	int i;
	unsigned short enc[2];
	unsigned short hall[2];
	static unsigned short _enc[2];
	static unsigned short _hall[2];
	static int _sign[2] = { 0, 0 };
	static int init = 0;
	static int cnt = 0;

	{
		// PWM周波数が高い場合は処理を間引く
		// PWM_resolution 1000以下で一回間引き
		static int thin = 0;

		thin ++;
		if( thin < PWM_thinning ) return;
		thin = 0;
	}
	

	if( driver_param.servo_level == SERVO_LEVEL_STOP || 
		driver_param.error_state )
	{
		// Short-mode brake
		for( i = 0; i < 3*2; i ++ )
		{
			THEVA.MOTOR[i%2].PWM[i/2].H = PWM_resolution;
			THEVA.MOTOR[i%2].PWM[i/2].L = PWM_resolution;
		}
		PWM_init = 0;
		init = 0;
		return;
	}

	for( i = 0; i < 2; i ++ )
	{
		unsigned short s;
		int __vel;

		motor[i].enc = enc[i] = THEVA.MOTOR[i].ENCODER;
		hall[i] = *(unsigned short *)&THEVA.MOTOR[i].ROT_DETECTER;
		s = THEVA.MOTOR[i].SPEED;
		__vel = ( short )( enc[i] - _enc[i] );

		if( __vel < 0 )
			_sign[i] = -1;
		else if( __vel > 0 )
			_sign[i] = 1;

		if( s < 256 * 16 * 4 )
		{
			if( _sign[i] > 0 )
				motor[i].spd_sum += s;
			else if( _sign[i] < 0 )
				motor[i].spd_sum -= s;
			motor[i].spd_num ++;
		}
	}

	if( !init )
	{
		init = 1;
		_hall[0] = hall[0];
		_hall[1] = hall[1];
		_enc[0] = enc[0];
		_enc[1] = enc[1];

		return;
	}

	for( i = 0; i < 2; i ++ )
	{
		motor[i].pos += ( short )( enc[i] - _enc[i] );
		normalize( &motor[i].pos, 0, motor_param[i].enc_rev, motor_param[i].enc_rev );
	}

	// PWM計算
	{
		int pwm[2][3];
		int phase[3];
		int j;

		if( PWM_init < 2048 )
		{
			PWM_init ++;
			for( i = 0; i < 2; i++ )
			{
				motor[i].pos = 0;
				motor[i].vel = 0;
				motor[i].error_integ = 0;
				if( hall[i] & HALL_U )
				{
					if( hall[i] & HALL_V )
						motor_param[i].enc0 = motor[i].pos - motor_param[i].enc_rev * 5 / 12;	// 150度
					else if( hall[i] & HALL_W )
						motor_param[i].enc0 = motor[i].pos - motor_param[i].enc_rev * 1 / 12;	// 30度
					else
						motor_param[i].enc0 = motor[i].pos - motor_param[i].enc_rev * 3 / 12;	// 90度
				}
				else
				{
					if( !( hall[i] & HALL_V ) )
						motor_param[i].enc0 = motor[i].pos - motor_param[i].enc_rev * 11 / 12;	// 330度
					else if( !( hall[i] & HALL_W ) )
						motor_param[i].enc0 = motor[i].pos - motor_param[i].enc_rev * 7 / 12;	// 210度
					else
						motor_param[i].enc0 = motor[i].pos - motor_param[i].enc_rev * 9 / 12;	// 270度
				}
				motor_param[i].enc0tran = motor_param[i].enc0;
			}
		}
		cnt ++;
		for( j = 0; j < 2; j++ )
		{
			int rate;

			rate = motor[j].ref.rate * PWM_init / 2048;

			if( driver_param.vsrc_rated )
			{
				rate = rate * driver_param.vsrc_factor / 32768;
				if( rate >= PWM_resolution )
					rate = PWM_resolution - 1;
				else if( rate <= -PWM_resolution )
					rate = -PWM_resolution + 1;
			}

			if( cnt % 64 == 2 + j )
			{
				int diff;
				diff = motor_param[j].enc0tran - motor_param[j].enc0;
				normalize( &diff, -motor_param[j].enc_rev / 2, motor_param[j].enc_rev / 2, motor_param[j].enc_rev );

				if( diff == 0 )
				{
					motor_param[j].enc0tran = motor_param[j].enc0;
				}
				else if( diff > 0 )
				{
					motor_param[j].enc0tran --;
					if( motor_param[j].enc0tran <= -motor_param[j].enc_rev )
						motor_param[j].enc0tran += motor_param[j].enc_rev;
				}
				else
				{
					motor_param[j].enc0tran ++;
					if( motor_param[j].enc0tran >= motor_param[j].enc_rev )
						motor_param[j].enc0tran -= motor_param[j].enc_rev;
				}
			}

			switch( motor_param[j].motor_type )
			{
			case MOTOR_TYPE_DC:
				{
					int pwmt;
					pwmt = PWM_center + rate / 2;
					if( pwmt < PWM_abs_min )
						pwmt = PWM_abs_min;
					if( pwmt > PWM_abs_max )
						pwmt = PWM_abs_max;
					pwm[j][2] = pwmt;
					pwmt = PWM_center - ( rate - rate / 2 );
					if( pwmt < PWM_abs_min )
						pwmt = PWM_abs_min;
					if( pwmt > PWM_abs_max )
						pwmt = PWM_abs_max;
					pwm[j][0] = pwmt;
				}
				break;
			case MOTOR_TYPE_AC3:
				phase[2] = ( ( motor[j].pos - motor_param[j].enc0tran ) ) - phase90[j];
				normalize( &phase[2], 0, motor_param[j].enc_rev, motor_param[j].enc_rev );

				phase[1] = phase[2] - phase_offset[j][0];
				normalize( &phase[1], 0, motor_param[j].enc_rev, motor_param[j].enc_rev );

				phase[0] = phase[2] - phase_offset[j][1];
				normalize( &phase[0], 0, motor_param[j].enc_rev, motor_param[j].enc_rev );

				{
					for( i = 0; i < 3; i++ )
					{
						int pwmt;
						pwmt = ( ( ( ( int )SinTB[j][phase[i]] ) * rate ) / 8192 );
						pwmt += PWM_center;
						if( pwmt < PWM_abs_min )
							pwmt = PWM_abs_min;
						if( pwmt > PWM_abs_max )
							pwmt = PWM_abs_max;
						pwm[j][i] = pwmt;
					}
				}
				break;
			}
		}
		for( j = 0; j < 2; j++ )
		{
			if( _abs( motor[j].ref.torque ) < driver_param.zero_torque )
			{
				for( i = 0; i < 3; i++ )
				{
					THEVA.MOTOR[j].PWM[i].H = 0;
					THEVA.MOTOR[j].PWM[i].L = 0;
				}
			}
			else
			{
				for( i = 0; i < 3; i++ )
				{
					THEVA.MOTOR[j].PWM[i].H = pwm[j][i];
					THEVA.MOTOR[j].PWM[i].L = PWM_resolution;
				}
			}
		}
	}

	// ゼロ点計算
	for( i = 0; i < 2; i++ )
	{
		char u, v, w;

		if( motor_param[i].motor_type != MOTOR_TYPE_DC )
		{
			char dir;
			unsigned short halldiff;

			u = v = w = 0;
			halldiff = ( hall[i] ^ _hall[i] ) & 0x07; 
			
			if( halldiff == 0 ) continue;
			dir = 0;

			if( ( hall[i] & 0x07 ) == ( HALL_U | HALL_V | HALL_W ) ||
				( hall[i] & 0x07 ) == 0 ||
				halldiff == 3 || halldiff >= 5 )
			{
				if( (hall[i] & 0x07) == ( HALL_U | HALL_V | HALL_W ) ) printf( "ENC error: 111\n\r" );
				if( (hall[i] & 0x07) == 0 ) printf( "ENC error: 000\n\r" );
				if( halldiff == 3 || halldiff >= 5 ) printf( "ENC error: %x->%x\n\r", _hall[i], hall[i] );
				// ホール素子信号が全相1、全相0のとき
				// ホース素子信号が2ビット以上変化したときはエラー
				if( driver_param.error.hall[i] < 128 )
						driver_param.error.hall[i] += 12;
				if( driver_param.error.hall[i] > 12 )
				{
					// エラー検出後、1周以内に再度エラーがあれば停止
					switch(i)
					{
					case 0:
						driver_param.error_state |= ERROR_HALL1;
						break;
					case 1:
						driver_param.error_state |= ERROR_HALL2;
						break;
					}
				}
				continue;
			}
			else
			{
				if( driver_param.error.hall[i] > 0 ) driver_param.error.hall[i] --;
			}

			switch( halldiff )
			{
			case HALL_U:
				if( !( _hall[i] & HALL_U ) )
					if( hall[i] & HALL_W )
					{
						u = 1; // 正回転 U立ち上がり 0度
					}
					else
					{
						u = -1; // 逆回転 U立ち上がり 180度
						dir = 1;
					}
				else
					if( hall[i] & HALL_V )
					{
						u = -1; // 正回転 U立ち下がり 180度
					}
					else
					{
						u = 1; // 逆回転 U立ち下がり 0度
						dir = 1;
					}
				break;
			case HALL_V:
				if( !( _hall[i] & HALL_V ) )
					if( hall[i] & HALL_U )
					{
						v = 1; // 正回転 V立ち上がり 120度
					}
					else
					{
						v = -1; // 逆回転 V立ち上がり 300度
						dir = 1;
					}
				else
					if( hall[i] & HALL_W )
					{
						v = -1; // 正回転 V立ち下がり 300度
					}
					else
					{
						v = 1; // 逆回転 V立ち下がり 120度
						dir = 1;
					}
				break;
			case HALL_W:
				if( !( _hall[i] & HALL_W ) )
					if( hall[i] & HALL_V )
					{
						w = 1; // 正回転 W立ち上がり 240度
					}
					else
					{
						w = -1; // 逆回転 W立ち上がり 60度
						dir = 1;
					}
				else
					if( hall[i] & HALL_U )
					{
						w = -1; // 正回転 W立ち下がり 60度
					}
					else
					{
						w = 1; // 逆回転 W立ち下がり 240度
						dir = 1;
					}
				break;
			default:
				// エラー
				break;
			}

			// ホール素子は高速域では信頼できない
			if( _abs( motor[i].vel ) > motor_param[i].enc_10hz )
				continue;

			// ゼロ点計算

			if( w == -1 )
				motor_param[i].enc0 = motor[i].pos - motor_param[i].enc_drev[0] + dir;
			else if( v == 1 )
				motor_param[i].enc0 = motor[i].pos - motor_param[i].enc_drev[1] + dir;
			else if( u == -1 )
				motor_param[i].enc0 = motor[i].pos - motor_param[i].enc_drev[2] + dir;
			else if( w == 1 )
				motor_param[i].enc0 = motor[i].pos - motor_param[i].enc_drev[3] + dir;
			else if( v == -1 )
				motor_param[i].enc0 = motor[i].pos - motor_param[i].enc_drev[4] + dir;
			else if( u == 1 )
				motor_param[i].enc0 = motor[i].pos - motor_param[i].enc_drev[5] + dir;
		}
	}

	_hall[0] = hall[0];
	_hall[1] = hall[1];

	_enc[0] = enc[0];
	_enc[1] = enc[1];

	return;
}

// ------------------------------------------------------------------------------
// / Configure velocity control loop
// ------------------------------------------------------------------------------
void controlPWM_init(  )
{
	int i;

	PIO_Configure( &pinPWMCycle2, 1 );
	AIC_ConfigureIT( AT91C_ID_IRQ0, 5 | AT91C_AIC_SRCTYPE_POSITIVE_EDGE, ( void ( * )( void ) )FIQ_PWMPeriod );
	AIC_EnableIT( AT91C_ID_IRQ0 );

	// PWM Generator init
	PWM_resolution = saved_param.PWM_resolution;
	PWM_thinning = 2000 / PWM_resolution;
	PWM_deadtime = saved_param.PWM_deadtime;

	THEVA.GENERAL.PWM.HALF_PERIOD = PWM_resolution;
	THEVA.GENERAL.PWM.DEADTIME = PWM_deadtime;

	// Short-mode brake
	for( i = 0; i < 3*2; i ++ )
	{
		THEVA.MOTOR[i%2].PWM[i/2].H = PWM_resolution;
		THEVA.MOTOR[i%2].PWM[i/2].L = PWM_resolution;
	}
	
	THEVA.GENERAL.PWM.COUNT_ENABLE = 1;
	THEVA.GENERAL.OUTPUT_ENABLE = 1;
	PIO_Clear( &pinPWMEnable );
}


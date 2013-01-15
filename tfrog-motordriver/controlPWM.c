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

extern int velcontrol;

int _abs( int x )
{
	if( x < 0 ) return -x;
	return x;
}

void controlPWM_config(  )
{
	static unsigned short hall[2];
	static int i, j;
	static unsigned short bwatchdog;
	int PWM_resolution;
	int deadtime;

	bwatchdog = driver_param.enable_watchdog;
	driver_param.enable_watchdog = 0;

	hall[0] = *( unsigned short * )&THEVA.MOTOR[0].ROT_DETECTER;
	hall[1] = *( unsigned short * )&THEVA.MOTOR[1].ROT_DETECTER;

	PWM_resolution = 1200;
	deadtime = 36;
	THEVA.GENERAL.PWM.HALF_PERIOD = PWM_resolution;
	THEVA.GENERAL.PWM.DEADTIME = deadtime;

	PWM_abs_max = PWM_resolution - deadtime - 1;
	PWM_abs_min = deadtime + 1;
	PWM_center = PWM_resolution / 2;

	// PIO_Clear( &pinsLeds[USBD_LEDPOWER] );
	for( i = 0; i < 2; i++ )
	{
		// enc2phase[i] = 2000 / motor_param[i].enc_rev;

		THEVA.MOTOR[i].PWM[0].H = 0;
		THEVA.MOTOR[i].PWM[1].H = 0;
		THEVA.MOTOR[i].PWM[2].H = 0;
		THEVA.MOTOR[i].PWM[0].L = PWM_resolution;
		THEVA.MOTOR[i].PWM[1].L = PWM_resolution;
		THEVA.MOTOR[i].PWM[2].L = PWM_resolution;

		motor[i].ref.rate = 0;

		motor_param[i].enc_drev[0] = motor_param[i].enc_rev / 6;
		motor_param[i].enc_drev[1] = motor_param[i].enc_rev * 2 / 6;
		motor_param[i].enc_drev[2] = motor_param[i].enc_rev * 3 / 6;
		motor_param[i].enc_drev[3] = motor_param[i].enc_rev * 4 / 6;
		motor_param[i].enc_drev[4] = motor_param[i].enc_rev * 5 / 6;
		motor_param[i].enc_drev[5] = motor_param[i].enc_rev;

		motor_param[i].enc_10hz = motor_param[i].enc_rev * 10 * 4 / 1000;

		if( hall[i] & HALL_U )
		{
			if( hall[i] & HALL_V )
			{
				motor_param[i].enc0 = motor[i].pos - motor_param[i].enc_rev * 5 / 12;	// 150度
			}
			else if( hall[i] & HALL_W )
			{
				motor_param[i].enc0 = motor[i].pos - motor_param[i].enc_rev * 1 / 12;	// 30度
			}
			else
			{
				motor_param[i].enc0 = motor[i].pos - motor_param[i].enc_rev * 3 / 12;	// 90度
			}
		}
		else
		{
			if( !( hall[i] & HALL_V ) )
			{
				motor_param[i].enc0 = motor[i].pos - motor_param[i].enc_rev * 11 / 12;	// 330度
			}
			else if( !( hall[i] & HALL_W ) )
			{
				motor_param[i].enc0 = motor[i].pos - motor_param[i].enc_rev * 7 / 12;	// 210度
			}
			else
			{
				motor_param[i].enc0 = motor[i].pos - motor_param[i].enc_rev * 9 / 12;	// 270度
			}
		}

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
			val = ( fp4sin( FP4_PI2 * j / motor_param[i].enc_rev )
					+ fp4mul( fp4sin( FP4_PI2 * 3 * j / motor_param[i].enc_rev ), DOUBLE2FP4( 0.1547 ) ) );
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
	driver_param.enable_watchdog = bwatchdog;
	// PIO_Set( &pinsLeds[USBD_LEDPOWER] );
}

// ------------------------------------------------------------------------------
// / PWM control interrupt (every PWM period) 20us/50us
// ------------------------------------------------------------------------------
// void FIQ_PWMPeriod( const Pin *pPin )
void FIQ_PWMPeriod(  )
{
	static unsigned short enc[2];
	static unsigned short _enc[2];
	static unsigned short hall[2];
	static unsigned short _hall[2];
	// static int test = 0;
	static int i;
	static int init = 0;
	static int cnt = 0;

	// AT91C_BASE_AIC->AIC_ICCR = 1 << AT91C_ID_IRQ0;

	if( driver_param.servo_level < SERVO_LEVEL_TORQUE )
		return;
	// PIO_Clear( &pinsLeds[USBD_LEDPOWER] );

	motor[0].enc = enc[0] = THEVA.MOTOR[0].ENCODER;
	motor[1].enc = enc[1] = THEVA.MOTOR[1].ENCODER;

	hall[0] = *( unsigned short * )&THEVA.MOTOR[0].ROT_DETECTER;
	hall[1] = *( unsigned short * )&THEVA.MOTOR[1].ROT_DETECTER;

	if( !init )
	{
		init = 1;
		_hall[0] = hall[0];
		_hall[1] = hall[1];
		_enc[0] = enc[0];
		_enc[1] = enc[1];

		return;
	}

	motor[0].pos += ( short )( enc[0] - _enc[0] );
	motor[1].pos += ( short )( enc[1] - _enc[1] );
	if( motor[0].pos >= motor_param[0].enc_rev )
		motor[0].pos -= motor_param[0].enc_rev;
	if( motor[1].pos >= motor_param[1].enc_rev )
		motor[1].pos -= motor_param[1].enc_rev;
	if( motor[0].pos < 0 )
		motor[0].pos += motor_param[0].enc_rev;
	if( motor[1].pos < 0 )
		motor[1].pos += motor_param[1].enc_rev;

	// PWM計算
	{
		static int pwm[2][3];
		static int phase[3];
		static int j;

		for( j = 0; j < 2; j++ )
		{
			phase[0] = ( ( motor[j].pos - motor_param[j].enc0 ) ) - phase90[j];
			while( phase[0] < 0 )
				phase[0] += motor_param[j].enc_rev;
			while( phase[0] >= motor_param[j].enc_rev )
				phase[0] -= motor_param[j].enc_rev;

			phase[1] = phase[0] - phase_offset[j][0];
			while( phase[1] < 0 )
				phase[1] += motor_param[j].enc_rev;
			while( phase[1] >= motor_param[j].enc_rev )
				phase[1] -= motor_param[j].enc_rev;

			phase[2] = phase[0] - phase_offset[j][1];
			while( phase[2] < 0 )
				phase[2] += motor_param[j].enc_rev;
			while( phase[2] >= motor_param[j].enc_rev )
				phase[2] -= motor_param[j].enc_rev;

			for( i = 0; i < 3; i++ )
			{
				static int pwmt;
				pwmt = ( ( ( int )SinTB[j][phase[i]] * motor[j].ref.rate ) / 8192 ) + PWM_center;
				if( pwmt < PWM_abs_min )
					pwmt = PWM_abs_min;
				if( pwmt > PWM_abs_max )
					pwmt = PWM_abs_max;
				pwm[j][i] = pwmt;
			}
		}
		for( i = 0; i < 3; i++ )
		{
			THEVA.MOTOR[0].PWM[i].H = pwm[0][i];
			THEVA.MOTOR[1].PWM[i].H = pwm[1][i];
		}
	}

	// ゼロ点計算
	for( i = 0; i < 2; i++ )
	{
		static char u;
		static char v;
		static char w;

		{
			u = v = w = 0;

			switch ( hall[i] ^ _hall[i] )
			{
			case HALL_U:
				if( !( _hall[i] & HALL_U ) )
					u = 1;						// 正回転 U立ち上がり 0度
				else
					u = -1;						// 正回転 U立ち下がり 180度
				break;
			case HALL_V:
				if( !( _hall[i] & HALL_V ) )
					v = 1;						// 正回転 V立ち上がり 120度
				else
					v = -1;						// 正回転 V立ち下がり 300度
				break;
			case HALL_W:
				if( !( _hall[i] & HALL_W ) )
					w = 1;						// 正回転 W立ち上がり 240度
				else
					w = -1;						// 正回転 W立ち下がり 60度
				break;
			default:
				// エラー
				break;
			}

			// 逆回転
			if( motor[i].vel < -8 )
			{
				u = -u;
				v = -v;
				w = -w;
			}
			else if( motor[i].vel <= 8 )
			{
				continue;
			}

			// ホール素子は高速域では信頼できない
			if( motor[i].vel < -motor_param[i].enc_10hz || motor[i].vel > motor_param[i].enc_10hz )
			{
			//	continue;
			}

			// ゼロ点計算

			if( w == -1 )
				motor_param[i].enc0 = motor[i].pos - motor_param[i].enc_drev[0];
			else if( v == 1 )
				motor_param[i].enc0 = motor[i].pos - motor_param[i].enc_drev[1];
			else if( u == -1 )
				motor_param[i].enc0 = motor[i].pos - motor_param[i].enc_drev[2];
			else if( w == 1 )
				motor_param[i].enc0 = motor[i].pos - motor_param[i].enc_drev[3];
			else if( v == -1 )
				motor_param[i].enc0 = motor[i].pos - motor_param[i].enc_drev[4];
			else if( u == 1 )
				motor_param[i].enc0 = motor[i].pos - motor_param[i].enc_drev[5];
		}
	}

	_hall[0] = hall[0];
	_hall[1] = hall[1];

	if( cnt++ % 20 == 0 )
	{
		static int _vel[2][16] = { {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0} };
		static unsigned short __enc[2];
		static unsigned char cnt = 0;
		int vel;

		velcontrol = 1;
		for( i = 0; i < 2; i++ )
		{
			int j, n;
			int __vel;
			
			__vel = ( short )( enc[i] - __enc[i] );
			_vel[i][cnt] = __vel;
			j = cnt;
			
			if( _abs( __vel ) < 8 )
			{
				vel = 0;
				for( n = 0; n < 16; n ++ ) vel += _vel[i][n];
			}
			else if( _abs( __vel ) < 16 )
			{
				vel = 0;
				for( n = 0; n < 8; n ++ )
				{
					vel += _vel[i][j];
					if( j == 0 ) j = 15;
					else j--;
				}
				vel *= 2;
			}
			else if( _abs( __vel ) < 32 )
			{
				vel = 0;
				for( n = 0; n < 4; n ++ )
				{
					vel += _vel[i][j];
					if( j == 0 ) j = 15;
					else j--;
				}
				vel *= 4;
			}
			else
			{
				vel = __vel * 16;
			}
			
			motor[i].vel = vel;
			__enc[i] = enc[i];
			motor[i].enc_buf = enc[i];
		}
		cnt++;
		if( cnt >= 16 )
			cnt = 0;
	}

	_enc[0] = enc[0];
	_enc[1] = enc[1];

	// PIO_Set( &pinsLeds[USBD_LEDPOWER] );

	return;
}

// ------------------------------------------------------------------------------
// / Configure velocity control loop
// ------------------------------------------------------------------------------
void controlPWM_init(  )
{
	/* 
	 * PIO_Configure( &pinPWMCycle, 1 ); AIC_ConfigureIT( AT91C_ID_FIQ, AT91C_AIC_SRCTYPE_EXT_NEGATIVE_EDGE,
	 * FIQ_PWMPeriod );
	 * 
	 * // AIC_EnableIT(AT91C_ID_FIQ); */

	PIO_Configure( &pinPWMCycle2, 1 );
	AIC_ConfigureIT( AT91C_ID_IRQ0, 0 | AT91C_AIC_SRCTYPE_POSITIVE_EDGE, ( void ( * )( void ) )FIQ_PWMPeriod );
	AIC_EnableIT( AT91C_ID_IRQ0 );

	/* 
	 * PIO_Configure(&pinPWMCycle, 1); PIO_ConfigureIt(&pinPWMCycle, FIQ_PWMPeriod); PIO_EnableIt(&pinPWMCycle); */
}

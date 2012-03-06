//-----------------------------------------------------------------------------
//         Headers
//------------------------------------------------------------------------------

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


static const Pin pinsLeds[] = {PINS_LEDS};
static const unsigned int numLeds = PIO_LISTSIZE(pinsLeds);

static const Pin pinPWMCycle  = PIN_PWM_CYCLE;
static const Pin pinPWMCycle2 = PIN_PWM_CYCLE2;

/// PWM Enable pin instance.
static const Pin pinPWMEnable = PIN_PWM_ENABLE;

//static long enc2phase[2];
short SinTB[2][4096];
int phase_offset[2][2];
int phase90[2];

extern int velcontrol;

void controlPWM_config( )
{
	static unsigned short hall[2];
	static int i, j;
	
	hall[0] = *(unsigned short*)&THEVA.MOTOR[0].ROT_DETECTER;
	hall[1] = *(unsigned short*)&THEVA.MOTOR[1].ROT_DETECTER;
	
	THEVA.GENERAL.PWM.HALF_PERIOD  = 1200;
	THEVA.GENERAL.PWM.DEADTIME	= 36;

//	PIO_Clear( &pinsLeds[USBD_LEDPOWER] );
	for( i = 0; i < 2; i ++ )
	{
	//	enc2phase[i] = 2000 / motor_param[i].enc_rev;

		THEVA.MOTOR[i].PWM[0].H = 0;
		THEVA.MOTOR[i].PWM[1].H = 0;
		THEVA.MOTOR[i].PWM[2].H = 0;
		THEVA.MOTOR[i].PWM[0].L = driver_param.PWM_max;
		THEVA.MOTOR[i].PWM[1].L = driver_param.PWM_max;
		THEVA.MOTOR[i].PWM[2].L = driver_param.PWM_max;

		motor_param[i].enc_drev[0] = motor_param[i].enc_rev     / 6;
		motor_param[i].enc_drev[1] = motor_param[i].enc_rev * 2 / 6;
		motor_param[i].enc_drev[2] = motor_param[i].enc_rev * 3 / 6;
		motor_param[i].enc_drev[3] = motor_param[i].enc_rev * 4 / 6;
		motor_param[i].enc_drev[4] = motor_param[i].enc_rev * 5 / 6;
		motor_param[i].enc_drev[5] = motor_param[i].enc_rev;

		if( hall[i] & HALL_U )
		{
			if( hall[i] & HALL_V )
			{
				motor_param[i].enc0 = motor[i].pos - motor_param[i].enc_rev *  5 / 12;	// 150“x
			}
			else if( hall[i] & HALL_W )
			{
				motor_param[i].enc0 = motor[i].pos - motor_param[i].enc_rev *  1 / 12;	// 30“x
			}
			else
			{
				motor_param[i].enc0 = motor[i].pos - motor_param[i].enc_rev *  3 / 12;	// 90“x
			}
		}
		else
		{
			if( !( hall[i] & HALL_V ) )
			{
				motor_param[i].enc0 = motor[i].pos - motor_param[i].enc_rev * 11 / 12;	// 330“x
			}
			else if( !( hall[i] & HALL_W ) )
			{
				motor_param[i].enc0 = motor[i].pos - motor_param[i].enc_rev *  7 / 12;	// 210“x
			}
			else
			{
				motor_param[i].enc0 = motor[i].pos - motor_param[i].enc_rev *  9 / 12;	// 270“x
			}
		}

		phase90[i] = motor_param[i].enc_rev - motor_param[i].enc_rev / 4;
		phase_offset[i][ 0 ] = motor_param[i].enc_rev / 3;
		phase_offset[i][ 1 ] = motor_param[i].enc_rev * 2 / 3;
		if( i == 1 && motor_param[1].enc_rev == motor_param[0].enc_rev )
		{
			for( j = 0; j < motor_param[i].enc_rev; j ++ )
			{
				SinTB[1][j] = SinTB[0][j];
			}
			break;
		}
		for( j = 0; j < motor_param[i].enc_rev/2; j ++ )
		{
			fixp4 val;
			int ival;
			val = ( fp4sin( FP4_PI2 * j / motor_param[i].enc_rev )
					 + fp4mul( fp4sin( FP4_PI2 * 3 * j / motor_param[i].enc_rev ), DOUBLE2FP4(0.1547) ) );
			// = ( 2.0 / sqrt( 3.0 ) - 1.0 )

			ival = val * /*4730*/3547 / FP4_ONE;
		
			if( ival > 4096 ) ival = 4096;
			else if( ival < -4096 ) ival = -4096;
		
			SinTB[i][j] = ival;
		}
		for( j = 0; j < motor_param[i].enc_rev/2; j ++ )
		{
			SinTB[i][j+motor_param[i].enc_rev/2] = -SinTB[i][j];
		}
	}
//	PIO_Set( &pinsLeds[USBD_LEDPOWER] );
}


//------------------------------------------------------------------------------
/// PWM control interrupt (every PWM period) 20us/50us
//------------------------------------------------------------------------------
//void FIQ_PWMPeriod( const Pin *pPin )
void FIQ_PWMPeriod( )
{
	static unsigned short enc[2];
	static unsigned short _enc[2];
	static unsigned short hall[2];
	static unsigned short _hall[2];
	//static int test = 0;
	static int i;
	static int init = 0;
	static int cnt = 0;

//    AT91C_BASE_AIC->AIC_ICCR = 1 << AT91C_ID_IRQ0;

	if( driver_param.servo_level < SERVO_LEVEL_TORQUE ) return;
	//PIO_Clear( &pinsLeds[USBD_LEDPOWER] );

	motor[0].enc = enc[0] = THEVA.MOTOR[0].ENCODER;
	motor[1].enc = enc[1] = THEVA.MOTOR[1].ENCODER;

	hall[0] = *(unsigned short*)&THEVA.MOTOR[0].ROT_DETECTER;
	hall[1] = *(unsigned short*)&THEVA.MOTOR[1].ROT_DETECTER;

	if( !init )
	{
		init = 1;
		_hall[0] = hall[0];
		_hall[1] = hall[1];
		_enc[0] = enc[0];
		_enc[1] = enc[1];

		return;
	}

	motor[0].pos += (short)( enc[0] - _enc[0] );
	motor[1].pos += (short)( enc[1] - _enc[1] );
	if( motor[0].pos >= motor_param[0].enc_rev ) motor[0].pos -= motor_param[0].enc_rev;
	if( motor[1].pos >= motor_param[1].enc_rev ) motor[1].pos -= motor_param[1].enc_rev;
	if( motor[0].pos < 0 ) motor[0].pos += motor_param[0].enc_rev;
	if( motor[1].pos < 0 ) motor[1].pos += motor_param[1].enc_rev;

	// PWMŒvŽZ
	{
		static int pwm[2][3];
		static int phase[3];
		static int j;

		for( j = 0; j < 2; j ++ )
		{
			phase[0] = ( ( motor[j].pos - motor_param[j].enc0) ) - phase90[j];
			while( phase[0] < 0 ) phase[0] += motor_param[j].enc_rev;
			while( phase[0] >= motor_param[j].enc_rev ) phase[0] -= motor_param[j].enc_rev;

			phase[1] = phase[0] - phase_offset[j][ 0 ];
			while( phase[1] < 0 ) phase[1] += motor_param[j].enc_rev;
			while( phase[1] >= motor_param[j].enc_rev ) phase[1] -= motor_param[j].enc_rev;

			phase[2] = phase[0] - phase_offset[j][ 1 ];
			while( phase[2] < 0 ) phase[2] += motor_param[j].enc_rev;
			while( phase[2] >= motor_param[j].enc_rev ) phase[2] -= motor_param[j].enc_rev;

			for( i = 0; i < 3; i ++ )
			{
				static int pwmt;
				pwmt = ( ( (int)SinTB[j][ phase[i] ] * motor[j].ref.rate ) / 8192 ) + driver_param.PWM_max / 2;
				if( pwmt < 0 ) pwmt = 0;
				if( pwmt >= driver_param.PWM_max ) pwmt = driver_param.PWM_max - 1;
				pwm[j][i] = pwmt;
			}
		}
		for( i = 0; i < 3; i ++ )
		{
			THEVA.MOTOR[0].PWM[i].H = pwm[0][i];
			THEVA.MOTOR[1].PWM[i].H = pwm[1][i];
		}
	}
	
	// ƒ[ƒ“_ŒvŽZ
	for( i = 0; i < 2; i ++ )
	{
		static char u;
		static char v;
		static char w;
		
		//if( -20 < motor[i].vel && motor[i].vel < 20 )
		{
			u = v = w = 0;

			switch( hall[i] ^ _hall[i] )
			{
			case HALL_U:
				if( !( _hall[i] & HALL_U ) )
					u = 1;  // ³‰ñ“] U—§‚¿ã‚ª‚è 0“x
				else
					u = -1; // ³‰ñ“] U—§‚¿‰º‚ª‚è 180“x
				break;
			case HALL_V:
				if( !( _hall[i] & HALL_V ) )
					v = 1;  // ³‰ñ“] V—§‚¿ã‚ª‚è 120“x
				else
					v = -1; // ³‰ñ“] V—§‚¿‰º‚ª‚è 300“x
				break;
			case HALL_W:
				if( !( _hall[i] & HALL_W ) )
					w = 1;  // ³‰ñ“] W—§‚¿ã‚ª‚è 240“x
				else
					w = -1; // ³‰ñ“] W—§‚¿‰º‚ª‚è 60“x
				break;
			default:
				// ƒGƒ‰[
				break;
			}

			// ‹t‰ñ“]
			if( motor[i].vel < 0 )
			{
				u = -u; v = -v; w = -w;
			}
		
			// ƒ[ƒ“_ŒvŽZ
		
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
	_enc[0] = enc[0];
	_enc[1] = enc[1];
	
	if( cnt ++ % 20 == 0 ) velcontrol = 1;

	//PIO_Set( &pinsLeds[USBD_LEDPOWER] );
	
	return;
}

//------------------------------------------------------------------------------
/// Configure velocity control loop
//------------------------------------------------------------------------------
void controlPWM_init( )
{
/*
    PIO_Configure( &pinPWMCycle, 1 );
	AIC_ConfigureIT( AT91C_ID_FIQ, AT91C_AIC_SRCTYPE_EXT_NEGATIVE_EDGE, FIQ_PWMPeriod );

//	AIC_EnableIT(AT91C_ID_FIQ);
*/

    PIO_Configure( &pinPWMCycle2, 1 );
	AIC_ConfigureIT( AT91C_ID_IRQ0, 0 | AT91C_AIC_SRCTYPE_POSITIVE_EDGE, (void (*)(void))FIQ_PWMPeriod );
	AIC_EnableIT(AT91C_ID_IRQ0);

/*
    PIO_Configure(&pinPWMCycle, 1);
    PIO_ConfigureIt(&pinPWMCycle, FIQ_PWMPeriod);
    PIO_EnableIt(&pinPWMCycle);
*/
}




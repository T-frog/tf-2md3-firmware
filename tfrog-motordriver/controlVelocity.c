// -----------------------------------------------------------------------------
// Headers
// ------------------------------------------------------------------------------

#include <board.h>
#include <pio/pio.h>
#include <pio/pio_it.h>
#include <aic/aic.h>
#include <tc/tc.h>
#include <pmc/pmc.h>
#include <utility/trace.h>
#include <utility/led.h>
#include <usb/device/cdc-serial/CDCDSerialDriver.h>
#include <usb/device/cdc-serial/CDCDSerialDriverDescriptors.h>
#include <string.h>
#include <stdint.h>

#include "power.h"
#include "controlPWM.h"
#include "controlVelocity.h"
#include "registerFPGA.h"
#include "filter.h"

MotorState motor[2];
MotorParam motor_param[2];
DriverParam driver_param;

extern int watchdog;
extern int velcontrol;

static const Pin pinsLeds[] = { PINS_LEDS };

static const unsigned int numLeds = PIO_LISTSIZE( pinsLeds );

// / PWM Enable pin instance.
static const Pin pinPWMEnable = PIN_PWM_ENABLE;

Filter1st accelf[2];
Filter1st accelf0;

// ------------------------------------------------------------------------------
// / Velocity control loop (1ms)
//   Called from main loop
// ------------------------------------------------------------------------------
void ISR_VelocityControl(  )
{
	// volatile unsigned int status;
	static int pwm_sum[2] = { 0, 0 };
	int i;

	// PIO_Clear(&pinsLeds[USBD_LEDOTHER]);

	// status = AT91C_BASE_TC0->TC_SR;

	if( driver_param.servo_level >= SERVO_LEVEL_TORQUE )
	{
		// servo_level 2(toque enable)
		int64_t toq[2];
		int64_t out_pwm[2];

		if( driver_param.servo_level >= SERVO_LEVEL_VELOCITY && 
			driver_param.servo_level != SERVO_LEVEL_OPENFREE )
		{
			// servo_level 3 (speed enable)
			int64_t toq_pi[2], s_a, s_b;

			for( i = 0; i < 2; i++ )
			{
				motor[i].ref.vel_interval++;
				if( motor[i].ref.vel_changed )
				{
					motor[i].ref.vel_buf = motor[i].ref.vel;
					if( motor[i].control_init )
					{
						motor[i].ref.vel_diff = 0;
						motor[i].control_init = 0;
					}
					else
					{
						motor[i].ref.vel_diff = ( motor[i].ref.vel_buf - motor[i].ref.vel_buf_prev )
							 * 1000000 / motor[i].ref.vel_interval;
						// [cnt/msms] * 1000[ms/s] * 1000[ms/s] = [cnt/ss]
					}

					motor[i].ref.vel_buf_prev = motor[i].ref.vel_buf;
					motor[i].ref.vel_interval = 0;

					motor[i].ref.vel_changed = 0;
				}

				// 積分
				if( motor[i].control_init )
				{
					motor[i].error = 0;
					motor[i].error_integ = 0;
				}
				else
				{
					motor[i].error = motor[i].ref.vel_buf - motor[i].vel;
					motor[i].error_integ += motor[i].error;
				}
				if( motor[i].error_integ > driver_param.integ_max )
				{
					motor[i].error_integ = driver_param.integ_max;
				}
				else if( motor[i].error_integ < driver_param.integ_min )
				{
					motor[i].error_integ = driver_param.integ_min;
				}

				// PI制御分 単位：加速度[cnt/ss]
				toq_pi[i]  = motor[i].error * 1000 * motor_param[i].Kp; // [cnt/ms] * 1000[ms/s] * Kp[1/s] = [cnt/ss]
				toq_pi[i] += motor[i].error_integ * motor_param[i].Ki; // [cnt] * Ki[1/ss] = [cnt/ss]
			}

			// PWSでの相互の影響を考慮したフィードフォワード
			s_a = ( toq_pi[0] + Filter1st_Filter( &accelf[0], motor[0].ref.vel_diff ) ) / 16;
			s_b = ( toq_pi[1] + Filter1st_Filter( &accelf[1], motor[1].ref.vel_diff ) ) / 16;

			// Kdynamics[TORQUE_UNIT 2pi/cntrev kgf m m], s_a/s_b[cnt/ss]
			toq[0] = ( s_a * driver_param.Kdynamics[0]
					   + s_b * driver_param.Kdynamics[2] + motor[0].ref.vel_buf * driver_param.Kdynamics[4] ) / 256;
			toq[1] = ( s_b * driver_param.Kdynamics[1]
					   + s_a * driver_param.Kdynamics[3] + motor[1].ref.vel_buf * driver_param.Kdynamics[5] ) / 256;
		}
		else
		{
			// servo_level 2(toque enable)
			toq[0] = 0;
			toq[1] = 0;
			motor[0].ref.vel_buf_prev = motor[0].vel;
			motor[1].ref.vel_buf_prev = motor[1].vel;
			motor[0].ref.vel_buf = motor[0].vel;
			motor[1].ref.vel_buf = motor[1].vel;
			motor[0].ref.vel = motor[0].vel;
			motor[1].ref.vel = motor[1].vel;
			motor[0].error_integ = motor[1].error_integ = 0;
			motor[0].ref.vel_diff = motor[1].ref.vel_diff = 0;
			Filter1st_Filter( &accelf[0], 0 );
			Filter1st_Filter( &accelf[1], 0 );
		}

		// 出力段
		for( i = 0; i < 2; i++ )
		{
			// トルクでクリッピング
			if( toq[i] >= motor_param[i].torque_max )
			{
				toq[i] = motor_param[i].torque_max;
			}
			if( toq[i] <= motor_param[i].torque_min )
			{
				toq[i] = motor_param[i].torque_min;
			}

			// 摩擦補償（線形）
			if( motor[i].vel > 0 )
			{
				toq[i] += ( motor_param[i].fr_wplus * motor[i].vel / 16 + motor_param[i].fr_plus );
			}
			else if( motor[i].vel < 0 )
			{
				toq[i] -= ( motor_param[i].fr_wminus * ( -motor[i].vel ) / 16 + motor_param[i].fr_minus );
			}
			// トルク補償
			toq[i] += motor_param[i].torque_offset;

			// トルクでクリッピング
			if( toq[i] >= motor_param[i].torque_limit )
			{
				toq[i] = motor_param[i].torque_limit;
			}
			if( toq[i] <= -motor_param[i].torque_limit )
			{
				toq[i] = -motor_param[i].torque_limit;
			}

			// トルク→pwm変換
			if( motor[i].dir == 0 )
			{
				out_pwm[i]  = 0;
			}
			else
			{
				out_pwm[i]  = ( (int64_t)motor[i].vel * motor_param[i].Kvolt ) / 16;
			}
			// PWMでクリッピング
			if( out_pwm[i] > driver_param.PWM_max * 65536 )
				out_pwm[i] = driver_param.PWM_max * 65536;
			if( out_pwm[i] < driver_param.PWM_min * 65536 )
				out_pwm[i] = driver_param.PWM_min * 65536;

			motor[i].ref.torque = toq[i] * motor_param[i].Kcurrent;
			out_pwm[i] += motor[i].ref.torque;
			out_pwm[i] /= 65536;

			// PWMでクリッピング
			if( out_pwm[i] > driver_param.PWM_max - 1 )
				out_pwm[i] = driver_param.PWM_max - 1;
			if( out_pwm[i] < driver_param.PWM_min + 1 )
				out_pwm[i] = driver_param.PWM_min + 1;

		}

		// 出力
		motor[0].ref.rate = out_pwm[0];
		motor[1].ref.rate = out_pwm[1];

		pwm_sum[0] += out_pwm[0];
		pwm_sum[1] += out_pwm[1];

		driver_param.cnt_updated++;
		if( driver_param.cnt_updated == 5 )
		{
			// static long cnt = 0;
			motor[0].ref.rate_buf = pwm_sum[0];
			motor[1].ref.rate_buf = pwm_sum[1];
			pwm_sum[0] = 0;
			pwm_sum[1] = 0;
		}
	}		// servo_level 2
	else
	{
		motor[0].ref.rate = 0;
		motor[1].ref.rate = 0;
		motor[0].ref.vel_buf_prev = motor[0].vel;
		motor[1].ref.vel_buf_prev = motor[1].vel;
		motor[0].ref.vel_buf = motor[0].vel;
		motor[1].ref.vel_buf = motor[1].vel;
		motor[0].ref.vel = motor[0].vel;
		motor[1].ref.vel = motor[1].vel;
		Filter1st_Filter( &accelf[0], 0 );
		Filter1st_Filter( &accelf[1], 0 );
	}
	// PIO_Set(&pinsLeds[USBD_LEDOTHER]);
}

void timer0_vel_calc( )
{
	static unsigned short __enc[2];
	unsigned short enc[2];
	int _nspd[2];
	int _spd[2];
	static char _spd_cnt[2]; 
	int i;
	volatile unsigned int dummy;

	if( driver_param.servo_level > SERVO_LEVEL_STOP )
	{
		driver_param.watchdog ++;
	}
	
	dummy = AT91C_BASE_TC0->TC_SR;
	dummy = dummy;
	
	LED_on(1);
	for( i = 0; i < 2; i++ )
	{
		enc[i] = motor[i].enc;
		_nspd[i] = motor[i].spd_num;
		_spd[i]  = motor[i].spd_sum;
		motor[i].spd_num = 0;
		motor[i].spd_sum = 0;
	}

	for( i = 0; i < 2; i++ )
	{
		int __vel;
		int vel;

		__vel = ( short )( enc[i] - __enc[i] );
		motor[i].vel1 = __vel;

		if( _abs( __vel ) > 6 || driver_param.fpga_version == 0 ) 
		{
			motor[i].spd = 1000 * 256;
			vel = __vel * 16;
		}
		else if( _nspd[i] >= 1 )
		{
			motor[i].spd = _spd[i] / _nspd[i];
			_spd_cnt[i] = 256 / _abs( motor[i].spd );
			vel = motor[i].spd / 256;
		}
		else if( _spd_cnt[i] > 0 )
		{
			_spd_cnt[i] --;
			vel = motor[i].spd / 256;
		}
		else
		{
			vel = 0;
		}
		if( vel < 0 ) motor[i].dir = -1;
		else if( vel > 0 ) motor[i].dir = 1;
		else motor[i].dir = 0;
		
		motor[i].vel = vel;
		__enc[i] = enc[i];
		motor[i].enc_buf = enc[i];
	}

	LED_off(1);

	velcontrol = 1;
}

// ------------------------------------------------------------------------------
// / Configure velocity control loop
// ------------------------------------------------------------------------------
void controlVelocity_init(  )
{
#define ACCEL_FILTER_TIME  15.0
	int i;

	Filter1st_CreateLPF( &accelf0, ACCEL_FILTER_TIME );
	accelf[0] = accelf[1] = accelf0;

	driver_param.cnt_updated = 0;
	driver_param.watchdog = 0;
	driver_param.watchdog_limit = 600;
	driver_param.servo_level = SERVO_LEVEL_STOP;
	driver_param.admask = 0;
	driver_param.io_mask[0] = 0;
	driver_param.io_mask[1] = 0;

	for( i = 0; i < 2; i ++ )
	{
		motor[i].ref.vel = 0;
		motor[i].ref.vel_buf = 0;
		motor[i].ref.vel_buf_prev = 0;
		motor[i].ref.vel_diff = 0;
		motor[i].error_integ = 0;
		motor[i].control_init = 0;
		motor_param[i].motor_type = MOTOR_TYPE_AC3;
		motor_param[i].enc_rev = 0;
		motor_param[i].phase_offset = 0;
	}

	{
		volatile unsigned int dummy;

		AT91C_BASE_PMC->PMC_PCER = 1 << AT91C_ID_TC0;

		AT91C_BASE_TC0->TC_CCR = AT91C_TC_CLKDIS;
		AT91C_BASE_TC0->TC_IDR = 0xFFFFFFFF;
		dummy = AT91C_BASE_TC0->TC_SR;
		dummy = dummy;

		// MCK/32 * 1500 -> 1ms
		AT91C_BASE_TC0->TC_CMR = AT91C_TC_CLKS_TIMER_DIV3_CLOCK | AT91C_TC_WAVE | AT91C_TC_WAVESEL_UP_AUTO;
		AT91C_BASE_TC0->TC_CCR = AT91C_TC_CLKEN;
		AT91C_BASE_TC0->TC_RC  = 1500;
		AT91C_BASE_TC0->TC_IER = AT91C_TC_CPCS;

		AIC_ConfigureIT( AT91C_ID_TC0, 4 | AT91C_AIC_SRCTYPE_POSITIVE_EDGE, ( void ( * )( void ) )timer0_vel_calc );
		AIC_EnableIT( AT91C_ID_TC0 );

		AT91C_BASE_TC0->TC_CCR = AT91C_TC_SWTRG;
	}

}


#ifndef __CONTROL_VELOCITY_H__
#define __CONTROL_VELOCITY_H__

#include "communication.h"

typedef struct _MotorState
{
	int				vel;		// count/ms
	int				pos;		// count
	int				enc_buf;	// count
	unsigned short	enc;

	struct{
		int			vel;		// count/ms
		int			vel_buf;	// count/ms
		char		vel_changed;
		int			vel_interval;
		int			vel_diff;	// count/ms
		int			torque;		// 1/100000 Nm
		int			rate;		// -PWM_max < rate < PWM_max
		int			rate_buf;	// 
	} ref;
	int			error_integ;
} MotorState;

typedef struct _MotorParam
{
	unsigned short	enc_rev;	// count/rev
	unsigned short	enc_drev[6];
	int				enc0;		// count
	int				vel_max;	// count/ms
	int				Kcurrent;
	int				Kvolt;
	int				Kp;			// 1/s
	int				Ki;			// 1/ss
	int				torque_max;
	int				torque_min;
	int				torque_offset;
	int 			fr_plus;
	int 			fr_wplus;
	int 			fr_minus;
	int 			fr_wminus;
} MotorParam;

typedef struct _DriverParam
{
	int				PWM_max;	// clock
	int				PWM_min;	// clock
	int				integ_max;	// 
	int				integ_min;	// 
	int				Kdynamics[6];
	YPSpur_servo_level servo_level;
	unsigned short	watchdog_limit;
	unsigned short	watchdog;
	unsigned short	enable_watchdog;
	unsigned char	cnt_updated;
	unsigned short	admask;
	unsigned short	io_dir;
	unsigned short	io_mask;
} DriverParam;

extern MotorState	motor[2];
extern MotorParam	motor_param[2];
extern DriverParam	driver_param;

void controlVelocity_init( );
RAMFUNC void ISR_VelocityControl( );

#endif


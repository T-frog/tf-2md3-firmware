#ifndef __CONTROL_VELOCITY_H__
#define __CONTROL_VELOCITY_H__

#include "communication.h"

typedef struct _MotorState
{
	int vel;									// count/ms
	int vel1;									// count/ms
	int pos;									// count
	int enc_buf;								// count
	int spd;
	int spd_sum;
	int spd_num;
	unsigned short enc;
	short dir;

	struct
	{
		int vel;								// count/ms
		int vel_buf;							// count/ms
		int vel_buf_prev;						// count/ms
		int vel_interval;
		int vel_diff;							// count/ms
		int torque;								// 1/100000 Nm
		int rate;								// -PWM_max < rate < PWM_max
		int rate2;								// 
		int rate_buf;							// 
		char vel_changed;
	} ref;
	int error;
	int error_integ;
	char control_init;
	YPSpur_servo_level servo_level;
} MotorState;

typedef struct _MotorParam
{
	unsigned short enc_rev;						// count/rev
	unsigned short phase_offset;
	unsigned short enc_10hz;
	unsigned short enc_drev[6];
	unsigned int enc_mul;
	int enc0;									// count
	int enc0tran;									// count
	int vel_max;								// count/ms
	int Kcurrent;
	int Kvolt;
	int Kp;										// 1/s
	int Ki;										// 1/ss
	int torque_max;
	int torque_min;
	int torque_limit;
	int torque_offset;
	int fr_plus;
	int fr_wplus;
	int fr_minus;
	int fr_wminus;
	enum _motor_type_t
	{
		MOTOR_TYPE_DC,
		MOTOR_TYPE_AC3
	} motor_type;
} MotorParam;

typedef struct _DriverParam
{
	int PWM_max;								// clock
	int PWM_min;								// clock
	int PWM_resolution;							// clock
	int integ_max;								// cnt
	int integ_min;								// cnt
	int vsrc_rated;
	int vsrc_factor;
	int vsrc;
	int zero_torque;
	int Kdynamics[6];
	unsigned short watchdog_limit;
	unsigned short watchdog;
	unsigned char cnt_updated;
	unsigned short admask;
	unsigned short io_dir;
	unsigned short io_mask[2];
	unsigned short fpga_version;
	unsigned char ifmode;
	struct
	{
		unsigned char low_voltage;
		unsigned char hall[2];
	} error;
	enum
	{
		ERROR_NONE = 0,
		ERROR_LOW_VOLTAGE	= 0x0001,
		ERROR_HALL1			= 0x0002,
		ERROR_HALL2			= 0x0004,
		ERROR_WATCHDOG		= 0x0008
	} error_state;
} DriverParam;

#define ERROR_NUM 4

extern MotorState motor[2];
extern MotorParam motor_param[2];
extern DriverParam driver_param;

#define COM_MOTORS 16
extern short com_cnts[COM_MOTORS];
extern short com_pwms[COM_MOTORS];
extern char com_en[COM_MOTORS];

void controlVelocity_init(  );
RAMFUNC void ISR_VelocityControl(  );
RAMFUNC void timer0_vel_calc(  );

#endif

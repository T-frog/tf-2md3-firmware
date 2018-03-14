#ifndef __CONTROL_VELOCITY_H__
#define __CONTROL_VELOCITY_H__

#include <board.h>
#include <stdint.h>
#include "communication.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct _MotorState
{
	int vel;									// count/ms
	int vel1;									// count/ms
	int pos;									// count
	unsigned int posc;							// count
	int enc_buf;								// count
	int enc_buf2;								// count
	int spd;
	int spd_cnt;
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
	int64_t error_integ;
	char control_init;
	YPSpur_servo_level servo_level;
} MotorState;

typedef struct _MotorParam
{
	unsigned int enc_rev;						// count/rev
	unsigned int phase_offset;
	unsigned int enc_rev_h;					// count/rev
	unsigned int enc_rev_1p;
	unsigned char enc_type;
	unsigned int enc_10hz;
	unsigned int enc_drev[6];
	unsigned int enc_mul;
	int enc0;									// count
	int enc0tran;								// count
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
	int integ_max;								// cnt
	int integ_min;								// cnt
	int inertia_self;
	int inertia_cross;
	enum _motor_type_t
	{
		MOTOR_TYPE_DC,
		MOTOR_TYPE_AC3
	} motor_type;
	unsigned short enc_div;
} MotorParam;

typedef struct _DriverParam
{
	int PWM_max;								// clock
	int PWM_min;								// clock
	int PWM_resolution;							// clock
	int control_cycle;
	int control_s;
	int vsrc_rated;
	int vsrc_factor;
	int vsrc;
	int zero_torque;
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
	enum
	{
		BOARD_R4,
		BOARD_R6A,
		BOARD_R6B
	} board_version;
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
void controlVelocity_config(  );
void ISR_VelocityControl(  );
void timer0_vel_calc(  );


#ifdef __cplusplus
}
#endif

#endif

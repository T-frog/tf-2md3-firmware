#ifndef __COMMUNICATION_H__
#define __COMMUNICATION_H__

typedef union _Integer4
{
	int integer;
	char byte[4];
} Integer4;

typedef union _Integer2
{
	short integer;
	char byte[2];
} Integer2;

// typedef 
typedef enum _YPSpur_servo_level
{
	SERVO_LEVEL_STOP = 0,
	SERVO_LEVEL_COUNTER,
	SERVO_LEVEL_TORQUE,
	SERVO_LEVEL_VELOCITY,
	SERVO_LEVEL_POSITION						// n/a
} YPSpur_servo_level;

typedef enum
{
	PARAM_w_ref = 0,
	PARAM_w_ref_highprec,
	PARAM_p_ki,
	PARAM_p_kv,
	PARAM_p_fr_plus,
	PARAM_p_fr_wplus,
	PARAM_p_fr_minus,
	PARAM_p_fr_wminus,
	PARAM_p_A,
	PARAM_p_B,
	PARAM_p_C,
	PARAM_p_D,
	PARAM_p_E,
	PARAM_p_F,
	PARAM_p_pi_kp,
	PARAM_p_pi_ki,
	PARAM_pwm_max,
	PARAM_pwm_min,
	PARAM_toq_max,
	PARAM_toq_min,
	PARAM_int_max,
	PARAM_int_min,
	PARAM_p_toq_offset,
	PARAM_toq_limit,
	PARAM_enc_rev,
	PARAM_motor_phase,
	PARAM_servo = 64,
	PARAM_watch_dog_limit,
	PARAM_io_dir = 96,
	PARAM_io_data,
	PARAM_ad_mask
} YPSpur_loco_param;

#define COMMUNICATION_START_BYTE	0x09
#define COMMUNICATION_END_BYTE		0x0a

/* firmware */
#define YP_FIRMWARE_NAME "2013.03.08"
/* parametor files dir */
#define YP_PARAMS_DIR "robot-params"
/* product */
#define YP_PRODUCT_NAME "Yamabico Project - Spur"
/* protocol */
#define YP_PROTOCOL_NAME "YPP:05:00"
/* vendor */
#define YP_VENDOR_NAME "Univ. of Tsukuba - Intelligent Robot Lab."

/* Driver parameters */
#define YP_DRIVERPARAM_MOTORNUM "2"
#define YP_DRIVERPARAM_TORQUEUNIT "100000"


int data_analyze(  );
int data_fetch( unsigned char *data, int len );
int extended_command_analyze( char *data );
int command_analyze( unsigned char *data, int len );
int data_send( short cnt1, short cnt2, short pwm1, short pwm2, short *analog, unsigned short analog_mask );
int decord( unsigned char *src, int len, unsigned char *dst, int buf_max );
int encode( unsigned char *src, int len, unsigned char *dst, int buf_max );

#endif

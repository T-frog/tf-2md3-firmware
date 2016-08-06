#ifndef __COMMUNICATION_H__
#define __COMMUNICATION_H__

#include <board.h>

#ifdef __cplusplus
extern "C" {
#endif

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
	SERVO_LEVEL_POSITION, // not used
	SERVO_LEVEL_OPENFREE
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
	PARAM_vsrc,
	PARAM_p_inertia_self,
	PARAM_p_inertia_cross,
	PARAM_enc_type,
	PARAM_control_cycle,
	PARAM_enc_div,
	PARAM_servo = 64,
	PARAM_watch_dog_limit,
	PARAM_io_dir = 96,
	PARAM_io_data,
	PARAM_ad_mask,
	PARAM_phase_offset,
} YPSpur_loco_param;

#define COMMUNICATION_START_BYTE	0x09
#define COMMUNICATION_END_BYTE		0x0a

#define COMMAND_LEN		(1/*START*/ + 1/*param*/ + 1/*id*/ + 4/*data*/ + 1/*end*/)

/* firmware */
#ifndef YP_FIRMWARE_NAME
#define YP_FIRMWARE_NAME "DEBUG_BUILD_VERSION"
#endif
/* parametor files dir */
#ifndef YP_PARAMS_DIR 
#define YP_PARAMS_DIR "robot-params"
#endif
/* protocol */
#ifndef YP_PROTOCOL_NAME 
#define YP_PROTOCOL_NAME "YPP:08:00"
#endif
/* vendor */
#ifndef YP_VENDOR_NAME 
#define YP_VENDOR_NAME "T-frog Prject"
#endif

/* Driver parameters */
#ifndef YP_DRIVERPARAM_MOTORNUM 
#define YP_DRIVERPARAM_MOTORNUM "2"
#endif


#define SEND_BUF_LEN  128
#define RECV_BUF_LEN  256


int data_analyze(  );
int data_analyze485(  );

int data_fetch( unsigned char *data, int len );
int data_fetch485( unsigned char *data, int len );

int buf_left();

int data_send( short *cnt, short *pwm, char *en, short *analog, unsigned short analog_mask );
int data_send485( short *cnt, short *pwm, char *en, short *analog, unsigned short analog_mask );
int data_pack( short *cnt, short *pwm, char *en, short *analog, unsigned short analog_mask, unsigned char *data );

int decord( unsigned char *src, int len, unsigned char *dst, int buf_max );
int encode( unsigned char *src, int len, unsigned char *dst, int buf_max );

int extended_command_analyze( char *data );
int command_analyze( unsigned char *data, int len );

inline int data_analyze_( unsigned char *receive_buf, 
		volatile int *w_receive_buf, volatile int *r_receive_buf, int fromto);
int data_fetch_( unsigned char *receive_buf, 
		volatile int *w_receive_buf, volatile int *r_receive_buf,
		unsigned char *data, int len );


int hextoi( char *buf );
int atoi( char *buf );
int nhex( char *buf, int data, int len );
int itoa10( char *buf, int data );
int send( char *buf );
int nsend( char *buf, int len );
int send485( char *buf );
int nsend485( char *buf, int len );
void sendclear( void );
void flush( void );
void flush485( void );


#ifdef __cplusplus
}
#endif

#endif

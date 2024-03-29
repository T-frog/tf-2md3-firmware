/* ----------------------------------------------------------------------------
 * Copyright 2011-2019 T-frog Project
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 * ----------------------------------------------------------------------------
 */

#ifndef __COMMUNICATION_H__
#define __COMMUNICATION_H__

#include <stdint.h>
#include <board.h>
#include <assert.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef union _Integer4
{
  int32_t integer;
  char byte[4];
} Integer4;

typedef union _Integer2
{
  int16_t integer;
  char byte[2];
} Integer2;

// typedef
typedef enum _YPSpur_servo_level
{
  SERVO_LEVEL_STOP = 0,
  SERVO_LEVEL_COUNTER,
  SERVO_LEVEL_TORQUE,
  SERVO_LEVEL_VELOCITY,
  SERVO_LEVEL_POSITION,  // not used
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
  PARAM_enc_denominator,
  PARAM_hall_delay_factor,
  PARAM_lr_cutoff_vel,
  PARAM_vmin,
  PARAM_BLOCK0_END,
  PARAM_servo = 64,
  PARAM_watch_dog_limit,
  PARAM_BLOCK1_END,
  PARAM_io_dir = 96,
  PARAM_io_data,
  PARAM_ad_mask,
  PARAM_phase_offset,
  PARAM_protocol_version,
  PARAM_ping,
  PARAM_dump,
  PARAM_BLOCK2_END,
} YPSpur_loco_param;

#ifdef static_assert
static_assert(PARAM_BLOCK0_END <= PARAM_servo, "Parameter enum overwrapped");
static_assert(PARAM_BLOCK1_END <= PARAM_io_dir, "Parameter enum overwrapped");
static_assert(PARAM_BLOCK2_END <= 255, "Parameter enum overflow");
#endif

typedef enum
{
  INT_enc_index_rise = 0,
  INT_enc_index_fall,
  INT_error_state,
  INT_ping_response,
  INT_debug_dump,
} YPSpur_loco_interrupt;

#define COMMUNICATION_START_BYTE 0x09
#define COMMUNICATION_INT_BYTE   0x07
#define COMMUNICATION_END_BYTE   0x0a

#define COMMUNICATION_ID_BROADCAST 0x3f

#define COMMAND_LEN (1 /*START*/ + 1 /*param*/ + 1 /*id*/ + 4 /*data*/ + 1 /*end*/)

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
#define YP_PROTOCOL_NAME "YPP:11:02"
#endif
/* vendor */
#ifndef YP_VENDOR_NAME
#define YP_VENDOR_NAME "T-frog Project"
#endif

/* Driver parameters */
#ifndef YP_DRIVERPARAM_MOTORNUM
#define YP_DRIVERPARAM_MOTORNUM "2"
#endif

#define SEND_BUF_LEN 128
#define RECV_BUF_LEN 256

int32_t data_analyze();
int32_t data_analyze485();

int32_t data_fetch(uint8_t* data, int32_t len);
int32_t data_fetch485(uint8_t* data, int32_t len);

int32_t buf_left();

int32_t data_send(int16_t* cnt, int16_t* pwm, char* en, int16_t* analog, uint16_t analog_mask);
int32_t data_send485(int16_t* cnt, int16_t* pwm, char* en, int16_t* analog, uint16_t analog_mask);
int32_t int_send(const char param, const char id, const int32_t value);
int32_t int_nsend(const void* data, const int32_t len);
int32_t int_send485(const char param, const char id, const int32_t value);
int32_t int_send485to(const char from, const char to, const char param, const char id, const int32_t value);
int32_t data_pack(int16_t* cnt, int16_t* pwm, char* en, int16_t* analog, uint16_t analog_mask, uint8_t* data);

int32_t decord(uint8_t* src, int32_t len, uint8_t* dst, int32_t buf_max);
int32_t encode(const uint8_t* src, int32_t len, uint8_t* dst, int32_t buf_max);

int32_t extended_command_analyze(char* data);
int32_t command_analyze(uint8_t* data, int32_t len);

int32_t data_fetch_(uint8_t* receive_buf,
                    volatile int32_t* w_receive_buf, volatile int32_t* r_receive_buf,
                    uint8_t* data, int32_t len);

int32_t hextoi(char* buf);
int32_t atoi(char* buf);
int32_t nhex(char* buf, int32_t data, int32_t len);
int32_t itoa10(char* buf, int32_t data);
int32_t send(char* buf);
int32_t nsend(char* buf, int32_t len);
int32_t send485(char* buf);
int32_t nsend485(char* buf, int32_t len);
void sendclear(void);
void flush(void);
void flush485(void);

extern volatile int32_t usb_timeout_cnt;

#ifdef __cplusplus
}
#endif

#endif

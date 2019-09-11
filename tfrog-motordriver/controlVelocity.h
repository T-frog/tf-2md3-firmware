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

#ifndef __CONTROL_VELOCITY_H__
#define __CONTROL_VELOCITY_H__

#include <board.h>
#include <stdint.h>
#include <assert.h>

#include "communication.h"
#include "filter.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum _ErrorID
{
  ERROR_NONE = 0,
  ERROR_LOW_VOLTAGE = 0x0001,
  ERROR_HALL_SEQ = 0x0002,
  ERROR_HALL_ENC = 0x0004,
  ERROR_WATCHDOG = 0x0008,
  ERROR_EEPROM = 0x0010,
  ERROR_INTERNAL = 0x0020,
} ErrorID;
#define ERROR_NUM 6

// ENC0_BUF_MAX must be 2^n to reduce computation cost.
#define ENC0_BUF_MAX 64
#define ENC0_DENOMINATOR_MAX ((int)(ENC0_BUF_MAX / 6))
#define ENC0_BUF_UNKNOWN 0x7FFFFFF

static_assert(
    ENC0_BUF_MAX && ((ENC0_BUF_MAX & (ENC0_BUF_MAX - 1)) == 0),
    "ENC0_BUF_MAX must be 2^n");

#define ACCEL_FILTER_TIME 15  // Timeconstant in velocity control steps
#define ENC0_FILTER_TIME 32   // Timeconstant in velocity control steps

typedef struct _MotorState
{
  int vel;                // count/ms
  int vel1;               // count/ms
  int pos;                // count
  unsigned int posc;      // count
  unsigned int enc_buf;   // count
  unsigned int enc_buf2;  // count
  int spd;
  unsigned int spd_cnt;
  unsigned short enc;
  short dir;

  struct
  {
    int vel;           // count/ms
    int vel_buf;       // count/ms
    int vel_buf_prev;  // count/ms
    int vel_interval;
    int vel_diff;       // count/ms
    int torque;         // 1/100000 Nm
    int torque_offset;  // 1/100000 Nm
    int rate;           // -PWM_max < rate < PWM_max
    int rate2;          //
    int rate_buf;       //
    char vel_changed;
  } ref;
  int error;
  int64_t error_integ;
  char control_init;

  char enc0_buf_updated;
  int enc0_buf[ENC0_BUF_MAX];
  unsigned short enc0_buf_len;

  YPSpur_servo_level servo_level;
  ErrorID error_state;
  FilterExp enc0_lpf;
} MotorState;

typedef struct _MotorParam
{
  int enc_rev;  // count/rev
  int phase_offset;
  int enc_rev_h;              // count/rev
  int deprecated_enc_rev_1p;  // [DEPRECATED]
  unsigned char enc_type;
  int vel_rely_hall;
  int enc_drev[6];
  int enc_mul;
  int enc0;      // count
  int enc0tran;  // count
  int vel_max;   // count/ms
  int Kcurrent;
  int Kvolt;
  int Kp;  // 1/s
  int Ki;  // 1/ss
  int torque_max;
  int torque_min;
  int torque_limit;
  int fr_plus;
  int fr_wplus;
  int fr_minus;
  int fr_wminus;
  int integ_max;  // cnt
  int integ_min;  // cnt
  int inertia_self;
  int inertia_cross;
  enum _motor_type_t
  {
    MOTOR_TYPE_DC,
    MOTOR_TYPE_AC3
  } motor_type;
  short enc_div;
  short enc_denominator;
  int enc_rev_raw;  // count/rev
  char rotation_dir;
  short hall_delay_factor;  // velocity * hall_delay_factor / 32768 = delay count
  int lr_cutoff_vel;
  int lr_cutoff_vel_inv;
} MotorParam;

typedef struct _DriverParam
{
  int PWM_max;
  int PWM_min;
  int control_cycle;
  int control_s;
  int vsrc_rated;
  unsigned short watchdog_limit;
} DriverParam;

typedef struct _DriverState
{
  int PWM_resolution;
  int vsrc_factor;
  int vsrc;
  int zero_torque;
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
    unsigned char hallenc[2];
  } error;
  enum
  {
    BOARD_R4,
    BOARD_R6A,
    BOARD_R6B
  } board_version;
  unsigned char protocol_version;
  unsigned int velcontrol;
} DriverState;

static_assert(sizeof(DriverParam) < 0x100, "DriverParam overflows reserved EEPROM capacity");
static_assert(sizeof(MotorParam) < 0x100, "MotorParam overflows reserved EEPROM capacity");

extern MotorState motor[2];
extern MotorParam motor_param[2];
extern DriverParam driver_param;
extern DriverState driver_state;

#define COM_MOTORS 16
extern short com_cnts[COM_MOTORS];
extern short com_pwms[COM_MOTORS];
extern char com_en[COM_MOTORS];

void controlVelocity_init();
void controlVelocity_config();
void ISR_VelocityControl();
void timer0_vel_calc();

#ifdef __cplusplus
}
#endif

#endif

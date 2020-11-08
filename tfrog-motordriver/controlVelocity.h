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

#ifdef static_assert
static_assert(
    ENC0_BUF_MAX && ((ENC0_BUF_MAX & (ENC0_BUF_MAX - 1)) == 0),
    "ENC0_BUF_MAX must be 2^n");
#endif

#define ACCEL_FILTER_TIME 15  // Timeconstant in velocity control steps
#define ENC0_FILTER_TIME 32   // Timeconstant in velocity control steps

typedef struct _MotorState
{
  int32_t vel;        // count/ms
  int32_t vel1;       // count/ms
  int32_t pos;        // count
  uint32_t posc;      // count
  uint32_t enc_buf;   // count
  uint32_t enc_buf2;  // count
  int32_t spd;
  uint32_t spd_cnt;
  uint16_t enc;
  int16_t dir;

  struct
  {
    int32_t vel;           // count/ms
    int32_t vel_buf;       // count/ms
    int32_t vel_buf_prev;  // count/ms
    int32_t vel_interval;
    int32_t vel_diff;       // count/ms
    int32_t torque;         // 1/100000 Nm
    int32_t torque_offset;  // 1/100000 Nm
    int32_t rate;           // -PWM_max < rate < PWM_max
    int32_t rate2;          //
    int32_t rate_buf;       //
    char vel_changed;
  } ref;
  int32_t error;
  int64_t error_integ;
  char control_init;

  char enc0_buf_updated;
  int32_t enc0_buf[ENC0_BUF_MAX];
  uint16_t enc0_buf_len;

  YPSpur_servo_level servo_level;
  ErrorID error_state;
  FilterExp enc0_lpf;
} MotorState;

typedef struct _MotorParam
{
  int32_t enc_rev;  // count/rev
  int32_t phase_offset;
  int32_t enc_rev_h;              // count/rev
  int32_t deprecated_enc_rev_1p;  // [DEPRECATED]
  uint8_t enc_type;
  int32_t vel_rely_hall;
  int32_t enc_drev[6];
  int32_t enc_mul;
  int32_t enc0;      // count
  int32_t enc0tran;  // count
  int32_t vel_max;   // count/ms
  int32_t Kcurrent;
  int32_t Kvolt;
  int32_t Kp;  // 1/s
  int32_t Ki;  // 1/ss
  int32_t torque_max;
  int32_t torque_min;
  int32_t torque_limit;
  int32_t fr_plus;
  int32_t fr_wplus;
  int32_t fr_minus;
  int32_t fr_wminus;
  int32_t integ_max;  // cnt
  int32_t integ_min;  // cnt
  int32_t inertia_self;
  int32_t inertia_cross;
  enum _motor_type_t
  {
    MOTOR_TYPE_DC,
    MOTOR_TYPE_AC3
  } motor_type;
  int16_t enc_div;
  int16_t enc_denominator;
  int32_t enc_rev_raw;  // count/rev
  char rotation_dir;
  int16_t hall_delay_factor;  // velocity * hall_delay_factor / 32768 = delay count
  int32_t lr_cutoff_vel;
  int32_t lr_cutoff_vel_inv;
} MotorParam;

typedef struct _DriverParam
{
  int32_t PWM_max;
  int32_t PWM_min;
  int32_t control_cycle;
  int32_t control_s;
  int32_t vsrc_rated;
  uint16_t watchdog_limit;
  int32_t vmin;
} DriverParam;

typedef struct _DriverState
{
  int32_t PWM_resolution;
  int32_t vsrc_factor;
  int32_t vsrc;
  int32_t zero_torque;
  uint16_t watchdog;
  uint8_t cnt_updated;
  uint16_t admask;
  uint16_t io_dir;
  uint16_t io_mask[2];
  uint16_t fpga_version;
  uint8_t ifmode;
  struct
  {
    uint8_t low_voltage;
    uint8_t hall[2];
    uint8_t hallenc[2];
  } error;
  enum
  {
    BOARD_R4,
    BOARD_R6A,
    BOARD_R6B
  } board_version;
  uint8_t protocol_version;
  uint32_t velcontrol;
  uint32_t ping_request;
} DriverState;

#ifdef static_assert
static_assert(sizeof(DriverParam) < 0x100, "DriverParam overflows reserved EEPROM capacity");
static_assert(sizeof(MotorParam) < 0x100, "MotorParam overflows reserved EEPROM capacity");
#endif

extern MotorState motor[2];
extern MotorParam motor_param[2];
extern DriverParam driver_param;
extern DriverState driver_state;

#define COM_MOTORS 16
extern int16_t com_cnts[COM_MOTORS];
extern int16_t com_pwms[COM_MOTORS];
extern char com_en[COM_MOTORS];

void controlVelocity_init();
void controlVelocity_config();
void ISR_VelocityControl();
void timer0_vel_calc();

#ifdef __cplusplus
}
#endif

#endif

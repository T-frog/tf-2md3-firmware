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

#include <stdint.h>
#include <string.h>

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

#include "power.h"
#include "eeprom.h"
#include "controlPWM.h"
#include "controlVelocity.h"
#include "registerFPGA.h"
#include "filter.h"
#include "utils.h"

MotorState motor[2];
MotorParam motor_param[2];
DriverParam driver_param;
DriverState driver_state;
int16_t com_cnts[COM_MOTORS];
int16_t com_pwms[COM_MOTORS];
char com_en[COM_MOTORS];
int32_t pwm_sum[2] = { 0, 0 };
int32_t pwm_num[2] = { 0, 0 };
int16_t soft_start[2] = { 0, 0 };

Filter1st accelf[2];
Filter1st accelf0;

extern Tfrog_EEPROM_data saved_param;

void ISR_VelocityControl() RAMFUNC;
void timer0_vel_calc() RAMFUNC;

// ------------------------------------------------------------------------------
// / Velocity control loop (1ms)
//   Called from main loop
// ------------------------------------------------------------------------------
void ISR_VelocityControl()
{
  int32_t i;
  int64_t toq[2];
  int64_t out_pwm[2];
  int64_t acc[2];

  YPSpur_servo_level servo_level[2] = {
    motor[0].servo_level, motor[1].servo_level,
  };
  if (motor[0].error_state || motor[1].error_state)
  {
    // Don't output PWM if error is detected
    servo_level[0] = SERVO_LEVEL_OPENFREE;
    servo_level[1] = SERVO_LEVEL_OPENFREE;
  }

  // Calculate acceleration from reference velocity by PI control
  for (i = 0; i < 2; i++)
  {
    if (servo_level[i] != SERVO_LEVEL_VELOCITY &&
        servo_level[i] != SERVO_LEVEL_POSITION)
    {
      motor[i].ref.rate = 0;
      motor[i].ref.vel_buf_prev = motor[i].vel;
      motor[i].ref.vel_buf = motor[i].vel;
      motor[i].ref.vel = motor[i].vel;
      motor[i].control_init = 1;
      Filter1st_Filter(&accelf[i], 0);
      acc[i] = 0;
      continue;
    }

    // Clear integral term
    if (motor[i].control_init)
    {
      motor[i].ref.vel_diff = 0;
      motor[i].error = 0;
      motor[i].error_integ = 0;
      motor[i].control_init = 0;
    }

    motor[i].ref.vel_interval++;
    if (motor[i].ref.vel_changed)
    {
      motor[i].ref.vel_buf = motor[i].ref.vel;
      motor[i].ref.vel_diff = (motor[i].ref.vel_buf - motor[i].ref.vel_buf_prev) / motor[i].ref.vel_interval;
      // [cnt/msms]

      motor[i].ref.vel_buf_prev = motor[i].ref.vel_buf;
      motor[i].ref.vel_interval = 0;

      motor[i].ref.vel_changed = 0;
    }
    else if (motor[i].ref.vel_interval > 128)
    {
      motor[i].ref.vel_diff = 0;
    }

    motor[i].error = motor[i].ref.vel - motor[i].vel;
    motor[i].error_integ += motor[i].error;
    if (motor[i].error_integ > motor_param[i].integ_max)
    {
      motor[i].error_integ = motor_param[i].integ_max;
    }
    else if (motor[i].error_integ < motor_param[i].integ_min)
    {
      motor[i].error_integ = motor_param[i].integ_min;
    }

    int64_t acc_pi;
    // PI制御分 単位：加速度[cnt/ss]
    acc_pi = ((int64_t)motor[i].error * motor_param[i].Kp) * driver_param.control_s;
    // [cnt/ms] * 1000[ms/s] * Kp[1/s] = [cnt/ss]
    acc_pi += motor[i].error_integ * motor_param[i].Ki;
    // [cnt] * Ki[1/ss] = [cnt/ss]

    acc[i] = (acc_pi +
              (int64_t)Filter1st_Filter(&accelf[i], motor[i].ref.vel_diff) *
                  driver_param.control_s * driver_param.control_s) /
             16;
    // [cnt/msms] * 1000[ms/s] * 1000[ms/s] = [cnt/ss]
  }

  // Calculate PWM duty ratio from acceleration and velocity
  for (i = 0; i < 2; i++)
  {
    if (servo_level[i] == SERVO_LEVEL_STOP ||
        servo_level[i] == SERVO_LEVEL_OPENFREE)
    {
      // Soft-start if it is uncontrolled state (STOP, OPENFREE)
      soft_start[i] = 0;
    }
    else if (soft_start[i] < 128)
    {
      soft_start[i]++;
    }

    if (servo_level[i] >= SERVO_LEVEL_TORQUE)
    {
      // servo_level 2(toque enable)

      if (servo_level[i] >= SERVO_LEVEL_VELOCITY &&
          servo_level[i] != SERVO_LEVEL_OPENFREE)
      {
        int32_t ipair;
        if (i == 0)
          ipair = 1;
        else
          ipair = 0;
        // Kdynamics[TORQUE_UNIT 2pi/cntrev kgf m m], acc[cnt/ss]
        toq[i] = (acc[i] * motor_param[i].inertia_self + acc[ipair] * motor_param[i].inertia_cross) / 256;
      }
      else
      {
        // servo_level 2(toque enable)
        toq[i] = 0;
      }

      // 出力段
      // トルクでクリッピング
      if (toq[i] >= motor_param[i].torque_max)
      {
        toq[i] = motor_param[i].torque_max;
      }
      if (toq[i] <= motor_param[i].torque_min)
      {
        toq[i] = motor_param[i].torque_min;
      }

      // 摩擦補償（線形）
      if (motor[i].vel > 0)
      {
        toq[i] += (motor_param[i].fr_wplus * motor[i].vel / 16 + motor_param[i].fr_plus);
      }
      else if (motor[i].vel < 0)
      {
        toq[i] -= (motor_param[i].fr_wminus * (-motor[i].vel) / 16 + motor_param[i].fr_minus);
      }
      // トルク補償
      toq[i] += motor[i].ref.torque_offset;

      // トルクでクリッピング
      if (toq[i] >= motor_param[i].torque_limit)
      {
        toq[i] = motor_param[i].torque_limit;
      }
      if (toq[i] <= -motor_param[i].torque_limit)
      {
        toq[i] = -motor_param[i].torque_limit;
      }

      // トルク→pwm変換
      if (motor[i].dir == 0)
      {
        out_pwm[i] = 0;
      }
      else
      {
        out_pwm[i] = ((int64_t)motor[i].vel * motor_param[i].Kvolt) / 16;
      }
      // PWMでクリッピング
      if (out_pwm[i] > driver_param.PWM_max * 65536)
        out_pwm[i] = driver_param.PWM_max * 65536;
      if (out_pwm[i] < driver_param.PWM_min * 65536)
        out_pwm[i] = driver_param.PWM_min * 65536;

      motor[i].ref.torque = toq[i] * motor_param[i].Kcurrent;
      out_pwm[i] += motor[i].ref.torque;
      out_pwm[i] = out_pwm[i] * soft_start[i] / (65536 * 128);

      // PWMでクリッピング
      if (out_pwm[i] > driver_param.PWM_max - 1)
        out_pwm[i] = driver_param.PWM_max - 1;
      if (out_pwm[i] < driver_param.PWM_min + 1)
        out_pwm[i] = driver_param.PWM_min + 1;

      // 出力
      motor[i].ref.rate = out_pwm[i];

      pwm_sum[i] += out_pwm[i];
      pwm_num[i]++;
    }
  }
}

void timer0_vel_calc()
{
  static uint32_t enc[2];
  static char _spd_cnt[2];
  uint32_t __enc[2];
  int32_t spd[2];
  int32_t i;
  volatile uint32_t dummy;

  dummy = AT91C_BASE_TC0->TC_SR;
  dummy = dummy;

  if (motor[0].servo_level > SERVO_LEVEL_STOP ||
      motor[1].servo_level > SERVO_LEVEL_STOP)
  {
    driver_state.watchdog++;
    if (driver_state.cnt_updated < 9)
      driver_state.cnt_updated++;
  }

  LED_on(1);
  __enc[0] = enc[0];
  __enc[1] = enc[1];
  for (i = 0; i < 2; i++)
  {
    enc[i] = motor[i].posc;
    spd[i] = motor[i].spd;
    motor[i].spd = 0;
  }

  // calculate motor velocities
  for (i = 0; i < 2; i++)
  {
    int32_t __vel;
    int32_t vel;

    __vel = (int32_t)(enc[i] - __enc[i]);
    motor[i].vel1 = __vel;

    if (_abs(__vel) > 6 || driver_state.fpga_version == 0)
    {
      vel = __vel * 16;
      _spd_cnt[i] = 0;

      if (vel < 0)
        motor[i].dir = -1;
      else if (vel > 0)
        motor[i].dir = 1;
    }
    else if (_abs(spd[i]) > 128)
    {
      // spd[i] is a 8+4bit fixed point speed calculated from pulse width.
      // In this speed, a next pulse should be observed after spd_cnt[i] counts,
      // otherwise, the speed is decreased.
      _spd_cnt[i] = 256 * 16 / _abs(spd[i]);
      vel = spd[i] / 256;

      if (vel < 0)
        motor[i].dir = -1;
      else if (vel > 0)
        motor[i].dir = 1;
    }
    else if (_spd_cnt[i] > 0)
    {
      _spd_cnt[i]--;
      vel = motor[i].vel;
    }
    else
    {
      vel = 0;
      motor[i].dir = 0;
    }

    if (motor[i].control_init)
    {
      vel = 0;
      motor[i].dir = 0;
    }

    if (motor_param[i].enc_type == 0)
    {
      motor[i].vel = motor[i].ref.vel;
      motor[i].vel1 = motor[i].ref.vel / 16;
      motor[i].enc += motor[i].vel1;
      enc[i] = motor[i].enc;

      if (motor[i].vel < 0)
        motor[i].dir = -1;
      else if (motor[i].vel > 0)
        motor[i].dir = 1;
      else
        motor[i].dir = 0;
    }
    else
    {
      motor[i].vel = vel;
    }
    motor[i].enc_buf = enc[i] >> motor_param[i].enc_div;
  }
  for (i = 0; i < 2; i++)
  {
    if (motor[i].servo_level == SERVO_LEVEL_STOP)
      continue;

    if (driver_state.cnt_updated == 5)
    {
      motor[i].ref.rate_buf = pwm_sum[i] * 5 / pwm_num[i];
      motor[i].enc_buf2 = motor[i].enc_buf;
      pwm_sum[i] = 0;
      pwm_num[i] = 0;
    }
  }
  for (i = 0; i < 2; i++)
  {
    if (motor[i].servo_level == SERVO_LEVEL_STOP)
      continue;

    // LPF encoder origin
    motor_param[i].enc0tran =
        FilterExp_FilterAngle(
            &motor[i].enc0_lpf, motor_param[i].enc0,
            motor_param[i].enc_rev, motor_param[i].enc_rev_raw);

    // Calculate 1 rev average of encoder origin
    if (!motor[i].enc0_buf_updated)
      continue;

    motor[i].enc0_buf_updated = 0;

    int32_t enc0 = motor_param[i].enc0;
    int32_t sum_enc0_err = 0, num_enc0 = 0;
    for (int32_t j = 0; j < motor[i].enc0_buf_len; ++j)
    {
      const int32_t e0 = motor[i].enc0_buf[j];
      if (e0 != ENC0_BUF_UNKNOWN)
      {
        int32_t err = e0 - enc0;
        normalize(&err, -motor_param[i].enc_rev_h, motor_param[i].enc_rev);

        sum_enc0_err += err;
        num_enc0++;
      }
    }
    enc0 += sum_enc0_err / num_enc0;
    normalize(&enc0, 0, motor_param[i].enc_rev_raw);
    motor_param[i].enc0 = enc0;
  }

  ISR_VelocityControl();
  LED_off(1);

  driver_state.velcontrol++;
}

// ------------------------------------------------------------------------------
// / Configure velocity control loop
// ------------------------------------------------------------------------------
void controlVelocity_init()
{
  int32_t i;

  Filter1st_CreateLPF(&accelf0, ACCEL_FILTER_TIME);
  accelf[0] = accelf[1] = accelf0;

  driver_param.watchdog_limit = 600;
  driver_param.control_cycle = 1;
  driver_state.protocol_version = 0;
  driver_state.cnt_updated = 0;
  driver_state.admask = 0;
  driver_state.io_mask[0] = 0;
  driver_state.io_mask[1] = 0;
  driver_state.velcontrol = 0;
  driver_state.ping_request = 0;

  for (i = 0; i < 2; i++)
  {
    motor[i].ref.vel = 0;
    motor[i].ref.vel_buf = 0;
    motor[i].ref.vel_buf_prev = 0;
    motor[i].ref.vel_diff = 0;
    motor[i].ref.torque_offset = 0;
    motor[i].error_integ = 0;
    motor[i].control_init = 1;
    motor[i].servo_level = SERVO_LEVEL_STOP;
    motor_param[i].motor_type = MOTOR_TYPE_AC3;
    motor_param[i].rotation_dir = 1;
    motor_param[i].enc_rev = 0;
    motor_param[i].enc_rev_raw = 0;
    motor_param[i].enc_div = 0;
    motor_param[i].enc_denominator = 1;
    motor_param[i].phase_offset = 0;
    motor_param[i].enc_type = 2;
    motor_param[i].hall_delay_factor = 0;
    motor_param[i].lr_cutoff_vel = 0;
  }
}
void controlVelocity_config()
{
  Filter1st_CreateLPF(&accelf0, ACCEL_FILTER_TIME / driver_param.control_cycle);
  accelf[0] = accelf[1] = accelf0;

  {
    volatile uint32_t dummy;

    AT91C_BASE_PMC->PMC_PCER = 1 << AT91C_ID_TC0;

    AT91C_BASE_TC0->TC_CCR = AT91C_TC_CLKDIS;
    AT91C_BASE_TC0->TC_IDR = 0xFFFFFFFF;
    dummy = AT91C_BASE_TC0->TC_SR;
    dummy = dummy;

    // MCK/32 * 1500 -> 1ms
    AT91C_BASE_TC0->TC_CMR = AT91C_TC_CLKS_TIMER_DIV3_CLOCK | AT91C_TC_WAVE | AT91C_TC_WAVESEL_UP_AUTO;
    AT91C_BASE_TC0->TC_CCR = AT91C_TC_CLKEN;
    AT91C_BASE_TC0->TC_RC = 1500 * driver_param.control_cycle;
    AT91C_BASE_TC0->TC_IER = AT91C_TC_CPCS;

    AIC_ConfigureIT(AT91C_ID_TC0, 4 | AT91C_AIC_SRCTYPE_POSITIVE_EDGE, (void (*)(void))timer0_vel_calc);
    AIC_EnableIT(AT91C_ID_TC0);

    AT91C_BASE_TC0->TC_CCR = AT91C_TC_SWTRG;
  }
}

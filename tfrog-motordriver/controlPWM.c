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

#include <board.h>
#include <pio/pio.h>
#include <pio/pio_it.h>
#include <aic/aic.h>
#include <tc/tc.h>
#include <stdint.h>
#include <utility/trace.h>
#include <utility/led.h>
#include <usb/device/cdc-serial/CDCDSerialDriver.h>
#include <usb/device/cdc-serial/CDCDSerialDriverDescriptors.h>

#define FP4_POINTBIT 17

#include "registerFPGA.h"
#include "fixpawd.h"
#include "fixpawd_math.h"
#include "controlPWM.h"
#include "controlVelocity.h"
#include "power.h"
#include "eeprom.h"
#include "utils.h"

static const Pin pinPWMCycle2 = PIN_PWM_CYCLE2;

// / PWM Enable pin instance.
static const Pin pinPWMEnable = PIN_PWM_ENABLE;

short SinTB[1024];
int PWM_abs_max = 0;
int PWM_abs_min = 0;
int PWM_center = 0;
int PWM_init = 0;
int PWM_resolution = 0;
int PWM_thinning = 0;
int PWM_deadtime;
int PWM_cpms;

void FIQ_PWMPeriod() RAMFUNC;

#define SinTB_2PI 4096
inline short sin_(int x)
{
  if (x < 1024)
    return SinTB[x];
  else if (x < 2048)
    return SinTB[2048 - x];
  else if (x < 2048 + 1024)
    return -SinTB[x - 2048];
  return -SinTB[4096 - x];
}

extern Tfrog_EEPROM_data saved_param;
extern volatile char rs485_timeout;

void controlPWM_config(int i)
{
  switch (motor_param[i].motor_type)
  {
    case MOTOR_TYPE_DC:
      THEVA.MOTOR[i].PWM[0].H = PWM_resolution;
      THEVA.MOTOR[i].PWM[1].H = 0;
      THEVA.MOTOR[i].PWM[2].H = PWM_resolution;
      THEVA.MOTOR[i].PWM[0].L = PWM_resolution;
      THEVA.MOTOR[i].PWM[1].L = 0;
      THEVA.MOTOR[i].PWM[2].L = PWM_resolution;
      break;
    case MOTOR_TYPE_AC3:
      THEVA.MOTOR[i].PWM[0].H = PWM_resolution;
      THEVA.MOTOR[i].PWM[1].H = PWM_resolution;
      THEVA.MOTOR[i].PWM[2].H = PWM_resolution;
      THEVA.MOTOR[i].PWM[0].L = PWM_resolution;
      THEVA.MOTOR[i].PWM[1].L = PWM_resolution;
      THEVA.MOTOR[i].PWM[2].L = PWM_resolution;
      break;
  }

  motor[i].ref.rate = 0;

  motor_param[i].enc_rev = (int)motor_param[i].enc_rev_raw / (int)motor_param[i].enc_denominator;

  motor_param[i].enc_drev[0] = motor_param[i].enc_rev * 1 / 6;
  motor_param[i].enc_drev[1] = motor_param[i].enc_rev * 2 / 6;
  motor_param[i].enc_drev[2] = motor_param[i].enc_rev * 3 / 6;
  motor_param[i].enc_drev[3] = motor_param[i].enc_rev * 4 / 6;
  motor_param[i].enc_drev[4] = motor_param[i].enc_rev * 5 / 6;
  motor_param[i].enc_drev[5] = motor_param[i].enc_rev * 6 / 6;

  motor_param[i].enc_10hz = motor_param[i].enc_rev * 10 * 16 / 1000;
  motor_param[i].enc_rev_1p = motor_param[i].enc_rev / 300;
  if (motor_param[i].enc_rev_1p == 0)
    motor_param[i].enc_rev_1p = 1;

  motor_param[i].enc_mul =
      (unsigned int)((uint64_t)SinTB_2PI * 0x40000 * motor_param[i].enc_denominator /
                     motor_param[i].enc_rev_raw);
  motor_param[i].enc_rev_h = motor_param[i].enc_rev / 2;

  // normalize phase offset
  motor_param[i].phase_offset = motor_param[i].phase_offset % motor_param[i].enc_rev;
  if (motor_param[i].phase_offset < 0)
    motor_param[i].phase_offset += motor_param[i].enc_rev;

  motor[i].ref.vel_diff = 0;
  motor[i].ref.vel_interval = 0;
  motor[i].ref.vel_changed = 0;
  motor[i].error_integ = 0;
  motor[i].vel = 0;
  motor[i].dir = 0;
  motor[i].pos = 0;

  motor_param[i].enc0 = 0;
  motor_param[i].enc0tran = 0;

  if (motor_param[i].motor_type != MOTOR_TYPE_DC &&
      motor_param[i].enc_type != 0)
  {
    const unsigned short hall = *(unsigned short*)&THEVA.MOTOR[i].ROT_DETECTER;

    if (hall & HALL_U)
    {
      if (hall & HALL_V)
        motor_param[i].enc0 = motor[i].pos - motor_param[i].enc_rev * 5 / 12;  // 150度
      else if (hall & HALL_W)
        motor_param[i].enc0 = motor[i].pos - motor_param[i].enc_rev * 1 / 12;  // 30度
      else
        motor_param[i].enc0 = motor[i].pos - motor_param[i].enc_rev * 3 / 12;  // 90度
    }
    else
    {
      if (!(hall & HALL_V))
        motor_param[i].enc0 = motor[i].pos - motor_param[i].enc_rev * 11 / 12;  // 330度
      else if (!(hall & HALL_W))
        motor_param[i].enc0 = motor[i].pos - motor_param[i].enc_rev * 7 / 12;  // 210度
      else
        motor_param[i].enc0 = motor[i].pos - motor_param[i].enc_rev * 9 / 12;  // 270度
    }
    if (motor_param[i].enc_type == 3)
    {
      motor[i].pos = motor[i].pos - motor_param[i].enc0;
      motor_param[i].enc0 = 0;
    }
  }
  motor_param[i].enc0tran = motor_param[i].enc0;
}

// ------------------------------------------------------------------------------
// / PWM control interrupt (every PWM period) 20us/50us
// ------------------------------------------------------------------------------
void FIQ_PWMPeriod()
{
  int i;
  static unsigned short enc[2];
  static unsigned short hall[2];
  static int init = 0;
  static unsigned int cnt = 0;
  unsigned short _enc[2];
  unsigned short _hall[2];

  cnt++;
  {
    // PWM周波数が高い場合は処理を間引く
    // PWM_resolution 2000以下で一回間引き
    static int thin = 0;

    thin++;
    if (thin < PWM_thinning)
      return;
    thin = 0;
  }

  _enc[0] = enc[0];
  _enc[1] = enc[1];
  _hall[0] = hall[0];
  _hall[1] = hall[1];

  for (i = 0; i < 2; i++)
  {
    hall[i] = *(unsigned short*)&THEVA.MOTOR[i].ROT_DETECTER;
    if (motor_param[i].enc_type == 2)
    {
      unsigned short s;
      int __vel;

      motor[i].enc = enc[i] = THEVA.MOTOR[i].ENCODER;
      s = THEVA.MOTOR[i].SPEED * driver_param.control_cycle;
      __vel = (short)(enc[i] - _enc[i]);

      if (s < 256 * 16 * 8 && __vel != 0)
      {
        if (__vel > 0)
          motor[i].spd = s;
        else if (__vel < 0)
          motor[i].spd = -s;
      }
    }
    else if (motor_param[i].enc_type == 0)
    {
      enc[i] = motor[i].enc;
    }
  }

  int disabled = 0;
  if (motor[0].error_state || motor[1].error_state)
  {
    // Short-mode brake
    for (i = 0; i < 3 * 2; i++)
    {
      THEVA.MOTOR[i % 2].PWM[i / 2].H = PWM_resolution;
      THEVA.MOTOR[i % 2].PWM[i / 2].L = PWM_resolution;
    }
    PWM_init = 0;
    init = 0;
    disabled = 1;
  }
  else if (!init)
  {
    init = 1;
    return;
  }

  for (i = 0; i < 2; i++)
  {
    if (motor_param[i].enc_type == 2 ||
        motor_param[i].enc_type == 0)
    {
      const short diff = (short)(enc[i] - _enc[i]);
      motor[i].pos += diff;
      motor[i].posc += diff;
      normalize(&motor[i].pos, 0, motor_param[i].enc_rev_raw, motor_param[i].enc_rev_raw);
    }
  }

  if (!disabled)
  {
    // PWM計算
    int pwm[2][3];
    int phase[3];
    int j;

    if (PWM_init < 2048)
    {
      PWM_init++;
      if (PWM_init > 2048)
        PWM_init = 2048;
    }
    for (j = 0; j < 2; j++)
    {
      int64_t rate;

      if (motor[j].servo_level == SERVO_LEVEL_STOP ||
          motor[j].servo_level == SERVO_LEVEL_OPENFREE)
      {
        continue;
      }
      rate = motor[j].ref.rate * PWM_init / 2048;

      if (driver_param.vsrc_rated != 0)
      {
        rate = rate * driver_param.vsrc_factor / 32768;
        if (rate >= PWM_resolution)
          rate = PWM_resolution - 1;
        else if (rate <= -PWM_resolution)
          rate = -PWM_resolution + 1;
        motor[j].ref.rate2 = rate;
      }

      switch (motor_param[j].motor_type)
      {
        case MOTOR_TYPE_DC:
        {
          int pwmt;
          pwmt = PWM_center - rate / 2;
          if (pwmt < PWM_abs_min)
            pwmt = PWM_abs_min;
          if (pwmt > PWM_abs_max)
            pwmt = PWM_abs_max;
          pwm[j][2] = pwmt;
          pwmt = PWM_center + (rate - rate / 2);
          if (pwmt < PWM_abs_min)
            pwmt = PWM_abs_min;
          if (pwmt > PWM_abs_max)
            pwmt = PWM_abs_max;
          pwm[j][0] = pwmt;
          break;
        }
        case MOTOR_TYPE_AC3:
        {
          phase[2] = motor[j].pos - motor_param[j].enc0tran;
          phase[2] = (int64_t)(phase[2] + motor_param[j].phase_offset) *
                         motor_param[j].enc_mul / 0x40000 +
                     SinTB_2PI + SinTB_2PI / 4;
          phase[1] = phase[2] - SinTB_2PI / 3;
          phase[0] = phase[2] - SinTB_2PI * 2 / 3;

          for (i = 0; i < 3; i++)
          {
            int pwmt;

            int p = phase[i] % SinTB_2PI;
            if (p < 0)
              p += SinTB_2PI;
            pwmt = (int)sin_(p) * rate / (4096 * 2);
            pwmt += PWM_center;
            if (pwmt < PWM_abs_min)
              pwmt = PWM_abs_min;
            if (pwmt > PWM_abs_max)
              pwmt = PWM_abs_max;
            pwm[j][i] = pwmt;
          }
          if (motor_param[j].rotation_dir != 1)
          {
            const int tmp = pwm[j][0];
            pwm[j][0] = pwm[j][2];
            pwm[j][2] = tmp;
          }
          break;
        }
      }
    }
    for (j = 0; j < 2; j++)
    {
      if (motor[j].servo_level == SERVO_LEVEL_STOP)
      {
        for (i = 0; i < 3; i++)
        {
          THEVA.MOTOR[j].PWM[i].H = PWM_resolution;
          THEVA.MOTOR[j].PWM[i].L = PWM_resolution;
        }
      }
      else if (_abs(motor[j].ref.torque) < driver_param.zero_torque ||
               motor[j].servo_level == SERVO_LEVEL_OPENFREE)
      {
        for (i = 0; i < 3; i++)
        {
          THEVA.MOTOR[j].PWM[i].H = 0;
          THEVA.MOTOR[j].PWM[i].L = 0;
        }
      }
      else
      {
        for (i = 0; i < 3; i++)
        {
          THEVA.MOTOR[j].PWM[i].H = pwm[j][i];
          THEVA.MOTOR[j].PWM[i].L = PWM_resolution;
        }
      }
    }
  }

  // ゼロ点計算
  for (i = 0; i < 2; i++)
  {
    int u, v, w;

    if (motor_param[i].motor_type != MOTOR_TYPE_DC &&
        motor_param[i].enc_type != 0)
    {
      char dir;
      unsigned short halldiff;

      u = v = w = 0;
      halldiff = (hall[i] ^ _hall[i]) & 0x07;

      if (halldiff == 0)
        continue;
      dir = 0;

      if ((hall[i] & 0x07) == (HALL_U | HALL_V | HALL_W) ||
          (hall[i] & 0x07) == 0 ||
          halldiff == 3 || halldiff >= 5)
      {
        //if( (hall[i] & 0x07) == ( HALL_U | HALL_V | HALL_W ) ) printf( "ENC error: 111\n\r" );
        //if( (hall[i] & 0x07) == 0 ) printf( "ENC error: 000\n\r" );
        //if( halldiff == 3 || halldiff >= 5 ) printf( "ENC error: %x->%x\n\r", _hall[i], hall[i] );
        // ホール素子信号が全相1、全相0のとき
        // ホース素子信号が2ビット以上変化したときはエラー
        if (driver_param.error.hall[i] < 128)
          driver_param.error.hall[i] += 12;
        if (driver_param.error.hall[i] > 12)
        {
          // エラー検出後、1周以内に再度エラーがあれば停止
          motor[i].error_state |= ERROR_HALL_SEQ;
        }
        continue;
      }
      else
      {
        if (driver_param.error.hall[i] > 0)
          driver_param.error.hall[i]--;
      }

      switch (halldiff)
      {
        case HALL_U:
          if (!(_hall[i] & HALL_U))
            if (hall[i] & HALL_W)
            {
              u = 1;  // 正回転 U立ち上がり 0度
            }
            else
            {
              u = -1;  // 逆回転 U立ち上がり 180度
              dir = 1;
            }
          else if (hall[i] & HALL_V)
          {
            u = -1;  // 正回転 U立ち下がり 180度
          }
          else
          {
            u = 1;  // 逆回転 U立ち下がり 0度
            dir = 1;
          }
          break;
        case HALL_V:
          if (!(_hall[i] & HALL_V))
            if (hall[i] & HALL_U)
            {
              v = 1;  // 正回転 V立ち上がり 120度
            }
            else
            {
              v = -1;  // 逆回転 V立ち上がり 300度
              dir = 1;
            }
          else if (hall[i] & HALL_W)
          {
            v = -1;  // 正回転 V立ち下がり 300度
          }
          else
          {
            v = 1;  // 逆回転 V立ち下がり 120度
            dir = 1;
          }
          break;
        case HALL_W:
          if (!(_hall[i] & HALL_W))
            if (hall[i] & HALL_V)
            {
              w = 1;  // 正回転 W立ち上がり 240度
            }
            else
            {
              w = -1;  // 逆回転 W立ち上がり 60度
              dir = 1;
            }
          else if (hall[i] & HALL_U)
          {
            w = -1;  // 正回転 W立ち下がり 60度
          }
          else
          {
            w = 1;  // 逆回転 W立ち下がり 240度
            dir = 1;
          }
          break;
        default:
          // エラー
          break;
      }

      if (motor_param[i].enc_type == 3)
      {
        int _pos;
        _pos = (int)motor[i].pos;
        if (w == -1)
          motor[i].pos = motor_param[i].enc_drev[0];
        else if (v == 1)
          motor[i].pos = motor_param[i].enc_drev[1];
        else if (u == -1)
          motor[i].pos = motor_param[i].enc_drev[2];
        else if (w == 1)
          motor[i].pos = motor_param[i].enc_drev[3];
        else if (v == -1)
          motor[i].pos = motor_param[i].enc_drev[4];
        else if (u == 1)
          motor[i].pos = motor_param[i].enc_drev[5];
        motor[i].pos -= dir - 1;
        normalize(&motor[i].pos, 0, motor_param[i].enc_rev, motor_param[i].enc_rev);
        motor_param[i].enc0 = 0;
        motor_param[i].enc0tran = 0;

        int diff;
        diff = (int)motor[i].pos - _pos;
        normalize(&diff, -motor_param[i].enc_rev_h, motor_param[i].enc_rev_h, motor_param[i].enc_rev);
        motor[i].enc += diff;

        if (diff > 0)
          motor[i].spd = PWM_cpms / (int)(cnt - motor[i].spd_cnt);
        else
          motor[i].spd = -PWM_cpms / (int)(cnt - motor[i].spd_cnt);
        motor[i].spd_cnt = cnt;
        continue;
      }

      int enc0 = 0;

      // ゼロ点計算
      if (w == -1)
        enc0 = motor[i].pos - motor_param[i].enc_drev[0] + dir - 1;
      else if (v == 1)
        enc0 = motor[i].pos - motor_param[i].enc_drev[1] + dir - 1;
      else if (u == -1)
        enc0 = motor[i].pos - motor_param[i].enc_drev[2] + dir - 1;
      else if (w == 1)
        enc0 = motor[i].pos - motor_param[i].enc_drev[3] + dir - 1;
      else if (v == -1)
        enc0 = motor[i].pos - motor_param[i].enc_drev[4] + dir - 1;
      else if (u == 1)
        enc0 = motor[i].pos - motor_param[i].enc_drev[5] + dir - 1;

      // Check hall signal consistency
      if (motor_param[i].enc_type == 2 && motor[i].servo_level > SERVO_LEVEL_STOP)
      {
        int err = motor_param[i].enc0 - enc0;
        normalize(&err, -motor_param[i].enc_rev_h, motor_param[i].enc_rev_h, motor_param[i].enc_rev);
        // In worst case, initial encoder origin can have offset of motor_param[i].enc_rev/12.
        if (_abs(err) > motor_param[i].enc_rev / 6)
        {
          motor[i].error_state |= ERROR_HALL_ENC;
        }
      }

      // ホール素子は高速域では信頼できない
      if (_abs(motor[i].vel) > motor_param[i].enc_10hz &&
          !saved_param.rely_hall)
        continue;

      motor_param[i].enc0 = enc0;
    }
  }

  return;
}

// ------------------------------------------------------------------------------
// / Configure velocity control loop
// ------------------------------------------------------------------------------
void controlPWM_init()
{
  int j;

  for (j = 0; j < SinTB_2PI / 4; j++)
  {
    fixp4 val, ang;
    int ival;

    ang = (uint64_t)FP4_PI2 * j / SinTB_2PI;
    val = (fp4sin(ang) + fp4mul(fp4sin(ang * 3), DOUBLE2FP4(0.1547)));

    ival = val * 3547 / FP4_ONE;

    if (ival > 4096)
      ival = 4096;
    else if (ival < -4096)
      ival = -4096;

    SinTB[j] = ival;
  }

  PIO_Configure(&pinPWMCycle2, 1);
  AIC_ConfigureIT(AT91C_ID_IRQ0, 5 | AT91C_AIC_SRCTYPE_POSITIVE_EDGE, (void (*)(void))FIQ_PWMPeriod);
  AIC_EnableIT(AT91C_ID_IRQ0);

  // PWM Generator init
  PWM_resolution = saved_param.PWM_resolution;
  PWM_thinning = 2000 / PWM_resolution;
  PWM_deadtime = saved_param.PWM_deadtime;

  THEVA.GENERAL.PWM.HALF_PERIOD = PWM_resolution;
  THEVA.GENERAL.PWM.DEADTIME = PWM_deadtime;

  // Short-mode brake
  for (j = 0; j < 3 * 2; j++)
  {
    THEVA.MOTOR[j % 2].PWM[j / 2].H = PWM_resolution;
    THEVA.MOTOR[j % 2].PWM[j / 2].L = PWM_resolution;
  }

  driver_param.PWM_resolution = PWM_resolution;

  PWM_cpms = 256 * 16 * driver_param.control_cycle * 48000 / (PWM_resolution * 2);
  driver_param.control_s = 1000 / driver_param.control_cycle;

  PWM_abs_max = PWM_resolution - PWM_deadtime - 1;
  PWM_abs_min = PWM_deadtime + 1;
  PWM_center = PWM_resolution / 2;

  THEVA.GENERAL.PWM.COUNT_ENABLE = 1;
  THEVA.GENERAL.OUTPUT_ENABLE = 1;
  PIO_Clear(&pinPWMEnable);

  PWM_init = 0;
}

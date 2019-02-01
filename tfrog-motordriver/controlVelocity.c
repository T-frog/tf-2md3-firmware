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
#include <string.h>
#include <stdint.h>

#include "power.h"
#include "eeprom.h"
#include "controlPWM.h"
#include "controlVelocity.h"
#include "registerFPGA.h"
#include "filter.h"

MotorState motor[2];
MotorParam motor_param[2];
DriverParam driver_param;
short com_cnts[COM_MOTORS];
short com_pwms[COM_MOTORS];
char com_en[COM_MOTORS];
int pwm_sum[2] = { 0, 0 };
int pwm_num[2] = { 0, 0 };

extern int watchdog;
extern int velcontrol;

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
  // volatile unsigned int status;
  int i;
  int64_t toq[2];
  int64_t out_pwm[2];
  int64_t acc[2];

  for (i = 0; i < 2; i++)
  {
    if (motor[i].servo_level >= SERVO_LEVEL_TORQUE)
    {
      // servo_level 2(toque enable)

      if (motor[i].servo_level >= SERVO_LEVEL_VELOCITY &&
          motor[i].servo_level != SERVO_LEVEL_OPENFREE)
      {
        int64_t acc_pi;

        // 積分
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

        motor[i].error = motor[i].ref.vel_buf - motor[i].vel;
        motor[i].error_integ += motor[i].error;
        if (motor[i].error_integ > motor_param[i].integ_max)
        {
          motor[i].error_integ = motor_param[i].integ_max;
        }
        else if (motor[i].error_integ < motor_param[i].integ_min)
        {
          motor[i].error_integ = motor_param[i].integ_min;
        }

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
      else
      {
        motor[i].ref.vel_buf_prev = motor[i].vel;
        motor[i].ref.vel_buf = motor[i].vel;
        motor[i].ref.vel = motor[i].vel;
        motor[i].error_integ = 0;
        motor[i].ref.vel_diff = 0;
        Filter1st_Filter(&accelf[i], 0);
        acc[i] = 0;
      }
    }
    else
    {
      motor[i].ref.rate = 0;
      motor[i].ref.vel_buf_prev = motor[i].vel;
      motor[i].ref.vel_buf = motor[i].vel;
      motor[i].ref.vel = motor[i].vel;
      Filter1st_Filter(&accelf[i], 0);
      acc[i] = 0;
    }
  }

  for (i = 0; i < 2; i++)
  {
    if (motor[i].servo_level >= SERVO_LEVEL_TORQUE)
    {
      // servo_level 2(toque enable)

      if (motor[i].servo_level >= SERVO_LEVEL_VELOCITY &&
          motor[i].servo_level != SERVO_LEVEL_OPENFREE)
      {
        int ipair;
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
      toq[i] += motor_param[i].torque_offset;

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
      out_pwm[i] /= 65536;

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
  static unsigned int __enc[2];
  unsigned int enc[2];
  static char _spd_cnt[2];
  int spd[2];
  int i;
  volatile unsigned int dummy;

  if (motor[0].servo_level > SERVO_LEVEL_STOP ||
      motor[1].servo_level > SERVO_LEVEL_STOP)
  {
    driver_param.watchdog++;
    driver_param.cnt_updated++;
  }

  dummy = AT91C_BASE_TC0->TC_SR;
  dummy = dummy;

  LED_on(1);
  for (i = 0; i < 2; i++)
  {
    enc[i] = motor[i].posc;
    spd[i] = motor[i].spd;
    motor[i].spd = 0;
  }

  for (i = 0; i < 2; i++)
  {
    int __vel;
    int vel;

    __vel = (int)(enc[i] - __enc[i]);
    motor[i].vel1 = __vel;

    if (_abs(__vel) > 6 || driver_param.fpga_version == 0)
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
    __enc[i] = enc[i];
    motor[i].enc_buf = enc[i] >> motor_param[i].enc_div;
  }
  for (i = 0; i < 2; i++)
  {
    if (motor[i].servo_level >= SERVO_LEVEL_TORQUE)
    {
      if (driver_param.cnt_updated == 5)
      {
        motor[i].ref.rate_buf = pwm_sum[i] * 5 / pwm_num[i];
        motor[i].enc_buf2 = motor[i].enc_buf;
        pwm_sum[i] = 0;
        pwm_num[i] = 0;
      }
    }
  }

  LED_off(1);

  velcontrol = 1;
}

// ------------------------------------------------------------------------------
// / Configure velocity control loop
// ------------------------------------------------------------------------------
void controlVelocity_init()
{
#define ACCEL_FILTER_TIME 15.0
  int i;

  Filter1st_CreateLPF(&accelf0, ACCEL_FILTER_TIME);
  accelf[0] = accelf[1] = accelf0;

  driver_param.protocol_version = 0;
  driver_param.cnt_updated = 0;
  driver_param.watchdog_limit = 600;
  driver_param.admask = 0;
  driver_param.io_mask[0] = 0;
  driver_param.io_mask[1] = 0;

  for (i = 0; i < 2; i++)
  {
    motor[i].ref.vel = 0;
    motor[i].ref.vel_buf = 0;
    motor[i].ref.vel_buf_prev = 0;
    motor[i].ref.vel_diff = 0;
    motor[i].error_integ = 0;
    motor[i].control_init = 0;
    motor[i].servo_level = SERVO_LEVEL_STOP;
    motor_param[i].motor_type = MOTOR_TYPE_AC3;
    motor_param[i].enc_rev = 0;
    motor_param[i].enc_rev_raw = 0;
    motor_param[i].enc_div = 0;
    motor_param[i].enc_denominator = 1;
    motor_param[i].phase_offset = 0;
    motor_param[i].enc_type = 2;
  }

  controlVelocity_config();
}
void controlVelocity_config()
{
  Filter1st_CreateLPF(&accelf0, 15 / driver_param.control_cycle);
  accelf[0] = accelf[1] = accelf0;

  {
    volatile unsigned int dummy;

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

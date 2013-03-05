#ifndef __CONTROL_PWM_H__
#define __CONTROL_PWM_H__

void controlPWM_init(  );
RAMFUNC void controlPWM_config(  );
RAMFUNC void FIQ_PWMPeriod(  );

#define ZERO_TORQUE 0

#endif

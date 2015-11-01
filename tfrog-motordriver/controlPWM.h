#ifndef __CONTROL_PWM_H__
#define __CONTROL_PWM_H__

void controlPWM_init(  );
void controlPWM_config(  );
RAMFUNC void FIQ_PWMPeriod(  );
RAMFUNC int _abs( int x );
RAMFUNC void normalize( int *val, int min, int max, int resolution );

#endif

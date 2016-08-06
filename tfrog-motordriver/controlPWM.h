#ifndef __CONTROL_PWM_H__
#define __CONTROL_PWM_H__

void controlPWM_init(  );
void controlPWM_config(  );
void FIQ_PWMPeriod(  );
int _abs( int x );
void normalize( int *val, int min, int max, int resolution );

#endif

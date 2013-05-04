#ifndef __POWER_H__
#define __POWER_H__

void LowPowerMode( void );
void NormalPowerMode( void );
RAMFUNC void LED_on( int num );
RAMFUNC void LED_off( int num );

#endif

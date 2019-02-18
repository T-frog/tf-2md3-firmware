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
#include <usart/usart.h>
#include <utility/trace.h>
#include <utility/led.h>
#include <pmc/pmc.h>

#include "power.h"

static const Pin pinsLED[] = { PIN_LED_0, PIN_LED_1, PIN_LED_2 };

void LED_off(int num) RAMFUNC;
void LED_on(int num) RAMFUNC;

void LED_on(int num)
{
  PIO_Clear(&pinsLED[num]);
}
void LED_off(int num)
{
  PIO_Set(&pinsLED[num]);
}

// ------------------------------------------------------------------------------
// / Put the CPU in 32kHz, disable PLL, main oscillator
// / Put voltage regulator in standby mode
// ------------------------------------------------------------------------------
void LowPowerMode(void)
{
  // MCK=48MHz to MCK=32kHz
  // MCK = SLCK/2 : change source first from 48 000 000 to 18. / 2 = 9M
  AT91C_BASE_PMC->PMC_MCKR = AT91C_PMC_PRES_CLK_2;
  while (!(AT91C_BASE_PMC->PMC_SR & AT91C_PMC_MCKRDY))
    ;
  // MCK=SLCK : then change prescaler
  AT91C_BASE_PMC->PMC_MCKR = AT91C_PMC_CSS_SLOW_CLK;
  while (!(AT91C_BASE_PMC->PMC_SR & AT91C_PMC_MCKRDY))
    ;
  // disable PLL
  AT91C_BASE_PMC->PMC_PLLR = 0;
  // Disable Main Oscillator
  AT91C_BASE_PMC->PMC_MOR = 0;

  // Voltage regulator in standby mode : Enable VREG Low Power Mode
  AT91C_BASE_VREG->VREG_MR |= AT91C_VREG_PSTDBY;

  PMC_DisableProcessorClock();
}

// ------------------------------------------------------------------------------
// / Put voltage regulator in normal mode
// / Return the CPU to normal speed 48MHz, enable PLL, main oscillator
// ------------------------------------------------------------------------------
void NormalPowerMode(void)
{
  // Voltage regulator in normal mode : Disable VREG Low Power Mode
  AT91C_BASE_VREG->VREG_MR &= ~AT91C_VREG_PSTDBY;

  // MCK=32kHz to MCK=48MHz
  // enable Main Oscillator
  AT91C_BASE_PMC->PMC_MOR = (((AT91C_CKGR_OSCOUNT & (0x06 << 8)) | AT91C_CKGR_MOSCEN));
  while (!(AT91C_BASE_PMC->PMC_SR & AT91C_PMC_MOSCS))
    ;

  // enable PLL@96MHz
  AT91C_BASE_PMC->PMC_PLLR = ((AT91C_CKGR_DIV & 0x0E) |
                              (AT91C_CKGR_PLLCOUNT & (28 << 8)) | (AT91C_CKGR_MUL & (0x48 << 16)));
  while (!(AT91C_BASE_PMC->PMC_SR & AT91C_PMC_LOCK))
    ;
  while (!(AT91C_BASE_PMC->PMC_SR & AT91C_PMC_MCKRDY))
    ;
  AT91C_BASE_CKGR->CKGR_PLLR |= AT91C_CKGR_USBDIV_1;
  // MCK=SLCK/2 : change prescaler first
  AT91C_BASE_PMC->PMC_MCKR = AT91C_PMC_PRES_CLK_2;
  while (!(AT91C_BASE_PMC->PMC_SR & AT91C_PMC_MCKRDY))
    ;
  // MCK=PLLCK/2 : then change source
  AT91C_BASE_PMC->PMC_MCKR |= AT91C_PMC_CSS_PLL_CLK;
  while (!(AT91C_BASE_PMC->PMC_SR & AT91C_PMC_MCKRDY))
    ;
}

// ------------------------------------------------------------------------------
// Callbacks re-implementation
// ------------------------------------------------------------------------------
// ------------------------------------------------------------------------------
// / Invoked when the USB device leaves the Suspended state. By default,
// / configures the LEDs.
// ------------------------------------------------------------------------------
void USBDCallbacks_Resumed(void)
{
}

// ------------------------------------------------------------------------------
// / Invoked when the USB device gets suspended. By default, turns off all LEDs.
// ------------------------------------------------------------------------------
void USBDCallbacks_Suspended(void)
{
}

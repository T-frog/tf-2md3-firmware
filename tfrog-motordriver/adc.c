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
#include <string.h>
#include <utility/led.h>
#include <flash/flashd.h>
#include <pmc/pmc.h>

#include "adc.h"

void ADC_Init()
{
  static const Pin pinsADC[] = { PINS_ADC };
  unsigned int prescal;
  unsigned int startup;
  unsigned int shtim;

  AT91C_BASE_PMC->PMC_PCER = 1 << AT91C_ID_ADC;
  PIO_Configure(pinsADC, PIO_LISTSIZE(pinsADC));

  prescal = (BOARD_MCK / (2 * 1000000)) - 1;  // 1MHz
  startup = (1 * 10 / 8);                     // 10us startup
  shtim = ((1 * 2000) / 1000) - 1;            // 2us sample and hold

  AT91C_BASE_ADC->ADC_CR = AT91C_ADC_SWRST;

  AT91C_BASE_ADC->ADC_MR =
      AT91C_ADC_TRGEN_DIS |
      AT91C_ADC_LOWRES_10_BIT |
      AT91C_ADC_SLEEP_NORMAL_MODE |
      (prescal << 8) | (startup << 16) | (shtim << 24);

  AT91C_BASE_ADC->ADC_CHER = 0xFF;
}

void ADC_Start()
{
  AT91C_BASE_ADC->ADC_CR = AT91C_ADC_START;
}

int ADC_Read(int i)
{
  return *(&AT91C_BASE_ADC->ADC_CDR0 + i);
}

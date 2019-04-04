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

#include "power.h"
#include "controlPWM.h"
#include "controlVelocity.h"
#include "registerFPGA.h"
#include "communication.h"
#include "eeprom.h"

void msleep(int ms)
{
  volatile unsigned int dummy = 0;
  int i = 0;

  AT91C_BASE_PITC->PITC_PIMR = AT91C_PITC_PITEN | 0x667;

  for (i = 0; i < ms * 3; i++)
  {
    while (!(AT91C_BASE_PITC->PITC_PISR & AT91C_PITC_PITS))
      ;
    dummy = AT91C_BASE_PITC->PITC_PIVR;
    dummy = dummy;
  }
}

void EEPROM_Init()
{
#if defined(tfrog_rev5)
  static const Pin pinsEEPROM[] = { PINS_EEPROM };
  static const Pin pinsEEPROM_reset[] = { PINS_EEPROM_TWD, PINS_EEPROM_TWCK };
  int i;

  PIO_Configure(pinsEEPROM_reset, PIO_LISTSIZE(pinsEEPROM_reset));
  PIO_Set(&pinsEEPROM_reset[0]);
  PIO_Clear(&pinsEEPROM_reset[1]);
  msleep(1);
  PIO_Set(&pinsEEPROM_reset[1]);
  msleep(1);
  PIO_Clear(&pinsEEPROM_reset[0]);
  msleep(1);
  PIO_Clear(&pinsEEPROM_reset[1]);
  msleep(1);
  PIO_Set(&pinsEEPROM_reset[0]);
  msleep(1);

  for (i = 0; i < 9; i++)
  {
    PIO_Set(&pinsEEPROM_reset[1]);
    msleep(1);
    PIO_Clear(&pinsEEPROM_reset[1]);
    msleep(1);
  }

  PIO_Set(&pinsEEPROM_reset[1]);
  msleep(1);
  PIO_Clear(&pinsEEPROM_reset[0]);
  msleep(1);
  PIO_Clear(&pinsEEPROM_reset[1]);
  msleep(1);
  PIO_Set(&pinsEEPROM_reset[1]);
  msleep(1);
  PIO_Set(&pinsEEPROM_reset[0]);
  msleep(1);
  PIO_Clear(&pinsEEPROM_reset[1]);
  msleep(1);

  AT91C_BASE_PMC->PMC_PCER = 1 << AT91C_ID_TWI;
  PIO_Configure(pinsEEPROM, PIO_LISTSIZE(pinsEEPROM));

  AT91C_BASE_TWI->TWI_CR = AT91C_TWI_SWRST;

  AT91C_BASE_TWI->TWI_CWGR = 0x00007575;
  AT91C_BASE_TWI->TWI_CR = AT91C_TWI_MSEN | AT91C_TWI_SVDIS;
  AT91C_BASE_TWI->TWI_MMR = AT91C_TWI_IADRSZ_1_BYTE;

  {
    char dummy[8];
    EEPROM_Read(0, dummy, 8);
  }
#elif defined(tfrog_rev4)
  FLASHD_Initialize(BOARD_MCK);
#endif
}

int EEPROM_Read(int addr, void* data, int len)
{
#if defined(tfrog_rev5)
  int page;
  int recieved;

  page = addr >> 8;
  addr = addr & 0xFF;
  AT91C_BASE_TWI->TWI_CR = AT91C_TWI_MSEN | AT91C_TWI_SVDIS;
  AT91C_BASE_TWI->TWI_MMR = ((0x50 | page) << 16) | AT91C_TWI_IADRSZ_1_BYTE | AT91C_TWI_MREAD;
  AT91C_BASE_TWI->TWI_IADR = addr;

  while (!(AT91C_BASE_TWI->TWI_SR & AT91C_TWI_TXCOMP_MASTER))
    ;
  if (len == 1)
  {
    AT91C_BASE_TWI->TWI_CR = AT91C_TWI_START | AT91C_TWI_STOP;
  }
  else
  {
    AT91C_BASE_TWI->TWI_CR = AT91C_TWI_START;
  }

  recieved = 0;
  do
  {
    if (AT91C_BASE_TWI->TWI_SR & AT91C_TWI_RXRDY)
    {
      ((unsigned char*)data)[recieved] = AT91C_BASE_TWI->TWI_RHR;
      recieved++;

      if (recieved < len)
      {
        if (recieved == len - 1)
          AT91C_BASE_TWI->TWI_CR = AT91C_TWI_STOP;
        continue;
      }

      while (!(AT91C_BASE_TWI->TWI_SR & AT91C_TWI_TXCOMP_MASTER))
        ;
      return recieved;
    }
  } while (1);
#elif defined(tfrog_rev4)
  memcpy(data, (void*)(FLASH_USERDATA_START + addr), len);
  return len;
#endif
  return -1;
}

int EEPROM_Write(int addr, void* data, int len)
{
#if defined(tfrog_rev5)
  int page, addr_l;
  int sent;

  sent = 0;

  do
  {
    AT91C_BASE_WDTC->WDTC_WDCR = 1 | 0xA5000000;
    while (!(AT91C_BASE_TWI->TWI_SR & AT91C_TWI_TXCOMP_MASTER))
      ;

    page = addr >> 8;
    addr_l = addr & 0xFF;
    AT91C_BASE_TWI->TWI_CR = AT91C_TWI_MSEN | AT91C_TWI_SVDIS;
    AT91C_BASE_TWI->TWI_MMR = ((0x50 | page) << 16) | AT91C_TWI_IADRSZ_1_BYTE;
    AT91C_BASE_TWI->TWI_IADR = addr_l;

    while (!(AT91C_BASE_TWI->TWI_SR & AT91C_TWI_TXRDY_MASTER))
      ;
    AT91C_BASE_TWI->TWI_THR = ((unsigned char*)data)[sent];

    do
    {
      while (!(AT91C_BASE_TWI->TWI_SR & AT91C_TWI_TXRDY_MASTER))
        ;
      sent++;
      addr++;
      if (sent < len)
      {
        if ((addr & 0x0F) == 0x00)
        {
          // Wait for 5ms
          AT91C_BASE_WDTC->WDTC_WDCR = 1 | 0xA5000000;
          msleep(5);
          break;
        }
        AT91C_BASE_TWI->TWI_THR = ((unsigned char*)data)[sent];
        continue;
      }
      else
      {
        while (!(AT91C_BASE_TWI->TWI_SR & AT91C_TWI_TXCOMP_MASTER))
          ;
        AT91C_BASE_WDTC->WDTC_WDCR = 1 | 0xA5000000;

        return sent;
      }
    } while (1);
  } while (1);
  return -2;
#elif defined(tfrog_rev4)
  FLASHD_Write((unsigned int)(FLASH_USERDATA_START + addr), data, len);
  return len;
#endif
  return -1;
}

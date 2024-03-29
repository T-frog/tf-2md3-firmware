/* ----------------------------------------------------------------------------
 * Copyright 2011-2019 T-frog Project, unless otherwise stated
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
 *
 * Libraries and some part of main.c are licensed under the following statements:
 *
 * ----------------------------------------------------------------------------
 *         ATMEL Microcontroller Software Support
 * ----------------------------------------------------------------------------
 * Copyright (c) 2008, Atmel Corporation
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * - Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the disclaimer below.
 *
 * Atmel's name may not be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * DISCLAIMER: THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
 * DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
 * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * ----------------------------------------------------------------------------
 */

#include <setjmp.h>
#include <stdint.h>
#include <string.h>

#include <board.h>
#include <pio/pio.h>
#include <pio/pio_it.h>
#include <aic/aic.h>
#include <tc/tc.h>
#include <usart/usart.h>
#include <utility/trace.h>
#include <utility/led.h>
#include <usb/common/core/USBStringDescriptor.h>
#include <usb/device/cdc-serial/CDCDSerialDriver.h>
#include <usb/device/cdc-serial/CDCDSerialDriverDescriptors.h>
#include <pmc/pmc.h>

#include "power.h"
#include "controlPWM.h"
#include "controlVelocity.h"
#include "registerFPGA.h"
#include "communication.h"
#include "eeprom.h"
#include "adc.h"
#include "io.h"
#include "filter.h"
#include "errors.h"
#include "debug.h"

#if defined(tfrog_rev5)
#warning "T-frog driver rev.5"
#endif
#if defined(tfrog_rev4)
#warning "T-frog driver rev.4"
#endif

volatile uint8_t rs485_timeout = 0;
volatile uint16_t tic = 0;

Tfrog_EEPROM_data saved_param = TFROG_EEPROM_DEFAULT;

extern uint8_t languageIdStringDescriptor[];
extern USBDDriverDescriptors cdcdSerialDriverDescriptors;
extern volatile int32_t w_receive_buf;
extern volatile int32_t r_receive_buf;

uint8_t manufacturerStringDescriptor2[64] = {
    USBStringDescriptor_LENGTH(0),
    USBGenericDescriptor_STRING,
};

uint8_t productStringDescriptor2[64] = {
    USBStringDescriptor_LENGTH(0),
    USBGenericDescriptor_STRING,
};

uint8_t* stringDescriptors2[3] = {
    languageIdStringDescriptor,
    productStringDescriptor2,
    manufacturerStringDescriptor2,
};

// ------------------------------------------------------------------------------
// Definitions
// ------------------------------------------------------------------------------

// / Size in bytes of the buffer used for reading data from the USB & USART
#define DATABUFFERSIZE \
  BOARD_USB_ENDPOINTS_MAXPACKETSIZE(CDCDSerialDriverDescriptors_DATAIN)

// ------------------------------------------------------------------------------
// Internal variables
// ------------------------------------------------------------------------------

char connecting = 0;
char connected = 0;

// / List of pins that must be configured for use by the application.
static const Pin pins[] = {
    PIN_PWM_ENABLE,
    PIN_PCK_PCK1,
    PINS_USERIO,
    PINS_RS485,
#if defined(tfrog_rev5)
    PIN_VERSION,
    PIN_BUZ,
#endif
    PIN_LED_0, PIN_LED_1, PIN_LED_2};
#if defined(tfrog_rev5)
static const Pin pinBuz[] = {PIN_BUZ};
static const Pin pinVer[] = {PIN_VERSION};
#endif

// / VBus pin instance.
static const Pin pinVbus = PIN_USB_VBUS;

// / PWM Enable pin instance.
const Pin pinPWMEnable = PIN_PWM_ENABLE;

// / Buffer for storing incoming USB data.
static uint8_t usbBuffer[DATABUFFERSIZE];

// ------------------------------------------------------------------------------
// Main
// ------------------------------------------------------------------------------

void SRAM_Init()
{
  static const Pin pinsSram[] = {PINS_SRAM};

  // Enable corresponding PIOs
  PIO_Configure(pinsSram, PIO_LISTSIZE(pinsSram));

#if defined(tfrog_rev1)
  AT91C_BASE_SMC->SMC2_CSR[0] =
      1 | AT91C_SMC2_WSEN | (0 << 8) | AT91C_SMC2_BAT | AT91C_SMC2_DBW_16 | (0 << 24) | (1 << 28);
#elif defined(tfrog_rev5)
  AT91C_BASE_SMC->SMC2_CSR[0] =
      0 | AT91C_SMC2_WSEN | (0 << 8) | AT91C_SMC2_BAT | AT91C_SMC2_DBW_16 | (0 << 24) | (0 << 28);
#else
  AT91C_BASE_SMC->SMC2_CSR[0] =
      0 | AT91C_SMC2_WSEN | (0 << 8) | AT91C_SMC2_BAT | AT91C_SMC2_DBW_16 | (0 << 24) | (0 << 28);
#endif
}

static char FPGA_test()
{
  THEVA.GENERAL.PWM.COUNT_ENABLE = 0;
  THEVA.GENERAL.OUTPUT_ENABLE = 0;

  do
  {
    THEVA.GENERAL.PWM.HALF_PERIOD = 1000;
    if ((volatile TVREG)(THEVA.GENERAL.PWM.HALF_PERIOD) != 1000)
      break;
    THEVA.GENERAL.PWM.HALF_PERIOD = 2000;
    if ((volatile TVREG)(THEVA.GENERAL.PWM.HALF_PERIOD) != 2000)
      break;
    THEVA.GENERAL.PWM.DEADTIME = 100;
    if ((volatile TVREG)(THEVA.GENERAL.PWM.DEADTIME) != 100)
      break;
    THEVA.GENERAL.PWM.DEADTIME = 30;
    if ((volatile TVREG)(THEVA.GENERAL.PWM.DEADTIME) != 30)
      break;
    return 1;
  } while (0);
  return 0;
}

// ------------------------------------------------------------------------------
// / Handles interrupts coming from PIO controllers.
// ------------------------------------------------------------------------------
/*
static void ISR_Vbus( const Pin * pPin )
{
}
*/

// ------------------------------------------------------------------------------
// / Configures the VBus pin to trigger an interrupt when the level on that pin
// / changes.
// ------------------------------------------------------------------------------
static void VBus_Configure(void)
{
  // Configure PIO
  PIO_Configure(&pinVbus, 1);
}

volatile uint8_t usb_read_pause = 0;
// ------------------------------------------------------------------------------
// / Callback invoked when data has been received on the USB.
// ------------------------------------------------------------------------------
static void UsbDataReceived(uint32_t unused, uint8_t status, uint32_t received, uint32_t remaining)
{
  // Check that data has been received successfully
  if (status == USBD_STATUS_SUCCESS)
  {
    int32_t remain = 0;

    // Check if bytes have been discarded
    if ((received == DATABUFFERSIZE) && (remaining > 0))
    {
      printf("USB:discard %luB\n\r", remaining);
    }

    LED_on(2);
    remain = data_fetch(usbBuffer, received);

    if (remain > 0)
    {
      printf("USB:remain %ldB\n\r", remain);
    }

    if (buf_left() < COMMAND_LEN * 2)
    {
      printf("USB:pause\n\r");
      usb_read_pause = 1;
    }
    else
    {
      CDCDSerialDriver_Read(usbBuffer, DATABUFFERSIZE, (TransferCallback)UsbDataReceived, 0);
    }
  }
  else
  {
    printf("USB:transfer err\n\r");
  }
}

void us0_received()
{
  rs485_timeout = 0;
  AIC_DisableIT(AT91C_ID_US0);
}

char buz_on = 0;
void timer1_tic()
{
  volatile uint32_t dummy;
  dummy = AT91C_BASE_TC1->TC_SR;
  dummy = dummy;

  tic++;
  if (rs485_timeout < 255)
    rs485_timeout++;
  AIC_EnableIT(AT91C_ID_US0);

#if defined(tfrog_rev5)
  if (driver_state.board_version == BOARD_R6B)
  {
    if (buz_on)
    {
      if ((tic & 0x7) < saved_param.buz_lvl)
        pinBuz->pio->PIO_SODR = pinBuz->mask;
      else
        pinBuz->pio->PIO_CODR = pinBuz->mask;
    }
    else
    {
      pinBuz->pio->PIO_CODR = pinBuz->mask;
    }
  }
#endif
}
void tic_init()
{
  volatile uint32_t dummy;

  AT91C_BASE_PMC->PMC_PCER = 1 << AT91C_ID_TC1;

  AT91C_BASE_TC1->TC_CCR = AT91C_TC_CLKDIS;
  AT91C_BASE_TC1->TC_IDR = 0xFFFFFFFF;
  dummy = AT91C_BASE_TC1->TC_SR;
  dummy = dummy;

  // MCK/32 * 1500 -> 1ms
  AT91C_BASE_TC1->TC_CMR = AT91C_TC_CLKS_TIMER_DIV3_CLOCK | AT91C_TC_WAVE | AT91C_TC_WAVESEL_UP_AUTO;
  AT91C_BASE_TC1->TC_CCR = AT91C_TC_CLKEN;
  AT91C_BASE_TC1->TC_RC = 1500 / 23;
  AT91C_BASE_TC1->TC_IER = AT91C_TC_CPCS;

  AIC_ConfigureIT(AT91C_ID_TC1, 5 | AT91C_AIC_SRCTYPE_POSITIVE_EDGE, (void (*)(void))timer1_tic);
  AIC_EnableIT(AT91C_ID_TC1);

  AT91C_BASE_TC1->TC_CCR = AT91C_TC_SWTRG;
}

// ------------------------------------------------------------------------------
// Main
// ------------------------------------------------------------------------------
// ------------------------------------------------------------------------------
// / Initializes drivers and start the USB <-> Serial bridge.
// ------------------------------------------------------------------------------
int main()
{
  static int16_t analog[16];
  static int32_t com_index[2][2];
  int32_t err_cnt;
  Filter1st voltf;
  int32_t vbuslv = 0;
  int32_t vbus = 0;
  int32_t _vbus = 0;
  uint32_t err_chk = 0;
  int16_t mscnt = 0;
  uint8_t errnum = 0;
  uint8_t blink = 0;

  motor[0].error_state = 0;
  motor[1].error_state = 0;

  // Configure IO
  PIO_Configure(pins, PIO_LISTSIZE(pins));

  LED_on(0);

  TRACE_CONFIGURE(DBGU_STANDARD, 230400, BOARD_MCK);
  printf("-- Locomotion Board %s --\n\r", SOFTPACK_VERSION);
  printf("-- %s\n\r", BOARD_NAME);
  printf("-- Compiled: %s %s --\n\r", __DATE__, __TIME__);

  switch (AT91C_BASE_RSTC->RSTC_RSR & AT91C_RSTC_RSTTYP)
  {
    case AT91C_RSTC_RSTTYP_POWERUP:
      printf("Power-up Reset. VDDCORE rising.\n\r");
      break;
    case AT91C_RSTC_RSTTYP_WAKEUP:
      printf("WakeUp Reset. VDDCORE rising.\n\r");
      break;
    case AT91C_RSTC_RSTTYP_WATCHDOG:
      printf("Watchdog Reset. Watchdog overflow occured.\n\r");
      break;
    case AT91C_RSTC_RSTTYP_SOFTWARE:
      printf("Software Reset. Processor reset required by the software.\n\r");
      break;
    case AT91C_RSTC_RSTTYP_USER:
      printf("User Reset. NRST pin detected low.\n\r");
      break;
    case AT91C_RSTC_RSTTYP_BROWNOUT:
      printf("Brownout Reset occured.\n\r");
      break;
  }

  switch (AT91C_BASE_RSTC->RSTC_RSR & AT91C_RSTC_RSTTYP)
  {
    case AT91C_RSTC_RSTTYP_SOFTWARE:
    case AT91C_RSTC_RSTTYP_USER:
      break;
    default:
#ifdef PINS_CLEAR
    {
      static const Pin pinsClear[] = {PINS_CLEAR};
      static const Pin pinsSet[] = {PINS_SET};

      AT91C_BASE_WDTC->WDTC_WDMR = AT91C_WDTC_WDDIS;
      AT91C_BASE_RSTC->RSTC_RMR = 0xA5000400;

      PIO_Configure(pinsClear, PIO_LISTSIZE(pinsClear));
      msleep(50);
      LED_off(0);
      PIO_Configure(pinsSet, PIO_LISTSIZE(pinsSet));
      msleep(50);
      LED_on(0);
      PIO_Configure(pinsClear, PIO_LISTSIZE(pinsClear));
      msleep(50);
      LED_off(0);
      PIO_Configure(pinsSet, PIO_LISTSIZE(pinsSet));
      msleep(50);
    }
#endif

      AT91C_BASE_RSTC->RSTC_RCR = 0xA5000000 | AT91C_RSTC_EXTRST | AT91C_RSTC_PROCRST | AT91C_RSTC_PERRST;
      while (1)
        ;
      break;
  }

  // Enable user reset
  AT91C_BASE_RSTC->RSTC_RMR = 0xA5000400 | AT91C_RSTC_URSTEN;

  // If they are present, configure Vbus & Wake-up pins
  PIO_InitializeInterrupts(0);

  // Disable PWM Output
  PIO_Set(&pinPWMEnable);

  printf("SRAM init\n\r");
  SRAM_Init();
  EEPROM_Init();

  err_cnt = 0;
  while (((volatile TVREG)(THEVA.GENERAL.ID) & 0xFF) != 0xA0)
  {
    volatile int32_t i;

#ifdef PINS_CLEAR
    static const Pin pinsClear[] = {PINS_CLEAR};
    static const Pin pinsSet[] = {PINS_SET};

    AT91C_BASE_WDTC->WDTC_WDMR = AT91C_WDTC_WDDIS;
    PIO_Configure(pinsClear, PIO_LISTSIZE(pinsClear));
    for (i = 0; i < 30000; i++)
      ;
    PIO_Configure(pinsSet, PIO_LISTSIZE(pinsSet));
#endif

    printf("Invalid FPGA %u !\n\r", THEVA.GENERAL.ID);
    for (i = 0; i < 30000; i++)
      ;
    err_cnt++;

    if (err_cnt > 2)
    {
      AT91C_BASE_RSTC->RSTC_RCR = 0xA5000000 | AT91C_RSTC_EXTRST | AT91C_RSTC_PROCRST | AT91C_RSTC_PERRST;
      while (1)
        ;
    }
  }
  printf("FPGA ID: %x\n\r", (volatile TVREG)(THEVA.GENERAL.ID));
  // Checking FPGA-version
  if (((volatile TVREG)(THEVA.GENERAL.ID) & 0xFF00) == 0x0000)
  {
    driver_state.zero_torque = 5 * 65536;
    driver_state.fpga_version = 0;
  }
  else if (((volatile TVREG)(THEVA.GENERAL.ID) & 0xFF00) == 0x0100)
  {
    driver_state.zero_torque = 0 * 65536;
    driver_state.fpga_version = 1;
  }
  else if (((volatile TVREG)(THEVA.GENERAL.ID) & 0xFF00) == 0x0200)
  {
    driver_state.zero_torque = 0 * 65536;
    driver_state.fpga_version = 2;
  }

  // FPGA test
  printf("FPGA test\n\r");
  if (!FPGA_test())
  {
    printf("  Failed\n\r");
    LED_on(0);
    msleep(200);
    LED_on(1);
    AT91C_BASE_RSTC->RSTC_RCR = 0xA5000000 | AT91C_RSTC_EXTRST | AT91C_RSTC_PROCRST | AT91C_RSTC_PERRST;
    while (1)
      ;
  }

  printf("ADC init\n\r");
  ADC_Init();

  // BOT driver initialization
  {
    const char manufacturer[] = {"T-frog project"};
    const char product[] = {"T-frog Driver"};
    int32_t i;

    manufacturerStringDescriptor2[0] = USBStringDescriptor_LENGTH(strlen(manufacturer));
    productStringDescriptor2[0] = USBStringDescriptor_LENGTH(strlen(product));

    for (i = 0; i < strlen(manufacturer); i++)
    {
      manufacturerStringDescriptor2[i * 2 + 2] = manufacturer[i];
      manufacturerStringDescriptor2[i * 2 + 2 + 1] = 0;
    }
    for (i = 0; i < strlen(product); i++)
    {
      productStringDescriptor2[i * 2 + 2] = product[i];
      productStringDescriptor2[i * 2 + 2 + 1] = 0;
    }
  }
  cdcdSerialDriverDescriptors.pFsDevice->iManufacturer = 2;
  cdcdSerialDriverDescriptors.pStrings = (const uint8_t**)stringDescriptors2;
  cdcdSerialDriverDescriptors.numStrings = 3;
  CDCDSerialDriver_Initialize();

  {
    Tfrog_EEPROM_data data_default = TFROG_EEPROM_DEFAULT;

#if defined(tfrog_rev1)
    data_default.PWM_deadtime = 18;
#elif defined(tfrog_rev5)
    data_default.PWM_deadtime = 7;
#else
    data_default.PWM_deadtime = 20;
#endif

    switch (EEPROM_Read(0x000, &saved_param, sizeof(Tfrog_EEPROM_data)))
    {
      case -1:
        // No EEPROM
        printf("No EEPROM\n\r");
        saved_param = data_default;
        break;
      case -2:
      case -3:
        // Read Error
        printf("EEPROM Read Error!\n\r");
        motor[0].error_state |= ERROR_EEPROM;
        motor[1].error_state |= ERROR_EEPROM;
        break;
      default:
        if (saved_param.key != TFROG_EEPROM_KEY)
        {
          char zero = 0;

          saved_param = data_default;

          EEPROM_Write(TFROG_EEPROM_ROBOTPARAM_ADDR, &zero, 1);
          msleep(5);
          EEPROM_Write(0, &data_default, sizeof(data_default));

          LED_on(0);
          LED_on(1);
          LED_on(2);
          msleep(200);

          AT91C_BASE_RSTC->RSTC_RCR = 0xA5000000 | AT91C_RSTC_EXTRST | AT91C_RSTC_PROCRST | AT91C_RSTC_PERRST;
          while (1)
            ;
        }
        else if (saved_param.size < TFROG_EEPROM_DATA_SIZE)
        {
          printf("Migrating EEPROM data\n\r");
          memcpy(
              ((char*)&saved_param) + saved_param.size,
              ((char*)&data_default) + saved_param.size,
              TFROG_EEPROM_DATA_SIZE - saved_param.size);
          saved_param.size = TFROG_EEPROM_DATA_SIZE;
          EEPROM_Write(0, &saved_param, sizeof(saved_param));

          LED_on(0);
          LED_on(1);
          LED_on(2);
          msleep(50);
          LED_off(0);
          LED_off(1);
          LED_off(2);
        }
        break;
    }
    if (saved_param.buz_lvl > 4)
      saved_param.buz_lvl = 0;

    THEVA.GENERAL.PWM.HALF_PERIOD = saved_param.PWM_resolution;
    THEVA.GENERAL.PWM.DEADTIME = saved_param.PWM_deadtime;

    if (saved_param.high_frequency_encoder)
    {
      THEVA.GENERAL.ENCODER.HFREQ = 1;
    }
    else
    {
      THEVA.GENERAL.ENCODER.HFREQ = 0;
    }

    driver_state.io_dir = saved_param.io_dir;
    set_io_dir(saved_param.io_dir);
    set_io_data(saved_param.io_data);
  }

  // Configure USB vbus pin
  VBus_Configure();

  driver_state.vsrc = 0;
  driver_state.error.low_voltage = 0;
  driver_state.error.hall[0] = 0;
  driver_state.error.hall[1] = 0;
  driver_state.error.hallenc[0] = 0;
  driver_state.error.hallenc[1] = 0;

  printf("Velocity Control init\n\r");
  // Configure velocity control loop
  controlVelocity_init();
  Filter1st_CreateLPF(&voltf, 10);

  if (saved_param.stored_data == TFROG_EEPROM_DATA_BIN ||
      saved_param.stored_data == TFROG_EEPROM_DATA_BIN_LOCKED)
  {
    if (saved_param.stored_param_version != TFROG_EEPROM_PARAM_VERSION)
    {
      printf("EEPROM has incompatible version of the parameter\n\r");
      motor[0].error_state |= ERROR_EEPROM;
      motor[1].error_state |= ERROR_EEPROM;
    }
    else
    {
      printf("Loading saved DriverParam\n\r");
      EEPROM_Read(TFROG_EEPROM_ROBOTPARAM_ADDR, &driver_param, sizeof(DriverParam));
      msleep(5);
      printf("Loading saved MotorParam[0]\n\r");
      EEPROM_Read(TFROG_EEPROM_ROBOTPARAM_ADDR + 0x100, &motor_param[0], sizeof(MotorParam));
      msleep(5);
      printf("Loading saved MotorParam[1]\n\r");
      EEPROM_Read(TFROG_EEPROM_ROBOTPARAM_ADDR + 0x200, &motor_param[1], sizeof(MotorParam));

      if (motor_param[0].enc_rev_raw / motor_param[0].enc_denominator != motor_param[0].enc_rev ||
          motor_param[1].enc_rev_raw / motor_param[1].enc_denominator != motor_param[1].enc_rev)
      {
        TRACE_ERROR("Embedded parameter has inconsistency\n\r");
        printf("enc_rev: %ld, %ld\n\r", motor_param[0].enc_rev, motor_param[1].enc_rev);
        printf("enc_denominator: %d, %d\n\r", motor_param[0].enc_denominator, motor_param[1].enc_denominator);
        printf("enc_rev_raw: %ld, %ld\n\r", motor_param[0].enc_rev_raw, motor_param[1].enc_rev_raw);
        motor[0].error_state |= ERROR_EEPROM;
        motor[1].error_state |= ERROR_EEPROM;
      }
    }
  }
  controlVelocity_config();

  printf("PWM control init\n\r");
  // Configure PWM control
  controlPWM_init();

  // Enable watchdog
  printf("Watchdog init\n\r");
  AT91C_BASE_WDTC->WDTC_WDMR = AT91C_WDTC_WDRSTEN | 0xFF00FF;  // 1s
  AT91C_BASE_WDTC->WDTC_WDCR = 1 | 0xA5000000;

  // Enable ticker
  printf("Start ticker\n\r");
  tic_init();

#define RS485BUF_SIZE 128
  uint8_t rs485buf_[2][RS485BUF_SIZE];
  uint8_t* rs485buf;
  uint8_t* rs485buf_next;
  uint16_t r_rs485buf_pos;

  rs485buf = &rs485buf_[0][0];
  rs485buf_next = &rs485buf_[1][0];
  r_rs485buf_pos = 0;
  printf("RS485 init\n\r");
  AT91C_BASE_PMC->PMC_PCER = 1 << AT91C_ID_US0;
  USART_Configure(AT91C_BASE_US0, AT91C_US_USMODE_RS485 | AT91C_US_CHRL_8_BITS, 3000000, BOARD_MCK);

  USART_SetTransmitterEnabled(AT91C_BASE_US0, 1);
  USART_SetReceiverEnabled(AT91C_BASE_US0, 1);
  USART_ReadBuffer(AT91C_BASE_US0, rs485buf, RS485BUF_SIZE);
  USART_ReadBuffer(AT91C_BASE_US0, rs485buf_next, RS485BUF_SIZE);

  AT91C_BASE_US0->US_IDR = 0xFFFFFFFF;
  AT91C_BASE_US0->US_IER = AT91C_US_RXRDY;
  AIC_ConfigureIT(AT91C_ID_US0, 3 | AT91C_AIC_SRCTYPE_POSITIVE_EDGE, (void (*)(void))us0_received);
  AIC_EnableIT(AT91C_ID_US0);

  {
    int32_t i;
    for (i = 0; i < COM_MOTORS; i++)
    {
      com_cnts[i] = 0;
      com_pwms[i] = 0;
      com_en[i] = 0;
    }
    com_en[0] = com_en[1] = 1;
  }

#if defined(tfrog_rev5)
  if (!(AT91C_BASE_PIOA->PIO_PDSR & pinVer->mask))
  {
    driver_state.board_version = BOARD_R6B;
    printf("Board version: R6B\n\r");
  }
  else
  {
    driver_state.board_version = BOARD_R6A;
    printf("Board version: R6, R6A\n\r");
  }
#else
  driver_state.board_version = BOARD_R4;
  printf("Board version: R4\n\r");
#endif
  printf("Entering main control loop\n\r------\n\r");

  ADC_Start();
  LED_off(0);

  err_cnt = 0;
  driver_state.error.low_voltage = 0;
  driver_state.error.hall[0] = 0;
  driver_state.error.hall[1] = 0;
  driver_state.error.hallenc[0] = 0;
  driver_state.error.hallenc[1] = 0;
  driver_state.ifmode = 0;
  driver_state.watchdog = 0;
  driver_state.odom_drop = 0;
  driver_state.vsrc_max = 0x3ff;
  switch (driver_state.board_version)
  {
    case BOARD_R6A:
    case BOARD_R4:
      break;
    case BOARD_R6B:
#if defined(tfrog_rev5)
      driver_state.vsrc_max = 0x3ff * VSRC_CONV_B;
#endif
      break;
  }

  // Driver loop
  while (1)
  {
    AT91C_BASE_WDTC->WDTC_WDCR = 1 | 0xA5000000;

    if (err_chk++ % 30 == 0)
    {
      if ((volatile int32_t)THEVA.GENERAL.PWM.HALF_PERIOD != saved_param.PWM_resolution ||
          (volatile int32_t)THEVA.GENERAL.PWM.DEADTIME != saved_param.PWM_deadtime ||
          THEVA.MOTOR[0].INVERT != 0 ||
          THEVA.MOTOR[1].INVERT != 0)
      {
        err_cnt++;
        controlPWM_init();
      }
      else
      {
        err_cnt = 0;
      }
      if (err_cnt > 3)
      {
        printf("FPGA-Value Error!\n\r");
        msleep(50);
        AT91C_BASE_RSTC->RSTC_RCR = 0xA5000000 | AT91C_RSTC_EXTRST | AT91C_RSTC_PROCRST | AT91C_RSTC_PERRST;
        while (1)
          ;
      }
    }

    if (driver_state.watchdog >= driver_param.watchdog_limit)
    {
      motor[0].servo_level = SERVO_LEVEL_STOP;
      motor[1].servo_level = SERVO_LEVEL_STOP;

      if (saved_param.stored_data == TFROG_EEPROM_DATA_BIN_SAVING)
      {
        LED_on(0);
        EEPROM_Write(TFROG_EEPROM_ROBOTPARAM_ADDR, &driver_param, sizeof(DriverParam));
        msleep(5);
        EEPROM_Write(TFROG_EEPROM_ROBOTPARAM_ADDR + 0x100, &motor_param[0], sizeof(MotorParam));
        msleep(5);
        EEPROM_Write(TFROG_EEPROM_ROBOTPARAM_ADDR + 0x200, &motor_param[1], sizeof(MotorParam));
        saved_param.stored_data = TFROG_EEPROM_DATA_BIN;
        saved_param.stored_param_version = TFROG_EEPROM_PARAM_VERSION;
        msleep(5);
        EEPROM_Write(0, &saved_param, sizeof(saved_param));
        LED_off(0);
      }
      if (!(saved_param.stored_data == TFROG_EEPROM_DATA_BIN ||
            saved_param.stored_data == TFROG_EEPROM_DATA_BIN_LOCKED))
      {
        controlVelocity_init();
        Filter1st_CreateLPF(&voltf, 10);
        controlPWM_init();
      }
      driver_state.error.hall[0] = 0;
      driver_state.error.hall[1] = 0;
      driver_state.error.hallenc[0] = 0;
      driver_state.error.hallenc[1] = 0;
      motor[0].error_state |= ERROR_WATCHDOG;
      motor[1].error_state |= ERROR_WATCHDOG;
      printf("Watchdog - init parameters\n\r");
      {
        int32_t i;
        printf("Motors: ");
        for (i = 0; i < COM_MOTORS; i++)
        {
          if (com_en[i])
            printf("%ld ", i);
          if (!(saved_param.stored_data == TFROG_EEPROM_DATA_BIN ||
                saved_param.stored_data == TFROG_EEPROM_DATA_BIN_LOCKED))
            com_cnts[i] = 0;
          com_pwms[i] = 0;
          com_en[i] = 0;
        }
        printf("\n\r");
        com_en[0] = com_en[1] = 1;
      }
      driver_state.watchdog = 0;
      if (!(saved_param.stored_data == TFROG_EEPROM_DATA_BIN ||
            saved_param.stored_data == TFROG_EEPROM_DATA_BIN_LOCKED))
      {
        driver_state.ifmode = 0;
      }
    }
    else
    {
      if (motor[0].servo_level != SERVO_LEVEL_STOP ||
          motor[1].servo_level != SERVO_LEVEL_STOP)
      {
        motor[0].error_state &= ~ERROR_WATCHDOG;
        motor[1].error_state &= ~ERROR_WATCHDOG;
      }
    }

    // Check current level on VBus
    if (PIO_Get(&pinVbus))
    {
      if (vbuslv < 10)
        vbuslv++;
      else if (vbuslv == 10)
        vbus = 1;
    }
    else
    {
      if (vbuslv > -10)
        vbuslv--;
      else if (vbuslv == -10)
        vbus = 0;
    }
    if (vbus != _vbus)
    {
      if (vbus == 1)
      {
        printf("USB:vbus connect\n\r");
        USBD_Connect();
        connecting = 1;
      }
      else
      {
        printf("USB:vbus disconnect\n\r");
        USBD_Disconnect();
      }
    }
    _vbus = vbus;

    if (AT91C_BASE_US0->US_RNCR == 0)
    {
      int16_t len;

      if (driver_state.ifmode == 1)
        LED_on(2);

      len = RS485BUF_SIZE - r_rs485buf_pos;
      if (data_fetch485(rs485buf + r_rs485buf_pos, len))
      {
        printf("485:buf ovf\n\r");
      }

      if (rs485buf == &rs485buf_[0][0])
      {
        rs485buf = &rs485buf_[1][0];
        rs485buf_next = &rs485buf_[0][0];
      }
      else
      {
        rs485buf = &rs485buf_[0][0];
        rs485buf_next = &rs485buf_[1][0];
      }
      USART_ReadBuffer(AT91C_BASE_US0, rs485buf_next, RS485BUF_SIZE);
      r_rs485buf_pos = 0;
    }
    else
    {
      int16_t len;
      len = (RS485BUF_SIZE - AT91C_BASE_US0->US_RCR) - r_rs485buf_pos;

      if (len > 0)
      {
        if (driver_state.ifmode == 1)
          LED_on(2);

        if (data_fetch485(rs485buf + r_rs485buf_pos, len))
        {
          printf("485:buf ovf\n\r");
        }
        r_rs485buf_pos += len;
      }
    }

    if (usb_read_pause)
    {
      printf("USB:flush r:%ld,w:%ld\n\r", r_receive_buf, w_receive_buf);
    }
    data_analyze();

    if (usb_read_pause)
    {
      usb_read_pause = 0;
      printf("USB:resume r:%ld,w:%ld\n\r", r_receive_buf, w_receive_buf);
      CDCDSerialDriver_Read(usbBuffer, DATABUFFERSIZE, (TransferCallback)UsbDataReceived, 0);
    }

    data_analyze485();
    if (driver_state.ping_request != 0)
    {
      LED_on(0);
      LED_on(2);
      printf("Ping: %lx\n\r", driver_state.ping_request);
      if (driver_state.ifmode == 0)
      {
        int_send(INT_ping_response, 0, driver_state.ping_request);
      }
      else
      {
        const char from =
            (saved_param.id485 == 0) ?
                (COMMUNICATION_ID_BROADCAST) :
                saved_param.id485;
        int_send485to(from, -1, INT_ping_response, 0, driver_state.ping_request);
      }
      driver_state.ping_request = 0;
      LED_off(0);
      LED_off(2);
    }

    if (connecting)
    {
      if (USBD_GetState() >= USBD_STATE_CONFIGURED)
      {
        printf("USB:start\n\r");
        // Start receiving data on the USB
        CDCDSerialDriver_Read(usbBuffer, DATABUFFERSIZE, (TransferCallback)UsbDataReceived, 0);
        connecting = 0;
        connected = 1;
      }
    }
    if (connected)
    {
      if (USBD_GetState() < USBD_STATE_DEFAULT)
      {
        printf("USB:disconnect\n\r");
        AT91C_BASE_RSTC->RSTC_RCR = 0xA5000000 | AT91C_RSTC_EXTRST;
        while (1)
          ;
      }
    }

    if (driver_state.cnt_updated >= 5)
    {
      uint16_t mask;
      int32_t i;
      /* 約5msおき */
      driver_state.cnt_updated -= 5;
      for (; driver_state.cnt_updated >= 5; driver_state.cnt_updated -= 5)
      {
        driver_state.odom_drop++;
      }

      mask = driver_state.admask;  // analog_mask;
      if (driver_state.io_mask[0])
        mask |= 0x100;
      if (driver_state.io_mask[1])
        mask |= 0x200;
      for (i = 0; i < 8; i++)
      {
        analog[i] = (i << 12) | ADC_Read(i);
      }
      analog[8] = (15 << 12) | get_io_data();
      analog[9] = (14 << 12) | THEVA.PORT[0];

      switch (driver_state.board_version)
      {
        case BOARD_R6A:
        case BOARD_R4:
          break;
        case BOARD_R6B:
#if defined(tfrog_rev5)
          analog[7] = (analog[7] & 0xFFF) * VSRC_CONV_B | (7 << 12);
#endif
          break;
      }

      com_pwms[0] = motor[0].ref.rate_buf;
      com_pwms[1] = motor[1].ref.rate_buf;
      com_cnts[0] = motor[0].enc_buf2;
      com_cnts[1] = motor[1].enc_buf2;
      if (driver_state.ifmode == 0)
        data_send(com_cnts, com_pwms, com_en, analog, mask);
      else
        data_send485(com_cnts, com_pwms, com_en, analog, mask);

      for (i = 2; i < COM_MOTORS; i++)
        com_pwms[i] = 0;

      for (i = 0; i < 2; i++)
      {
        const int16_t index_r = THEVA.MOTOR[i].INDEX_RISE_ANGLE;
        const int16_t index_f = THEVA.MOTOR[i].INDEX_FALL_ANGLE;

        if (index_r != com_index[i][0])
        {
          // New rising edge
          if (driver_state.ifmode == 0)
            int_send(INT_enc_index_rise, i, index_r);
          else
            int_send485(INT_enc_index_rise, i, index_r);
        }
        else if (index_f != com_index[i][1])
        {
          // New falling edge
          if (driver_state.ifmode == 0)
            int_send(INT_enc_index_fall, i, index_f);
          else
            int_send485(INT_enc_index_fall, i, index_f);
        }
        com_index[i][0] = index_r;
        com_index[i][1] = index_f;
      }

      driver_state.vsrc = Filter1st_Filter(&voltf, (int32_t)(analog[7] & 0x0FFF));
      ADC_Start();

      if (driver_param.vsrc_rated >= driver_state.vsrc_max)
      {
        driver_state.vsrc_factor = 32768;
      }
      else
      {
        driver_state.vsrc_factor = driver_param.vsrc_rated * 32768 / driver_state.vsrc;
      }
      if (driver_state.vsrc > 310 * 8 * VSRC_DIV && driver_state.vsrc > driver_param.vmin)
      {
        if (driver_state.error.low_voltage < 100)
        {
          driver_state.error.low_voltage++;
        }
        else
        {
          motor[0].error_state &= ~ERROR_LOW_VOLTAGE;
          motor[1].error_state &= ~ERROR_LOW_VOLTAGE;
        }
      }
      else
      {
        driver_state.vsrc_factor = 0;
        driver_state.error.low_voltage = 0;
        motor[0].error_state |= ERROR_LOW_VOLTAGE;
        motor[1].error_state |= ERROR_LOW_VOLTAGE;
      }
    }

    dump_send();  // return state dump if requested

    if (driver_state.velcontrol > 0)
    {
#define ERROR_BLINK_MS 200
      driver_state.velcontrol = 0;

      if (++mscnt >= ERROR_BLINK_MS)
      {
        if (usb_timeout_cnt > 0)
        {
          printf("USB:w timeout (%ld)\n\r", usb_timeout_cnt);
          usb_timeout_cnt = 0;
        }

        if (driver_state.protocol_version >= 10 &&
            (motor[0].servo_level >= SERVO_LEVEL_TORQUE ||
             motor[1].servo_level >= SERVO_LEVEL_TORQUE))
        {
          if (driver_state.ifmode == 0)
          {
            int_send(INT_error_state, 0, motor[0].error_state);
            int_send(INT_error_state, 1, motor[1].error_state);
          }
          else
          {
            int_send485(INT_error_state, 0, motor[0].error_state);
            int_send485(INT_error_state, 1, motor[1].error_state);
          }
        }
        mscnt = 0;

        if (motor[0].error_state || motor[1].error_state)
        {
          const int32_t motor_id = errnum / ERROR_NUM;
          const int32_t error_id = errnum % ERROR_NUM;
          if ((motor[motor_id].error_state & (1 << (error_id))) &&
              error_pat[motor_id][error_id] != 0)
          {
            if (error_pat[motor_id][error_id] & (1 << blink))
            {
              LED_on(0);
              buz_on = 1;
            }
            else
            {
              LED_off(0);
              buz_on = 0;
            }
            blink++;
            if (blink > 10)
            {
              blink = 0;
              errnum++;
            }
          }
          else
          {
            errnum++;
            if (errnum >= ERROR_NUM * 2)
              errnum = 0;
            blink = 0;
            LED_off(0);
            buz_on = 0;
            mscnt = ERROR_BLINK_MS;
          }
        }
        else
        {
          LED_off(0);
          buz_on = 0;
        }
      }
    }
    LED_off(2);
  }
}

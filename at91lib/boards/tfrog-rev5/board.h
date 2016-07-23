/* ----------------------------------------------------------------------------
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

//------------------------------------------------------------------------------
/// ¥dir
/// !Purpose
/// 
/// Definition and functions for using AT91SAM7SE-related features, such
/// has PIO pins, memories, etc.
/// 
/// !Usage
/// -# The code for booting the board is provided by board_cstartup.S and
///    board_lowlevel.c.
/// -# For using board PIOs, board characteristics (clock, etc.) and external
///    components, see board.h.
/// -# For manipulating memories (remapping, SDRAM, etc.), see board_memories.h.
//------------------------------------------------------------------------------
 
//------------------------------------------------------------------------------
/// ¥unit
/// !Purpose
/// 
/// Definition of AT91SAM7SE-EK characteristics, AT91SAM7SE-dependant PIOs and
/// external components interfacing.
/// 
/// !Usage
/// -# For operating frequency information, see "SAM7SE-EK - Operating frequencies".
/// -# For using portable PIO definitions, see "SAM7SE-EK - PIO definitions".
/// -# Several USB definitions are included here (see "SAM7SE-EK - USB device").
/// -# For external components definitions, see "SAM7SE-EK - External components".
/// -# For memory-related definitions, see "SAM7SE-EK - Memories".
//------------------------------------------------------------------------------

#ifndef BOARD_H 
#define BOARD_H 

//------------------------------------------------------------------------------
//         Headers
//------------------------------------------------------------------------------

#if defined(at91sam7se256)
    #include "at91sam7se256/AT91SAM7SE256.h"
#else
    #error Board does not support the specified chip.
#endif

//------------------------------------------------------------------------------
//         Definitions
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
/// ¥page "SAM7SE-EK - Board Description"
/// This page lists several definition related to the board description
///
/// !Definitions
/// - BOARD_NAME

/// Name of the board.
#define BOARD_NAME "T-frog Motor Driver rev.5" 
/// Board definition.
#define tfrog_rev5
/// Family definition.
#define at91sam7se 
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
/// ¥page "SAM7SE-EK - Operating frequencies"
/// This page lists several definition related to the board operating frequency
/// (when using the initialization done by board_lowlevel.c).
/// 
/// !Definitions
/// - BOARD_MAINOSC
/// - BOARD_MCK

/// Frequency of the board main oscillator.
#define BOARD_MAINOSC           18432000

/// Master clock frequency (when using board_lowlevel.c).
#define BOARD_MCK               48000000
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// ADC
//------------------------------------------------------------------------------
/// ADC clock frequency, at 10-bit resolution (in Hz)
#define ADC_MAX_CK_10BIT         5000000
/// ADC clock frequency, at 8-bit resolution (in Hz)
#define ADC_MAX_CK_8BIT          8000000
/// Startup time max, return from Idle mode (in ｵs)
#define ADC_STARTUP_TIME_MAX       20
/// Track and hold Acquisition Time min (in ns)
#define ADC_TRACK_HOLD_TIME_MIN   600



#define BOARD_FLASH_EFC

//------------------------------------------------------------------------------
/// ¥page "SAM7SE-EK - USB device"
/// This page lists constants describing several characteristics (controller
/// type, D+ pull-up type, etc.) of the USB device controller of the chip/board.
/// 

/// Chip has a UDP controller.
#define BOARD_USB_UDP

/// Indicates the D+ pull-up is internal to the USB controller.
#define BOARD_USB_PULLUP_INTERNAL

/// Number of endpoints in the USB controller.
#define BOARD_USB_NUMENDPOINTS                  8

/// Returns the maximum packet size of the given endpoint.
#define BOARD_USB_ENDPOINTS_MAXPACKETSIZE(i)    (((i == 4) || (i == 5)) ? 512 : 64)

/// Returns the number of FIFO banks for the given endpoint.
#define BOARD_USB_ENDPOINTS_BANKS(i)            (((i == 0) || (i == 3)) ? 1 : 2)

/// USB attributes configuration descriptor (bus or self powered, remote wakeup)
#define BOARD_USB_BMATTRIBUTES                  USBConfigurationDescriptor_SELFPOWERED_NORWAKEUP
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
/// ¥page "SAM7SE - PIO definitions"
/// This pages lists all the pio definitions contained in board.h. The constants
/// are named using the following convention: PIN_* for a constant which defines
/// a single Pin instance (but may include several PIOs sharing the same
/// controller), and PINS_* for a list of Pin instances.
///

/// DBGU pins (DTXD and DRXD) definitions.
#define PINS_DBGU  {0x00000600, AT91C_BASE_PIOA, AT91C_ID_PIOA, PIO_PERIPH_A, PIO_DEFAULT}

/// LED #0 pin definition.
#define PIN_LED_0  {1 << 11, AT91C_BASE_PIOA, AT91C_ID_PIOA, PIO_OUTPUT_1, PIO_DEFAULT}
/// LED #1 pin definition.
#define PIN_LED_1  {1 << 12, AT91C_BASE_PIOA, AT91C_ID_PIOA, PIO_OUTPUT_1, PIO_DEFAULT}
/// LED #2 pin definition.
#define PIN_LED_2  {1 << 13, AT91C_BASE_PIOA, AT91C_ID_PIOA, PIO_OUTPUT_1, PIO_DEFAULT}
/// List of all LEDs pin definitions.
#define PINS_LEDS  PIN_LED_0, PIN_LED_1, PIN_LED_2

/// ADC_AD0 pin definition.
#define PIN_ADC_ADC0 {1 << 17, AT91C_BASE_PIOA, AT91C_ID_PIOA, PIO_INPUT, PIO_PULLUP}
/// ADC_AD1 pin definition.
#define PIN_ADC_ADC1 {1 << 18, AT91C_BASE_PIOA, AT91C_ID_PIOA, PIO_INPUT, PIO_PULLUP}
/// ADC_AD2 pin definition.
#define PIN_ADC_ADC2 {1 << 19, AT91C_BASE_PIOA, AT91C_ID_PIOA, PIO_INPUT, PIO_PULLUP}
/// ADC_AD3 pin definition.
#define PIN_ADC_ADC3 {1 << 20, AT91C_BASE_PIOA, AT91C_ID_PIOA, PIO_INPUT, PIO_PULLUP}
/// Pins ADC
#define PINS_ADC PIN_ADC_ADC0, PIN_ADC_ADC1, PIN_ADC_ADC2, PIN_ADC_ADC3

/// USB VBus monitoring pin definition.
#define PIN_USB_VBUS    {1 << 19, AT91C_BASE_PIOC, AT91C_ID_PIOC, PIO_INPUT, PIO_PULLUP}

#define PINS_CLEAR  \
	{0x000000FE, AT91C_BASE_PIOB, AT91C_ID_PIOB, PIO_OUTPUT_0, PIO_DEFAULT}, \
	{0x0000FFFF, AT91C_BASE_PIOC, AT91C_ID_PIOC, PIO_OUTPUT_0, PIO_DEFAULT}, \
	{0x00E00000, AT91C_BASE_PIOC, AT91C_ID_PIOC, PIO_OUTPUT_0, PIO_DEFAULT}, \
	{1 << 19, AT91C_BASE_PIOB, AT91C_ID_PIOB, PIO_OUTPUT_0, PIO_DEFAULT}, \
	{1 << 20, AT91C_BASE_PIOB, AT91C_ID_PIOB, PIO_OUTPUT_0, PIO_DEFAULT}, \
	{1 << 17, AT91C_BASE_PIOB, AT91C_ID_PIOB, PIO_OUTPUT_0, PIO_DEFAULT}

#define PINS_SET  \
	{0x000000FE, AT91C_BASE_PIOB, AT91C_ID_PIOB, PIO_OUTPUT_1, PIO_DEFAULT}, \
	{0x0000FFFF, AT91C_BASE_PIOC, AT91C_ID_PIOC, PIO_OUTPUT_1, PIO_DEFAULT}, \
	{0x00E00000, AT91C_BASE_PIOC, AT91C_ID_PIOC, PIO_OUTPUT_1, PIO_DEFAULT}, \
	{1 << 19, AT91C_BASE_PIOB, AT91C_ID_PIOB, PIO_OUTPUT_1, PIO_DEFAULT}, \
	{1 << 20, AT91C_BASE_PIOB, AT91C_ID_PIOB, PIO_OUTPUT_1, PIO_DEFAULT}, \
	{1 << 17, AT91C_BASE_PIOB, AT91C_ID_PIOB, PIO_OUTPUT_1, PIO_DEFAULT}


#define PINS_SRAM  \
    {0x000000FE, AT91C_BASE_PIOB, AT91C_ID_PIOB, PIO_PERIPH_B, PIO_PULLUP}, \
    {0x0000FFFF, AT91C_BASE_PIOC, AT91C_ID_PIOC, PIO_PERIPH_A, PIO_PULLUP}, \
    {0x00E00000, AT91C_BASE_PIOC, AT91C_ID_PIOC, PIO_PERIPH_B, PIO_PULLUP}


#define PINS_USERIO  \
    {0xFF000000, AT91C_BASE_PIOB, AT91C_ID_PIOB, PIO_INPUT, PIO_PULLUP}

#define PINS_RS485  {0x000000E0, AT91C_BASE_PIOA, AT91C_ID_US0, PIO_PERIPH_A, PIO_PULLUP}
#define PIN_VERSION  {0x00010000, AT91C_BASE_PIOA, AT91C_ID_PIOA, PIO_INPUT, PIO_PULLUP}
#define PIN_BUZ      {0x00004000, AT91C_BASE_PIOA, AT91C_ID_PIOA, PIO_OUTPUT_1, PIO_DEFAULT}
//------------------------------------------------------------------------------

#define PIN_PWM_ENABLE    {1 << 16, AT91C_BASE_PIOC, AT91C_ID_PIOC, PIO_OUTPUT_1, PIO_DEFAULT}

#define PIN_PWM_CYCLE     {1 << 19, AT91C_BASE_PIOB, AT91C_ID_PIOB, PIO_PERIPH_A, PIO_PULLUP}
#define PIN_PWM_CYCLE2    {1 << 20, AT91C_BASE_PIOB, AT91C_ID_PIOB, PIO_PERIPH_A, PIO_PULLUP}
#define PIN_PCK_PCK1      {1 << 17, AT91C_BASE_PIOB, AT91C_ID_PIOB, PIO_PERIPH_A, PIO_DEFAULT}

#define PINS_EEPROM_TWD    {0x00000008, AT91C_BASE_PIOA, AT91C_ID_PIOA, PIO_OUTPUT_1, PIO_PULLUP | PIO_OPENDRAIN}
#define PINS_EEPROM_TWCK   {0x00000010, AT91C_BASE_PIOA, AT91C_ID_PIOA, PIO_OUTPUT_1, PIO_PULLUP | PIO_OPENDRAIN}
#define PINS_EEPROM        {0x00000018, AT91C_BASE_PIOA, AT91C_ID_PIOA, PIO_PERIPH_A, PIO_PULLUP | PIO_OPENDRAIN}

#define FLASH_USERDATA_START   ((char*)0x0013F800)

#define RAMFUNC __attribute__ ((long_call, section (".ramfunc")))


#define VSRC_DIV         10/130
#define VSRC_CONV_B      25/13 //(10*125)/(130*5)


#endif //#ifndef BOARD_H


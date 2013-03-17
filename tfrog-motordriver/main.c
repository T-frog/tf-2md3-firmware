/* ---------------------------------------------------------------------------- ATMEL Microcontroller Software Support
 * ---------------------------------------------------------------------------- Copyright (c) 2008, Atmel Corporation
 * All rights reserved. Redistribution and use in source and binary forms, with or without modification, are permitted
 * provided that the following conditions are met: - Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the disclaimer below. Atmel's name may not be used to endorse or promote
 * products derived from this software without specific prior written permission. DISCLAIMER: THIS SOFTWARE IS PROVIDED 
 * * * BY ATMEL "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE DISCLAIMED. IN NO EVENT SHALL ATMEL BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED 
 * TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * ---------------------------------------------------------------------------- */

// -----------------------------------------------------------------------------
// Headers
// ------------------------------------------------------------------------------

#include <board.h>
#include <pio/pio.h>
#include <pio/pio_it.h>
#include <aic/aic.h>
#include <tc/tc.h>
#include <usart/usart.h>
#include <utility/trace.h>
#include <string.h>
#include <utility/led.h>
#include <usb/common/core/USBStringDescriptor.h>
#include <usb/device/cdc-serial/CDCDSerialDriver.h>
#include <usb/device/cdc-serial/CDCDSerialDriverDescriptors.h>
#include <pmc/pmc.h>
#include <setjmp.h>

#include "power.h"
#include "controlPWM.h"
#include "controlVelocity.h"
#include "registerFPGA.h"
#include "communication.h"
#include "eeprom.h"
#include "adc.h"
#include "filter.h"

// extern int getStackPointer( void );
// extern int getIrqStackPointer( void );

int velcontrol = 0;
Tfrog_EEPROM_data saved_param = TFROG_EEPROM_DEFAULT;

extern unsigned char languageIdStringDescriptor[];
extern USBDDriverDescriptors cdcdSerialDriverDescriptors;

unsigned char manufacturerStringDescriptor2[64] = {
	USBStringDescriptor_LENGTH( 0 ),
	USBGenericDescriptor_STRING
};

unsigned char productStringDescriptor2[64] = {
	USBStringDescriptor_LENGTH( 0 ),
	USBGenericDescriptor_STRING
};

unsigned char *stringDescriptors2[3] = {
	languageIdStringDescriptor,
	productStringDescriptor2,
	manufacturerStringDescriptor2
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
	PIN_LED_0, PIN_LED_1, PIN_LED_2
};
static const Pin pinsLED[] = { PIN_LED_0, PIN_LED_1, PIN_LED_2 };

// / VBus pin instance.
static const Pin pinVbus = PIN_USB_VBUS;

// / PWM Enable pin instance.
const Pin pinPWMEnable = PIN_PWM_ENABLE;

// / Buffer for storing incoming USB data.
static unsigned char usbBuffer[DATABUFFERSIZE];


void LED_on( int num )
{
	PIO_Clear(&pinsLED[num]);
}
void LED_off( int num )
{
	PIO_Set(&pinsLED[num]);
}

// ------------------------------------------------------------------------------
// Main
// ------------------------------------------------------------------------------

void SRAM_Init(  )
{
	static const Pin pinsSram[] = { PINS_SRAM };

	// Enable corresponding PIOs
	PIO_Configure( pinsSram, PIO_LISTSIZE( pinsSram ) );

	AT91C_BASE_SMC->SMC2_CSR[0] =
		1 | AT91C_SMC2_WSEN | ( 0 << 8 ) | AT91C_SMC2_BAT | AT91C_SMC2_DBW_16 | ( 0 << 24 ) | ( 1 << 28 );
}


static int FPGA_test( )
{
	THEVA.GENERAL.PWM.COUNT_ENABLE = 0;
	THEVA.GENERAL.OUTPUT_ENABLE = 0;

	do
	{
		THEVA.GENERAL.PWM.HALF_PERIOD = 1000;
		if( (volatile TVREG)(THEVA.GENERAL.PWM.HALF_PERIOD) != 1000 )
			break;
		THEVA.GENERAL.PWM.HALF_PERIOD = 2000;
		if( (volatile TVREG)(THEVA.GENERAL.PWM.HALF_PERIOD) != 2000 )
			break;
		THEVA.GENERAL.PWM.DEADTIME = 100;
		if( (volatile TVREG)(THEVA.GENERAL.PWM.DEADTIME) != 100 )
			break;
		THEVA.GENERAL.PWM.DEADTIME = 30;
		if( (volatile TVREG)(THEVA.GENERAL.PWM.DEADTIME) != 30 )
			break;
		return 1;
	}
	while( 0 );
	return 0;
}

// ------------------------------------------------------------------------------
// / Handles interrupts coming from PIO controllers.
// ------------------------------------------------------------------------------
static void ISR_Vbus( const Pin * pPin )
{
	// Check current level on VBus
	if( PIO_Get( &pinVbus ) )
	{
		TRACE_INFO( "VBUS conn\n\r" );
		USBD_Connect(  );
		connecting = 1;
	}
	else
	{
		TRACE_INFO( "VBUS discon\n\r" );
		USBD_Disconnect(  );
	}
}

// ------------------------------------------------------------------------------
// / Configures the VBus pin to trigger an interrupt when the level on that pin
// / changes.
// ------------------------------------------------------------------------------
static void VBus_Configure( void )
{
	TRACE_INFO( "VBus configuration\n\r" );

	// Configure PIO
	PIO_Configure( &pinVbus, 1 );
	PIO_ConfigureIt( &pinVbus, ISR_Vbus );
	PIO_EnableIt( &pinVbus );

	ISR_Vbus( &pinVbus );
}


// ------------------------------------------------------------------------------
// / Callback invoked when data has been received on the USB.
// ------------------------------------------------------------------------------
static void UsbDataReceived( unsigned int unused, unsigned char status, unsigned int received, unsigned int remaining )
{
	// Check that data has been received successfully
	if( status == USBD_STATUS_SUCCESS )
	{
		static int remain = 0;

		LED_on( 2 );
		remain = data_fetch( usbBuffer, received + remain );

		CDCDSerialDriver_Read( usbBuffer + remain, DATABUFFERSIZE - remain, ( TransferCallback ) UsbDataReceived, 0 );

		// Check if bytes have been discarded
		if( ( received == DATABUFFERSIZE ) && ( remaining > 0 ) )
		{

			TRACE_WARNING( "UsbDataReceived: %u bytes discarded\n\r", remaining );
		}
	}
	else
	{

		TRACE_WARNING( "UsbDataReceived: Transfer error\n\r" );
	}
}

// ------------------------------------------------------------------------------
// Main
// ------------------------------------------------------------------------------

// ------------------------------------------------------------------------------
// / Initializes drivers and start the USB <-> Serial bridge.
// ------------------------------------------------------------------------------
int main(  )
{
	short analog[16];
	short enc_buf2[2];
	int err_cnt;
	Filter1st voltf;

	// Configure IO
	PIO_Configure( pins, PIO_LISTSIZE( pins ) );

	LED_on( 0 );

	switch( AT91C_BASE_RSTC->RSTC_RSR & AT91C_RSTC_RSTTYP )
	{
	case AT91C_RSTC_RSTTYP_SOFTWARE:
	case AT91C_RSTC_RSTTYP_USER:
		break;
	default:
		#ifdef PINS_CLEAR
		{
			static const Pin pinsClear[] = { PINS_CLEAR };
			static const Pin pinsSet[] = { PINS_SET };
			volatile int i;

			AT91C_BASE_WDTC->WDTC_WDMR = AT91C_WDTC_WDDIS;
			AT91C_BASE_RSTC->RSTC_RMR = 0xA5000400;

			PIO_Configure( pinsClear, PIO_LISTSIZE( pinsClear ) );
			for( i = 0; i < 40000; i ++ );
			LED_off( 0 );
			PIO_Configure( pinsSet, PIO_LISTSIZE( pinsSet ) );
			for( i = 0; i < 40000; i ++ );
		}
		#endif

		AT91C_BASE_RSTC->RSTC_RCR = 0xA5000000 | AT91C_RSTC_EXTRST | AT91C_RSTC_PROCRST | AT91C_RSTC_PERRST;
		while( 1 );
		break;
	}

	// Enable user reset
	AT91C_BASE_RSTC->RSTC_RMR = 0xA5000400 | AT91C_RSTC_URSTEN;

	TRACE_CONFIGURE( DBGU_STANDARD, 230400, BOARD_MCK );
	printf( "-- Locomotion Board %s --\n\r", SOFTPACK_VERSION );
	printf( "-- %s\n\r", BOARD_NAME );
	printf( "-- Compiled: %s %s --\n\r", __DATE__, __TIME__ );

	// If they are present, configure Vbus & Wake-up pins
	PIO_InitializeInterrupts( 0 );

	// Disable PWM Output
	PIO_Set( &pinPWMEnable );

	printf( "SRAM init\n\r" );
	SRAM_Init(  );
	EEPROM_Init(  );

	err_cnt = 0;
	while( (volatile TVREG)THEVA.GENERAL.ID != 0xA0 )
	{
		volatile int i;

		#ifdef PINS_CLEAR
		static const Pin pinsClear[] = { PINS_CLEAR };
		static const Pin pinsSet[] = { PINS_SET };

		AT91C_BASE_WDTC->WDTC_WDMR = AT91C_WDTC_WDDIS;
		PIO_Configure( pinsClear, PIO_LISTSIZE( pinsClear ) );
		for( i = 0; i < 30000; i ++ );
		PIO_Configure( pinsSet, PIO_LISTSIZE( pinsSet ) );
		#endif

		TRACE_ERROR( "Invalid FPGA %u !\n\r", THEVA.GENERAL.ID );
		for( i = 0; i < 30000; i ++ );
		err_cnt ++;

		if( err_cnt > 2 )
			AT91C_BASE_RSTC->RSTC_RCR = 0xA5000000 | AT91C_RSTC_EXTRST;
	}

	// FPGA test
	printf( "FPGA test\n\r" );
	if( !FPGA_test() )
	{
		volatile int i;

		printf( "  Failed\n\r" );
		LED_on( 0 );
		LED_on( 1 );
		for( i = 0; i < 60000; i ++ );
		AT91C_BASE_RSTC->RSTC_RCR = 0xA5000000 | AT91C_RSTC_EXTRST;
	}

	printf( "ADC init\n\r" );
	ADC_Init(  );

	// BOT driver initialization
	{
		const char manufacturer[] = { "T-frog project" };
		const char product[] = { "T-frog Driver" };
		int i;

		manufacturerStringDescriptor2[0] = USBStringDescriptor_LENGTH( strlen( manufacturer ) );
		productStringDescriptor2[0] = USBStringDescriptor_LENGTH( strlen( product ) );

		for( i = 0; i < strlen( manufacturer ); i++ )
		{
			manufacturerStringDescriptor2[i * 2 + 2] = manufacturer[i];
			manufacturerStringDescriptor2[i * 2 + 2 + 1] = 0;
		}
		for( i = 0; i < strlen( product ); i++ )
		{
			productStringDescriptor2[i * 2 + 2] = product[i];
			productStringDescriptor2[i * 2 + 2 + 1] = 0;
		}
	}
	cdcdSerialDriverDescriptors.pFsDevice->iManufacturer = 2;
	cdcdSerialDriverDescriptors.pStrings = ( const unsigned char ** )stringDescriptors2;
	cdcdSerialDriverDescriptors.numStrings = 3;
	CDCDSerialDriver_Initialize(  );

	{
		Tfrog_EEPROM_data data_default = TFROG_EEPROM_DEFAULT;

		#if defined(tfrog_rev1)
		data_default.PWM_deadtime = 18;
		#elif defined(tfrog_rev5)
		data_default.PWM_deadtime = 10;
		#else
		data_default.PWM_deadtime = 20;
		#endif

		switch( EEPROM_Read( 0x000, &saved_param, sizeof(Tfrog_EEPROM_data) ) )
		{
		case -1:
			// No EEPROM
			printf( "No EEPROM\n\r" );
			saved_param = data_default;
			break;
		case -2:
		case -3:
			// Read Error
			TRACE_ERROR( "EEPROM Read Error!\n\r" );
			AT91C_BASE_RSTC->RSTC_RCR = 0xA5000000 | AT91C_RSTC_EXTRST;
			break;
		default:
			if( saved_param.key != TFROG_EEPROM_KEY )
			{
				char zero = 0;

				saved_param = data_default;

				EEPROM_Write( TFROG_EEPROM_ROBOTPARAM_ADDR, &zero, 1 );
				msleep( 5 );
				EEPROM_Write( 0, &data_default, sizeof(data_default) );

				LED_on( 0 );
				LED_on( 1 );
				LED_on( 2 );
				msleep( 200 );

				AT91C_BASE_RSTC->RSTC_RCR = 0xA5000000 | AT91C_RSTC_EXTRST;
			}
			break;
		}

	}

	// connect if needed
	VBus_Configure(  );

	printf( "PWM control init\n\r" );
	// Configure PWM control
	controlPWM_init(  );

	printf( "Velocity Control init\n\r" );
	// Configure velocity control loop
	controlVelocity_init(  );

	enc_buf2[0] = enc_buf2[1] = 0;

	// Enable watchdog
	AT91C_BASE_WDTC->WDTC_WDMR = AT91C_WDTC_WDRSTEN | 0xFF00FF; // 1s
	AT91C_BASE_WDTC->WDTC_WDCR = 1 | 0xA5000000;
	
	LED_off( 0 );
	ADC_Start();
	driver_param.vsrc = 0;
	Filter1st_CreateLPF( &voltf, 10 ); // 50ms

	// Driver loop
	while( 1 )
	{
		// static int i;
		// int j;

		AT91C_BASE_WDTC->WDTC_WDCR = 1 | 0xA5000000;

		if( driver_param.servo_level >= SERVO_LEVEL_TORQUE )
		{
			if( driver_param.watchdog >= driver_param.watchdog_limit )
			{
				controlVelocity_init( );
				controlPWM_init(  );
				LED_on( 1 );
			}
			else
			{
				LED_off( 1 );
			}
		}
		data_analyze(  );
		if( connecting )
		{
			if( USBD_GetState(  ) < USBD_STATE_CONFIGURED )
				continue;

			// Start receiving data on the USB
			CDCDSerialDriver_Read( usbBuffer, DATABUFFERSIZE, ( TransferCallback ) UsbDataReceived, 0 );
			connecting = 0;
			connected = 1;
		}
		if( connected )
		{
			if( USBD_GetState(  ) < USBD_STATE_DEFAULT )
			{
				AT91C_BASE_RSTC->RSTC_RCR = 0xA5000000 | AT91C_RSTC_EXTRST;
				while( 1 );
			}
		}

		if( driver_param.cnt_updated >= 5 )
		{
			unsigned short mask;
			int i;
			// static long cnt = 0;
			/* 約5msおき */

			mask = driver_param.admask;			// analog_mask;
			if( driver_param.io_mask )
				mask |= 0x100;
			for( i = 0; i < 8; i ++ )
			{
				analog[i] = ( i << 12 ) | ADC_Read( i );
			}
			analog[8] = ( 15 << 12 ) | THEVA.PORT[0];
			driver_param.vsrc = Filter1st_Filter( &voltf, (int)( analog[ 0 ] & 0x03FF ) );

			data_send( ( short )( ( short )motor[0].enc_buf - ( short )enc_buf2[0] ),
					   ( short )( ( short )motor[1].enc_buf - ( short )enc_buf2[1] ),
					   motor[0].ref.rate_buf, motor[1].ref.rate_buf, analog, mask );

			enc_buf2[0] = motor[0].enc_buf;
			enc_buf2[1] = motor[1].enc_buf;

			driver_param.cnt_updated = 0;
			ADC_Start();

			if( driver_param.vsrc < driver_param.vsrc_rated / 4 )
			{
				driver_param.vsrc_factor = 0;
			}
			else
			{
				driver_param.vsrc_factor = driver_param.vsrc_rated * 32768 / driver_param.vsrc;
			}
		}

		if( velcontrol == 1 )
		{
			velcontrol = 0;
			ISR_VelocityControl(  );
		}
		LED_off( 2 );
	}
}

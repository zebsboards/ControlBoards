/******************************************************************************
	LW2CloneU2

	Copyright (C) 2013
	All sourcecode parts that are not from LUFA are free!
*******************************************************************************/
 
#include <avr/io.h>
#include <avr/wdt.h>
#include <avr/power.h>
#include <avr/interrupt.h>
#include <util/atomic.h>
#include <string.h>
#include <LUFA/Drivers/USB/USB.h>

#include "descriptors.h"


//**************************************************************************************************************
// edit this table to specify which port/pin is used for the LEDs or output driver and the polarity,
// e.g. ( D, 0, 1 ) for PD0 and inverted output ('light on' <==> 'pin low', 'light off' <==> 'pin high')
// the following example corresponds to the given schematic "lwcloneu2.sch"

#if defined(__AVR_ATmega32U2__)

#define LED_MAPPING_TABLE(_map_) \
	_map_( C, 2, 0 ) \
	_map_( C, 4, 0 ) \
	_map_( C, 5, 0 ) \
	_map_( C, 6, 1 ) \
	_map_( C, 7, 1 ) \
	_map_( D, 0, 1 ) \
	_map_( D, 1, 1 ) \
	_map_( D, 2, 0 ) \
	_map_( D, 3, 0 ) \
	_map_( D, 4, 0 ) \
	_map_( D, 5, 0 ) \
	_map_( D, 6, 0 ) \
	_map_( D, 7, 0 )

#elif defined(__AVR_ATmega32U4__)

#define LED_MAPPING_TABLE(_map_) \
	/* the following corresponds to the Arduino Leonardo mapping */ \
	\
	_map_( D, 5, 0 ) /* Digital Pin 1  Output 1*/ \
	_map_( D, 3, 0 ) /* Digital Pin 12 Output 2*/ \
	_map_( D, 2, 0 ) /* Digital Pin 13 Output 3*/ \
	_map_( D, 1, 0 ) /* Analog Pin 4   Output 4*/ \
	_map_( D, 0, 0 ) /* Analog Pin 5   Output 5*/ \
	\
	_map_( B, 7, 0 ) /* Digital Pin 0 Output 6*/ \
	_map_( B, 3, 0 ) /* Digital Pin 1 Output 7*/ \
	_map_( B, 2, 0 ) /* Digital Pin 2 Output 8*/ \
	_map_( B, 1, 0 ) /* Digital Pin 3 Output 9*/ \
	_map_( B, 0, 0 ) /* Digital Pin 4 Output 10*/ \
	_map_( E, 6, 0 ) /* Digital Pin 5 Output 11*/ \
	_map_( F, 0, 0 ) /* Digital Pin 6 Output 12*/ \
	_map_( F, 1, 0 ) /* Digital Pin 7 Output 13*/ \
	_map_( F, 4, 0 ) /* Digital Pin 8 Output 14*/ \
	_map_( F, 5, 0 ) /* Digital Pin 9 Output 15*/ \
	_map_( F, 6, 0 ) /* Digital Pin 10 Output 16*/ \
	_map_( C, 7, 0 ) /* Digital Pin 10 Output 16*/ \
	_map_( C, 6, 0 ) /* Digital Pin 10 Output 16*/ \
	_map_( B, 6, 0 ) /* Digital Pin 10 Output 16*/ \
	_map_( B, 5, 0 ) /* Digital Pin 10 Output 16*/ \
	_map_( B, 4, 0 ) /* Digital Pin 10 Output 16*/ \
	_map_( D, 7, 0 ) /* Digital Pin 10 Output 16*/ \
	_map_( D, 6, 0 ) /* Digital Pin 10 Output 16*/ \
	_map_( D, 4, 0 ) /* Digital Pin 10 Output 16*/ \
	
	
	
	
	
#else
#error "unsupported platform"
#endif
	
//**************************************************************************************************************

#define MAP(X, pin, inv) X##pin##_index,
enum { LED_MAPPING_TABLE(MAP) NUMBER_OF_LEDS };
#undef MAP

#define NUMBER_OF_BANKS   ((NUMBER_OF_LEDS + 7) / 8)


static void HID_Task(void);
static void SetupHardware(void);
static void Timer_init(void);
static void Ports_init(void);

static void update_state(uint8_t * p5bytes);
static void update_profile(int8_t nbank, uint8_t * p8bytes);


// Main program entry point. This routine configures the hardware required by the application, then
// enters a loop to run the application tasks in sequence.

int main(void){
	
	/* Disables JTAG functionality of 4 ACD pins. This allows OUTPUT LOW values on them */
	MCUCR=(1<<JTD);
    MCUCR=(1<<JTD);
	
	SetupHardware();
	sei();

	for (;;)
	{
		HID_Task();
		USB_USBTask();
	}
}

// Configures the board hardware and chip peripherals for the demo's functionality.

void SetupHardware(void)
{
	/* Disable watchdog if enabled by bootloader/fuses */
	MCUSR &= ~(1 << WDRF);
	wdt_disable();
	
	/* Disable clock division */
	clock_prescale_set(clock_div_1);

	/* Hardware Initialization */
	
	Ports_init();

	// USB initialization
	USB_Init();
	
	// Timer
	Timer_init();
}

// Event handler for the USB_Connect event. This indicates that the device is enumerating via the status LEDs and
// starts the library USB task to begin the enumeration and USB management process.
 
void EVENT_USB_Device_Connect(void)
{
}

// Event handler for the USB_Disconnect event. This indicates that the device is no longer connected to a host via
// the status LEDs and stops the USB management and joystick reporting tasks.
 
void EVENT_USB_Device_Disconnect(void)
{
}

// Event handler for the library USB Configuration Changed event.

void EVENT_USB_Device_ConfigurationChanged(void)
{
	/* Setup HID Report Endpoint */
	
	Endpoint_ConfigureEndpoint(
		GENERIC_IN_EPADDR,
		EP_TYPE_INTERRUPT,
		GENERIC_EPSIZE,
		1);
}

// Event handler for the USB_ControlRequest event. This is used to catch and process control requests sent to
// the device from the USB host before passing along unhandled control requests to the library for processing
// internally.
 
void EVENT_USB_Device_ControlRequest(void)
{
	/* Handle HID Class specific requests */
	switch (USB_ControlRequest.bRequest)
	{
	case HID_REQ_GetReport:
		if (USB_ControlRequest.bmRequestType == (REQDIR_DEVICETOHOST | REQTYPE_CLASS | REQREC_INTERFACE))
		{
			Endpoint_ClearSETUP();
			
			uint8_t zero = 0;
			
			/* Write one 'zero' byte report data to the control endpoint */
			Endpoint_Write_Control_Stream_LE(&zero, 1);
			Endpoint_ClearOUT();
		}
		break;
		
	case HID_REQ_SetReport:
		if (USB_ControlRequest.bmRequestType == (REQDIR_HOSTTODEVICE | REQTYPE_CLASS | REQREC_INTERFACE))
		{
			uint8_t data[8];
			static uint8_t nbank = 0;

			Endpoint_ClearSETUP();

			/* Read the report data from the control endpoint */
			Endpoint_Read_Control_Stream_LE(data, sizeof(data));
			Endpoint_ClearIN();

			if (data[0] == 64)
			{
				update_state(&data[1]);
				nbank = 0;
			}
			else
			{
				update_profile(nbank, data);
				nbank = (nbank + 1) & 0x03;
			}
		}
		break;
	}
}

// Function to manage HID report generation and transmission to the host.

void HID_Task(void)
{
	// nothing to do here because
	// we don't have anything to report or receive to/from the host via the normal report transmission since
	// the data from the host is received in the control request
}

void Timer_init(void)
{
	cli();

	// ----- TIMER0 (8bit) -----
	// timer for int routine

	#define T0_CYCLE_US  200

	OCR0A = (((T0_CYCLE_US * (F_CPU / 1000L)) / (64 * 1000L)) - 1);
	TCNT0 = 0x00;

	// clear timer/counter on compare0 match
	TCCR0A = _BV(WGM01);
	TCCR0B = _BV(CS01) |_BV(CS00); // prescale 64

	// enable Output Compare 0 overflow interrupt
	TIMSK0 = _BV(OCIE0A);

	sei();
}


struct {
	volatile uint8_t enable;
	volatile uint8_t mode;
} g_LED[NUMBER_OF_BANKS * 8];

volatile uint16_t g_dt = 256;

#define MAX_PWM 49


static void update_state(uint8_t * p5bytes)
{
	for (int8_t k = 0; k < NUMBER_OF_BANKS; k++)
	{
		uint8_t b = p5bytes[k];

		for (int8_t i = 0; i < 8; i++)
		{
			g_LED[k * 8 + i].enable = b & 0x01;
			b >>= 1;
		}
	}

	uint8_t pulse_speed = p5bytes[4];

	if (pulse_speed > 7)
	    pulse_speed = 7;

	if (pulse_speed == 0)
	    pulse_speed = 1;
		
	g_dt = pulse_speed * 128;
}

static void update_profile(int8_t k, uint8_t * p8bytes)
{
	if (k >= NUMBER_OF_BANKS)
		return;

	for (int8_t i = 0; i < 8; i++)
	{
		g_LED[k * 8 + i].mode = p8bytes[i];
	}
}

static void update_pwm(uint8_t *pwm, int8_t n, uint16_t t)
{
	for (int8_t i = 0; i < n; i++) 
	{
		if (g_LED[i].enable == 0)
		{
			pwm[i] = 0;
		}
		else
		{
			uint8_t b = g_LED[i].mode;

			if ((b >= 0) && (b <= 48))
			{
				// constant brightness

				pwm[i] = b;
			}
			else if (b == 129)
			{
				// triangle

				uint16_t x = t >> 8;
				if (x & 0x80)	// 128..255
					x = 255 - x;

				pwm[i] = (48 * x) >> 7;

			}
			else if (b == 130)
			{
				// rect
				
				pwm[i] = (t & 0x8000) ? 48 : 0;
			}
			else if (b == 131)
			{
				// fall

				uint16_t x = 255 - (t >> 8);

				pwm[i] = (48 * x) >> 8;
			}
			else if (b == 132)
			{
				// rise

				uint16_t x = t >> 8;

				pwm[i] = (48 * x) >> 8;
			}
			else
			{
				// unexpected!

				pwm[i] = 48;
			}
		}
	}
}

// Timer0 compare A handler

ISR(TIMER0_COMPA_vect)
{
	static int8_t counter = 0;
	static uint16_t t = 0;
	static uint8_t pwm[NUMBER_OF_LEDS];

	counter--;
	
	if (counter < 0)
	{
		// reset counter
		counter = MAX_PWM - 1;	// pwm value of MAX_PWM should be allways 'on', 0 should be allways 'off'

		// increment time counter
		t += g_dt;

		// update pwm values
		update_pwm(pwm, sizeof(pwm) / sizeof(pwm[0]), t);
	}

	#define MAP(X, pin, inv) if ((pwm[X##pin##_index] > counter) == (!inv)) { PORT##X |= (1 << pin); } else { PORT##X &= ~(1 << pin); }
	LED_MAPPING_TABLE(MAP)
	#undef MAP
}

static void Ports_init(void)
{
	
	/**Set Ports to OUTPUT**/
	
	#define MAP(X, pin, inv) \
		PORT##X &= ~(1 << pin); \
		DDR##X  |= (1 << pin);
	LED_MAPPING_TABLE(MAP)
	#undef MAP
}


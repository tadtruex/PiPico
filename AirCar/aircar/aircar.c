/*
 * AirCar MarkII
 *
 * Tad Truex, F23
 *
 * Goals:
 * 	At a minimum, replicate the existing 8051 firmware.  Ideally,
 * 	modernize a bit, potentially adding additional sensors (accelerometer),
 * 	possibly a small display, an RTC, and (hopefully) eliminate the need for
 * 	9V battery.
 *
 * 	At a minimum
 *
 * 	1) Count the pulses from the rotary encoder.
 * 	2) Accurately record the value every 28.2uS (CHECK)
 * 	3) Appear as a USB-Drive when connected to a host.
 *
 *	Extras
 *	4) Add an oled display
 *	5) ??
 *
 * 	Status: (Main status at https://github.com/users/tadtruex/projects/14)
 *
 * 	9/21: Pulse counting works fine.  The encoder is 360 PPR.  2.5"-ish
 * 	      diameter on a 16' track = 32 revolutions * 360 * 4 (quadrature) =
 * 	      46080 counts WC.  Signed int32_t is plenty.
 *
 * 	      Question:  The lab handout says 40 PPR - did Doug divide down to
 * 	      fit into an 8 bit number (it looks like the old data format is
 * 	      delta rather than absolute)
 *
 *	TBD:
 *		Establish a time base (I'm not even sure what clock rate the PiPico is
 *		really running at.)
 *
 *		Establish USB mass storage device.
 *
 *		Disable the reset button.
 *
 *
 *
 *
 */



#include "pico/stdlib.h"
#include "hardware/gpio.h"

#include <stdint.h>
#include <stdio.h>


typedef struct _encoder {
	uint leadPinNum;
	uint lagPinNum;
} encoderT;

const encoderT encoder = { 16, 17 };


// Gray coded transitions on the encoder.
#define Quadrant0 ((uint32_t)0)
#define Quadrant1 ((uint32_t)(1<<16))
#define Quadrant2 ((uint32_t)(1<<16 | 1<<17))
#define Quadrant3 ((uint32_t)(1<<17))


// Track the wheel rotations in a 30.2 format (quarter rotations)
volatile int32_t pulseCount30p2 = 0;

// Track encoder errors
volatile uint32_t errorCount = 0;

// Interrupt handler for the encoder pin changes
void stateUpdate( uint gpio, uint32_t eventMask );

// Timer callback to sample the encoder data
bool timerFunc( repeating_timer_t *rt );

const uint led = PICO_DEFAULT_LED_PIN;

int main() {

	repeating_timer_t rt;

	gpio_init(4);
	gpio_set_dir(4, GPIO_OUT);
	gpio_put(4,1);

	gpio_init(led);
	gpio_set_dir(led, GPIO_OUT);
	gpio_put(led,1);

	// Prepare for some cave man debugging
	stdio_uart_init();

	// Set the encoder pins to be inputs with no pullup/pulldown
	//  (the encoder has the pullups)
	gpio_init_mask(1<< encoder.lagPinNum | 1 << encoder.leadPinNum);

	// Remove this in production
	gpio_set_pulls(16, true, true);
	gpio_set_pulls(17, true, true);

    gpio_set_irq_enabled_with_callback(encoder.lagPinNum, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &stateUpdate);
    gpio_set_irq_enabled_with_callback(encoder.leadPinNum, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &stateUpdate);

    add_repeating_timer_us( 100, timerFunc, NULL, &rt);

    while(1);

}

void stateUpdate( uint gpio, uint32_t eventMask ){
	static uint currentState = 0;
	static uint nextState = 0;
	const uint32_t encMask = 1<<encoder.leadPinNum | 1 << encoder.lagPinNum;

	nextState = gpio_get_all() & encMask;

	switch (currentState) {
	case Quadrant0 :
		if (nextState == Quadrant1) { pulseCount30p2++; }
		if (nextState == Quadrant2) { errorCount++; }
		if (nextState == Quadrant3) { pulseCount30p2--; }
		break;
	case Quadrant1 :
		if (nextState == Quadrant2) { pulseCount30p2++; }
		if (nextState == Quadrant3) { errorCount++; }
		if (nextState == Quadrant0) { pulseCount30p2--; }
		break;
	case Quadrant2 :
		if (nextState == Quadrant3) { pulseCount30p2++; }
		if (nextState == Quadrant0) { errorCount++; }
		if (nextState == Quadrant1) { pulseCount30p2--; }
		break;
	case Quadrant3 :
		if (nextState == Quadrant0) { pulseCount30p2++; }
		if (nextState == Quadrant1) { errorCount++; }
		if (nextState == Quadrant2) { pulseCount30p2--; }
		break;
	}

	currentState = nextState;

}

bool timerFunc( repeating_timer_t *rt ){
	bool val = gpio_get(4);
	gpio_put(4, !val );

	return true;
}


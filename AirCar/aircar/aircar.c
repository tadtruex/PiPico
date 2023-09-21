
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include <stdint.h>

void gpioInt( uint gpio, uint32_t eventMask );

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


int main() {
	// Set the encoder pins to be inputs with no pullup/pulldown
	//  (the encoder has the pullups)
	gpio_init_mask(1<< encoder.lagPinNum | 1 << encoder.leadPinNum);

	// Remove this in production
	gpio_set_pulls(16, true, true);
	gpio_set_pulls(17, true, true);

    gpio_set_irq_enabled_with_callback(encoder.lagPinNum, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &stateUpdate);
    gpio_set_irq_enabled_with_callback(encoder.leadPinNum, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &stateUpdate);

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


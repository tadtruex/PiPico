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
#include "hardware/clocks.h"
#include "hardware/pwm.h"
#include <stdint.h>
#include <stdio.h>

#include "bsp/board.h"
#include "ff.h"
#include <diskio.h>
#include <disk.h>
#include <ramdisk.h>

#include "version.h"


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
volatile int32_t nextRotationCount = 0x7fffffff;

// This seems to be what the old aircar did.
#define NUM_SAMPLES 800
volatile int32_t SampleBuffer[NUM_SAMPLES];

// Track the number of 28.2us pulses.
volatile int32_t counterTicks = 0;

// Track encoder errors
volatile uint32_t errorCount = 0;

// Interrupt handler for the encoder pin changes
void stateUpdate( uint gpio, uint32_t eventMask );

const uint led = PICO_DEFAULT_LED_PIN;

int do_test(void);

uint8_t pdisk[1<<15];

PARTITION VolToPart[FF_VOLUMES] = {
			  {0,1},
};


uint32_t millis;

void ledTask(uint32_t cycleTime);

void pwmInterrupt(void);

uint32_t msPerLedCycle = 5000;


int main() {

  
  board_init();

	gpio_init(led);
	gpio_set_dir(led, GPIO_OUT);
	gpio_put(led,1);

	// Prepare for some cave man debugging
	stdio_uart_init();

  // init device stack on configured roothub port
	tud_init(BOARD_TUD_RHPORT);

  initRamDisk();

  {
    BYTE buf[FF_MAX_SS];
    FATFS fs;
    FIL fil;

    int n;
    
    // Initialize the ram disk
    if (f_fdisk(0, (LBA_t[]){100,0}, buf) ) msPerLedCycle = 200;

    // Create a filesystem
    if (f_mkfs( "0:", 0, buf, FF_MAX_SS ) ) msPerLedCycle = 200;

    if (f_mount( &fs, "", 0 ) ) msPerLedCycle = 200;

    if (f_open( &fil, "hello.txt", FA_CREATE_NEW | FA_WRITE )) msPerLedCycle = 200;

    /* Write a message */
    n = sprintf( buf, "%s", _version_str );
    f_write( &fil, buf, n, &n );
    
    /* Close the file */
    f_close(&fil);
  }
  
  //do_test();
	
	// Set the encoder pins to be inputs with no pullup/pulldown
	//  (the encoder has the pullups)
	gpio_init_mask(1<< encoder.lagPinNum | 1 << encoder.leadPinNum);

	// Remove this in production
	gpio_set_pulls(16, true, true);
	gpio_set_pulls(17, true, true);

    gpio_set_irq_enabled_with_callback(encoder.lagPinNum, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &stateUpdate);
    gpio_set_irq_enabled_with_callback(encoder.leadPinNum, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &stateUpdate);

    // This is not cool.  Using -28 here causes the chip to hang.
    //  Updating to -28 in the handler is fine.
    //add_repeating_timer_us( -100, timerFunc, NULL, &rt);

    // Use GPIO 4 (for now) as an indicator of interrupt entry.
    gpio_init(4);
    gpio_set_function(4, GPIO_FUNC_SIO);
    gpio_set_dir(4, GPIO_OUT);
    gpio_put(4, 0);
    
    uint slice_num = pwm_gpio_to_slice_num(4);

    // 125MHz cycle time for PWM clock.  3525 ticks gets us ~28.2us
    pwm_set_wrap(slice_num, 3524);

    // Set channel A output high for one cycle before dropping
    pwm_set_chan_level(slice_num, PWM_CHAN_A, 2000);

    // We're going to need an interrupt for this
    pwm_set_irq_enabled(slice_num, true);

    irq_set_exclusive_handler( PWM_IRQ_WRAP, pwmInterrupt );
    irq_set_priority( PWM_IRQ_WRAP, 0 );
    
    // Set the PWM running
    pwm_set_enabled(slice_num, true);

    pwm_clear_irq(slice_num);
        irq_set_enabled(PWM_IRQ_WRAP, true);

    for( int i = 0; i < 26; i++ ) {
      if ( irq_is_enabled(i) ) {
	printf( "IRQ %d: %d\n", i, irq_get_priority(i) );
      }
    }

    msPerLedCycle = ~0;
    printf("Main loop start\n");
    
    while(1){
      millis = to_ms_since_boot( get_absolute_time() );

      // Start data collection after 1/4 rotation
      if ( pulseCount30p2 > 90*4 && nextRotationCount == 0x7fffffff ) {
	msPerLedCycle = 1000;
	nextRotationCount = pulseCount30p2 + 36;  // Nine degrees counting 1/4 degrees
	printf( "Sampling enabled\n" );
      }
      ledTask(msPerLedCycle);
      tud_task();
    }

}

void stateUpdate( uint gpio, uint32_t eventMask ){

  static uint currentState = 0;
  static uint nextState = 0;
  static int sampleCount = 0;
  static bool done = false;

  if ( !done ) {
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

    if ( pulseCount30p2 > nextRotationCount ) {
      SampleBuffer[sampleCount++] = counterTicks;
      nextRotationCount += 36;  // 9 degrees
      if ( sampleCount == NUM_SAMPLES ) {
	done = true;
	msPerLedCycle = 500;
	printf( "Sampling complete\n");
      }
    }
  }
}

void pwmInterrupt( void ) {
  //  counterTicks++;
  //  gpio_put( 4, !gpio_get(4) );
}

  
void ledTask( uint32_t msPerCycle ) {
  static uint32_t lastMillis = 0;

  if ( msPerCycle = 0 ) {
    gpio_put(led, 0);
  } else if ( msPerCycle = ~0 ) {
    gpio_put(led, 1);
  } else if ( millis - lastMillis > msPerCycle>>2 ) {
    lastMillis = millis;
    gpio_put( led, !gpio_get(led) );
  }
}



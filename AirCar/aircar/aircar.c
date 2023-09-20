
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include <stdint.h>

void gpioInt( uint gpio, uint32_t eventMask );


int main() {

	stdio_init_all();

	gpio_set_function(16, GPIO_FUNC_SIO );
	gpio_set_function(17, GPIO_FUNC_SIO );

	gpio_pull_up(16);
    gpio_pull_up(17);

    gpio_set_irq_callback(gpioInt);
    gpio_set_irq_enabled(16, GPIO_IRQ_EDGE_RISE|GPIO_IRQ_EDGE_FALL, true);

    while(1);
}




void gpioInt( uint gpio, uint32_t eventMask ){
	while(1);
}

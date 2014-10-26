#include <avr/interrupt.h>
#include <avr/io.h>
#include <avr/power.h>
#include <avr/sleep.h>
#include <avr/wdt.h>
#include <util/delay.h>

static inline void led_set(unsigned char bits)
{
	unsigned char mask = (1<<5)|(1<<6);
	PORTD &= ~(mask);
	PORTD |= (bits<<5) & mask;
}

void init_hardware(void)
{
	/* Disable watchdog if enabled by bootloader/fuses */
	MCUSR &= ~(1<<WDRF);
	wdt_disable();

	/* Disable prescaler */
	clock_prescale_set(clock_div_1);

	/* Enable port B pin 0 as an output */
	DDRB = (1<<DDB0);
	PORTB = 0;

	/* LEDs */
	DDRD |= (1<<5)|(1<<6);
}


int main(void)
{
	init_hardware();
	
	while (1)
	{
		_delay_us(100);
		PORTB = 1;
		_delay_us(100);
		PORTB = 0;
	}
}


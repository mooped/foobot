#include <avr/interrupt.h>
#include <avr/io.h>
#include <avr/power.h>
#include <avr/sleep.h>
#include <avr/wdt.h>
#include <util/delay.h>

void init_hardware(void)
{
	// Disable watchdog if enabled by bootloader/fuses
	MCUSR &= ~(1<<WDRF);
	wdt_disable();

	// Disable prescaler
	clock_prescale_set(clock_div_1);

	// Enable port A pin 0:1 as output
	DDRA = (1<<0)|(1<<1);
	PORTA = 0x01;

  // Enable port D pin 3 as input
  DDRD = (0<<PD3);
  PORTD = 0;

  // Enable interrupts on INT1 (PD3) on all state transitions
  MCUCR = (0<<ISC11)|(1<<ISC10);
  GIMSK |= (1<<INT1);

  sei();
}

int main(void)
{
	init_hardware();
	
	while (1)
	{
	}
}

ISR(INT1_vect, ISR_BLOCK)
{
  PORTA = (PIND & (1<<PD3)) ? 1 : 2;
}


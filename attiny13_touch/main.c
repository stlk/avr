#define F_CPU 8000000UL
#include <avr/io.h>
#include <util/delay.h>
#include "TinyTouchLib.c"

int main(void)
{
	CLKPR=_BV(CLKPCE);
	CLKPR=0;			// set clock prescaler to 1 (attiny 25/45/85/24/44/84/13/13A)		

	DDRB=_BV(PB4);		// Enable LED output pin

	tinytouch_init();

	while(1) {
		if (tinytouch_sense()==tt_push) {
			PORTB |= _BV(PB4);
			_delay_ms(500);	
			PORTB &= ~_BV(PB4);
		}
		_delay_ms(10);	
	}	
}


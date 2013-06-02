/*
 *
 *#include <avr/io.h>
 *#include <stdlib.h>
 *#include <util/delay.h>
 * 
 *int main(void)
 *{
 *    const int msecsDelayPost = 1000;
 * 
 *    // Set up Port B pin 4 mode to output
 *    DDRB = 1<<DDB4;
 * 
 *    // Set up Port B data to be all low
 *    PORTB = 0;  
 * 
 *    while (1) {
 *        // Toggle Port B pin 4 output state
 *        PORTB ^= 1<<PB4;
 * 
 *        // Pause a little while
 *        _delay_ms (msecsDelayPost);
 *    }
 * 
 *    return 0;
 *}

 */
 
 
 #define F_CPU 1200000UL
 #include <avr/io.h> 
 #include <util/delay.h> 
 #include <avr/interrupt.h> 
 
 
 #define LED PB4
 #define PIN_RESET PB3
 
 #define output_low(port,pin) port &= ~(1<<pin) 
 #define output_high(port,pin) port |= (1<<pin) 
 volatile uint8_t ledStatus;

 int main(void) { 
	ledStatus = 0;
    GIMSK = _BV (INT0); // int - Enable external interrupts int0 
    MCUCR = _BV (ISC01); // int - INT0 is falling edge 
    sei(); // int - Global enable interrupts 
    DDRB |= (1 << LED); // Set direction register output 
	
	//Enable internal pull-up resistor
	PORTB |= (1 << PB1);
	PORTB |= (1 << PIN_RESET);
	
    for (;;) {
		_delay_us(10);  //optional

		if(bit_is_clear(PINB, PINB3))
		{
			ledStatus = 0;
		}

		if(ledStatus == 1)
		{
			output_high(PORTB, LED); 
		}
		else
		{
			output_low(PORTB, LED); 
		}
	}
    return 0; 
 } 

 ISR (INT0_vect) // Interrupt on Int0 vector 
 {                  
	ledStatus = 1;
 } 
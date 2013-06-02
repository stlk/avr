/*
Program to flicker 2 leds when it's dark.
See http://spritesmods.com/?art=minimalism&f=gpl for more info

Copyright (C) 2008 Sprite_tm
    
This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#define F_CPU 1200000UL
#include <util/delay.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>
#include <avr/pgmspace.h>
#include <avr/sleep.h>
#include <avr/wdt.h>

#define LED1	PB4 //ADC2, used for measuring light
#define LED2 	PB3
#define CALEN 	PB0

//Bunch o' random numbers: I'm too lazy to write or port a real random number
//generator.
//generated using bash:
//for x in `seq 0 255`; do echo -n $(($RANDOM%256)),; done
unsigned char randomvals[] PROGMEM = {
 234,191,103,250,144,74,39,34,215,128,9,122,144,74,137,105,123,218,158,175,205,
 118,149,13,98,7,173,179,194,97,115,110,213,80,220,142,102,102,36,152,90,135,
 105,176,173,49,6,197,48,140,176,122,4,53,83,216,212,202,170,180,214,53,161,
 225,129,185,106,22,12,190,97,158,170,92,160,194,134,169,98,246,128,195,24,
 198,165,156,77,126,113,136,58,156,196,136,41,246,164,84,138,171,184,42,214,
 203,128,89,39,198,85,140,148,149,36,215,78,170,234,131,124,152,239,154,214,
 130,194,49,3,69,248,120,179,101,163,131,124,184,148,213,118,213,81,177,149,
 58,213,33,201,63,10,195,215,190,7,86,245,128,9,8,40,102,51,125,94,92,5,159,
 75,253,158,40,4,6,178,241,92,124,73,248,1,157,61,50,86,136,113,22,16,171,209,
 230,144,240,14,188,2,167,22,88,57,50,86,171,73,114,175,34,226,245,57,180,111,
 220,186,170,242,141,229,49,158,30,82,161,49,124,65,139,24,95,14,133,65,238,
 116,180,190,49,130,30,30,59,93,173,139,19,187,2,163,102,26,255,23,239,196,19,
 6,162
};

//Gets a semi-random number between 0 and 255
unsigned char GetRandom(void) {
    //This'll probably give a warning because we use it uninitialised. Little 
    //does the compiler know: that's actually what we _want_ :)
    static unsigned char random1, random2;
    random1++;
    if (random1==0) random2+=0x41;
    return pgm_read_byte(randomvals+random1)^random2;
}


int GetLight(void) {
    int val;
    //set up measuring
    ADMUX=0x42; //measure pb4 using internal ref
    ADCSRA=0x83;//enable adc, max prescaler
    
    //kill led2
    DDRB|=(1<<LED1)|(1<<LED2);
    PORTB&=~(1<<LED2);
    //kill remaining charge in led
    PORTB&=~(1<<LED1);
    _delay_ms(50);
    //let led generate some voltage
    DDRB&=~(1<<LED1);
    _delay_ms(50);
    ADCSRA|=0x40; //go do adc
    while(ADCSRA&0x40) ; //wait till adc is done
    val=ADC;
    ADCSRA=0; //disable adc
    DDRB|=(1<<LED1); //re-enable led
    return val;
}

void PowerOffUc(void) {
    //Go to sleep until we're reset by the wdt.
    cli();
    WDTCR&=~0x40;
    PORTB=0; DDRB=0;
    set_sleep_mode(SLEEP_MODE_PWR_DOWN);
    sleep_mode();
}

#define CAL_ADDR 2


void Calibrate() {
    int i,j;
    //Measure current light level and store in EEPROM
    //Wait for the level to stabilize first.
    i=GetLight();
    eeprom_write_word(CAL_ADDR,i);
    DDRB|=(1<<LED1)|(1<<LED2);
    PORTB|=(1<<LED1)|(1<<LED2);
    while((PINB&(1<<CALEN))==0);

}

//Retrieves dark level calibration from EEPROM
int GetDarkCal(void) {
    return eeprom_read_word(CAL_ADDR);
}

ISR(WDT_vect) {
    //check if it's still dark
    if (GetLight()>GetDarkCal()) {
	PowerOffUc();
    }
}

int main(void) {
    unsigned char lval1, lval2;
    unsigned char x,y;

    //set up wdt
    wdt_enable(WDTO_2S);

    //check if we should calibrate
    PORTB=(1<<CALEN);
    _delay_us(10);
    if ((PINB&(1<<CALEN))==0) Calibrate();
    
        
    if (GetLight()>GetDarkCal()) {
	PowerOffUc();
    }

    WDTCR|=0x40; //WDT generates interrupts instead of resets now.
		 //We want interrupts because a reset clears our nice random
		 //seeds, and an interrupt doesn't.

    sei();
    //enable leds
    DDRB=(1<<LED2)|(1<<LED1);
    while(1) {
	//get a random value for the leds intensity
	lval1=GetRandom();
	lval2=GetRandom();
	//Manually do some pwm
	for (x=0; x<20; x++) {
	    PORTB|=(1<<LED1)|(1<<LED2);
	    for (y=0; y!=255; y++) {
		if (y==lval1) PORTB&=(1<<LED1);
		if (y==lval2) PORTB&=(1<<LED2);
		_delay_us(5);
	    }
	}
	WDTCR|=0x40; //make sure wdt keeps generating an int instead of a reset
    }
}


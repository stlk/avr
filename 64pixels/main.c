/* -----------------------------------------------------------------------
 * Title:    8x8 LED dot matrix animations
 * Author:   Alexander Weber alex@tinkerlog.com
 * Date:     21.12.2008
 * Hardware: ATtiny2313V
 * Software: AVRMacPack
 * 
 */

#define F_CPU 4000000
#include <inttypes.h>
#include <stdlib.h>
#include <string.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>
#include <util/delay.h>
#include <avr/pgmspace.h>
#include "font.h"

// Change these values to adjust scroll speeds and animation iterations
#define ANIMATION_SCROLL_SPEED 80  // how fast to scroll the animations
#define TEXT_SCROLL_SPEED 200      // how fast to scrill the text
#define REPEAT_ANIMATION 10        // how often to repeat the animation if in cycling mode
#define REPEAT_TEXT 5              // how often to repeat the text if in cycling mode

// How to add a new message:
// * add the new message (only upper case, see font.h)
// * adjust MAX_MESSAGES
// * add the new message to messages
// NOTE: messages may not be longer than 59 chars. Otherwise they will not fit in the buffer.
//                                              123456789012345678901234567890123456789012345678901234567890
const prog_char PROGMEM message_00[] PROGMEM = "   LIDUSCE Z LASKY ";
const prog_char PROGMEM message_01[] PROGMEM = "   VSECHNO NEJLEPSI ! ";
const prog_char PROGMEM message_02[] PROGMEM = "   LIDUSKAAA ";
const prog_char PROGMEM message_03[] PROGMEM = "   HTTP://ROUSEK.NAME ";

#define MAX_MESSAGES 4
PGM_P PROGMEM messages[] = {
  message_00
  ,message_01
  ,message_02
  ,message_03
}; 

#define MAX_ANIMATIONS 1

const prog_uint8_t PROGMEM sprite_01[8] =
  {
    0x00,    // ________ 
    0x00,    // ________
    0x14,    // ___X_X__
    0x3E,    // __XXXXX_
    0x1C,    // ___XXX__
    0x08,    // ____X___
    0x00,    // ________
    0x00     // ________
  };

const prog_uint8_t PROGMEM sprite_02[8] =
  {
    0x00,    // ________ 
    0x00,    // ________
    0x14,    // ___X_X__
    0x3E,    // __XXXXX_
    0x3E,    // __XXXXX_
    0x1C,    // ___XXX__
    0x08,    // ____X___
    0x00     // ________
  };

const prog_uint8_t PROGMEM sprite_03[8] =
  {
    0x00,    // ________ 
    0x66,    // _XX__XX_
    0xFF,    // XXXXXXXX
    0xFF,    // XXXXXXXX
    0xFF,    // XXXXXXXX
    0x7E,    // _XXXXXX_
    0x3C,    // __XXXX__
    0x18     // ___XX___
  };



uint8_t mode_ee EEMEM = 0;                      // stores the mode in eeprom
static uint8_t screen_mem[8];			// screen memory
static uint8_t active_row;			// active row
static uint8_t buffer[60];                      // stores the active message or sprite
static uint8_t message_ptr = 0;                 // points to the active char in the message
static uint8_t message_displayed = 0;           // how often has the message been displayed?
static uint8_t active_char = 0;                 // stores the active char
static uint8_t message_length = 0;              // stores the length of the active message
static uint8_t char_ptr = 0;                    // points to the active col in the char
static uint8_t char_length = 0;                 // stores the length of the active char
static volatile uint16_t counter = 0;           // used for delay function

// prototypes
void delay_ms(uint16_t delay);
void copy_to_display(int8_t x, int8_t y, uint8_t sprite[]);
void display_active_row(void);
void show_char();
void clear_screen(void);
void copy_to_buffer(const prog_uint8_t sprite[8]);
void scroll_animation(const prog_uint8_t sprite_1[], const prog_uint8_t sprite_2[]);



/*
 * ISR TIMER0_OVF_vect
 * Handles overflow interrupts of timer 0.
 *
 * 4MHz
 * ----
 * Prescaler 8 ==> 1953.1 Hz
 * Complete display = 244 Hz
 *
 */
ISR(TIMER0_OVF_vect) {	
  display_active_row();
  counter++;
}



/*
 * delay_ms
 * Uses the counter that is incremented by the ISR.
 * Max delay is 32767ms.
 */
void delay_ms(uint16_t delay) {
  while (!(PIND & (1 << PD6))) {}   // used to stop the animation when PD6 goes LOW
  uint16_t t = delay * 2;
  counter = 0;
  while (counter < t) {}
}



/*
 * copy_to_display
 * Copies sprite data to the screen memory at the given position. 
 */
void copy_to_display(int8_t x, int8_t y, uint8_t sprite[8]) {
  int8_t i, t;
  uint8_t row;
  for (i = 0; i < 8; i++) {
    t = i-y;
    row = ((t >= 0) && (t < 8)) ? sprite[t] : 0x00;
    row = (x >= 0) ? (row >> x) : (row << -x);
    screen_mem[i] = row;
  }
}



/*
 * display_active_col
 * Deactivates the active column and displays the next one.
 * Data is read from screen_mem.
 *
 *      ATtiny2313
 * 16 - PD0    PB7 - 1
 * 15 - PD1    PB6 - 2
 * 14 - PA1    PB5 - 3
 * 13 - PA0    PB4 - 4
 * 12 - PD2    PB3 - 5
 * 11 - PD3    PB2 - 6
 * 10 - PD4    PB1 - 7
 *  9 - PD5    PB0 - 8
 *
 * NFM-12883 common anode          |
 *     A0B5B4D4B2D3D1D0      +-----+
 * PD5 o o o o o o o o       |     |
 * PA1 o o o o o o o o      _+_    |
 * PB0 o o o o o o o o      \ /    |
 * PD2 o o o o o o o o     __V__   |
 * PB7 o o o o o o o o       |     |
 * PB1 o o o o o o o o    ---+-----C---
 * PB6 o o o o o o o o             |
 * PB3 o o o o o o o o
 
  *
 *      ATtiny2313
 *  9 - PD0    PB7 - 8
 * 10 - PD1    PB6 - 7
 * 11 - PA1    PB5 - 6
 * 12 - PA0    PB4 - 5
 * 13 - PD2    PB3 - 4
 * 14 - PD3    PB2 - 3
 * 15 - PD4    PB1 - 2
 * 16 - PD5    PB0 - 1
 *
 *  common anode          |
 *     1303041006111516      +-----+
 *   9 o o o o o o o o       |     |
 *  14 o o o o o o o o      _+_    |
 *   8 o o o o o o o o      \ /    |
 *  12 o o o o o o o o     __V__   |
 *   1 o o o o o o o o       |     |
 *   7 o o o o o o o o    ---+-----C---
 *   2 o o o o o o o o             |
 *   5 o o o o o o o o
 *
 */
 
void display_active_row(void) {

  uint8_t row;

  // shut down all rows and columns
  PORTA = (1 << PA0) | (0 << PA1);
  PORTB = (0 << PB5) | (1 << PB4) | (0 << PB2) | (1 << PB0) | 
          (1 << PB7) | (1 << PB1) | (1 << PB6) | (0 << PB3);
  PORTD = (0 << PD4) | (1 << PD3) | (0 << PD1) | (1 << PD0) |
          (0 << PD5) | (0 << PD2) | (1 << PD6);

  // next row
  active_row = (active_row+1) % 8;
  row = screen_mem[active_row];

  // output all columns, switch leds on.
  // column 1
  if ((row & 0x80) == 0x80) {
    PORTD |= (1 << PD2);    
  }
  // column 2
  if ((row & 0x40) == 0x40) {
    PORTB |= (1 << PB2);    
  }
  // column 3
  if ((row & 0x20) == 0x20) {
    PORTB |= (1 << PB3);    
  }
  // column 4
  if ((row & 0x10) == 0x10) {
    PORTD |= (1 << PD1);    
  }
  // column 5
  if ((row & 0x08) == 0x08) {
    PORTB |= (1 << PB5);    
  }
  // column 6
  if ((row & 0x04) == 0x04) {
    PORTA |= (1 << PA1);    
  }
  // column 7
  if ((row & 0x02) == 0x02) {
    PORTD |= (1 << PD4);    
  }
  // column 8
  if ((row & 0x01) == 0x01) {
    PORTD |= (1 << PD5);    
  }

  // activate row
  switch (active_row) {
  case 0:
    PORTD &= ~(1 << PD0);
    break;
  case 1:
    PORTD &= ~(1 << PD3);
    break;
  case 2:
    PORTB &= ~(1 << PB7);
    break;
  case 3:
    PORTA &= ~(1 << PA0);
    break;
  case 4:
    PORTB &= ~(1 << PB0);
    break;
  case 5:
    PORTB &= ~(1 << PB6);
    break;
  case 6:
    PORTB &= ~(1 << PB1);
    break;
  case 7:
    PORTB &= ~(1 << PB4);
    break;
  }

}

/*
 * show_char
 * Displays the actual message. 
 * Scrolls the screen to the left and draws new pixels on the right.
 */
void show_char() {
  uint8_t i;
  uint8_t b;

  // blit the screen to the left
  for (i = 0; i < 8; i++) {
    screen_mem[i] <<= 1; 
  }
  // advance a char if needed
  if (char_ptr == char_length) {
    message_ptr++;
    if (message_ptr == message_length) {
      message_ptr = 0;
      message_displayed++;
    }
    active_char = buffer[message_ptr] - CHAR_OFFSET;
    char_length = pgm_read_byte(&font[active_char * 4 + 3]);
    char_ptr = 0;
    return; // this makes the space between two chars
  }
  // read pixels for current column of char
  b = pgm_read_byte(&font[active_char * 4 + char_ptr++]);
  // write pixels into screen memory
  for (i = 0; i < 7; i++) {
    if ((b & (1 << i)) == (1 << i)) {
      screen_mem[i+1] |= 0x01;
    } 
  }
}



/*
 * clear_screen
 */
void clear_screen(void) {
  uint8_t i;
  for (i = 0; i < 8; i++) {
    screen_mem[i] = 0x00;
  }
}



/*
 * copy_to_buffer
 * Copies the given sprite from PROGMEM to RAM.
 */
void copy_to_buffer(const prog_uint8_t sprite[8]) {
  memcpy_P(buffer, sprite, 8);
}


int main(void) {

  uint8_t i = 0;
  uint8_t mode = 0;
  uint8_t cycle = 0;

  // timer 0 setup, prescaler 8
  TCCR0B |= (1 << CS01);
 
  // enable timer 0 interrupt
  TIMSK |= (1 << TOIE0);	

  // define outputs
  DDRA |= 0x03;  
  DDRB |= 0xFF;
  DDRD |= 0x3F;
  
  // shut down all rows and columns, enable column 1
  PORTA = (1 << PA0) | (1 << PA1);
  PORTB = (0 << PB5) | (0 << PB4) | (0 << PB2) | (1 << PB0) | 
          (1 << PB7) | (1 << PB1) | (1 << PB6) | (1 << PB3);
  PORTD = (0 << PD4) | (0 << PD3) | (0 << PD1) | (0 << PD0) |
          (1 << PD5) | (1 << PD2);

  // enable pull ups
  PORTD |= (1 << PD6);

  // say hello, toggle row 1 (pixel 0,0)
  for (i = 0; i < 5; i++) {
    PORTD &= ~(1 << PD5);
    _delay_ms(50);
    PORTD |= (1 << PD5);
    _delay_ms(50);
  }

  // read last mode from eeprom
  // 0 mean cycle through all modes and messages
  mode = eeprom_read_byte(&mode_ee);
  if ((mode == 0) || (mode >= (MAX_ANIMATIONS + MAX_MESSAGES + 1))) {
    mode = 1;
    cycle = 1;
  }
  eeprom_write_byte(&mode_ee, mode + 1);  

  sei();

  while (1) {

    switch (mode) {
    case 1:
      for (i = 0; i < REPEAT_ANIMATION; i++) {
        copy_to_buffer(sprite_01);
        copy_to_display(0, 0, buffer);
        delay_ms(180);
        copy_to_buffer(sprite_02);
        copy_to_display(0, 0, buffer);
        delay_ms(180);
        copy_to_buffer(sprite_03);
        copy_to_display(0, 0, buffer);
        delay_ms(750);
      }
      break;
    default:
      strcpy_P(buffer, (uint8_t*)pgm_read_word(&(messages[mode - (MAX_ANIMATIONS+1)])));
      message_length = strlen(buffer);
      while (message_displayed < REPEAT_TEXT) {
        show_char();
        delay_ms(TEXT_SCROLL_SPEED);
      }
      message_displayed = 0;
      break;
    }

    // cycle through all modes
    if (cycle) {
      mode++;
      clear_screen();
      if (mode >= (MAX_ANIMATIONS + MAX_MESSAGES + 1)) {
        mode = 1;
      }
    }

  }

  return 0;

}


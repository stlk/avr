#include <avr/io.h>
#define F_CPU 8000000UL
#include <util/delay.h>
#include <avr/interrupt.h>
#include <avr/wdt.h>
#include <avr/sleep.h>

#define REMOTE_TEMP		 30 // packet id
#define REMOTE_VOLTAGE	 31 // packet id
#define PAYLOAD_LENGTH   21
#define AT_CMD_PAYLOAD_LENGTH   8

#define SLEEP            PB0
#define SENSOR           0x20 // pin9 - sensor

#define TX(pin)          DDRD |= pin;
#define RX(pin)          DDRD &= ~pin;
#define RXPIN(pin)       PIND&pin
#define BAUD_RATE_4800   0x0067


#define output_low(port, pin) port &= ~(1<<pin) 
#define output_high(port, pin) port |= (1<<pin) 

uint8_t payload[PAYLOAD_LENGTH] = { 0x7E,
0x00, // size MSB
0x11, // size LSB
0x10, // frame type
0x01, // id
0x00, // 64bit address - controller
0x00,
0x00,
0x00,
0x00,
0x00,
0x00,
0x00,
0xFF, // destination MSB
0xFE, // destination LSB - uknown 16bit address
0x00, // broadcast radius
0x00, // options
REMOTE_TEMP, // data
0x00,
0x00,
0x00 // checksum
};

uint8_t at_cmd_payload[AT_CMD_PAYLOAD_LENGTH] = { 0x7E,
0x00, // size MSB
0x04, // size LSB
0x08, // frame type
0x01, // id
0x25, // %
0x56, // V
0x7B // checksum
};

// resets and tests ds-18b20 presence on the bus 
uint8_t ow_detect_presence(uint8_t pin) {
	uint8_t out = 1;
	RX(pin);                          // initial state
	_delay_us(1000);                  // give it time to stabilize
	TX(pin);                          // bus low
	_delay_us(480);                   // reset
	RX(pin);                          // release bus
	_delay_us(70);                    // wait for confirmation
	if (RXPIN(pin)) out = 0;             // sensor should pull bus down
	_delay_us(410);                   // delay before following communication
	return out;                       // 1=sensor found, 0= sensor not found
}

// sends log.1
void ow_write_one(uint8_t pin) {
	TX(pin);                          // bus low
	_delay_us(6);                     // pause defining log.1
	RX(pin);                          // release bus
	_delay_us(64);                    // delay before following communication
}

// sends log.0
void ow_write_zero(uint8_t pin) {
	TX(pin);                          // bus low
	_delay_us(60);                    // pause defining log.0
	RX(pin);                          // release bus
	_delay_us(10);                    // delay before following communication
}

// reads one bit
uint8_t ow_read_bit(uint8_t pin) {
	uint8_t out = 0;
	TX(pin);                          // bus low
	_delay_us(6);                     // pause idicating we want to read bit
	RX(pin);                          // release bus
	_delay_us(9);                     // pause before reading
	if (RXPIN(pin)) out = 1;             // read bit
	_delay_us(55);                    // delay before following communication
	return out;
}

// LSB gets sent first
void ow_write_byte(uint8_t tosend, uint8_t pin) {
	int n = 8;
	while (n--) {
		if (tosend & 1) ow_write_one(pin); else ow_write_zero(pin);
		tosend >>= 1;
	}
}

// LSB is received first
uint8_t ow_read_byte(uint8_t pin) {
	int n = 8, out = 0;
	while (n--) {
		out >>= 1;                          // shift to right
		if (ow_read_bit(pin)) out |= 0x80;   // set highest bit to 1
	}
	return out;
}

uint8_t crc8(uint8_t *addr, uint8_t len) {
	uint8_t crc = 0;

	while (len--) {
		uint8_t inbyte = *addr++;
		for (uint8_t i = 8; i; i--) {
			uint8_t mix = (crc ^ inbyte) & 0x01;
			crc >>= 1;
			if (mix) crc ^= 0x8C;
			inbyte >>= 1;
		}
	}
	return crc;
}

uint8_t read_temperature(uint8_t pin, uint8_t* data) {
	uint8_t out = 1;
	if (!ow_detect_presence(pin)) out = 0;
	ow_write_byte(0x0CC, pin); // Skip ROM
	ow_write_byte(0x44, pin); // Start measuring temperature
	_delay_ms(750);
	if (!ow_detect_presence(pin)) out = 0;
	ow_write_byte(0x0CC, pin); // Skip ROM
	ow_write_byte(0x0BE, pin); // Read scratchpad

	uint8_t scratchpad[8];
	for (int i = 0; i < 8; i++)
	{
		uint8_t b = ow_read_byte(pin);
		if (i < 2)
		{
			// 1st byte of scratchpad = LSB of temperature
			// 2nd byte of scratchpad = MSB of temperature
			data[i] = b;
		}
		scratchpad[i] = b;
	}

	if (ow_read_byte(pin) != crc8(scratchpad, 8)) //read 9th byte of scratchpad and compare it to calculated CRC
	{
		out = 0;
	}

	return out;
}

void init_uart(uint8_t baudrate) {
	/* Set the baud rate */
	UBRRL = baudrate;

	/* Enable UART transmitter */
	UCSRB = (1 << TXEN);

	/* set to 8 data bits, 1 stop bit */
	UCSRC = (1 << UCSZ1) | (1 << UCSZ0);

}

uint8_t receive_byte(void) {
	while (!(UCSRA & (1 << RXC))); // Wait for incomming data 
	return UDR;
}

void transmit_byte(uint8_t data) {
	/* Wait for empty transmit buffer */
	while (!(UCSRA & (1 << UDRE)));

	/* Start transmittion */
	UDR = data;
}

void calculate_checksum(uint8_t* payload, uint8_t length) {
	uint8_t checksum = 0xFF;
	for (uint8_t i = 3; i < length - 1; i++)
	{
		checksum -= payload[i];
	}
	payload[length - 1] = checksum;
}

void send_tx_request(uint8_t type, uint8_t* data) {
	payload[PAYLOAD_LENGTH - 4] = type;
	payload[PAYLOAD_LENGTH - 3] = data[0];
	payload[PAYLOAD_LENGTH - 2] = data[1];

	calculate_checksum(payload, PAYLOAD_LENGTH);
	for (uint8_t i = 0; i < PAYLOAD_LENGTH; i++){
		transmit_byte(payload[i]);
	}
}

void read_and_send_temperature(void) {
	uint8_t data[2];
	if (read_temperature(SENSOR, data))
	{
		send_tx_request(REMOTE_TEMP, data);
	}
}

void enable_xbee(void) {
	DDRB |= (1 << SLEEP); // Set direction register output
	output_low(PORTB, SLEEP);
}

void disable_xbee(void) {
	_delay_ms(100);
	DDRB &= ~_BV(SLEEP);
	output_high(PORTB, SLEEP);
}

void read_voltage(void) {
	for (uint8_t i = 0; i < AT_CMD_PAYLOAD_LENGTH; i++){
		transmit_byte(at_cmd_payload[i]);
	}

	UCSRB |= (1 << RXEN); // enable UART receiver

	uint8_t data[2];
	data[0] = 0;
	data[1] = 0;

	for(uint8_t i = 0; i < 10; i++) {
		uint8_t received_byte = receive_byte();
		if(i == 0 && received_byte != 0x7E)
			break;
		if(i == 3 && received_byte != 0x88)
			break;

		if(i == 8) data[0] = received_byte;
		if(i == 9) data[1] = received_byte;
	}

	UCSRB &= ~(1 << RXEN); // disable UART receiver

	if(data[1] != 0) send_tx_request(REMOTE_VOLTAGE, data);
}

volatile int timer_overflow_count = 0;

int main(void) {

	if (MCUSR & _BV(WDRF)){            // If a reset was caused by the Watchdog Timer...
		MCUSR &= ~_BV(WDRF);                 // Clear the WDT reset flag
		WDTCSR |= (_BV(WDCE) | _BV(WDE));   // Enable the WD Change Bit
		WDTCSR = 0x00;                      // Disable the WDT
	}

	init_uart(BAUD_RATE_4800);

	WDTCSR |= (_BV(WDCE) | _BV(WDE));   // Enable the WD Change Bit
	WDTCSR = _BV(WDIE) |              // Enable WDT Interrupt
		_BV(WDP3) | _BV(WDP0);   // Set Timeout to 8 seconds

	// Enable Sleep Mode for Power Down
	set_sleep_mode(SLEEP_MODE_PWR_DOWN);// Set Sleep Mode: Power Down
	sleep_enable();                     // Enable Sleep Mode  
	sei();                              // Enable Interrupts 

	/****************************
	*  Enter Main Program Loop  *
	****************************/

	for (;;)
	{
		if (timer_overflow_count > 3) {
			timer_overflow_count = 0;
			enable_xbee();

			read_and_send_temperature();

			read_voltage();

			disable_xbee();
		}
		if (MCUCR & _BV(SE)){    // If Sleep is Enabled...
			sleep_mode();        // Go to Sleep

			/****************************
			*   Sleep Until WDT Times Out
			*   -> Go to WDT ISR
			****************************/

		}
	}

	return 0;
}

ISR(WDT_OVERFLOW_vect) {
	sleep_disable();          // Disable Sleep on Wakeup
	timer_overflow_count++;
	sleep_enable();           // Enable Sleep Mode
}
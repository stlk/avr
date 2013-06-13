#include <avr/io.h>
#define F_CPU 16000000UL
#include <util/delay.h>

#define CT				 0x74 // ascii value for T
#define C1				 0x31 // ascii value for 1
#define C2				 0x32 // ascii value for 2

#define PD6				 0x40 // pin11 - sensor2, outside
#define PD5				 0x20 // pin9 - sensor1, inside
#define TX(pin)          DDRD |= pin;
#define RX(pin)          DDRD &= ~pin;
#define RXPIN(pin)       PIND&pin


// resets and tests ds-18b20 presence on the bus 
unsigned char ow_detect_presence(unsigned char pin) {
	unsigned char out=1;
	RX(pin);                          // initial state
	_delay_us(1000);                  // give it time to stabilize
	TX(pin);                          // bus low
	_delay_us(480);                   // reset
	RX(pin);                          // release bus
	_delay_us(70);                    // wait for confirmation
	if(RXPIN(pin)) out=0;             // sensor should pull bus down
	_delay_us(410);                   // delay before following communication
	return out;                       // 1=sensor found, 0= sensor not found
}

// sends log.1
void ow_write_one(unsigned char pin) {
	TX(pin);                          // bus low
	_delay_us(6);                     // pause defining log.1
	RX(pin);                          // release bus
	_delay_us(64);                    // delay before following communication
}

// sends log.0
void ow_write_zero(unsigned char pin) {
	TX(pin);                          // bus low
	_delay_us(60);                    // pause defining log.0
	RX(pin);                          // release bus
	_delay_us(10);                    // delay before following communication
}

// reads one bit
unsigned char ow_read_bit(unsigned char pin) {
	unsigned char out=0;
	TX(pin);                          // bus low
	_delay_us(6);                     // pause idicating we want to read bit
	RX(pin);                          // release bus
	_delay_us(9);                     // pause before reading
	if(RXPIN(pin)) out=1;             // read bit
	_delay_us(55);                    // delay before following communication
	return out;
}

// LSB gets sent first
void ow_write_byte(unsigned char tosend, unsigned char pin) {
	int n=8;
	while(n--) {
		if(tosend&1) ow_write_one(pin); else ow_write_zero(pin);
		tosend >>= 1;
	}
}

// LSB is received first
unsigned char ow_read_byte(unsigned char pin) {
	int n=8, out=0;
	while(n--) {
		out >>= 1;                          // shift to right
		if(ow_read_bit(pin)) out |= 0x80;   // set highest bit to 1
	}
	return out;
}

unsigned char crc8(unsigned char *addr, unsigned char len)
{
	unsigned char crc = 0;

	while (len--) {
		unsigned char inbyte = *addr++;
		for (unsigned char i = 8; i; i--) {
			unsigned char mix = (crc ^ inbyte) & 0x01;
			crc >>= 1;
			if (mix) crc ^= 0x8C;
			inbyte >>= 1;
		}
	}
	return crc;
}

unsigned char read_temperature(unsigned char pin, unsigned char* data)
{
	unsigned char out=1;
	if(!ow_detect_presence(pin)) out=0;
	ow_write_byte(0x0CC, pin); // Skip ROM
	ow_write_byte(0x44, pin); // Start measuring temperature
	_delay_ms(300);
	if(!ow_detect_presence(pin)) out=0;
	ow_write_byte(0x0CC, pin); // Skip ROM
	ow_write_byte(0x0BE, pin); // Read scratchpad

	unsigned char scratchpad[8];
	for(int i = 0; i < 8;i++)
	{
		unsigned char b = ow_read_byte(pin);
		if(i < 2)
		{
			// 1st byte of scratchpad = LSB of temperature
			// 2nd byte of scratchpad = MSB of temperature
			data[i] = b;
		}
		scratchpad[i] = b;
	}

	if(ow_read_byte(pin) != crc8(scratchpad, 8)) //read 9th byte of scratchpad and compare it to calculated CRC
	{
		out=0;
	}

	return out;
}

void init_uart(unsigned char baudrate)
{
	/* Set the baud rate */
	UBRRL = baudrate;

	/* Enable UART receiver and transmitter */
	UCSRB = (1 << RXEN) | (1 << TXEN); 

	/* set to 8 data bits, 1 stop bit */
	UCSRC = (1 << UCSZ1) | (1 << UCSZ0);

}

unsigned char receive_byte(void) 
{ 
	unsigned char data; 

	while(!(UCSRA & (1<<RXC))); // Wait for incomming data 

	data = UDR; 

	return(data); 
} 

void transmit_byte(unsigned char data)
{
	/* Wait for empty transmit buffer */
	while (!(UCSRA & (1 << UDRE)));

	/* Start transmittion */
	UDR = data;
}

int main(void) {
	unsigned char data[2];
	init_uart(0x0067); // Initialize UART with baud rate 4800

	while(1) {
		unsigned char commandChar = receive_byte();

		if(commandChar == CT)
		{
			commandChar = receive_byte();
			if(commandChar == C1)
			{
				if(read_temperature(PD5, data))
				{
					transmit_byte(CT);
					transmit_byte(C1);
					transmit_byte(data[0]);
					transmit_byte(data[1]);
				}
			}
			else if(commandChar == C2)
			{
				if(read_temperature(PD6, data))
				{
					transmit_byte(CT);
					transmit_byte(C2);
					transmit_byte(data[0]);
					transmit_byte(data[1]);
				}
			}
		}
	}

	return 0;
}
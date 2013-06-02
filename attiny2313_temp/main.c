#include <avr/io.h>
#define F_CPU 16000000UL
#include <util/delay.h>
#include <avr/interrupt.h> 

#define CT				 0x74
#define CP				 0x70
#define C1				 0x31 // in
#define C2				 0x32 // out

#define PD6				 0x40 // pin11 - C2
#define PD5				 0x20 // pin9 - C1
#define TX(pin)          DDRD |= pin;
#define RX(pin)          DDRD &= ~pin;
#define RXPIN(pin)       PIND&pin


// provede reset a test prezence ds-18b20 na sbernici 
unsigned char ow_detect_presence(unsigned char pin) {
	unsigned char out=1;              // vychozi navratova hodnota
	RX(pin);                          // vychozi stav sbernice
	_delay_us(1000);                  // pro ustaleni
	TX(pin);                          // bus low
	_delay_us(480);                   // cas pro prikaz reset
	RX(pin);                          // uvolneni sbernice
	_delay_us(70);                    // cekani na potvrzeni teplomerem
	if(RXPIN(pin)) out=0;             // pokud detekovana log.1, tak teplomer na sbernici neni
	_delay_us(410);                   // pauza pred dalsi komunikaci
	return out;                       // vrati stav 1=teplomer nalezen,  0=teplomer nenalezen
}

// posle na sbernici log.1
void ow_write_one(unsigned char pin) {
	TX(pin);                               // bus low
	_delay_us(6);                     // pauza definujici log.1
	RX(pin);                               // uvolneni sbernice
	_delay_us(64);                    // pauza pred dalsi komunikaci 
}

// posle na sbernici log.0
void ow_write_zero(unsigned char pin) {
	TX(pin);                               // bus low
	_delay_us(60);                    // pauza definujici log.0
	RX(pin);                               // uvolneni sbernice
	_delay_us(10);                    // pauza pred dalsi komunikaci 
}

// precte jeden bit ze sbernice
unsigned char ow_read_bit(unsigned char pin) {
	unsigned char out=0;              // vychozi navratova hodnota bitu
	TX(pin);                               // bus low
	_delay_us(6);                     // pauza pro stav cteni
	RX(pin);                               // uvolneni sbernice
	_delay_us(9);                     // pauza pro reakci teplomeru
	if(RXPIN(pin)) out=1;                  // test stavu sbernice, vlastni cteni
	_delay_us(55);                    // pauza pred dalsi komunikaci  
	return out;                       // prectena hodnota, 1 nebo 0
}

// odeslne na sbernici jeden byte. Odesila se prvni LSB
void ow_write_byte(unsigned char tosend, unsigned char pin) {
	int n=8;
	while(n--) {
		if(tosend&1) ow_write_one(pin); else ow_write_zero(pin);
		tosend >>= 1;
	}
}

// prijde ze sbernice jeden byte. Prijima jako prvni LSB.
unsigned char ow_read_byte(unsigned char pin) {
	int n=8, out=0;
	while(n--) {
		out >>= 1;                       // bitovy posuv doprava
		if(ow_read_bit(pin)) out |= 0x80;   // nastaveni nejvyssiho bitu na 1
	}
	return out;
}

// nacte teplotu z teplomeru a vrati ji ve formatu 1000+t*10
// priklad: 23.5°C = 1235,  -10.5°C = 895
// tento format lze snadneji zpracovavat nez nejake floaty (zerou moc pameti)
unsigned char zmer(unsigned char pin, unsigned char* data) {
	unsigned char out=1;
	if(!ow_detect_presence(pin)) out=0;
	ow_write_byte(0x0CC, pin);
	ow_write_byte(0x44, pin);
	_delay_ms(300);
	if(!ow_detect_presence(pin)) out=0;
	ow_write_byte(0x0CC, pin);
	ow_write_byte(0x0BE, pin);
	
	/*usingned char shift_register = 0;
	for(int i = 0; i < 8;i++)
	{
		unsigned char b = ow_read_byte(pin);
		if(i < 2)
		{
			data[i] = b;
		}
		shift_register = calc_CRC_bit(shift_register, b);
	}*/
	
	
	
	data[0] = ow_read_byte(pin);     // 1. byte scratchpadu teplomeru = spodni byte teploty
	data[1] = ow_read_byte(pin);     // 2. byte scratchpadu teplomeru = horni byte teploty
	//teplota = (data_lo & 0x0F0) >> 4 | (data_hi & 0x0F) << 4 ;   // signed teplota
	//tmp = 10 * teplota + 1000;
	//desetiny = (data_lo & 0x0F) * 0.625;
	//if(tmp<1000) tmp -= desetiny; else tmp += desetiny;
	//if((tmp<700)||(tmp>2200)) tmp=0;

	return out;
}

void InitUART(unsigned char baudrate)
{
	/* Set the baud rate */
	UBRRL = baudrate;

	/* Enable UART receiver and transmitter */
	UCSRB = (1 << RXEN) | (1 << TXEN); 

	/* set to 8 data bits, 1 stop bit */
	UCSRC = (1 << UCSZ1) | (1 << UCSZ0);

}

unsigned char receiveByte(void) 
{ 
	unsigned char data; 

	while(!(UCSRA & (1<<RXC))); // Wait for incomming data 

	data = UDR; 

	return(data); 
} 

void TransmitByte(unsigned char data)
{
	/* Wait for empty transmit buffer */
	while (!(UCSRA & (1 << UDRE)));

	/* Start transmittion */
	UDR = data;
}

int main(void) {

	unsigned char data[2];
	InitUART(0x0067);
	DDRB |= 1<<PB1; /* set PB0 to output */
	
	GIMSK = _BV (INT0); // int - Enable external interrupts int0 
    MCUCR = _BV (ISC01); // int - INT0 is falling edge 
	
	TIMSK |= (1 << TOIE1); // Enable overflow interrupt 
	//TCCR1B |= (1 << CS11); // Start timer at Fcpu/8

    sei(); // int - Global enable interrupts 

	
	//Enable internal pull-up resistor
	PORTD |= (1 << PD2);


	while(1) {
		//PORTB |= 1<<PB1;
		//_delay_ms(200);
		//PORTB &= ~(1<<PB1);

		unsigned char commandChar = receiveByte();

		if(commandChar == CT)
		{
			commandChar = receiveByte();
			if(commandChar == C1)
			{
				if(zmer(PD5, data))
				{
					TransmitByte(CT);
					TransmitByte(C1);
					TransmitByte(data[0]);
					TransmitByte(data[1]);
				}
			}
			else if(commandChar == C2)
			{
				if(zmer(PD6, data))
				{
					TransmitByte(CT);
					TransmitByte(C2);
					TransmitByte(data[0]);
					TransmitByte(data[1]);
				}
			}
		}
	}
	
	return 0;
}

void DisablePirInterrupt(void) {
	GIMSK = 0;
	MCUCR &= ~(1<<ISC01);
}

void EnablePirInterrupt(void) {
	GIMSK = _BV (INT0); // int - Enable external interrupts int0
    MCUCR = _BV (ISC01); // int - INT0 is falling edge 
}

 ISR (INT0_vect) // Interrupt on Int0 vector 
 {    
	DisablePirInterrupt();
	TransmitByte(CP);

	TCCR1B |= (1<<CS12) | (1<<CS10);
 } 
 

volatile int timer_overflow_count = 0;

 ISR(TIMER1_OVF_vect) {
	   if (++timer_overflow_count > 5) {   // a timer overflow occurs 4.6 times per second

		TCCR1B = 0;
		EnablePirInterrupt();

		timer_overflow_count = 0;
   }
 }
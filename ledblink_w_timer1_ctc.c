#define F_CPU 8000000UL

#include<avr/io.h>
#include<avr/interrupt.h>

int Position = 0;

#define OUTPUT_VAL 0xFF
#define TIMER_CNT 31250 

ISR (TIMER1_COMPA_vect)
{
		PORTC = (0x01 << (Position++));
		if(Position == 8)
		{
			Position = 0;
		}
		
}


int main(void)
{
	DDRC = OUTPUT_VAL;
	TCNT1 = 0;
	TCCR1B |= (1<<WGM12)|(1<<CS12);
	OCR1A = 31250;
	OCR1B = 0;
	TIMSK |= (1<<OCIE1A);
	sei();
	while(1){}
}

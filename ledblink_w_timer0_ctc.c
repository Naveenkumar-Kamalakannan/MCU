#define F_CPU 8000000UL

#include<avr/io.h>
#include<avr/interrupt.h>

int Position = 0;
volatile int count;

#define OUTPUT_VAL 0xFF
#define CTC_w_PRESCL_1024 0x0D 

ISR (TIMER0_COMP_vect)
{
	if (count == 33)
	{	
		PORTC = (0x01 << (Position++));
		count = 0;
		if(Position == 8)
		{
			Position = 0;
		}
		
	}
	else
		count++;
}


int main(void)
{
	DDRC = OUTPUT_VAL;
	count = 0;
	TCNT0 = 0;
	TCCR0 = CTC_w_PRESCL_1024;
	OCR0 = 240;
	TIMSK |= (1<<OCIE0);
	sei();
	while(1){}
}

#define F_CPU 8000000UL

#include <avr/io.h>
#include <avr/interrupt.h>

int Position = 0;
int count;

#define OUTPUT_VAL 0xFF
#define PRESCL_VAL_1024 0x05

int main(void)
{
	DDRC = OUTPUT_VAL; 
	TCNT0 = 0;
	count = 0;
	TCCR0 = PRESCL_VAL_1024; // PRESCALER 1024
	TIMSK = (1<<TOIE0);
	sei();
	while(1){}
}

ISR (TIMER0_OVF_vect)
{
	if (count == 30)
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

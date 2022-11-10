#define F_CPU 8000000UL

#include <avr/io.h>
#include <avr/interrupt.h>

int Position = 0;

#define OUTPUT_VAL 0xFF
#define PRESCL_VAL_1024 0x05
#define TIMER_CNT 34285
int main(void)
{
	DDRC = OUTPUT_VAL; 
	TCNT1 = TIMER_CNT;
	TCCR1B |= (1<<CS12); // PRESCALER 256
	TIMSK = (1<<TOIE1);
	sei();
	while(1){}
}

ISR (TIMER1_OVF_vect)
{
		PORTC = (0x01 << (Position++));
		TCNT1 = TIMER_CNT;
		if(Position == 8)
		{
			Position = 0;
		}
}

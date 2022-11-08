#define F_CPU 8000000UL

#include<avr/io.h>
#include<util/delay.h>
#include<stdio.h>
#define LED_COUNT 8
			    
void my_delay()
{
	int count = 100;
	while(count --)
	{
		TCNT0 = 0xB2;
		TCCR0 = 0x05;   // this has the prescale value of 1024.
		while((TIFR & 0x01) == 0); 
		TCCR0 = 0x00; 
		TIFR = 0x01;
	}
}

int main(void)
{
	DDRC = 0xff;
	int Position = 0;
	printf("Give the delay timing you need: ");
	while(1)
	{
	
		PORTC = (0x01 << (Position++));
		my_delay();
		PORTC = 0x00;
		my_delay();
		if(Position == LED_COUNT)
		{
			Position = 0;
		}
	}
}

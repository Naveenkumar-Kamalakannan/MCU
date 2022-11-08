/****************** This Program is designed to make all the 8 LED's blink at a time with a time period of 1 second ******************/

#define F_CPU 8000000UL 
#include<avr/io.h>
#include <util/delay.h> 

int main(void)
{
	DDRC = 0xff;
	while(1)
	{
		PORTC = 0xff;
		_delay_ms(1000);
		PORTC  = 0x00;
		_delay_ms(1000);
	}
}

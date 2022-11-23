/*This is the Program for interfacing the "7 segment LED".
 */

#define F_CPU 8000000UL

#include<avr/io.h>
#include<util/delay.h>

int main(void)
{
	DDRB = 0xFF;
	DDRA = 0xFF;
	PORTA = 0xFF;
	unsigned char n[] = {0x0C, 0xB6, 0x9E, 0xCC, 0xDA, 0xFA, 0x0E, 0xFE, 0xDE, 0x7E, 0xEE, 0xFE, 0x72, 0x7E, 0xF2, 0xE2};
	int k;
	while(1)
	{	
		for(k = 0 ; k < 16 ; k++)
		{	
			PORTB = n[k];
			_delay_ms(1000);
		}
	}
				

}

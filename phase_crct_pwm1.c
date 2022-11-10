#define F_CPU 8000000UL

#include<avr/io.h>
#include<avr/interrupt.h>

#define D_Cycle_A  0x9C
#define D_Cycle_B  0x7C
#define PCP_w_1024 0xA3 //For inverting change this value to 0xF3
#define OUTPUT_VAL 0xFF

int main(void) 
{
	DDRD = 0xFF;
	TCCR1A = PCP_w_1024;//(1 << COM1B1)|(1 << WGM10); // 1024 prescaler with fast pwm mode.
	TCCR1B = (1 << WGM13)|(1 << CS12)|(1 << CS10);
	OCR1A = D_Cycle_A; // duty cycle will be 80 and the frequency will be 25.04
	OCR1B = D_Cycle_B;
}





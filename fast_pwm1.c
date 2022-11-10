#define F_CPU 8000000UL

#include<avr/io.h>
#include<avr/interrupt.h>

#define D_Cycle_50 32767
#define TOP 65535
#define FP_w_8 0xA3  //For inverting change this value to 0x73
#define OUTPUT_VAL 0xFF

int main(void) 
{
	DDRD = OUTPUT_VAL;
	TCCR1A = FP_w_8; //8 prescaler with fast pwm mode and frequency is 15.25.
	TCCR1B = (1 << WGM13)|(1 << WGM12)|(1 << CS11);
	OCR1A = TOP; 
	OCR1B = D_Cycle_50;
}



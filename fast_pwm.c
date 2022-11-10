#define F_CPU 8000000UL

#include<avr/io.h>
#include<avr/interrupt.h>

#define D_Cycle_75 191
#define FP_w_1024 0x6D  //For inverting change this value to 0x7D
#define OUTPUT_VAL 0xFF

int main(void) 
{
	DDRB = OUTPUT_VAL;
	TCCR0 |= FP_w_1024; //1024 prescaler with fast pwm mode.
	OCR0 = D_Cycle_75;
}



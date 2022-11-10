#define F_CPU 8000000UL

#include<avr/io.h>
#include<avr/interrupt.h>

#define D_Cycle_60 153
#define PCP_w_1024 0x65   //For inverting change this value to 0x75.
#define OUTPUT_VAL 0xFF

int main(void) 
{
	DDRB = OUTPUT_VAL;
	TCCR0 |= PCP_w_1024; // 1024 prescaler with fast pwm mode.
	OCR0 = D_Cycle_60; // duty cycle will be 60 and the frequency will be 15.31
}




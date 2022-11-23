/* This Program is for interfacing the Matrix Keypad of Atmega32 then identify and display the equivalent code of the pressed key in the LCD
 * and in the 7 segment led.
 */

//telling controller crystal frequency attached.
#define F_CPU 8000000UL

//header to enable data flow control over pins.
#include<avr/io.h>

//header to enable delay function.
#include<util/delay.h>

#define BIT_HIGH                0xFF
#define BIT_LOW                 0x00
#define TWO_LINE_8_BIT          0x38      
#define DISPLAY_ON_CURSOR_BLINK 0x0C
#define CLR_DPLAY_SCR           0x01
#define INCR_CURSOR             0x06
#define CURSOR_FIRST_LINE       0x80
#define CURSOR_SECOND_LINE      0xC0
#define LCD                     PORTB
#define EN                      6
#define RW                      5
#define RS                      4
#define COMMAND_REG             ~(1<<RS)
#define DATA_REG                (1<<RS)
#define WRITE_REG               ~(1<<RW)
#define ENABLE                  (1<<EN)
#define NOT_ENABLE              ~(1<<EN)        
#define MAX_SEC                 8000              
#define LINE_END                0x10

void delay(int);
void my_delay(int);
void send_cmd(unsigned char);
void send_data(unsigned char);
void display(char *);
void LCD_INIT(void);
unsigned char keypad(void);

void delay(int time)
{
        uint16_t top = time/0.128;
        TCNT1 = 0x00;
        TCCR1B |= 0x05;
        OCR1A = top;

        while((TIFR & 0x10) == 0);

        TCNT1 |= 0x00;
        TCCR1B |= 0x00;
        TIFR |= 0x10;
}

void my_delay(int time)
{
        if(time > MAX_SEC)
        {
                delay(MAX_SEC);
                time = time - MAX_SEC;
        }
        delay(time);
}

void send_cmd(unsigned char CMD)
{

        PORTD &= COMMAND_REG;                           // making RS as 0 PD4 = 0.
        PORTD &= WRITE_REG;                             // making RW as 0 PD5 = 0.
        LCD = CMD;
        PORTD |= ENABLE;                                // EN = 1
        my_delay(1);                                    // 1ms delay for the enable to happen.
        PORTD &= NOT_ENABLE;                            // EN = 0
}

void send_data(unsigned char DATA)
{

        PORTD |= DATA_REG;                              // making RS as 1 for data PD4 = 1.
        PORTD &= WRITE_REG;                             // RW = 0 for write.
        LCD = DATA;
        PORTD |= ENABLE;
        my_delay(5);
        PORTD &= NOT_ENABLE;                            // EN = 0
}

void display(char *str)
{
        int level = 0;
        unsigned char i = 0;
        while(str[i]!=0)
        {
                send_data(str[i]);
                i++;
                if(level  == LINE_END)
                {
                        send_cmd(CURSOR_SECOND_LINE);
                }
                level++;
        }

}

void LCD_INIT()
{

        DDRB = BIT_HIGH;
        DDRD = BIT_HIGH;
        PORTD &= ~(1<<EN);
	
        send_cmd(TWO_LINE_8_BIT);
        send_cmd(DISPLAY_ON_CURSOR_BLINK);
        send_cmd(CLR_DPLAY_SCR);
        send_cmd(INCR_CURSOR);
        send_cmd(CURSOR_FIRST_LINE);
}


unsigned char keypad()
{	
	//Making First Row as 0 and remaining rows(PC6, PC7) and columns(PA4, PA5, PA6, PA7) as 1.
	PORTC = 0xC0;				// 11000000 
	PORTA = 0xF0;				// 11110000
	
	if((PINA & (1<<4)) == 0) 		// Consider PINA is 11100000 and doing '&' with 00010000 will make this as 00000000.
	{			 		// else if  PINA is 11110000 and doing '&' with 00010000 will make this as 00010000.
		DDRA = 0xFF;
		PORTA = 0xFF;
		PORTB = 0x0C;
		_delay_ms(500);
		DDRA = 0x00;
		PORTA = 0xF0;
		return '1';
	}
	else if ((PINA & (1<<5)) == 0)
	{
		DDRA = 0xFF;
                PORTA = 0xFF;
                PORTB = 0xB6;
                _delay_ms(500);
                DDRA = 0x00;
                PORTA = 0xF0;		
                return '2';
	}
	else if ((PINA & (1<<6)) == 0)
        {
		DDRA = 0xFF;
                PORTA = 0xFF;
                PORTB = 0x9E;
                _delay_ms(500);
                DDRA = 0x00;
                PORTA = 0xF0;
                return '3';
        }
	else if ((PINA & (1<<7)) == 0)
        {
                DDRA = 0xFF;
                PORTA = 0xFF;
                PORTB = 0xCC;
                _delay_ms(500);
                DDRA = 0x00;
                PORTA = 0xF0;
		return '4';        
	}

	PORTC = 0xA0;                           // 11000000 
        PORTA = 0xF0;                           // 11110000

        if((PINA & (1<<4)) == 0)                // Consider PINA is 11100000 and doing '&' with 00010000 will make this as 00000000.
        {                                       // else if  PINA is 11110000 and doing '&' with 00010000 will make this as 00010000.
 		DDRA = 0xFF;
                PORTA = 0xFF;
                PORTB = 0xDA;
                _delay_ms(500);
                DDRA = 0x00;
                PORTA = 0xF0;
                return '5';
        }
        else if ((PINA & (1<<5)) == 0)
        {
		DDRA = 0xFF;
                PORTA = 0xFF;
                PORTB = 0xFA;
                _delay_ms(500);
                DDRA = 0x00;
                PORTA = 0xF0;
                return '6';
        }
        else if ((PINA & (1<<6)) == 0)
        {
		DDRA = 0xFF;
                PORTA = 0xFF;
                PORTB = 0x0E;
                _delay_ms(500);
                DDRA = 0x00;
                PORTA = 0xF0;
                return '7';
        }
        else if ((PINA & (1<<7)) == 0)
        {
                DDRA = 0xFF;
                PORTA = 0xFF;
                PORTB = 0xFE;
                _delay_ms(500);
                DDRA = 0x00;
                PORTA = 0xF0;
                return '8';
        }

	PORTC = 0x60;                           // 11000000
        PORTA = 0xF0;                           // 11110000

        if((PINA & (1<<4)) == 0)                // Consider PINA is 11100000 and doing '&' with 00010000 will make this as 00000000.
        {                                       // else if  PINA is 11110000 and doing '&' with 00010000 will make this as 00010000.
                DDRA = 0xFF;
                PORTA = 0xFF;
                PORTB = 0xDE;
                _delay_ms(500);
                DDRA = 0x00;
                PORTA = 0xF0;
                return '9';
        }
        else if ((PINA & (1<<5)) == 0)
        {
                DDRA = 0xFF;
                PORTA = 0xFF;
                PORTB = 0x7E;
                _delay_ms(500);
                DDRA = 0x00;
                PORTA = 0xF0;
                return '0';
        }
        else if ((PINA & (1<<6)) == 0)
        {
                DDRA = 0xFF;
                PORTA = 0xFF;
                PORTB = 0xEE;
                _delay_ms(500);
                DDRA = 0x00;
                PORTA = 0xF0;
                return 'A';
        }
        else if ((PINA & (1<<7)) == 0)
        {
                DDRA = 0xFF;
                PORTA = 0xFF;
                PORTB = 0xFE;
                _delay_ms(500);
                DDRA = 0x00;
                PORTA = 0xF0;
                return 'B';
        }

	
	
	return 1;
}

int main(void)
{

/*	Columns PA4 , PA5 , PA6 , PA7
 *	Rows	PC5 , PC6 , PC7.
 */
	LCD_INIT();

	unsigned char x;
	
	//Making the column port PA4 to PA7 as I/P (00000000).
	DDRA = 0x00;	
	
	//Making the row port PC5 to PC7 as O/P (11100000).
	DDRC = 0xE0;

	display("PRESS A KEY");
	while(1)
	{
		//Making all the 4 columns 1.
		PORTA = 0xF0;
		//Making all the 3 rows 0. 
		PORTC = 0x00;
		
		_delay_ms(2);

		//Now starting the Scanning Process.
		if(PINA != 0xF0)	//Previously we kept PORTA value as 0xF0, and after this if any switch has been pushed then that      										column will become 0 and then this condition will become true.
		{
			x = keypad();	// Will come inside this loop only when a particular key is pressed. 
			send_cmd(CLR_DPLAY_SCR);
                        send_cmd(DISPLAY_ON_CURSOR_BLINK);
			send_data(x);
			_delay_ms(1000);
			send_cmd(CLR_DPLAY_SCR);
                        send_cmd(DISPLAY_ON_CURSOR_BLINK);
        		display("PRESS A KEY");

			/*PORTC = 0xFF;
			_delay_ms(1000);
			PORTC = 0x00;
			_delay_ms(1000);*/
	
		}
	}
	
	return 0;
}




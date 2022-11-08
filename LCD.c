/*#ifndef F_CPU
#define F_CPU 8000000UL
#endif

#include<avr/io.h>
#include<util/delay.h>

#define en PD6
#define rw PD5
#define rs PD4

void LCD_Command(unsigned char cmnd)
{
	LCD_Port = (LCD_Port & 0x0F) | (cmnd & 0xF0);
	LCD_Port &= ~ (1<<RS);
	LCD_Port |= (1<<EN);
	_delay_ms(1000);
	LCD_Port &= ~ (1<<EN);

	_delay_ms(1000);
	
	LCD_Port = (LCD_Port & 0x0F) | (cmnd << 4);
	LCD_Port |= (1<<EN);
	_delay_ms(1);
	LCD_Port &= ~ (1<<EN);
	_delay_ms(1000);
}

void LCD_Char(unsigned char data)
{
	LCD_Port = (LCD_Port & 0x0F) | (data & 0xF0);
	LCD_Port |= (1<<RS);
	LCD_Port |= (1<<EN);
	_delay_ms(1);
	LCD_Port &= ~ (1<<EN);

	_delay_ms(1000);

	LCD_Port = (LCD_Port & 0x0F) | (data << 4);
	LCD_Port |= (1<<EN);
	_delay_ms(1000);
	LCD_Port &= ~ (1<<EN);
	_delay_ms(1000);
}

void LCD_Init(void)
{
	LCD_Dir = 0xFF;
	_delay_ms(1000);

	LCD_Command(0x02);
	LCD_Command(0x28);
	LCD_Command(0x0C);
	LCD_Command(0x06);
	LCD_Command(0x01);
	_delay_ms(1000);
}

void LCD_String(char *str)
{
	int i;
	for(i = 0; str[i]!=0; i++)
	{
		LCD_Char (str[i]);
	}
}

void LCD_String_xy(char row,char pos,char *str)
{
	if(row == 0 && pos<16)
		LCD_Command((pos & 0x0F)|0x80);
	else if (row == 1 && pos<16)
	LCD_Command((pos & 0x0F)|0xC0);
	LCD_String(str);
}

void LCD_Clear()
{
	LCD_Command(0x01);
	_delay_ms(1000);
	LCD_Command(0x80);
}

int main(void)
{
	DDRB = 0xFF;
	DDRD = 	
	
	LCD_String("TestingProgram");
	LCD_Command(0xC0);
	LCD_String("Done");
	while(1);
}
*/




#define F_CPU 8000000UL

#include <avr/io.h>
#include <util/delay.h>

#define LCD PORTB

#define EN 6
#define RW 5
#define RS 4

void lcdcmd(unsigned char cmd)
{
	PORTD &= ~(1<<RS);  // making RS as 0 PD4 = 0.
	PORTD &= ~(1<<RW);  // making RW as 0 PD5 = 0.
	LCD = cmd & 0xF0;   // sending first part.	
	PORTD |= (1<<EN);   // EN = 1
	_delay_ms(1); 	    // 1ms delay for the enable to happen.
	PORTD &= ~(1<<EN);  // EN = 0
	LCD = cmd<<4;       // sending second part.
	PORTD |= (1<<EN); 
	_delay_ms(1);
	PORTD &= ~(1<<EN);
}

void lcddata(unsigned char data)
{
	PORTD |= (1<<RS);   // making RS as 1 for data PD4 = 1.
	PORTD &= ~(1<<RW);  // RW = 0 for write.
	LCD = data & 0xF0;
	PORTD |= (1<<EN);
	_delay_ms(1);
	PORTD &= ~(1<<EN);  // EN = 0
        LCD = data<<4;       // sending second part.
        PORTD |= (1<<EN);
	_delay_ms(1);
	PORTD &= ~(1<<EN);
}

void lcdinit()
{
	DDRB = 0xFF;
	DDRD = 0xFF;
	PORTD &= ~(1<<EN);
	lcdcmd(0x33);
	lcdcmd(0x32);
	lcdcmd(0x28);
	lcdcmd(0x0E);
	lcdcmd(0x01);
	_delay_ms(2);
}

void lcd_print(char *str)
{
	unsigned char i = 0;
	while(str[i]!=0)
	{
		lcddata(str[i]);
		i++;
	}

}

int main(void)
{
	lcdinit();
	lcd_print("NaveenKumar");
	while(1);
	return 0;
}

 





	

/*This is the program for writing datas into EEPROM via I2C by Bit Banging method.And here time delay is used in milliseconds and every write and read call will be done in main function itself."


#define F_CPU 8000000UL

#include <avr/io.h>
#include <util/delay.h>
#include <stdbool.h>


/****************************************** LCD Details *****************************************************/

#define BIT_HIGH                0xFF
#define BIT_LOW                 0x00
#define TWO_LINE_8_BIT          0x38      
#define DISPLAY_ON_CURSOR_BLINK 0x0C
#define CLR_DPLAY_SCR           0x01
#define INCR_CURSOR             0x06
#define CURSOR_FIRST_LINE       0x80
#define SECOND_FIRST_LINE      0xC0
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

/************************************************************************************************************/


/**************************************I2C related details******************************************/

#define ACK 0 
#define NACK 1
#define SUCCESS 0

#define SDA PC1
#define SCL PC0

#define OUT_REG PORTC
#define IN_REG PINC

#define SDA_HIGH (OUT_REG |= (1 << SDA))
#define SDA_LOW (OUT_REG &= ~(1 << SDA))
#define SCL_HIGH (OUT_REG |= (1 << SCL))
#define SCL_LOW (OUT_REG &= ~(1 << SCL))

#define SDA_READ (IN_REG & (1 << SDA))
#define SCL_READ (IN_REG & (1 << SCL))

//#define I2C_READ 0x01
//#define I2C_WRITE 0x00
#define ADDR 0xA0 //Slave Address

/***************************************************************************************************/


int levels;	

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
                        send_cmd(SECOND_FIRST_LINE);
                }
                level++;
        }

}

void LCD_INIT()
{

        DDRB = BIT_HIGH;
        DDRD = (1<<PD4)|(1<<PD5)|(1<<PD6);
        PORTD &= ~(1<<EN);

        send_cmd(TWO_LINE_8_BIT);
        send_cmd(DISPLAY_ON_CURSOR_BLINK);
        send_cmd(CLR_DPLAY_SCR);
        send_cmd(INCR_CURSOR);
        send_cmd(CURSOR_FIRST_LINE);
}

void I2C_INIT()
{
	DDRC = (1<<PC0) | (1<<PC1);
	SCL_HIGH;
	SDA_HIGH;
}

/*  i2c start sequence */
void I2C_START()
{
    DDRC = (1<<PC0) | (1<<PC1);
    SDA_HIGH;
    _delay_us(5);
    SCL_HIGH;
    _delay_us(5);
    SDA_LOW;
    _delay_us(5);
    SCL_LOW;
    _delay_us(5);
}

/*  i2c stop sequence */
void stop()
{
    DDRC |= (1<<PC1);
    SDA_LOW;
    _delay_us(5);
    SCL_HIGH;
    _delay_us(5);
    SDA_HIGH;
    _delay_us(5);
}


/* Transmit 8 bit data to slave */
uint8_t i2c_write(char data)
{
    uint8_t iter, ack_val;

    for (iter = 0 ; iter < 8 ; iter++)
    {
        (data & 0x80) ? SDA_HIGH : SDA_LOW;
        //_delay_us(5);
        
	SCL_HIGH;
        _delay_us(5);
        SCL_LOW;
	_delay_us(5);
        data <<= 1;		 
    }
    DDRC &= ~(1 << PC1);
    SCL_HIGH;
 

    if(SDA_READ)
    {
    	ack_val = NACK; // Acknowledge bit
    }
    else
    {
	ack_val = ACK;
    }

    _delay_us(5);
    SCL_LOW;
    DDRC |= (1<<PC1);
    _delay_us(5);

    return ack_val;
}

uint8_t data_write(char *input)
{
    uint8_t iter, ack_val=0;
    unsigned char data,i = 0;
    while(input[i]!=0)
    {	
	data = input[i];
	i++;
    	for (iter = 0 ; iter < 8 ; iter++)
    	{
        	(data & 0x80) ? SDA_HIGH : SDA_LOW;
	
        	SCL_HIGH;
        	_delay_us(5);
       	 	SCL_LOW;
       	 	_delay_us(5);
        	data <<= 1;
    	}
    	DDRC &= ~(1 << PC1);
    	SCL_HIGH;


    	if(SDA_READ)
    	{
        	ack_val = NACK; // Acknowledge bit
		return ack_val;
    	}
    	else
    	{
        	ack_val = ACK;
		levels++;
    	}

    	_delay_us(5);
    	SCL_LOW;
    	DDRC |= (1<<PC1);
    	_delay_us(5);	
    }
    return ack_val;
}


uint8_t i2c_read(uint8_t assign_ack)
{
    uint8_t data = 0, iter;
    DDRC &= ~(1 << PC1);
    for (iter = 0; iter < 8; iter++)
    {
        data <<= 1;
	SCL_HIGH;

	if(SDA_READ)
	{
		data = data | 0x01;
	}
	else
	{
		data = data | 0x00;
	}

	_delay_us(5);

	SCL_LOW;
	_delay_us(5);
    }
    DDRC |= (1 << PC1);
    if(assign_ack)
    {
	    SDA_LOW;
	    _delay_us(5);
    }
    else
    {
	    SDA_HIGH;
	    _delay_us(5);
    }
    SCL_HIGH;
    _delay_us(5);
    SCL_LOW;
    _delay_us(5);

    return data;

}


int main(void)
{
	uint8_t data,ack;
	LCD_INIT();
	
	//I2C_INIT();
	//_delay_us(5);
	
	I2C_START();

	ack = i2c_write(0xA0);
	if(ack)
	{
		stop();
		display("ADDR_WRITE_ERROR1!");
	}

	
	ack = i2c_write(0x00);
	if(ack)
	{
		stop();
                display("POSITION_PLACING_ERROR!");
		return 0;
	}

	ack = data_write("WELCOME ATMEGA32");
	if(ack)
        {
                stop();
                display("DATA_WRITING_ERROR!");
              	return 0;
        }
	
	stop();
	_delay_ms(5);

	//while(1)
	//{
		I2C_START();
		ack = i2c_write(0xA0);
		if(ack)
		{
			stop();
			display("ADDR_WRITE_ERROR2!");
			return 0;
		}

		
		ack = i2c_write(0x00);
		if(ack)
		{
			stop();
			display("POSITION_PLACE_ERROR2!");
			return 0;
		}
		
		I2C_START();
		ack = i2c_write(0xA1);
		if(ack)
		{
			stop();
			display("ADDR_READ_ERROR!");
			return 0;
		}
		send_cmd(CLR_DPLAY_SCR);
        	send_cmd(CURSOR_FIRST_LINE);			

	for(int i = 1 ; i <= levels ; i++)
	{
		if(i==levels)
		{
			data = i2c_read(0);
			send_data(data);
		}
		else
		{
			data = i2c_read(1);
			send_data(data);
		}
	}

	stop();
	//_delay_ms(1000);
	//}
	
	return SUCCESS;

}


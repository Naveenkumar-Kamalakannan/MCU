/*This is the program for writing datas into RTC via I2C by Bit Banging method.And here time delay is used in milliseconds.
 */

#define F_CPU 8000000UL

#include <avr/io.h>
#include <util/delay.h>
#include <stdbool.h>

												
/*********************************************** LCD Details *****************************************************/

#define BIT_HIGH                0xFF
#define BIT_LOW                 0x00
#define TWO_LINE_8_BIT          0x38      
#define DISPLAY_ON_CURSOR_BLINK 0x0C
#define CLR_DPLAY_SCR           0x01
#define INCR_CURSOR             0x06
#define CURSOR_FIRST_LINE       0x80
#define SECOND_FIRST_LINE       0xC0
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

/*****************************************************************************************************************/

												
/*********************************************** I2C related details *********************************************/

#define MSB 			0x80
#define ACK 			0 
#define NACK 			1
			
#define SDA 			PC1
#define SCL 			PC0
			
#define DATA_0			(data = data | 0x00)
#define DATA_1			(data = data | 0x01)

#define SDA_LOW 		(PORTC &= ~(1 << SDA))
#define SCL_LOW 		(PORTC &= ~(1 << SCL))
#define SDA_HIGH 		(PORTC |= (1 << SDA))
#define SCL_HIGH 		(PORTC |= (1 << SCL))

#define SDA_READ 		(PINC & (1 << SDA))
#define SCL_READ 		(PINC & (1 << SCL))
			
#define SDA_DIR_IN 		(DDRC &= ~(1 << PC1))
#define SDA_DIR_OUT 		(DDRC |= (1 << PC1))
#define SDA_SCL_DIR_OUT		(DDRC = (1 << PC0) | (1 << PC1))

/****************************************************************************************************************/

int levels = 0;
char sec,min,hour;
char BCD[6];

void delay(int time)
{
        uint16_t top = time/0.128;
        TCNT1 = 0x00;
        TCCR1B |= 0x05;
        OCR1A = top;

        while((TIFR & 0x10) == 0);

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

void command_mode()
{
        PORTD &= COMMAND_REG;  // making RS as 0 PD4 = 0.
        PORTD &= WRITE_REG;  // making RW as 0 PD5 = 0.
}

void data_mode()
{
        PORTD |= DATA_REG;   // making RS as 1 for data PD4 = 1.
        PORTD &= WRITE_REG;  // RW = 0 for write.
}

void send_cmd(unsigned char CMD)
{

        LCD = CMD;
        PORTD |= ENABLE;                                // EN = 1
        my_delay(1);                                    // 1ms delay for the enable to happen.
        PORTD &= NOT_ENABLE;                            // EN = 0
}

void send_data(unsigned char DATA)
{

        LCD = DATA;
        PORTD |= ENABLE;
        my_delay(5);
        PORTD &= NOT_ENABLE;                          	 
}

void display(char *str)
{
	data_mode();
        int level = 0;
        unsigned char i = 0;
        while(str[i]!=0)
        {
                send_data(str[i]);
                i++;
                if(level  == LINE_END)
                {
			command_mode();
                        send_cmd(SECOND_FIRST_LINE);  	//To move the cursor to the second line.
                	data_mode();
		}	
                level++;
        }

}

void LCD_INIT()
{

        DDRB = BIT_HIGH;
        DDRD = (1<<PD4)|(1<<PD5)|(1<<PD6);		
        PORTD &= ~(1<<EN);
	
	command_mode();
        send_cmd(TWO_LINE_8_BIT);
        send_cmd(DISPLAY_ON_CURSOR_BLINK);
        send_cmd(CLR_DPLAY_SCR);
        send_cmd(INCR_CURSOR);
        send_cmd(CURSOR_FIRST_LINE);
}

void I2C_INIT()
{
	SDA_SCL_DIR_OUT;
	SCL_HIGH;
	SDA_HIGH;
}

void I2C_START()
{
    SDA_HIGH;
    _delay_us(5);
    SCL_HIGH;
    _delay_us(5);
    SDA_LOW;
    _delay_us(5);
    SCL_LOW;
    _delay_us(5);
}

void STOP()
{
    SDA_DIR_OUT;
    SDA_LOW;
    _delay_us(5);
    SCL_HIGH;
    _delay_us(5);
    SDA_HIGH;
    _delay_us(5);
}

uint8_t addr_write(char data)
{
    uint8_t iter, ack_val;

    for (iter = 0 ; iter < 8 ; iter++)
    {
        (data & MSB) ? SDA_HIGH : SDA_LOW;			//Moving the datas to the slave from MSB side.
        
	SCL_HIGH;
        _delay_us(5);
        SCL_LOW;
	_delay_us(5);
        data <<= 1;		 
    }
    SDA_DIR_IN;							//Getting the Acknowledgement.
    SCL_HIGH;
 

    if(SDA_READ)						//Checking the Read acknowledgement.
    {
       	ack_val = NACK;
        return ack_val;
    }
    else
    {
       	ack_val = ACK;
    }


    _delay_us(5);
    SCL_LOW;
    SDA_DIR_OUT;
    _delay_us(5);

    return ack_val;						
}

void length(char *input)
{
	unsigned char i = 0;
	while(input[i]!=0)
	{	
		levels++;
		i++;
	}
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
        	(data & MSB) ? SDA_HIGH : SDA_LOW;
	
        	SCL_HIGH;
        	_delay_us(5);
       	 	SCL_LOW;
       	 	_delay_us(5);
        	data <<= 1;
    	}
    	SDA_DIR_IN;
    	SCL_HIGH;


    	if(SDA_READ)
    	{
        	ack_val = NACK;
		return ack_val;
    	}
    	else
    	{
        	ack_val = ACK;
    	}

    	_delay_us(5);
    	SCL_LOW;
    	SDA_DIR_OUT;
    	_delay_us(5);	
    }
    return ack_val;
}


uint8_t i2c_read(uint8_t assign_ack)
{
    uint8_t data = 0, iter;
    SDA_DIR_IN;
    for (iter = 0; iter < 8; iter++)
    {
        data <<= 1;
	SCL_HIGH;
	
	(SDA_READ)? DATA_1 : DATA_0; 
	_delay_us(5);

	SCL_LOW;
	_delay_us(5);
    }
    SDA_DIR_OUT;
    (assign_ack)? SDA_LOW : SDA_HIGH;
    _delay_us(5);
    SCL_HIGH;
    _delay_us(5);
    SCL_LOW;
    _delay_us(5);

    return data;

}

uint8_t I2C_WRITE(unsigned char SL_W_ADDR, unsigned char SL_PLACE)
{
	uint8_t ack = 0;
        
	I2C_START();						//Sending the Start Condition.

        ack = addr_write(SL_W_ADDR);				//Sending the Slave Address with Write.
        if(ack)
        {
                STOP();
                display("ADDR_WRITE_ERROR1!");
		return 0;
        }


        ack = addr_write(SL_PLACE);				
        if(ack)
        {
                STOP();
                display("POSITION_PLACING_ERROR!");
                return 0;
        }

        ack = addr_write(0x00);				//Writing the datas.
        if(ack)
        {
                STOP();
                display("DATA_WRITING_ERROR1!");
                return 0;
        }

	ack = addr_write(0x40);
	if(ack)
        {
                STOP();
                display("DATA_WRITING_ERROR2!");
                return 0;
        }
	
	ack = addr_write(0x20);
	if(ack)
        {
                STOP();
                display("DATA_WRITING_ERROR3!");
                return 0;
        }

        STOP();							//Sending the Stop Condition.
        _delay_ms(5);
	return 1;
}

uint8_t I2C_READ(unsigned char SLA_W_ADDR, unsigned char SL_PLACE, unsigned char SLA_R_ADDR)
{
	uint8_t ack = 0 , loop = 0;
        
	I2C_START();						//Sending the Start Condition.

        ack = addr_write(SLA_W_ADDR);				//Sending the slave address with Write.
        if(ack)
        {
         	STOP();
                display("ADDR_WRITE_ERROR2!");
                return 0;
        }

	ack = addr_write(SL_PLACE);
        if(ack)
        {
                STOP();
                display("POSITION_PLACE_ERROR2!");
                return 0;
        }

        I2C_START();						

        ack = addr_write(SLA_R_ADDR);				//Sending the slave address with Read
        if(ack)
        {
                STOP();
                display("ADDR_READ_ERROR!");
                return 0;
        }

	command_mode();
        send_cmd(CLR_DPLAY_SCR);
        send_cmd(CURSOR_FIRST_LINE);
	data_mode();

	sec = i2c_read(1);
	min = i2c_read(1);
	hour = i2c_read(0);

	BCD[5] = 0x30 + (sec & 0x0F);           //characters second part will be stored here.
        BCD[4] = 0x30 + ((sec & 0x70) >> 4);    //characters first part will be stored here.

        BCD[3] = 0x30 + (min & 0x0F);
        BCD[2] = 0x30 + ((min & 0xF0) >> 4);

        BCD[1] = 0x30 + (hour & 0x0F);
        BCD[0] = 0x30 + ((hour & 0xF0) >> 4);
        display("TIME-");
        while(loop < 6)
        {
           	send_data(BCD[loop]);
               	loop++;
                if(loop ==2 || loop == 4)
                {
                      	send_data(':');
                }
        }

       	STOP();
	return 1;
}

int main(void)
{
	uint8_t check;

/*******LCD Initialization*******/
	LCD_INIT();				
	
/*******I2C Portion*************/
	I2C_INIT();					//Initializing the I2C.
	_delay_us(5);

	length("ATMEGA32"); 				//Finding the length of the string.

	check = I2C_WRITE(0xD0, 0x00);			//Write Function starts.
	if(check == 0)					//If any error occurs program will get terminated.
	{
		return 0;
	}
	
	while(1)
	{
		my_delay(1000);
		check = I2C_READ(0xD0,0x00,0xD1);		//Read Function starts.
		if(check == 0)					//If any error occurs program will get terminated.
		{
			return 0;
		}
	}
	
	return 0;

}


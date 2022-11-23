/*This is the program for writing datas into EEPROM via I2C registers.
 */


#define F_CPU 8000000UL

#include<avr/io.h>
#include<util/delay.h> 

/****************************************** Status Registers ************************************************/
					      								     
#define START        0x08	// SLA+W will be transmitted and ACK or NOT ACK will be received.	     
#define REP_START    0x10	// A repeated START condition has been transmitted then SLA+R will be transmitted and Logic will switch to Mas																   ter Receiver mode
#define MT_SLA_ACK   0x18	// SLA+W has been transmitted and ACK has been received.		     
#define MT_DATA_ACK  0x28	// Data byte has been transmitted and ACK has been received.		     
#define MT_SLA_NACK  0x20	// SLA+W has been transmitted and NOT ACK has been received.		     
#define MT_DATA_NACK 0x30	// Data byte has been transmitted and NOT ACK has been received.	     
#define MR_SLA_ACK   0x40	// SLA+R has been transmitted and ACK has been received.		     
#define MR_SLA_NACK  0x48	// SLA+R has been transmitted and NOT ACK has been received.		     
#define MR_DATA_ACK  0x50 	// Data byte has been received and ACK has been returned		     
#define MR_DATA_NACK 0x58	// Data byte has been received and NOT ACK has been returneid.		     
					  								     
/************************************************************************************************************/
					  
/****************************************** LCD Details *****************************************************/
					  								     			     
#define BIT_HIGH  		0xFF
#define BIT_LOW 		0x00
#define TWO_LINE_8_BIT 		0x38	  
#define DISPLAY_ON_CURSOR_BLINK 0x0C
#define CLR_DPLAY_SCR 		0x01
#define INCR_CURSOR 		0x06
#define CURSOR_FIRST_LINE 	0x80
#define SECOND_FIRST_LINE 	0xC0
#define LCD 			PORTB
#define EN  			6
#define RW  			5
#define RS  			4
#define COMMAND_REG 		~(1<<RS)
#define DATA_REG 		(1<<RS)
#define WRITE_REG 		~(1<<RW)
#define ENABLE 			(1<<EN)
#define NOT_ENABLE  		~(1<<EN)	
#define MAX_SEC 		8000	  	  
#define LINE_END      		0x10

/************************************************************************************************************/
					  								     
/****************************************** I2C Details *****************************************************/

#define I2C_START 		(1<<TWINT)|(1<<TWSTA)|(1<<TWEN)
#define I2C_STOP  		(1<<TWINT)|(1<<TWSTO)|(1<<TWEN)
#define INFO_PASS 		(1<<TWINT)|(1<<TWEN)
#define INFO_READ		(1<<TWINT)|(1<<TWEN)|(1<<TWEA)
#define SLAVE_ADDR_W		0xA0
#define SLAVE_CONTROL_ADDR 	0x07
#define WORD_ADDR		0x00
#define SLAVE_ADDR_R 		0xA1

/************************************************************************************************************/

uint8_t check;	
char sec,min,hour,store;
char time[6];
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
				
        PORTD &= COMMAND_REG;  				// making RS as 0 PD4 = 0.
        PORTD &= WRITE_REG;  				// making RW as 0 PD5 = 0.
        LCD = CMD;         	
        PORTD |= ENABLE;   				// EN = 1
        my_delay(1);        				// 1ms delay for the enable to happen.
        PORTD &= NOT_ENABLE;  				// EN = 0
}

void send_data(unsigned char DATA)
{

        PORTD |= DATA_REG;				// making RS as 1 for data PD4 = 1.
        PORTD &= WRITE_REG;  				// RW = 0 for write.
        LCD = DATA;
        PORTD |= ENABLE;
        my_delay(5);
        PORTD &= NOT_ENABLE;  				// EN = 0
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

void i2c_init()
{
	TWSR |= BIT_LOW; 				// Setting the prescaler bit for SCL frequency calculation.
	TWBR |= 0x20;					// Setting the bit rate for SCL frequency calculation.
	TWCR |= (1<<TWEN);				// Enabling the I2C.
}

int rep_start()
{
        TWCR |= I2C_START;                              //Setting TWI interrupt as 0.
        while ((TWCR & (1<<TWINT))==0);                 //When TWI interrupt becomes one it comes out.

        if (((TWSR & 0xF8) != REP_START)){
                PORTC = BIT_HIGH;
                return 0;
        }

        else{
        return 1;
        }
}

int start()
{
									
	TWCR |= I2C_START;				//Setting TWI interrupt as 0.
	while ((TWCR & (1<<TWINT))==0); 		//When TWI interrupt becomes one it comes out.
	
	if ((TWSR & 0xF8) != START){			
		PORTC = BIT_HIGH;
		return 0;
	}
	
	else{
	return 1;
	}
}

void stop()
{

	TWCR |= I2C_STOP;				//Setting TWI interrupt as 0.
}

int word_write(unsigned char data)
{
        TWDR = data;                                    //move the value to I2C reg.
        TWCR = INFO_PASS;                            	//Enable I2C and clear the interrupt.
        while (!(TWCR & (1<<TWINT)));                   //Waiting for the current task to over, if over it will become 1 and will come out of                                                                                                      the loop,Then the TWDR will become free.
        if ((TWSR & 0xF8) != MT_DATA_ACK){
                PORTC = BIT_HIGH;
                return 0 ;
        }
        else{
        return 1;
        }

}

int data_write(char *data)
{
	unsigned char i = 0;
	while(data[i]!=0)
	{
		TWDR = data[i];				//move the value to I2C reg.
		i++;
		TWCR = INFO_PASS;			//Enable I2C and clear the interrupt.
	  	while (!(TWCR & (1<<TWINT)));		//Waiting for the current task to over, if over it will become 1 and will come out of 													   the loop,Then the TWDR will become free.
		if ((TWSR & 0xF8) != MT_DATA_ACK){
			PORTC = BIT_HIGH;
			return 0 ;
		}
		else{
			levels++;
		}
	}
	return 1;	

}

int data_read()
{

        TWCR = INFO_READ;                               //Enable I2C and clear the interrupt.
        while ((TWCR & (1<<TWINT))==0);                 //Waiting for the current task to over, if over it will become 1 and will come out of                                                                                           the loop.Then all data will be received in the TWDR.

        if((TWSR & 0xF8) != MR_DATA_ACK){
                PORTC = BIT_HIGH;
                return 0;
        }
	else{
		store = TWDR;
		send_data(store);
                return 1;
        }
}

int l_data_read()
{

        TWCR = INFO_PASS;                               //Enable I2C and clear the interrupt.
        while ((TWCR & (1<<TWINT))==0);                 //Waiting for the current task to over, if over it will become 1 and will come out of                                                                                           the loop.Then all data will be received in the TWDR.

        if((TWSR & 0xF8) != MR_DATA_NACK){
                PORTC = BIT_HIGH;
                return 0;
        }
        else{
                store = TWDR;
                send_data(store);
                return 1;
        }
}



int address_pass_with_w(unsigned char data)
{

	TWDR = data;  					//move the value to I2C reg.
	TWCR |= INFO_PASS; 				//Enable I2C and clear the interrupt.
	while (!(TWCR & (1<<TWINT)));			//Waiting for the current task to over, if over it will become 1 and will come out of 													   the loop,Then the TWDR will become free.
	if((TWSR & 0xF8) != MT_SLA_ACK){
		PORTC = BIT_HIGH;
		return 0;
	}
	else{
	return 1;
	}

}


int address_pass_with_r(unsigned char data)
{

        TWDR = data;                                    //move the value to I2C reg.
        TWCR |= INFO_PASS;		              	//Enable I2C and clear the interrupt.
        while (!(TWCR & (1<<TWINT)));                   //Waiting for the current task to over, if over it will become 1 and will come out of                                                                                                      the loop,Then the TWDR will become free.
        if((TWSR & 0xF8) != MR_SLA_ACK){
                PORTC = BIT_HIGH;
                return 0;
        }
        else{
        return 1;
        }

}

void LED_LCD_INIT()
{
	DDRC = BIT_HIGH;

        DDRB = BIT_HIGH;
        DDRD = BIT_HIGH;
        PORTD &= ~(1<<EN);

        send_cmd(TWO_LINE_8_BIT);
        send_cmd(DISPLAY_ON_CURSOR_BLINK);
        send_cmd(CLR_DPLAY_SCR);
        send_cmd(INCR_CURSOR);
        send_cmd(CURSOR_FIRST_LINE);
}

int main(void)
{
	LED_LCD_INIT();

	i2c_init();						//This is for I2C initialization.
	
	check = start();					//Here we are sending the start bit of I2C.
	if(check == 0)
	{
		display("START_BIT_ERROR!");
		return 0;
	}

	check = address_pass_with_w(SLAVE_ADDR_W);		//Here we are sending the address of slave(DS1307_RTC-1101000) with write bit(0).
	if(check == 0)
	{
       		send_cmd(CLR_DPLAY_SCR);
		send_cmd(CURSOR_FIRST_LINE);
		display("ADDRESS_PASS_W_WRITE_ERROR!");
		return 0;
	}
	
	check = word_write(WORD_ADDR); 		//To write data into control register,selecting that address here.
	if(check == 0)
	{
		send_cmd(CLR_DPLAY_SCR);
                send_cmd(CURSOR_FIRST_LINE);
                display("WORD_ADDRESS_WRITE!");
		return 0;
	}
	
 	check = data_write("WELCOME ATMEGA32");               //To write data into control register,selecting that address here.
        if(check == 0)
        {
                send_cmd(CLR_DPLAY_SCR);
                send_cmd(CURSOR_FIRST_LINE);
                display("DATA_PASSING!");
                return 0;
        }
	
	stop(); 						//Here we are sending the Stop bit.
	_delay_ms(500);

/*********************************************************************************************************************/
	check = start();                                        //Here we are sending the start bit of I2C.
        if(check == 0)
        {
                send_cmd(CLR_DPLAY_SCR);
                send_cmd(CURSOR_FIRST_LINE);
                display("READING_START_ERROR!");
                return 0;
        }
	
	check = address_pass_with_w(SLAVE_ADDR_W);              //Slave address sending with write bit.
        if(check == 0)
        {
                send_cmd(CLR_DPLAY_SCR);
                send_cmd(CURSOR_FIRST_LINE);
                display("Reading_Part_ADDR_PASS1_ERROR!");
                return 0;
        }

	check = word_write(WORD_ADDR);               //To write data into control register,selecting that address here.
        if(check == 0)
        {
                send_cmd(CLR_DPLAY_SCR);
                send_cmd(CURSOR_FIRST_LINE);
                display("WORD_ADDRESS_READ!");
                return 0;
        }
	
	check = rep_start();                                    //Sending the repeated start of I2C.
        if(check == 0)
        {
                send_cmd(CLR_DPLAY_SCR);
                send_cmd(CURSOR_FIRST_LINE);
                display("REPEATED_START_ERROR!");
                return 0;
        }
	
	check = address_pass_with_r(SLAVE_ADDR_R);              //Slave address sending with read bit.
        if(check == 0)
        {
                send_cmd(CLR_DPLAY_SCR);
                send_cmd(CURSOR_FIRST_LINE);
                display("SLAVE_ADDR_W_RD");
                return 0;
        }
	
	for(int i = 1 ; i <=levels  ; i++)
	{
		if(i == levels)
		{ 
			check = l_data_read();                 //Reading first data of RTC which is in 0 address and it is Second's related.
        		if(check == 0)
        		{
                		send_cmd(CLR_DPLAY_SCR);
                		send_cmd(CURSOR_FIRST_LINE);
                		display("DATA_READ!");
                		return 0;
        		}
		}
		else{
			check = data_read();
			if(check == 0)
			{
                		send_cmd(CLR_DPLAY_SCR);
                		send_cmd(CURSOR_FIRST_LINE);
                		display("DATA_READ!");
                		return 0;
        		}
		}
	}

	/*check = l_data_read();
	if(check == 0)
        {
            	send_cmd(CLR_DPLAY_SCR);
                send_cmd(CURSOR_FIRST_LINE);
                display("DATA_READ!");
               	return 0;
        }*/


	stop();
	/*_delay_ms(1000);
	send_cmd(CLR_DPLAY_SCR);
	send_cmd(CURSOR_FIRST_LINE);*/
	
	
	return 0;
}	



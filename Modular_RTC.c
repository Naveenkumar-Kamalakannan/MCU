/* This Program is for writing the datas to RTC by communicating via I2C and then reading those datas and displaying it on LCD.
 * Both Writing and Reading the datas are handled in this same program itself.
 * Functionalities of most of the functions are written inside their function bodies.
 * Here the input data used is 15hrs,43min and 00sec.
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
#define CURSOR_SECOND_LINE 	0xC0
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
#define SLAVE_ADDR_W		0xD0	  
#define SLAVE_CONTROL_ADDR 	0x07	  
#define SLAVE_SEC_ADDR		0x00	  
#define SLAVE_ADDR_R 		0xD1	  
					  
/************************************************************************************************************/
					  
/****************************************** Functions Declarations ******************************************/

void i2c_init(void);
int start(void);
int rep_start(void);
int address_pass_with_w(unsigned char);
int address_pass_with_r(unsigned char);
int data_write(unsigned char);
int data_read(void);
int l_data_read(void);
void stop(void);
void read_stop(void);
void delay(int);
void my_delay(int);
void send_cmd(unsigned char);
void send_data(unsigned char);
void display(char *);
void command_mode(void);
void data_mode(void);
int I2C_WRITE(unsigned char, unsigned char, unsigned char , unsigned char, unsigned char);
int I2C_READ(unsigned char, unsigned char, unsigned char);
/*************************************************************************************************************/

uint8_t check;	
char sec,min,hour;
char BCD[6];

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

void command_mode()
{
        PORTD &= ~(1<<RS);  // making RS as 0 PD4 = 0.
        PORTD &= ~(1<<RW);  // making RW as 0 PD5 = 0.
}

void data_mode()
{
        PORTD |= (1<<RS);   // making RS as 1 for data PD4 = 1.
        PORTD &= ~(1<<RW);  // RW = 0 for write.
}

void send_cmd(unsigned char CMD)
{
				
       	//PORTD &= COMMAND_REG;  				// making RS as 0 PD4 = 0.
        //PORTD &= WRITE_REG;  				// making RW as 0 PD5 = 0.
        LCD = CMD;         	
        PORTD |= ENABLE;   				// EN = 1
        my_delay(1);        				// 1ms delay for the enable to happen.
        PORTD &= NOT_ENABLE;  				// EN = 0
}

void send_data(unsigned char DATA)
{

        //PORTD |= DATA_REG;				// making RS as 1 for data PD4 = 1.
        //PORTD &= WRITE_REG;  				// RW = 0 for write.
        LCD = DATA;
        PORTD |= ENABLE;
        my_delay(5);
        PORTD &= NOT_ENABLE;  				// EN = 0
}

void display(char *str)
{
	data_mode();					// shifted to data mode to set the datas.
	int level = 0;
        unsigned char i = 0;
        while(str[i]!=0)
        {
                send_data(str[i]);
                i++;
		if(level  == LINE_END)
                {
			command_mode();			// shifted to command mode to apply this command of moving the cursor to second line.
                        send_cmd(CURSOR_SECOND_LINE);
			data_mode();			// again shifted to data mode to continue passing the datas. 
                }
                level++;
        }

}

void i2c_init()
{
/* This Function is for initializing the I2C by setting the SCL frequency.
 * at first in TWSR(Two Wire serial Interface Status Register) we are setting the Prescaler value of bit 1 and 0  as 0.
 * then in TWBR(Two Wire serial Interface Bit rate Register) we are setting the TWBR value for the SCL Frequency calculation.
 * SCL frequency = ((CPU Clock frequency)/(16 + 2(TWBR)*4^TWPS))
 */
	TWSR |= BIT_LOW; 				// Setting the prescaler bit for SCL frequency calculation.
	TWBR |= 0x20;					// Setting the bit rate for SCL frequency calculation.
	TWCR |= (1<<TWEN);				// Enabling the I2C.
}


int rep_start()
{
/* This Function is for initializing the I2C communication again by sending the REPEATED START bit
 * To send the REPEATED START bit we have to enable three bits in TWCR(Two Wire serial Interface Control Register),they are:
     	TWINT(Two Wire serial Interface Interrupt) - We have to make this bit as 0, So that when this action gets completed the TWI Module wil							   l set this bit to 1 indicating the completion of its current action. 
       	TWSTA(Two Wire serial Interface Start) 	   - Making this bit as 1 will generate the START condition if the bus is free.
      	TWEN(Two Wire serial Interface Enable)     - Making this bit as 1 will enable the TWI Module.
 * Then we have to wait for the TWI Interrupt bit to become 1, So that we can know that the current action was completed.
 * Then with the help of the respective Status Register checking correct completion of the process.
 */
	TWCR |= I2C_START; 				//Setting TWI interrupt as 0.
	while ((TWCR & (1<<TWINT))==0); 		//When TWI interrupt becomes one it comes out.

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
/* This Function is for initializing the I2C communication by sending the START bit
 * To send the START bit we have to enable three bits in TWCR(Two Wire serial Interface Control Register),they are:
 	TWINT(Two Wire serial Interface Interrupt) - We have to make this bit as 0, So that when this action gets completed the TWI Module wil							   l set this bit to 1 indicating the completion of its current action. 
	TWSTA(Two Wire serial Interface Start) 	   - Making this bit as 1 will generate the START condition if the bus is free.
	TWEN(Two Wire serial Interface Enable) 	   - Making this bit as 1 will enable the TWI Module.
 * Then we have to wait for the TWI Interrupt bit to become 1, So that we can know that the current action was completed.
 * Then with the help of the respective Status Register checking correct completion of the process.
 */
									
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
/* This Function is for terminating the I2C Communication by sending the STOP bit.
 * To send the STOP bit we have to enable three bits in TWCR(Two Wire serial Interface Control Register),they are:
     	TWINT(Two Wire serial Interface Interrupt)  - We have to make this bit as 0, So that when this action gets completed the TWI Module wi							    ll set this bit to 1 indicating the completion of its current action.
      	TWSTO(Two Wire serial Interface Stop) 	    - Making this bit as 1 will generate the STOP condition and will be cleared by the hardwar							    e when the STOP condition is transmitted.
     	TWEN(Two Wire serial Interface Enable)	    - Making this bit as 1 will enable the TWI Module.
 */

	TWCR |= I2C_STOP;				//Setting TWI interrupt as 0.
}


void read_stop()
{
/* This Function is for terminating the I2C Communication by sending the STOP bit.
 * To send the STOP bit we have to enable three bits in TWCR(Two Wire serial Interface Control Register),they are:
      	TWINT(Two Wire serial Interface Interrupt) - We have to make this bit as 0, So that when this action gets completed the TWI Module wil							   l set this bit to 1 indicating the completion of its current action.
       	TWSTO(Two Wire serial Interface Stop)      - Making this bit as 1 will generate the STOP condition and will be cleared by the hardware 							   when the STOP condition is transmitted.
       	TWEN(Two Wire serial Interface Enable)     - Making this bit as 1 will enable the TWI Module.
 */

        TWCR |= I2C_STOP;                		//Setting TWI interrupt as 0.
}



int data_write(unsigned char data)
{
/* This Function is for writing the data into the slave.
 * First we have to give the data to the TWDR(Two Wire serial Interface Data Register) and from here the datas will be moved.
 * To write the data we have to enable two bit is TWCR(Two Wire serial Interface Control Register),they are:
 	TWINT(Two Wire serial Interface Interrupt) - We have to make this bit as 0, So that when this action gets completed the TWI Module wil							   l set this bit to 1 indicating the completion of its current action.
	TWEN(Two Wire serial Interface Enable)     - Making this bit as 1 will enable the TWI Module.
 * Then we have to wait for the TWI Interrupt bit to become 1,So that we can know that the current action was completed.After Completion TWDR 																  will become free.
 * Then with the help of the respective Status Register checking correct completion of the process.
 */
	TWDR = data;  					//move the value to I2C reg.
	TWCR = INFO_PASS;				//Enable I2C and clear the interrupt.
	while (!(TWCR & (1<<TWINT)));			//Waiting for the current task to over, if over it will become 1 and will come out of 													   the loop,Then the TWDR will become free.
	if ((TWSR & 0xF8) != MT_DATA_ACK){
		PORTC = BIT_HIGH;
		return 0 ;
	}
	else{
	return 1;
	}	

}

int address_pass_with_w(unsigned char data)
{
/* This Function is for writing the address with write bit to match the respective slave.
 * First we have to give the address to the TWDR(Two Wire serial Interface Data Register) and from here the address will be moved.
 * To write the address we have to enable two bit is TWCR(Two Wire serial Interface Control Register),they are:
     	TWINT(Two Wire serial Interface Interrupt) - We have to make this bit as 0, So that when this action gets completed the TWI Module wil							   l set this bit to 1 indicating the completion of its current action.
     	TWEN(Two Wire serial Interface Enable)     - Making this bit as 1 will enable the TWI Module.
 * Then we have to wait for the TWI Interrupt bit to become 1,So that we can know that the current action was completed.After Completion TWDR 																  will become free.
 * Then with the help of the respective Status Register checking correct completion of the process.
 */

	TWDR = data;  					//move the value to I2C reg.
	TWCR |= INFO_PASS;				//Enable I2C and clear the interrupt.
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
/* This Function is for writing the address with read bit to match the respective slave.
 * First we have to give the address to the TWDR(Two Wire serial Interface Data Register) and from here the address will be moved.
 * To write the address we have to enable two bit is TWCR(Two Wire serial Interface Control Register),they are:
      	TWINT(Two Wire serial Interface Interrupt) - We have to make this bit as 0, So that when this action gets completed the TWI Module wil							   l set this bit to 1 indicating the completion of its current action.
      	TWEN(Two Wire serial Interface Enable)     - Making this bit as 1 will enable the TWI Module.
 * Then we have to wait for the TWI Interrupt bit to become 1,So that we can know that the current action was completed.After Completion the 
 														       TWDR will become free.
 * Then with the help of the respective Status Register checking correct completion of the process.
 */

	TWDR = data;					//move the value to I2C reg.
        TWCR |= INFO_PASS;		   		//Enable I2C and clear the interrupt.
        while (!(TWCR & (1<<TWINT)));   		//Waiting for the current task to over, if over it will become 1 and will come out of 													the loop,Then the TWDR will become free.
	if((TWSR & 0xF8) != MR_SLA_ACK)
        {
                PORTC = BIT_HIGH;
                return 0;
        }
        else{
        return 1;
	}
}

int l_data_read()
{
/* This Function is for reading the datas from the respective address of the slave.
 * The Datas which we are reading from the slave will automatically be saved in the TWDR(Two Wire serial Interface Data Register).
 * To read the datas from the slave we have to enable three bits is TWCR(Two Wire serial Interface Control Register),they are:
    	TWINT(Two Wire serial Interface Interrupt) 		- We have to make this bit as 0, So that when this action gets completed the 
								  TWI Module will set this bit to 1 indicating the completion of its current 
								  action.
       	TWEN(Two Wire serial Interface Enable)     		- Making this bit as 1 will enable the TWI Module.
	TWEA(Two Wire serial Interface Enable Acknowledgement) 	- Making this bit as 1 will enable the generation of Ack in receiver mode (or) 									when needed in slave.
 * Then we have to wait for the TWI Interrupt bit to become 1,So that we can know that the current action was completed.After Completion the
  														       TWDR will become free.
 * Then with the help of the respective Status Register checking correct completion of the process.
 */

	int loop = 0;
        TWCR = INFO_PASS;  				//Enable I2C and clear the interrupt.
        while ((TWCR & (1<<TWINT))==0); 		//Waiting for the current task to over, if over it will become 1 and will come out of 												the loop.Then all data will be received in the TWDR.
        
	if((TWSR & 0xF8) != MR_DATA_NACK){
                PORTC = BIT_HIGH;
                return 0;
	}

	else{
		hour = TWDR;
		BCD[5] = 0x30 + (sec & 0x0F);		//characters second part will be stored here.
		BCD[4] = 0x30 + ((sec & 0x70) >> 4);	//characters first part will be stored here.

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
		return 1;	
	}
}

int data_read()
{
/* This Function is for reading the datas from the respective address of the slave.
 * The Datas which we are reading from the slave will automatically be saved in the TWDR(Two Wire serial Interface Data Register).
 * To read the datas from the slave we have to enable three bits is TWCR(Two Wire serial Interface Control Register),they are:
    	TWINT(Two Wire serial Interface Interrupt)		- We have to make this bit as 0, So that when this action gets completed the 
							          TWI Module will set this bit to 1 indicating the completion of its current 
								  action.
     	TWEN(Two Wire serial Interface Enable)     		- Making this bit as 1 will enable the TWI Module.
 	TWEA(Two Wire serial Interface Enable Acknowledgement) 	- Making this bit as 1 will enable the generation of Ack in receiver mode (or) 									when needed in slave.
 * Then we have to wait for the TWI Interrupt bit to become 1,So that we can know that the current action was completed.After Completion the 
		 												      TWDR will become free.
 * Then with the help of the respective Status Register checking correct completion of the process.
 */

        TWCR = INFO_READ;  				//Enable I2C and clear the interrupt.
        while ((TWCR & (1<<TWINT))==0); 		//Waiting for the current task to over, if over it will become 1 and will come out of 												the loop. All data will be received in the TWDR.

        if((TWSR & 0xF8) != MR_DATA_ACK){
                PORTC = BIT_HIGH;
                return 0;
        }

        else{
                return 1;
        }
}


void LCD_INIT()
{
	DDRC = BIT_HIGH;

        DDRB = BIT_HIGH;
        DDRD = BIT_HIGH;
        PORTD &= ~(1<<EN);

        command_mode();                                         //Called the command mode to set the below commands settings.
        send_cmd(TWO_LINE_8_BIT);
        send_cmd(DISPLAY_ON_CURSOR_BLINK);
        send_cmd(CLR_DPLAY_SCR);
        send_cmd(INCR_CURSOR);
        send_cmd(CURSOR_FIRST_LINE);
}

int I2C_WRITE(unsigned char SL_ADDR, unsigned char SL_PLACE, unsigned char SECONDS, unsigned char MINUTES, unsigned char HOURS)
{

	check = start();                                        //Here we are sending the start bit of I2C.
        if(check == 0)
        {
                display("START_BIT_ERROR!");
                return 0;
        }


	check = address_pass_with_w(SL_ADDR);           //Here we are sending the address of slave(DS1307_RTC-1101000) with write bit(0).
        if(check == 0)
        {
                send_cmd(CLR_DPLAY_SCR);
                send_cmd(CURSOR_FIRST_LINE);
                display("ADDRESS_PASS_W_WRITE_ERROR!");
                return 0;
        }
	
	check = data_write(SL_PLACE);                     //Address field for Seconds.
        if(check == 0)
        {
                send_cmd(CLR_DPLAY_SCR);
                send_cmd(CURSOR_FIRST_LINE);
                display("SECONDS_ADDR_FIELD_ERROR!");
                return 0;
        }

	check = data_write(SECONDS);                               //Second related data. 
        if(check == 0)
        {
                send_cmd(CLR_DPLAY_SCR);
                send_cmd(CURSOR_FIRST_LINE);
                display("SECOND'S_DATA_ERROR!");
                return 0;
        }

	check = data_write(MINUTES);                               //Second related data.
        if(check == 0)
        {
                send_cmd(CLR_DPLAY_SCR);
                send_cmd(CURSOR_FIRST_LINE);
                display("MINUTE'S_DATA_ERROR!");
                return 0;
        }
	
	check = data_write(HOURS);                               //Second related data.
        if(check == 0)
        {
                send_cmd(CLR_DPLAY_SCR);
                send_cmd(CURSOR_FIRST_LINE);
                display("HOUR'S_DATA_ERROR!");
                return 0;
        }

	stop();
	_delay_ms(500);
	return 1;
}

int I2C_READ(unsigned char SL_ADDR, unsigned char SL_PLACE, unsigned char SL_ADDR_R)
{
	check = start();                                        //Here we are sending the start bit of I2C.
        if(check == 0)
        {
                send_cmd(CLR_DPLAY_SCR);
                send_cmd(CURSOR_FIRST_LINE);
                display("READING_START_ERROR!");
                return 0;
        }

	check = address_pass_with_w(SL_ADDR);              //Slave address sending with write bit.
        if(check == 0)
        {
                send_cmd(CLR_DPLAY_SCR);
                send_cmd(CURSOR_FIRST_LINE);
                display("Reading_Part_ADDR_PASS1_ERROR!");
                return 0;
        }

	check = data_write(SL_PLACE);                     //Address field for Seconds
        if(check == 0)
        {
                send_cmd(CLR_DPLAY_SCR);
                send_cmd(CURSOR_FIRST_LINE);
                display("SECONDS_ADDR_FIELD1_ERROR!");
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


	check = address_pass_with_r(SL_ADDR_R);              //Slave address sending with read bit.
        if(check == 0)
        {
                send_cmd(CLR_DPLAY_SCR);
                send_cmd(CURSOR_FIRST_LINE);
                display("Reading_Part_ADDR_PASS2_ERROR!");
                return 0;
        }
        command_mode();  	
	
	send_cmd(CLR_DPLAY_SCR);
	send_cmd(CURSOR_FIRST_LINE);
	for(int i = 0 ; i < 3 ; i++)
	{
		if(i == 2){
			data_mode();
			check = l_data_read();              //Reading third data of RTC which is in 2 address and it is Hour's related.
        		if(check == 0)
        		{
                		send_cmd(CLR_DPLAY_SCR);
                		send_cmd(CURSOR_FIRST_LINE);
                		display("HOUR'S_DATA_READ_ERROR!");
               			return 0;
        		}
		}

		else{
		check = data_read();			//Reading first data of RTC which is in 0 address and it is Second's related.
		if(check == 0)
		{
			send_cmd(CLR_DPLAY_SCR);
                	send_cmd(CURSOR_FIRST_LINE);
                	display("DATA_READ_ERROR!");
			return 0;
		}
		else
		{
			if(i == 0)
			{
				sec = TWDR;
			}
			else
			{
				min = TWDR;
			}
		}
		}
	}

        stop();
	return 1;
}

		

int main(void)
{

	uint8_t check;
      	LCD_INIT();

	i2c_init();						//This is for I2C initialization.

	check = I2C_WRITE(0xD0, 0x00, 0x00, 0x59, 0x23);
	if(check==0)
	{
		return 0;
	}

		
	while(1){
		my_delay(900);
	
		check = I2C_READ(0xD0, 0x00, 0xD1);
		if(check == 0)
		{
			return 0;
		}
	
	}

	return 0;

}


/**************************************************************************************************
TWI.c

This contains all the functions needed by TWI masters and slaves for ATmega328p
**************************************************************************************************/

#include "TWI.h"
/**************************************************************************************************
 Initializes the TWI (I2C) interface with the following options:
		-Speed 100KHz
**************************************************************************************************/

void Initialize_TWI_Master(){
	//TWCR = (1 << TWEN); // turn on TWI
	TWBR = 72;  //100KHz at F_CPU = 16MHz
}



void TWI_Error(unsigned char data){
	/*USART_Transmit('\n');
	USART_Transmit('\r');*/
	USART_Transmit('E');
	USART_Transmit(data);
}

int TWI_Send_Start(unsigned char expectedResult){
	TWCR = (1<<TWINT)|(1<<TWSTA)|(1 << TWEN); //send START condition
		
	while (!(TWCR & (1<<TWINT))) {}; //Wait for START to transmit
		
//	if ((TWSR & 0xF8) != expectedResult){ //check and make sure START was actually sent
//		TWI_Error(TWSR);
//		return 0;
//	}
	return 1;
}

int TWI_Send_Data(unsigned char data, unsigned char expectedResult){
	TWDR = data; //eeprom I2C address and set to write	
	TWCR = (1<<TWINT) | (1<<TWEN);	//clear TWINT flag to start transmission	
		
	while (!(TWCR & (1<<TWINT))) {}; //Wait for transmission to complete
			
//	if ((TWSR & 0xF8) != expectedResult){ //check and make sure SLA_W was actually sent
//		TWI_Error(TWSR);
//		return 0;
//	}
	return 1;
}

int TWI_Send_Stop(){
	TWCR = (1<<TWINT)|(1<<TWEN)|(1<<TWSTO); //transmit stop condition

	
		
	return 1;
}

unsigned char TWI_Receive_Data(unsigned char expectedResult){
	
	if (expectedResult == MR_DATA_ACK)
		TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWEA);	//clear TWINT flag to start reading
	else
		TWCR = (1<<TWINT) | (1<<TWEN);	//clear TWINT flag to start reading	
		
	while (!(TWCR & (1<<TWINT))) {}; //Wait for transmission to complete
			
//	if ((TWSR & 0xF8) != expectedResult){ //check and make sure SLA_R was actually sent
//		TWI_Error(TWSR);
//	}
	
	return TWDR;
}
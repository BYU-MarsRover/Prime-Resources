/**************************************************************************************************
SPI.c

This contains all the functions needed by SPI masters and slaves for ATmega328p
**************************************************************************************************/

#include "SPI.h"

/**************************************************************************************************
 Initializes the SPI interface as a slave with interrupts with the following options:
		-MSB or LSB first
**************************************************************************************************/
void Initialize_SPI_Slave(unsigned int dataMode){
	
	SPCR |= (1 << SPE); //Enable SPI
	SPCR |= (1 << SPIE); //Enable SPI interrupts
	//SPCR |= (1 << MSTR); //set to SPI master
	//SPCR |= (dataMode << DORD); //LSB or MSB first
	DDRB |= (1 << MISO); //set MISO (PB4) as an output, all others are inputs
	SPDR = 0; //clear out the SPI data register before it is used	
}

//*****************************************************************************************************
// Receives a byte over SPI and returns it
//*****************************************************************************************************

char SPI_Slave_Receive(void){
	
	while(!(SPSR & (1<<SPIF))) {}; // Wait for reception to finish

	return SPDR; //return data in register
}
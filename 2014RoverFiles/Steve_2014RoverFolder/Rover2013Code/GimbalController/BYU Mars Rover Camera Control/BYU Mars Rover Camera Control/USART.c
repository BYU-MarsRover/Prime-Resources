/**************************************************************************************************
USART.c

This contains all the functions needed by the USART for ATmega328p
**************************************************************************************************/

#include "USART.h"
//*****************************************************************************************************
// Initializes the USART to the predefined baud rate (see the #defines in USART.h) and 8-N-1 framing
//*****************************************************************************************************

void Initialize_USART(unsigned int myubrr){
	UBRR0H = ((myubrr) >> 8);  //Load upper 8 bits of baud rate prescaler into to UBRR0 register
	UBRR0L = myubrr;  //load the lower 8 bits of the baud rate prescaler into the UBRR0 register
	UCSR0B |= (1 << RXEN0) | (1 << TXEN0);  //enable RX and TX circuitry
	//UCSR0C |= (1 << UPM01); // enable this for even parity
	//UCSR0C &= (0 << UPM00);
}

//*****************************************************************************************************
// Waits until the USART data register is ready to send and then sends the byte
//*****************************************************************************************************

void USART_Transmit(unsigned char data){
	while((UCSR0A & (1 << UDRE0)) == 0 ) {};  //do nothing until UDR is ready to send
	UDR0 = data; //transmit byte
}



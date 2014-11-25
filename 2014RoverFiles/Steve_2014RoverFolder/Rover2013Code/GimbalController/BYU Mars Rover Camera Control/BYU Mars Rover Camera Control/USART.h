/******************************************************************************
USART.h

This contains all the functions needed by the USART for ATmega328p
******************************************************************************/

#ifndef __USART_H
#define __USART_H

//Baud rate definitions
//#define F_CPU 16000000UL  //crystal frequency
#define USART_BAUDRATE 9600 //desired baud rate
#define BAUD_PRESCALE (((F_CPU / (USART_BAUDRATE * 16UL))) - 1) //this formula will automatically calculate the baudrate prescale for you


void Initialize_USART(unsigned int myubrr);
void USART_Transmit(unsigned char data);
//char USART_Receive();

#endif
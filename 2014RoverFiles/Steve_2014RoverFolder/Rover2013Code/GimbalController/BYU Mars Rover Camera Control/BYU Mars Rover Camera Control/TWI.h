/******************************************************************************
TWI.h

This contains all the functions needed by TWI masters and slaves for ATmega328p
******************************************************************************/

#ifndef __TWI_H
#define __TWI_H

#include "USART.h"


//Pin definitions

//Status Code Definitions DO NOT CHANGE
#define START 0x08
#define REPEATED_START 0x10
#define MT_SLAW_ACK 0x18
#define MT_DATA_ACK 0x28
#define MR_SLAR_ACK 0x40
#define MR_DATA_ACK 0x50
#define MR_DATA_NACK 0x58


void Initialize_TWI_Master(void);
void TWI_Error(unsigned char data);
int TWI_Send_Start(unsigned char expectedResult);
int TWI_Send_Data(unsigned char data, unsigned char expectedResult);
int TWI_Send_Stop(void);
unsigned char TWI_Receive_Data(unsigned char expectedResult);

#endif
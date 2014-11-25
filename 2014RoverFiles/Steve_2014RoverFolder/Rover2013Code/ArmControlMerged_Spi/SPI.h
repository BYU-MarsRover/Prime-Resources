/******************************************************************************
SPI.h

This contains all the functions needed by SPI masters and slaves for ATmega328p
******************************************************************************/

#ifndef __SPI_H
#define __SPI_H

#include <avr/io.h>

//Pin definitions
//#define MISO DDB4 //PB4 //COMMENTED OUT BECAUSE IT DOESN'T PLAY NICE WITH ARDUINO

//other definitions
#define MSB 0
#define LSB 1

void Initialize_SPI_Slave(unsigned int dataMode);
char SPI_Slave_Receive(void);

#endif

/**************************************************************************************************
BYU Mars Rover Camera Control.h		Created By: Daniel Hearn		27 Dec 2011

This is the code for the camera control board.  It functions as follows:

It is an SPI slave, I2C master, UART, and servo controller. Commands come in over the SPI bus, 
they are checked for errors and loaded into a buffer.  The main loop parses the commands in the
order they were received, and generates responses which are loaded into an output buffer.  The
SPI slave returns data as it is made available.  The I2C master is responsible for controlling the
on board compass.  The UART is used to control the camera.  The two 16-bit timers are used to
control two servos.  Most of the functions are interrupt based, and leave little for the main loop
to do.
**************************************************************************************************/
#ifndef BYU_MRCP_H
#define BYU_MRCP_H

#define F_CPU 16000000UL //clock speed of the uC
#define BUFFER_SIZE 512 //change this to change the size of all buffers
#define PACKET_SIZE 24

#include "SPI.h"
#include "SPI.c"
#include "RingBuffer.h"
#include "RingBuffer.c"

//Mars Rover Com Protocol  DO NOT CHANGE
#define DEVICE     0b11110000 //mask
#define NO_OP      0b00000000
#define GENERAL    0b00010000
#define ARM        0b01000000 

#define READ_WRITE 0b00000001 //mask
#define READ       0b00000001
#define WRITE      0b00000000

#define DEVICE_RW  0b11110001 //mask
#define GENERAL_R  0b00010000
#define GENERAL_W  0b00010001
// #define ARM_R      0b01000000
#define ARM_W      0b01000001

#define DATA_STATE 0b00000010 //mask
#define DATA_READY 0b00000010
#define DATA_EMPTY 0b00000000

#define BUFR_STATE 0b00000100 //mask
#define BUFR_FULL  0b00000100
#define BUFR_READY 0b00000000

#define ERR_STATE  0b00001000 //mask
#define ERROR      0b00001000
#define NO_ERROR   0b00000000 

//states so the MRCP can know what it's doing when the interrupt comes again
#define SEND 0
#define RECEIVE 1
#define SEND_RECEIVE 2

//function prototypes
void HandleFirstByte(void);
void HandleMiddleBytes(void);
void HandleLastByte(void);
unsigned char CalculateStatus(void);
#endif

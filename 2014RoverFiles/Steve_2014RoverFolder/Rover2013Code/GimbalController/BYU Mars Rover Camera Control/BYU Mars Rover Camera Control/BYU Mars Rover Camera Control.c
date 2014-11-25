/**************************************************************************************************
BYU Mars Rover Camera Control.c		Created By: Daniel Hearn		27 Dec 2011

This is the code for the camera control board.  It functions as follows:

It is an SPI slave, I2C master, UART, and servo controller. Commands come in over the SPI bus, 
they are checked for errors and loaded into a buffer.  The main loop parses the commands in the
order they were received, and generates responses which are loaded into an output buffer.  The
SPI slave returns data as it is made available.  The I2C master is responsible for controlling the
on board compass.  The UART is used to control the camera.  The two 16-bit timers are used to
control two servos.  Most of the functions are interrupt based, and leave little for the main loop
to do.
**************************************************************************************************/
#include <avr/io.h>
#include <avr/interrupt.h>
#include "BYU Mars Rover Camera Control.h"
#include <util/delay.h>

/**************************************************************************************************
Global variable list
**************************************************************************************************/
//servos
unsigned int servo1_offset = 450;
unsigned int servo2_offset = 500;

static unsigned char servoState = 0;

//input and output SPI buffers
struct RingBufferData commandBufferData;
struct RingBufferData outputBufferData;
RingBuffer commandBuffer = &commandBufferData;
RingBuffer outputBuffer = &outputBufferData;

//arrays for loading and unloading commands to and from the buffers
unsigned char incomingCommand[PACKET_SIZE];
unsigned char outgoingData[PACKET_SIZE];
int ci = 0; //incomingCommand index
int oi = 1; //outgoingData index

//status variables related to the Mars Rover SPI communication protocol
unsigned char checksumResult = 0;  //running checksum for incoming commands
unsigned char status = 0; 
int spiState;
int error = 0; //error condition - keeps error light flashing if set to 1 and FlashErrorLight() is called


/**************************************************************************************************
Main Loop - initializes everything, executes received commands, loads outgoing data buffer
**************************************************************************************************/
int main(void)
{
	//Initialize all faculties needed for operation
	Initialize_SPI_Slave(MSB);
	Initialize_USART(BAUD_PRESCALE);
	Initialize_TWI_Master();
	RingBufferInit(commandBuffer);
	RingBufferInit(outputBuffer);
	
	//Port I/O settings
	DDRC |= (1 << 0); //PORTC0 is output - 5v normally
	DDRC |= (1 << 3); //PORTC3 is output - Error Light
	DDRD |= 0b01100000; //PORTD5 and D6 output - Servos 1 and 2
	PORTD |= 0b01100000;
	
	//Timer 1 settings
	//Phase and Frequency Correct
	//Freq = 40Hz
	//TOP = ICR1 = 25000
	ICR1 = 25000;
	TCCR1A = 0b00000000;
	TCCR1B |=  (1 << CS11); // Set timer 1 prescaler to /8     (1 << WGM13) |
	TIMSK1 |= (1 << OCIE1A); //(1 << OCIE1B) | 
	
	TCCR0A = (1<<COM0A1)|(1<<COM0B1);
	TCCR0B = (1<<CS02);
	
	//Mars Rover com settings
	status = (DATA_EMPTY | BUFR_READY | NO_ERROR);
	SPDR = status; //initial state of machine 
	
	sei(); //enable interrupts

	//Variables for the main loop
	unsigned char header = 0; //used to store the header of the current command
	unsigned char byte = 0;
	unsigned char outgoingChecksum = 0;

	int i; //generic index
	
	//Main loop
	while(1){ //Execute commands as they come in		
		outgoingChecksum = 0;//reset the checksum
		
		if (RingBufferFillCount(commandBuffer) >= PACKET_SIZE){ //if we have a command in the buffer

			RingBufferDequeue(commandBuffer, &header); //get the first byte (header) out
			header &= DEVICE_RW; //only look at device ID and whether its read/write

			switch (header){
				case GENERAL_R: //reserved for later use - a read command issued directly to the uC
					RingBufferMultDelete(commandBuffer, 23); //delete the rest of the command
					break;
					
				case GENERAL_W: //reserved for later use - a write command issued directly to the uC
					RingBufferMultDelete(commandBuffer, 23); //delete the rest of the command
					break;
					
				case CAMERA_R: //read camera state - optical zoom level, digital zoom level, (what else?)
					RingBufferMultDelete(commandBuffer, 23); //delete the rest of the command
					break;
					
				case CAMERA_W: //passes a command from the base station to the camera
					for (i = 0; i < 22; i++){ //transmit all 22 data bytes to the camera
						RingBufferDequeue(commandBuffer, &byte);
						USART_Transmit(byte);
					}
					RingBufferMultDelete(commandBuffer, 1); //throw last byte away - its the checksum
					break;
					
				case COMPASS_R: //reads all values in from the compass
					CompassRead(&outgoingChecksum);
					break;
					
				case COMPASS_W: //writes a command directly to the compass
					RingBufferMultDelete(commandBuffer, 23); //throw last byte away - its the checksum
					break;
					
				case SERVO_R: //returns current offset of the servos
					ServoRead(&outgoingChecksum);
					break;
					
				case SERVO_W:
					ServoWrite();
					break;
					
				case PANTILT_W:
					PanTilt();
					break;
					
				default: //error state - device ID wasn't valid
					RingBufferMultDelete(commandBuffer, 23); //delete the rest of the bad command
					//FlashErrorLight(8); //turn on error light
			}		
		}
	}
}

/**************************************************************************************************
Timer 1 Comparator Interrupt vector - this vector triggers when the background timer reaches a
predetermined value (stored in 'OCR1A').  This vector is responsible for starting a new pulse on
both servos every 20ms (when the timer reaches 40000). OCR1A is the set to the offset of servo1 
+ 2000. When timer 1 reaches that offset (stored in 'servo1_offset'), this vector brings servo 1's
pulse low and waits for the timer to reach 40000 again so it can set both servo pulses high.
**************************************************************************************************/
ISR(TIMER1_COMPA_vect) {
	TCNT0 = 0;
	PORTD |= 0b01100000;
	
}

// ISR(TIMER0_COMPA_vect){
	
// }

// ISR(TIMER0_COMPB_vect){
// }

/**************************************************************************************************
SPI Interrupt vector - this vector triggers when a byte transmission is complete.  It is being used
to implement the Mars Rover Com Protocol.  The vector uses the index variable 'ci' to keep track of
what byte of the 24 byte packet it is on.  The variable 'status' contains the current header to
transmit.  It is reevaluated at the end of a 24 byte transmission, and each time the master polls
the slave to see if data is available or if there is room for a command to be received.  The 
variable 'spiState' is set on the first byte so that the vector can remember what it was doing when
it comes back.
**************************************************************************************************/
ISR(SPI_STC_vect) {
	if (ci == 0) //first byte = status header
		HandleFirstByte();	
	else if (ci > 0 && ci < 23) //middle bytes = data	
		HandleMiddleBytes();
	else { //last byte = checksum
		HandleLastByte();		
	}									
}

/**************************************************************************************************
HandleFirstByte - this mess of code is called by the SPI Interrupt vector to figure out what to do
with the first byte (the header) of each transmission. One thing to remember is that once we've
gotten here, the first byte of data (status header) has already been sent.
**************************************************************************************************/
void HandleFirstByte(void) {	
	checksumResult = 0; //reset checksum result
	unsigned char input = status & (BUFR_STATE | DATA_STATE); 

	//if there was an error on data sent, the master must know to start the transmission over again and discard the 
	//header it received.  since that header was peeked out of the buffer it is not lost, and will be resent upon 
	//successful transmission of the previous packet.
	if ((SPDR & ERR_STATE) == ERROR){ //if the master reports an error
		if ((spiState == SEND_RECEIVE) || (spiState == SEND)){   //there was an error in the data we sent out
			status = outgoingData[0]; //get the previous header back out		
			status |= DATA_READY; //recalculate the status
			if(!RingBufferHasRoom(commandBuffer, PACKET_SIZE)){
				status |= BUFR_FULL;
			}
			SPDR = status; //the header will be resent next time
			oi = 1;
			ci = 0;
		}
		else{ //some other error - reset the Com Protocol
			oi = 1;
			ci = 0;
			status = CalculateStatus();
			SPDR = status;
		}					
	}
	//if the slave (us) reported an error at the end of the last transmission the master will know to resend the
	//command over again.  this means it will abort the transmission it just started and start over again.  Here
	//we discard the header we received since it was part of the aborted transmission.  then we reset the com
	//protocol to wait for the old header to be sent again.
	else if ((status & ERR_STATE) == ERROR){
		status = CalculateStatus();
		SPDR = status;
		oi = 1;
		ci = 0;
	}
	//if there was no error and there is data to send, dequeue all 24 bytes of data into 'outgoingData'.  
	//This takes care of the header that was only peeked at.  start the index oi at 1, because the header was already sent.
	//continue transmission as normal
	else if ((SPDR & DEVICE) == NO_OP) { //header received is a NO_OP		
		switch (input) {
			case (BUFR_READY | DATA_READY):
			case (BUFR_FULL | DATA_READY):	
				RingBufferDequeueArray(outputBuffer, outgoingData, PACKET_SIZE);
				SPDR = outgoingData[oi];
				ci++;
				oi++;
				spiState = SEND;				
				break;
			case (BUFR_FULL | DATA_EMPTY):
			case (BUFR_READY | DATA_EMPTY): //recalculate status for next time
				status = CalculateStatus();
				SPDR = status;
				break;
			default: //never gets here
				//FlashErrorLight(2); //turn on error light
				;	
		}
	}
	else { //header received is any other command
		switch (input) {
			case (BUFR_READY | DATA_READY):
				incomingCommand[ci] = SPDR; //put it in a temp array
				checksumResult ^= SPDR; //calculate checksum on the fly			
				RingBufferDequeueArray(outputBuffer, outgoingData, PACKET_SIZE);
				SPDR = outgoingData[oi];
				ci++;
				oi++;
				spiState = SEND_RECEIVE;
				break;
			case (BUFR_READY | DATA_EMPTY):
				incomingCommand[ci] = SPDR; //put it in a temp array
				checksumResult ^= SPDR; //calculate checksum on the fly
				SPDR = 0; //clear the SPDR
				ci++;
				spiState = RECEIVE;
				break;
			case (BUFR_FULL | DATA_READY):
			case (BUFR_FULL | DATA_EMPTY):
				status = CalculateStatus();	
				SPDR = status;
				break;
			default: //never gets here, if it does something is very wrong 
				//FlashErrorLight(3); //turn on error light
				;	
		}
	}		
}

/**************************************************************************************************
HandleMiddleBytes - this is called by the SPI Interrupt vector to handle bytes 2 - 23 of a 
transmission.  HandleFirstByte() determines the type of transmission and sets 'spiState' to one of
three possibilities - send, receive, send and receive simultaneously.  When bytes are not being
sent back, 'SPDR' (SPI data register) is cleared.  I.E. during a receive only, the master only gets
zeros back from the slave.  Received data is loaded into 'incomingCommand[]' and held. Each byte is
then sent out. After byte 23 the checksum for outgoing data is calculated using the status header 
sent in byte 1 and loaded into 'SPDR' for transmission as byte 24.
**************************************************************************************************/
void HandleMiddleBytes(void) {
	//depends on what we were last doing as stored in 'spiState'
	switch (spiState) {
		case SEND_RECEIVE:
			incomingCommand[ci] = SPDR; //load SPDR into the array
			checksumResult ^= SPDR; //calculate running checksum as we go		
			SPDR = outgoingData[oi];
			if(ci == 23) //if we're about to load the checksum
				SPDR ^= status; //XOR the final status in to calculate the final checksum
			break;
		case RECEIVE:
			incomingCommand[ci] = SPDR; //load SPDR into the array
			checksumResult ^= SPDR; //calculate checksum on the fly
			SPDR = 0; //clear the SPDR
			break;
		case SEND:
			SPDR = outgoingData[oi];
			if(ci == 23) //if we're about to load the checksum
				SPDR ^= status; //XOR the final status in to calculate the final checksum
			break;
		default: //unreachable
			//FlashErrorLight(4); //turn on error light
			;
	}
	ci++; //move on to next byte
	oi++;
}

/**************************************************************************************************
HandleLastByte - this is called on byte 24 by the SPI Interrupt vector.  For sent data it does
nothing. For received data it verifies the checksum and loads the data from 'incomingCommand' into 
the buffer. If the checksum fails, it discards the received data and sets the error flag in the 
status header of the next transmission.  It also gets the current state of the machine (data 
ready?, buffer full?, device address?) and creates the status header for the next transmission.
**************************************************************************************************/
void HandleLastByte(void){
	status = 0; //clear status for this past transmission
	
	//if we received a command verify the checksum and add it to the command buffer
	if ((spiState == SEND_RECEIVE) || (spiState == RECEIVE)) {
		incomingCommand[ci] = SPDR; //load SPDR into the array
		checksumResult ^= SPDR; //calculate checksum on the fly
		
		if (checksumResult == 0) //if result is 0, no error - add newly received command to buffer
			RingBufferEnqueueArray(commandBuffer, incomingCommand, PACKET_SIZE);
		else //else set the error flag so the master can resend
			status |= ERROR;
	}
	
	//check status, load SPDR with status header for when the next command comes
	status |= CalculateStatus();		
	SPDR = status;
	ci = 0; //move on to first byte	of next command
	oi = 1;
}

/**************************************************************************************************
CalculateStatus - looks at the current state of com protocol to see if there is data to send, if
the command buffer is full, it appends the header from the output buffer to it and returns it
**************************************************************************************************/
unsigned char CalculateStatus(void){
	unsigned char newStatus = 0;
	if (RingBufferFillCount(outputBuffer) >= PACKET_SIZE) {
		RingBufferPeek(outputBuffer, &newStatus); //copy the header out, but leave it in the buffer
		newStatus &= DEVICE; //we only want the device ID of the header
		newStatus |= DATA_READY;
	}
	if (!RingBufferHasRoom(commandBuffer, PACKET_SIZE)) {
		newStatus |= BUFR_FULL;
	}
	
	return newStatus;
}

/**************************************************************************************************
FlashErrorLight - flashes the LED on PORTC3 'numFlashes' times. Useful for debug purposes.
**************************************************************************************************/
void FlashErrorLight(int numFlashes){
	PORTC = 0;//clear the LED
	error = 1;
	int i;
	while(error){
		for (i = 0; i < numFlashes; i++){
			PORTC = (1<<PORTC3);
			_delay_ms(350);
			PORTC = 0;
			_delay_ms(350);
		}
		_delay_ms(650); //long delay at end		
	}
}

/**************************************************************************************************
ServoRead - called by the main loop to read the current offset of each servo and create a packet
of outgoing data to return.  Packet format is as follows - [Header][Servo1High][Servo1Low]
[Servo2High][Servo2Low][0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0][Checksum].  For the checksum,
everything is calculated except the header, because the header is modified before transmission.
**************************************************************************************************/
void ServoRead(unsigned char *checksum){
	unsigned char servo1High = (servo1_offset >> 8);
	unsigned char servo1Low = (servo1_offset);
	unsigned char servo2High = (servo2_offset >> 8);
	unsigned char servo2Low = (servo2_offset);
	
	while (RingBufferSpaceFree(outputBuffer) < PACKET_SIZE); //if the output buffer is full we have to wait for space to free up
					
	RingBufferEnqueue(outputBuffer, SERVO); //header
	*checksum = 0; //the header is not XORed into the checksum here, because it will be modified later
	RingBufferEnqueue(outputBuffer, servo1High);
	*checksum ^= servo1High;
	RingBufferEnqueue(outputBuffer, servo1Low);
	*checksum ^= servo1Low;
	RingBufferEnqueue(outputBuffer, servo2High);
	*checksum ^= servo2High;
	RingBufferEnqueue(outputBuffer, servo2Low);
	*checksum ^= servo2High;
	
	int i;				
	for (i = 0; i < 18; i++){ //pad the data to 22 data bytes
		RingBufferEnqueue(outputBuffer, 0);
		*checksum ^= 0; //I know it doesn't do anything...
	}
					
	RingBufferEnqueue(outputBuffer, *checksum); //byte 24 is the (incomplete) checksum
	RingBufferMultDelete(commandBuffer, 23); //all the other bytes in the command are don't cares - delete them				
}

/**************************************************************************************************
ServoWrite - this is called by the main loop to set the servos to new angles.  The new offsets are
checked for bounds and are discarded if they are out of bounds.
**************************************************************************************************/
void ServoWrite(void){
	unsigned char servo1High;
	unsigned char servo1Low;
	unsigned int servo1Temp;
	unsigned char servo2High;
	unsigned char servo2Low;
	unsigned int servo2Temp;
	
	RingBufferDequeue(commandBuffer, &servo1High); //the first 4 bytes are the servo offsets
	RingBufferDequeue(commandBuffer, &servo1Low);
	RingBufferDequeue(commandBuffer, &servo2High);
	RingBufferDequeue(commandBuffer, &servo2Low);				
	RingBufferMultDelete(commandBuffer, 19); //the next 19 bytes are don't-cares ->throw them away
					
	servo1Temp = ((servo1High << 8) | servo1Low); //concatenate the 2 bytes and set the new offset if in bounds
	servo2Temp = ((servo2High << 8) | servo2Low);
	if(servo1Temp <= 2000){
		servo1_offset = servo1Temp>>1;
		OCR0A = (1000 + servo1_offset)>>4; //set interrupt to trigger at servo offset
	}		
	if(servo2Temp <= 2000){
		servo2_offset = servo2Temp>>1;
		OCR0B = (1000 + servo2_offset)>>4; //set the interrupt to the offset every time in case it changes
	}							
}

/**************************************************************************************************
PanTilt - this is called by the main loop to increment the angles of the servos gradually
**************************************************************************************************/
void PanTilt(void){
	unsigned char servo1High;
	unsigned char servo1Low;
	int servo1Temp;
	unsigned char servo2High;
	unsigned char servo2Low;
	int servo2Temp;
	
	RingBufferDequeue(commandBuffer, &servo1High); //the first 4 bytes are the servo offsets
	RingBufferDequeue(commandBuffer, &servo1Low);
	RingBufferDequeue(commandBuffer, &servo2High);
	RingBufferDequeue(commandBuffer, &servo2Low);				
	RingBufferMultDelete(commandBuffer, 19); //the next 19 bytes are don't-cares ->throw them away
					
	servo1Temp = ((servo1High << 8) | servo1Low); //concatenate the 2 bytes and set the new offset if in bounds
	servo2Temp = ((servo2High << 8) | servo2Low);
	
	if(servo1_offset + servo1Temp <= 2000 && servo1_offset + servo1Temp >= 0){
		servo1_offset += servo1Temp;
		OCR0A = (1000 + servo1_offset)>>4; //set interrupt to trigger at servo offset
	}		
	if(servo2_offset + servo2Temp <= 2000 && servo2_offset + servo2Temp >= 0){
		servo2_offset += servo2Temp;
		OCR0B = (1000 + servo2_offset)>>4; //set the interrupt to the offset every time in case it changes
	}							
}

/**************************************************************************************************
CompassRead - Reads the heading data from the compass and constructs a return packet
**************************************************************************************************/
void CompassRead(unsigned char *checksum){
	char headingHigh, headingLow, pitchHigh, pitchLow, rollHigh, rollLow;
	
	TWI_Send_Start(START);
	TWI_Send_Data(HMC6343_WRITE_ADDR, MT_SLAW_ACK); //I2C address and set to write	
	TWI_Send_Data(HMC6343_HEADING_REG, MT_DATA_ACK); //send command to read heading data
	TWI_Send_Stop();
	
	_delay_us(60);
	
	TWI_Send_Start(START);
	TWI_Send_Data(HMC6343_READ_ADDR, MR_SLAR_ACK); //I2C address and set to read
	_delay_us(60);
	headingHigh = TWI_Receive_Data(MR_DATA_ACK);
	_delay_us(60);
	headingLow = TWI_Receive_Data(MR_DATA_ACK);
	_delay_us(60);
	pitchHigh = TWI_Receive_Data(MR_DATA_ACK);
	_delay_us(60);
	pitchLow = TWI_Receive_Data(MR_DATA_ACK);
	_delay_us(60);
	rollHigh = TWI_Receive_Data(MR_DATA_ACK);
	_delay_us(60);
	rollLow = TWI_Receive_Data(MR_DATA_NACK);
	TWI_Send_Stop();
	
	while (RingBufferSpaceFree(outputBuffer) < PACKET_SIZE); //if the output buffer is full we have to wait for space to free up
					
	RingBufferEnqueue(outputBuffer, COMPASS); //header
	*checksum = 0; //the header is not XORed into the checksum here, because it will be modified later
	RingBufferEnqueue(outputBuffer, headingHigh);
	*checksum ^= headingHigh;
	RingBufferEnqueue(outputBuffer, headingLow);
	*checksum ^= headingLow;
	RingBufferEnqueue(outputBuffer, pitchHigh);
	*checksum ^= pitchHigh;
	RingBufferEnqueue(outputBuffer, pitchLow);
	*checksum ^= pitchLow;
	RingBufferEnqueue(outputBuffer, rollHigh);
	*checksum ^= rollHigh;
	RingBufferEnqueue(outputBuffer, rollLow);
	*checksum ^= rollLow;
	
	int i;				
	for (i = 0; i < 16; i++){ //pad the data to 22 data bytes
		RingBufferEnqueue(outputBuffer, 0);
		*checksum ^= 0; //I know it doesn't do anything...
	}
					
	RingBufferEnqueue(outputBuffer, *checksum); //byte 24 is the (incomplete) checksum
	RingBufferMultDelete(commandBuffer, 23); //all the other bytes in the command are don't cares - delete them
}	
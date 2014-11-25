/**************************************************************************************************
BYU Mars Rover Arm Control		
Created By: Daniel Hearn(SPI), Andrew Norgrant(Controls), Jacob Young(Controls)		16 May 2012

**************************************************************************************************/
#include <avr/io.h>
#include <avr/interrupt.h>
#include "BYU_MRCP.h"
#include <util/delay.h>
#include "DualVNH5019MotorShieldMEGA.h"
#include "DynamixelSerial1.h"
#include "PID_v1.h"
#include <Servo.h>
 
/**************************************************************************************************
Global variable list
**************************************************************************************************/

//-----------------------------SPI Variables----------------------------------
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
unsigned char header = 0; //used to store the header of the current command
unsigned char ShoulderYawHigh = 0, ShoulderYawLow = 0, ShoulderPitchHigh = 0, ShoulderPitchLow = 0, ElbowHigh = 0, ElbowLow = 0,
              WristHigh = 0, WristLow = 0, GripperHigh = 0, GripperLow = 0;
unsigned char outgoingChecksum = 0;

//-----------------------------Controls Variables----------------------------------

// Device			Control Type		Pins&Devices		Feedback Connection		Assert Method
// ShoulderYaw		Servo(PWM)			11					-						ShoulderYaw.write(ShoulderYawSetpoint); 
// ShoulderPitch	PID->VNH5019		3(PWM), 7,8			A14						md.setM2Speed()
// Elbow			PID->VNH5019		5(PWM), 2,4			A15						md.setM1Speed()
// Wrist			Dynamixel(Serial)	Serial1 on Pin 18	-						Dynamixel.move(dynamixelID,WristSetpoint);
// Gripper			Servo(PWM)			9					-						Gripper.write(GripperSetpoint);


// What Neil needs to know in soldering stuff
// Shoulder Yaw		Servo PWM			11
// ShoulderPitch	Motor Driver		Motor 2 Wire Clamps, Pot goes on A14, Give POT 5v
// Elbow			Motor Driver		Motor 1 Wire Clamps, Pot goes on A15, Give POT 5v
// Wrist			Serial1 Tx			TX1 / 18
// Grabber			Servo PWM			9










double ShoulderYawSetpoint = 90, WristSetpoint = 0, GripperSetpoint = 0;
const int shoulderYawPin= 11, gripperPin = 9;
const int ShoulderPitchInputPin=A14;
const int elbowInputPin=A15;
const int dynamixelID=1;

// PID Constructors and Variables
double ShoulderPitchSetpoint, ShoulderSetpointMap, ShoulderPitchInput, ShoulderInputMap, ShoulderOutput, ShoulderKp = 5.0, ShoulderKi = 1.0, ShoulderKd = 0;
double ElbowSetpoint, ElbowInput, ElbowOutput, Kp_elbow = .25, Ki_elbow = .01, Kd_elbow = .05;


int ShoulderMidpoint = -100, ShoulderMaxAngle = 750, ShoulderMinAngle = 400, ShoulderPitchMaxOutput = 300;
int ElbowMidpoint = 380, ElbowMaxAngle = 440, ElbowMinAngle = -330, ElbowMaxOutput = 375;

PID ShoulderPID(&ShoulderPitchInput, &ShoulderOutput, &ShoulderPitchSetpoint, ShoulderKp, ShoulderKi, ShoulderKd, DIRECT, ShoulderPitchMaxOutput);
PID ElbowPID(&ElbowInput, &ElbowOutput, &ElbowSetpoint, Kp_elbow, Ki_elbow, Kd_elbow, DIRECT, ElbowMaxOutput);

Servo ShoulderYaw, Gripper;
DualVNH5019MotorShield md;

void setup() {
	Initialize_SPI_Slave(MSB);
	RingBufferInit(commandBuffer);
	RingBufferInit(outputBuffer);

	status = (DATA_EMPTY | BUFR_READY | NO_ERROR);
	SPDR = status;

	md.init();

	pinMode(ShoulderPitchInputPin, INPUT);
	pinMode(elbowInputPin, INPUT);

	ShoulderPitchInput = (analogRead(ShoulderPitchInputPin) - ShoulderMidpoint);
	ShoulderPitchSetpoint = ShoulderMinAngle;
	
	ElbowInput = (analogRead(elbowInputPin) - ElbowMidpoint);
	ElbowSetpoint = ElbowMidpoint + ElbowMinAngle;

	ShoulderPID.SetMode(AUTOMATIC);
	ElbowPID.SetMode(AUTOMATIC);

	ShoulderYaw.attach(shoulderYawPin);
	Dynamixel.begin(1000000);
	Gripper.attach(gripperPin);
	
	Serial.begin(115200); // *** -

	sei();
}


void loop() {
  //-----------------------------SPI code---------------------------------
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

      /* The packets are 24 bytes long.  Once you've gotten here you've identified who the packet is
      *  directed to and have eaten the first byte.  That leaves 23 bytes.  The last byte is the
      *  checksum and has already been verified by this point.  You will always have to delete it
      *  before leaving the case.  That leaves 22 bytes of data containing the raw pot values for
      *  each joint.  Each pot will take 2 bytes of data, for 5 joints, that will be 10 bytes.  You
      *  will need to dequeue the first 10 bytes of the command and save them to a useful location
      *  so you can update the joint angles with them. Then delete the the remaining 13 bytes from the
      *  buffer so that it will not interfere with later commands.
      */			
      case ARM_W: //ADD CODE HERE TO SET THE ARM JOINTS TO ANGLES
        RingBufferDequeue(commandBuffer, &ShoulderYawHigh);
        RingBufferDequeue(commandBuffer, &ShoulderYawLow);
        ShoulderYawSetpoint = (ShoulderYawHigh << 8) | ShoulderYawLow;
        RingBufferDequeue(commandBuffer, &ShoulderPitchHigh);
        RingBufferDequeue(commandBuffer, &ShoulderPitchLow);
        ShoulderPitchSetpoint = (ShoulderPitchHigh << 8) | ShoulderPitchLow;
        ShoulderPitchSetpoint = ShoulderPitchSetpoint - ShoulderMidpoint;
        if(ShoulderPitchSetpoint > ShoulderMaxAngle){
          ShoulderPitchSetpoint = ShoulderMaxAngle;
        }
        if(ShoulderPitchSetpoint < ShoulderMinAngle){
          ShoulderPitchSetpoint = ShoulderMinAngle;
        }
        RingBufferDequeue(commandBuffer, &ElbowHigh);
        RingBufferDequeue(commandBuffer, &ElbowLow);
        ElbowSetpoint = (ElbowHigh << 8) | ElbowLow;
        ElbowSetpoint = (ElbowSetpoint - ElbowMidpoint);
        if (ElbowSetpoint > ElbowMaxAngle){
            ElbowSetpoint = ElbowMaxAngle;
        }
        if (ElbowSetpoint < ElbowMinAngle){
            ElbowSetpoint = ElbowMinAngle;
        }
        RingBufferDequeue(commandBuffer, &WristHigh);
        RingBufferDequeue(commandBuffer, &WristLow);
        WristSetpoint = (WristHigh << 8) | WristLow;
        RingBufferDequeue(commandBuffer, &GripperHigh);
        RingBufferDequeue(commandBuffer, &GripperLow);
        GripperSetpoint = (GripperHigh << 8) | GripperLow;
        
        RingBufferMultDelete(commandBuffer, 13); //delete the rest of the command
        break;
										
      default: //error state - device ID wasn't valid
	RingBufferMultDelete(commandBuffer, 23); //delete the rest of the bad command
    }		
  }
  
	//-----------------------------PID-related code---------------------------------
	const int numReadings = 32;
	int shoulderTotal = 0;
	int shoulderAverage = 0;
	int elbowTotal = 0;
	int elbowAverage = 0;
	for(int index = 0; index < numReadings; index++){
		shoulderTotal += analogRead(ShoulderPitchInputPin);
		elbowTotal += analogRead(elbowInputPin);
	}
	
	shoulderAverage = shoulderTotal/numReadings;
	elbowAverage = elbowTotal/numReadings;
	
	ShoulderPitchInput = (shoulderAverage - ShoulderMidpoint);
	ShoulderPID.Compute();
	ShoulderOutput = 200; // *** -
	md.setM2Speed(ShoulderOutput); // [-400:400]
	stopIfFault();

	ElbowInput = (elbowAverage - ElbowMidpoint);
	ElbowPID.ComputeElbow();
	ElbowOutput = 200; // *** -
	if(ElbowOutput <= 0) md.setM1Speed((ElbowOutput-30)); // [-400:400]
	if(ElbowOutput >= 0) md.setM1Speed((ElbowOutput+30));
	stopIfFault();
	
	// *** -
	ShoulderYawSetpoint = 90;
	GripperSetpoint = 90;
	WristSetpoint = 90;
	
	ShoulderYaw.write(ShoulderYawSetpoint);		// [0:180]
	Gripper.write(GripperSetpoint);				// [0:180]
	Dynamixel.move(dynamixelID,WristSetpoint); 	// [0:1023]
	
	// *** -
	Serial.print("Yaw: ");
	Serial.print(ShoulderYawSetpoint);
	Serial.print("\tPitch: ");
	Serial.print(ShoulderOutput);
	Serial.print("\tElbow: ");
	Serial.print(ElbowOutput);
	Serial.print("\tWrist: ");
	Serial.print(WristSetpoint);
	Serial.print("\tGrip: ");
	Serial.println(GripperSetpoint);
}


void stopIfFault(){
  if (md.getM1Fault()){
    //Serial.println("M1 fault");
    // while(1);
  }
  if (md.getM2Fault()){
    //Serial.println("M2 fault");
    // while(1);
  }
}


//union {                // This Data structure lets
//  byte asBytes[24];    // us take the byte array
//  float asFloat[6];    // sent from processing and
//}                      // easily convert it to a
//foo;                   // float array
//
//
//
//// getting float values from processing into the arduino
//// was no small task.  the way this program does it is
//// as follows:
////  * a float takes up 4 bytes.  in processing, convert
////    the array of floats we want to send, into an array
////    of bytes.
////  * send the bytes to the arduino
////  * use a data structure known as a union to convert
////    the array of bytes back into an array of floats
//
////  the bytes coming from the arduino follow the following
////  format:
////  0: 0=Manual, 1=Auto, else = ? error ?
////  1: 0=Direct, 1=Reverse, else = ? error ?
////  2-5: float setpoint
////  6-9: float input
////  10-13: float output  
////  14-17: float P_Param
////  18-21: float I_Param
////  22-245: float D_Param
//void SerialReceive()
//{
//
//  // read the bytes sent from Processing
//  int index=0;
//  byte Auto_Man = -1;
//  byte Direct_Reverse = -1;
//  while(Serial.available()&&index<26)
//  {
//    if(index==0) Auto_Man = Serial.read();
//    else if(index==1) Direct_Reverse = Serial.read();
//    else foo.asBytes[index-2] = Serial.read();
//    index++;
//  } 
//  
//  // if the information we got was in the correct format, 
//  // read it into the system
//  if(index==26  && (Auto_Man==0 || Auto_Man==1)&& (Direct_Reverse==0 || Direct_Reverse==1))
//  {
//    ShoulderPitchSetpoint=double(foo.asFloat[0]);
//    //Input=double(foo.asFloat[1]);       // * the user has the ability to send the 
//                                          //   value of "Input"  in most cases (as 
//                                          //   in this one) this is not needed.
//    if(Auto_Man==0)                       // * only change the output if we are in 
//    {                                     //   manual mode.  otherwise we'll get an
//      ShoulderOutput=double(foo.asFloat[2]);      //   output blip, then the controller will 
//    }                                     //   overwrite.
//    
//    double Shoulderp, Shoulderi, Shoulderd;                       // * read in and set the controller tunings
//    Shoulderp = double(foo.asFloat[3]);           //
//    Shoulderi = double(foo.asFloat[4]);           //
//    Shoulderd = double(foo.asFloat[5]);           //
//    ShoulderPID.SetTunings(Shoulderp, Shoulderi, Shoulderd);            //
//    
//    if(Auto_Man==0) ShoulderPID.SetMode(MANUAL);// * set the controller mode
//    else ShoulderPID.SetMode(AUTOMATIC);             //
//    
//    if(Direct_Reverse==0) ShoulderPID.SetControllerDirection(DIRECT);// * set the controller Direction
//    else ShoulderPID.SetControllerDirection(REVERSE);          //
//  }
//  Serial.flush();                         // * clear any random data from the serial buffer
//}
//
//// unlike our tiny microprocessor, the processing ap
//// has no problem converting strings into floats, so
//// we can just send strings.  much easier than getting
//// floats from processing to here no?
//void SerialSend()
//{
//  Serial.print("PID ");
//  ShoulderSetpointMap = map(ShoulderPitchSetpoint, 400, 800, 0, 85);
//  Serial.print(ShoulderSetpointMap);   
//  Serial.print(" ");
//  ShoulderInputMap = map(ShoulderPitchInput, 400, 800, 0, 85);
//  Serial.print(ShoulderInputMap);   
//  Serial.print(" ");
//  Serial.print(ShoulderOutput);   
//  Serial.print(" ");
//  Serial.print(ShoulderPID.GetKp());   
//  Serial.print(" ");
//  Serial.print(ShoulderPID.GetKi());   
//  Serial.print(" ");
//  Serial.print(ShoulderPID.GetKd());   
//  Serial.print(" ");
//  if(ShoulderPID.GetMode()==AUTOMATIC) Serial.print("Automatic");
//  else Serial.print("Manual");  
//  Serial.print(" ");
//  if(ShoulderPID.GetDirection()==DIRECT) Serial.println("Direct");
//  else Serial.println("Reverse");
//}


//ONLY THE BRAVE VENTURE BEYOND HERE.  DON'T CHANGE ANYTHING BELOW THIS POINT!!

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

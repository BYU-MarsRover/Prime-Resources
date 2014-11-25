/*******************************************************************
BYTE  VARIABLE
0      LeftMotors_H
1      LeftMotors_L
2      RightMotors_H
3      RightMotors_L
4      Package1_H
5      Package1_L
6      Package2_H
7      Package2_L
8      Package3_H
9      Package3_L
10     Package4_H
11     Package4_L
12     GimbalPan_H
13     GimbalPan_L
14     GimbalTilt_H
15     GimbalTilt_L
['H' XX XX XX XX XX XX XX XX XX XX XX XX XX XX XX XX 'L']
*******************************************************************/ 


/***********************************************************************
Incudes
/***********************************************************************/
#include <Servo.h>

#define HEADER_HIGH 0xFA
#define HEADER_LOW 0xDE

/***********************************************************************
Globals
/***********************************************************************/
int APM_Select = 4; //PG5: DIO 4
//int OSD_Select0 = 1;
//int OSD_Select1 = 2;

char receiveBuffer [17];

Servo LeftMotors; //PE4: APM 8: DIO 2
Servo RightMotors; //PE5: APM 7: DIO 3
Servo Package1; //PH4: APM 6: DIO 7
Servo Package2; //PH5: APM 5: DIO 8
Servo Package3; //PB7: APM 4: DIO 13
Servo Package4; //PB6: APM 3: DIO 12
Servo GimbalPan; //PL5: APM 2: DIO 44
Servo GimbalTilt; //PL4: APM 1: DIO 45


void setup() {
  // initialize serial communication with the XBEE radio
  Serial3.begin(57600);
  
  
  
  
  //attach servos outputs
  LeftMotors.attach(2);
  RightMotors.attach(3);
  Package1.attach(7);
  Package2.attach(8);
  Package3.attach(13);
  Package4.attach(12);
  GimbalPan.attach(44);
  GimbalTilt.attach(45);
  
  //set APM mux high to get all servo outputs
  pinMode(APM_Select, OUTPUT);
  digitalWrite(APM_Select, HIGH);
  
  //set OSD mode
  //add this later
  
  //set servos to default position
  LeftMotors.write(90);
  RightMotors.write(90);
  Package1.write(90);
  Package2.write(90);
  Package3.write(90);
  Package4.write(90);
  GimbalPan.write(90);
  GimbalTilt.write(90);
}


void loop() {
char current = 0;
  Serial3.println("waiting");  

  while(Serial3.available()){
    current = (char)Serial3.read();
	Serial3.println("searching");

	if (current == 'H'){

	  Serial3.println("Starting");
	  Serial3.readBytes(receiveBuffer, 17);
	  
	  if(receiveBuffer[16] == 'L'){
	      unsigned int LeftMotors_val = (receiveBuffer[0] << 8) & receiveBuffer[1];
		  unsigned int RightMotors_val = (receiveBuffer[2] << 8) & receiveBuffer[3];
		  unsigned int Package1_val = (receiveBuffer[4] << 8) & receiveBuffer[5];
		  unsigned int Package2_val = (receiveBuffer[6] << 8) & receiveBuffer[7];
		  unsigned int Package3_val = (receiveBuffer[8] << 8) & receiveBuffer[9];
		  unsigned int Package4_val = (receiveBuffer[10] << 8) & receiveBuffer[11];
		  unsigned int GimbalPan_val = (receiveBuffer[12] << 8) & receiveBuffer[13];
		  unsigned int GimbalTilt_val = (receiveBuffer[14] << 8) & receiveBuffer[15];
		  
		  // LeftMotors.writeMicroseconds(map(LeftMotors_val,0,2000,1000,2000));
		  // RightMotors.writeMicroseconds(map(RightMotors_val,0,2000,1000,2000));
		  // Package1.writeMicroseconds(map(Package1_val,0,2000,1000,2000));
		  // Package2.writeMicroseconds(map(Package2_val,0,2000,1000,2000));
		  // Package3.writeMicroseconds(map(Package3_val,0,2000,1000,2000));
		  // Package4.writeMicroseconds(map(Package4_val,0,2000,1000,2000));
		  // GimbalPan.writeMicroseconds(map(GimbalPan_val,0,2000,1000,2000));
		  // GimbalTilt.writeMicroseconds(map(GimbalTilt_val,0,2000,1000,2000));
		  
		  // LeftMotors.writeMicroseconds(LeftMotors_val + 1000);
		  // RightMotors.writeMicroseconds(RightMotors_val + 1000);
		  // Package1.writeMicroseconds(Package1_val + 1000);
		  // Package2.writeMicroseconds(Package2_val + 1000);
		  // Package3.writeMicroseconds(Package3_val + 1000);
		  // Package4.writeMicroseconds(Package4_val + 1000);
		  // GimbalPan.writeMicroseconds(GimbalPan_val + 1000);
		  // GimbalTilt.writeMicroseconds(GimbalTilt_val + 1000);
		  
		  LeftMotors.write(map(LeftMotors_val,0,2000,45,160));
		  RightMotors.write(map(RightMotors_val,0,2000,45,160));
		  Package1.write(map(Package1_val,0,2000,45,160));
		  Package2.write(map(Package2_val,0,2000,45,160));
		  Package3.write(map(Package3_val,0,2000,45,160));
		  Package4.write(map(Package4_val,0,2000,45,160));
		  GimbalPan.write(map(GimbalPan_val,0,2000,45,160));
		  GimbalTilt.write(map(GimbalTilt_val,0,2000,45,160));
	}
  } 
}
}





#include <DynamixelSerial1.h> 

const int uartTx2Pin = 16;

void setup(){
	// pinMode(uartTxPin,INPUT);
	Serial.begin(115200);
	Dynamixel.begin(1000000,uartTx2Pin);
	delay(1000);
}

void loop(){
	delay(20);
	static uint8_t addr = 0;
	
	addr = (addr >= 0xFF)? 0 : addr +1;
	
	uint8_t response = Dynamixel.ping(addr);
	//Serial.print("Response ");
	if( (response != 0xFF) || (addr == 0x00) ){
		Serial.print(addr);
		Serial.print(" = ");
		Serial.print(response);
		Serial.print("\r\n");
	}
}

// rf22_client.pde
// -*- mode: C++ -*-
// Example sketch showing how to create a simple messageing client
// with the RF22 class. RF22 class does not provide for addressing or reliability.
// It is designed to work with the other example rf22_server
#include <SPI.h>
#include <RF22.h>


#define HIGH			1
#define LOW				0

// Port Definitions and Macros
typedef struct{
  unsigned int bit0:1;
  unsigned int bit1:1;
  unsigned int bit2:1;
  unsigned int bit3:1;
  unsigned int bit4:1;
  unsigned int bit5:1;
  unsigned int bit6:1;
  unsigned int bit7:1;
} _io_reg; 
#define REGISTER_BIT(rg,bt) ((volatile _io_reg*)&rg)->bit##bt

#define LED_OR		REGISTER_BIT(PORTB,0)
#define LED_BL		REGISTER_BIT(PORTB,1)




// Singleton instance of the radio
RF22 rf22;
void setup(){
	Serial.begin(115200);
	if (!rf22.init()){
		Serial.println("RF22 init failed");
		LED_BL = HIGH;
	}
	// Defaults after init are 434.0MHz, 0.05MHz AFC pull-in, modulation FSK_Rb2_4Fd36
}
void loop(){
	static int lastTime = 0;
	while (1){
		if((lastTime+1000) > millis()){
			lastTime = millis();
			flashOrangeLED(2,10,20);
			Serial.print("Sending @ ");
			Serial.print(millis());
			Serial.println("\n");
			uint8_t data[] = "0123456789abcdef012345678";
			rf22.send(data, sizeof(data));
			//rf22.waitPacketSent();
		}
	}
}

void flashOrangeLED(uint8_t count, uint8_t high, uint8_t low){
	for(;count>0; count--){
		LED_OR = HIGH;
		delay(high);
		LED_OR = LOW;
		delay(low);
	}
}

void flashBlueLED(uint8_t count, uint8_t high, uint8_t low){
	for(;count>0; count--){
		LED_BL = HIGH;
		delay(high);
		LED_BL = LOW;
		delay(low);
	}
}
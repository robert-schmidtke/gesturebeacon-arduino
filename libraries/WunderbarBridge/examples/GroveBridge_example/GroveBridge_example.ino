/* 
	*** Simple example that shows how to use the Wunderbar Bridge module
    *** to connect an Ardino to the relayr open sensor cloud.

    The application wait for receiving data from the Bridge and shows the
    received value in a Grove Module
 */

#include <SoftwareSerial.h>
#include <WunderbarBridge.h>
#include <LED_Bar.h>

#define BRIDGE_DEBUG true

const int DEBUG_TX = 11;
const int DEBUG_RX = 10;

const int LED_BAR_CLK = 13;
const int LED_BAR_DI = 12;

/*  Config the bridge module, the 3rd parameter is the baudrate,
    which has to be the same used in the PC serial monitor application*/
Bridge bridge = Bridge(DEBUG_RX, DEBUG_TX, 115200); 

/*  Example of using the Grove LED_BAR module to show the received value 
	from the "open sensor cloud".
*/
LED_Bar bar(LED_BAR_CLK, LED_BAR_DI);   // config IOs here, (clk, dio)

const int led = 7;
const int button = 2;

void setup() {
	pinMode(led, OUTPUT);
	pinMode(button, INPUT);
	  
	bridge.begin();

	bar.setLevel(0);
}

/* Main Loop */
void loop() {
	 
	static uint8_t dataOut[2] = {1, 2};
	static bridge_payload_t rxPayload;

	if (bridge.newData == true){
		
		rxPayload = bridge.getData();
	
		digitalWrite(led, HIGH);

		/* Use the payload[1] to set the LED_BAR level */
		bar.setLevel(rxPayload.payload[1]);	

    	delay(1000);

    	digitalWrite(led, LOW);
	}

	
	/* On a button press Send a fixed test packet */
	if (digitalRead(button))
	{
		bridge.sendData(dataOut, 2);

		digitalWrite(led, HIGH);
		delay(1000);
		digitalWrite(led, LOW);
	}

}

/* the serialEvent() handler is called on every received data 
from the serial port. */
void serialEvent(){
	bridge.processSerial();
}





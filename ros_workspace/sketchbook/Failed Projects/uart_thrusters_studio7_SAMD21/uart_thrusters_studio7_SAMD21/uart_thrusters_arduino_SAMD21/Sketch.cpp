/*Begining of Auto generated code by Atmel studio */
#include <Arduino.h>

/*End of auto generated code by Atmel studio */

#include <Servo.h>
//Beginning of Auto generated function prototypes by Atmel Studio
void SerialUSBEvent();
//End of Auto generated function prototypes by Atmel Studio



int thrusters[8] = {1500, 1500 ,1500, 1500, 1500 ,1500, 1500, 1500};

const int numValues = 8; //number of values separated by spaces in packet being sent down
int packetInput[numValues] = {0, 0, 0, 0, 0, 0, 0, 0};

//for SerialUSB event
String packet = "";         // a String to hold incoming data
boolean packetComplete = false;  // whether the string is complete

int power(int base, int exponent){
	int value = 1;
	for(int i = 0; i < exponent; i++){
		value*=base;
	}
	return value;
}

inline boolean receivePacket() {
	
	int bitIndex = 0;
	int startValue = 0;
	int endValue = 0;
	
	for (int valueIndex=0; valueIndex < numValues; valueIndex++) {
		startValue = bitIndex;
		while(bitIndex < packet.length() && packet.charAt(bitIndex) != ' '){
			bitIndex++;
		}
		
		if(bitIndex <= packet.length()){
			
			endValue = bitIndex;
			int temp = 0;
			boolean negative = false;
			
			for(int i = startValue; i < endValue; i++){
				if(packet.charAt(i) >= 48 && packet.charAt(i) <= 57){;
					temp = temp + ((int)packet.charAt(i) - 48)*power(10, endValue - i - 1);
					} else if(packet.charAt(i) == '-' && i == startValue){
					negative = true;
					} else {
					return false;
				}
			}
			if(negative){
				temp*=-1;
			}
			SerialUSB.println(temp);
			packetInput[valueIndex] = 0;
			
			} else {
			return false; //if not all 8 value are read return false
		}
		bitIndex++;
	}
	return true;
}

void setup() {
  // initialize SerialUSB:
  SerialUSB.begin(115200);
  // reserve 100 bytes for the packet:
  packet.reserve(100);
}

void loop() {
  // print the string when a newline arrives:
  if (packetComplete) {
    //SerialUSBUSB.println(packet); 
    
    if(!receivePacket()){
      SerialUSB.println("Packet read failed! ");
    } else {
      for(int i = 0; i < numValues; i++){
        SerialUSB.print(packetInput[i] + ", ");
      }
      SerialUSB.println("\n--------------------------------");
    }
    
    // clear the string:
    packet = "";
    packetComplete = false;
  }
}


void SerialUSBEvent() {
  while (SerialUSB.available()) {
    // get the new byte:
    char inChar = (char)SerialUSB.read();
    // add it to the packet:
    packet += inChar;
    // if the incoming character is a newline, set a flag so the main loop can
    // do something about it:
    if (inChar == '\n') {
      packetComplete = true;
    }
  }
}

#include <Servo.h>

//For thrusters
int thrusters[8] = {1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500}; //value in microseconds between 1100 and 1900 to control thrusters 1-8
Servo T1, T2, T3, T4, T5, T6, T7, T8; //servo objects on pins 3, 5, 6, 9, 10, 11 respectively (328P can not handle more than 6 PWM, production plan is for an ATMSAME51 or ATSAMD21)

//For incomming data processing
const int numValues = 8; //number of values seperated by spaces in packet being sent down
int packetInput[numValues] = {0, 0, 0, 0, 0, 0, 0, 0}; //stores values after being parsed directly from the incomming packed string

//For serial event
String packet = "";         // a String to hold incoming data
boolean packetComplete = false;  // whether the string is complete

//For watchdog timer
unsigned long now = 0;
unsigned long lastPacket = 0;
boolean wdt_isTripped = false; //so  the timer is not tripped continuously 

//Function for converting thruster input (-1000 to 1000) to microseconds (1100 to 1900)
int calculateThrusterValues(int input){
  input = constrain(input, -1000, 1000);
  return map(input, -1000, 1000, 1100, 1900);
  }

void setThrusters(){ //update thrusters

  /*for(int i = 0; i < 8; i++){           //DEBUGGING
    Serial.print(thrusters[i]);         //DEBUGGING
    Serial.print(" ");                  //DEBUGGING
  }                                     //DEBUGGING
  Serial.println();                     //DEBUGGING*/  
  
  T1.writeMicroseconds(thrusters[0]);
  T2.writeMicroseconds(thrusters[1]);
  T3.writeMicroseconds(thrusters[2]);
  T4.writeMicroseconds(thrusters[3]);
  T5.writeMicroseconds(thrusters[4]);
  T6.writeMicroseconds(thrusters[5]);
  T7.writeMicroseconds(thrusters[6]);
  T8.writeMicroseconds(thrusters[7]);
  }


/*------------------------------Start of runtime program------------------------------*/
void setup() {
  // initialize serial to 115200 baud (SERIAL_8E1) 8 bit, even parity, 1 stop bit
  Serial.begin(115200);
  // reserve 100 bytes for the packet:
  packet.reserve(100);

  //Setup Servo pins
  T1.attach(2);
  T2.attach(3);
  T3.attach(4);
  T4.attach(5);
  T5.attach(6);
  T6.attach(7);
  T7.attach(8);
  T8.attach(9);

  //Set thrusters
  T1.writeMicroseconds(thrusters[0]);
  T2.writeMicroseconds(thrusters[1]);
  T3.writeMicroseconds(thrusters[2]);
  T4.writeMicroseconds(thrusters[3]);
  T5.writeMicroseconds(thrusters[4]);
  T6.writeMicroseconds(thrusters[5]);
  T7.writeMicroseconds(thrusters[6]);
  T8.writeMicroseconds(thrusters[7]);

  delay(1000); //delay to make sure thrusters get setup properly
  Serial.println("Thruster controller initialized");

  lastPacket = millis(); //start watchdog timer for the first packet
}

void loop() {

  now = millis();//get current time to  ensure connection to main contorller 

  if(wdt_isTripped || now - lastPacket > 500){ //If the contorller hasn't recived a new packet in half a second (short circuit limits calcs)
    for(int i = 0; i < numValues; i++){
      packetInput[i] = 0; //Set all values to neutral: 0
      if(i >= 0 && i < 8){ //If i is a thruster value
          thrusters[i] = calculateThrusterValues(packetInput[i]); //convert -1000 to 1000 from ROS to 1100 to 1900
      }
    }
    if(wdt_isTripped == false){
     Serial.println("Thruster controller watchdog tripped!");
     }
     wdt_isTripped = true;
     
   }
    
  // When a new packet arrives indicated by a newline '\n' char:
  if (packetComplete) {
    if(receivePacket()){ //check to make sure new packet is valid
      //If packet is valid
      
      for(int i = 0; i < numValues; i++){   
        
        if(i >= 0 && i < 8){ //If i is a thruster value
          thrusters[i] = calculateThrusterValues(packetInput[i]); //convert -1000 to 1000 from ROS to 1100 to 1900
          }
           
        //For more functionality to be added
          
        //Serial.print(packetInput[i]);       //DEBUGGING
        //Serial.print(" ");                  //DEBUGGING
      }                                      
      
      lastPacket = millis(); //Pet the watchdog timer
      wdt_isTripped = false;
      
    } else { //Packet invalid
      Serial.println("Thruster controller packet invalid!"); //Packet is invlaid
    }
    
    // clear the packet:
    packet = "";
    packetComplete = false;
  }

  setThrusters(); //update thrusters to new values

  delay(1);
}


void serialEvent() {
  while (Serial.available()) {
    // get the new byte:
    char inChar = (char)Serial.read();
    // add it to the packet:
    packet += inChar;
    // if the incoming character is a newline, set a flag so the main loop can
    // do something about it:
    if (inChar == '\n') {
      packetComplete = true;
    }
  }
}

//TCU Board Code ROS Version 1.0

#include <Arduino.h>
#include <Wire.h>

#define USE_USBCON //For Arduino DUE

#include <ros.h>
#include <tcu_board_msgs/tcu_board_data.h>
#include <std_msgs/Bool.h>
#include <std_msgs/ColorRGBA.h> //A = W for RGBW

//TCU board i2c bus needs to be switched!!! 
//Serial is on programming port not the native port

//https://learn.adafruit.com/adafruit-sht31-d-temperature-and-humidity-sensor-breakout/wiring-and-test
#include <Adafruit_SHT31.h>

//https://learn.adafruit.com/adafruit-ina219-current-sensor-breakout/arduino-code
#include <Adafruit_INA219.h>

//pins and hardware interface
#define LED 13

//12V switches
#define ROVpower 5 //to relay (optocoupler closest to cap)
#define mainSolenoid 3 //middle optocoupler 
#define extraOptoCoupler 2

ros::NodeHandle  nh;

String inputString = "";         // a String to hold incoming data
boolean stringComplete = false;  // whether the string is complete

Adafruit_SHT31 sht31 = Adafruit_SHT31();
float tempurature;
float humidity;

Adafruit_INA219 ina219;
float shuntvoltage;
float busvoltage;
float current_mA;
float loadvoltage;
float power_mW;

uint8_t neoPixelSetting; //1-3 determines neopixel routine (More to be added soon) OUTDATED
//New neopixel implementation comming soon

tcu_board_msgs::tcu_board_data msg;
ros::Publisher tcuPub("tcu/tcu_data", &msg);

//Subscriber callbacks
void ledCb(const std_msgs::ColorRGBA& npxSetting){
  //neoPixelSetting = npxMode.led_routine;
}

void mainRelayCb(const std_msgs::Bool& relayToggle){
  if(!relayToggle.data){
   digitalWrite(ROVpower, LOW);
   digitalWrite(LED, LOW);
  } 
  else{
    digitalWrite(ROVpower, HIGH);
    digitalWrite(LED, HIGH);
  }
}

void mainSolCb(const std_msgs::Bool& solenoidToggle){
  if(!solenoidToggle.data){
   digitalWrite(mainSolenoid, LOW);
  } 
  else{
    digitalWrite(mainSolenoid, HIGH);
  }
}

ros::Subscriber<std_msgs::ColorRGBA> ledSub("tcu/leds", ledCb);
ros::Subscriber<std_msgs::Bool> mainRelaySub("tcu/main_relay", mainRelayCb);
ros::Subscriber<std_msgs::Bool> mainSolSub("tcu/main_solenoid", mainSolCb);

void setup() { 

  //define pin directions 
  pinMode(LED, OUTPUT);
  pinMode(ROVpower, OUTPUT);
  pinMode(mainSolenoid, OUTPUT);
  pinMode(extraOptoCoupler, OUTPUT);

  //Init ROSserial functionality
  nh.initNode();
  nh.advertise(tcuPub);
  
  nh.subscribe(ledSub);
  nh.subscribe(mainRelaySub);
  nh.subscribe(mainSolSub);
 
  //Serial to ATMEGA328P for neopixels
  Serial1.begin(115200);
  Serial1.flush();
  
  //Initialize SHT-31D temp and humidity sensor
  if (!sht31.begin(0x45)) {   // Addr is 0x45 for i2c
    nh.logerror("Couldn't find SHT31!");
  }
  
  //Initialize INA219 temp and humidity sensor
  ina219.begin();

  nh.logdebug("Initialized...");
}

void loop() {

  //Update neopixels
  Serial1.println(neoPixelSetting); 
  
  //Serial from ATMEGA328P for neopixels
  if (stringComplete) {
    char logString[] = "NPX Controller Feedback Recieved!";
    nh.logdebug(logString);
    //clear the string:
    inputString = "";
    stringComplete = false;
    Serial1.flush();
  }

  //Read all the sensor data
  delay(1);
  tempurature = sht31.readTemperature();
  delay(1);
  humidity = sht31.readHumidity();
  delay(1);

  shuntvoltage = ina219.getShuntVoltage_mV();
  delay(1);
  busvoltage = ina219.getBusVoltage_V();
  delay(1);
  current_mA = ina219.getCurrent_mA();
  delay(1);
  power_mW = ina219.getPower_mW();
  
  loadvoltage = busvoltage + (shuntvoltage / 1000);

  //if load voltage makes no sense
  if(loadvoltage > 30){
    loadvoltage = NAN;
    shuntvoltage = NAN;
    busvoltage = NAN;
    current_mA = NAN;
    power_mW = NAN;
    }

  //Construct msg to send
  msg.tempC = tempurature;
  msg.humidity = humidity;
  msg.currentMA = current_mA;
  msg.voltage = loadvoltage;

  tcuPub.publish(&msg);

  nh.spinOnce();

  delay(100);
}

//Serial from ATMEGA328P for neopixels
void serialEvent1() {
  while (Serial1.available()) {
    // get the new byte:
    char inChar = (char)Serial1.read();
    // add it to the inputString:
    inputString += inChar;
    // if the incoming character is a newline, set a flag
    // so the main loop can do something about it:
    if (inChar == '\n') {
      stringComplete = true;
    }
  }
}

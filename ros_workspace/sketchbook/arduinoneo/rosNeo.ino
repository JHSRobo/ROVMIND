#include <ros.h>
#include <neopixels/flash.msg> 
//neopixels is the name of the package this .ino and flash.msg will be in
#include <Adafruit_NeoPixel.h>
#define NUM_LEDS 20
#define PIN 6
Adafruit_NeoPixel strip = Adafruit_NeoPixel(NUM_LEDS, PIN, NEO_GRBW + NEO_KHZ800);
ros::NodeHandle nh;


void messageCb(const neopixels::flash& statusChange)
{ 
  Strobe(statusChange.red, statusChange.green, statusChange.blue, statusChange.count, statusChange.wait);
}
ros::Subscriber<neopixels::flash> sub("/rov/neopixels", &messageCb );

void setup() 
{
strip.begin();
strip.show();
nh.initNode();
nh.subscribe(sub);
}

void loop() 
{
  nh.spinOnce();
}

void showStrip() {
 #ifdef ADAFRUIT_NEOPIXEL_H 
   // NeoPixel
   strip.show();
 #endif
 #ifndef ADAFRUIT_NEOPIXEL_H
   // FastLED
   FastLED.show();
 #endif
}

void setPixel(int Pixel, byte red, byte green, byte blue) {
 #ifdef ADAFRUIT_NEOPIXEL_H 
   // NeoPixel
   strip.setPixelColor(Pixel, strip.Color(red, green, blue));
 #endif
 #ifndef ADAFRUIT_NEOPIXEL_H 
   // FastLED
   leds[Pixel].r = red;
   leds[Pixel].g = green;
   leds[Pixel].b = blue;
 #endif
}

void setAll(byte red, byte green, byte blue) {
  for(int i = 0; i < NUM_LEDS; i++ ) {
    setPixel(i, red, green, blue); 
  }
  showStrip();
}

void Strobe(byte red, byte green, byte blue, int StrobeCount, int FlashDelay){
  for(int j = 0; j < StrobeCount; j++) {
    setAll(red,green,blue);
    showStrip();
    delay(FlashDelay);
    setAll(0,0,0);
    showStrip();
    delay(FlashDelay);
  }
 
 delay(1000);
}

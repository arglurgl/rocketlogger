#include <led.h>

#define NUMPIXELS  1 // Number of LEDs in strip
#define DATAPIN    8
#define CLOCKPIN   6
#define INITIAL_COLOR 0,16,0 //color after setup

Adafruit_DotStar rgbled(NUMPIXELS, DATAPIN, CLOCKPIN, DOTSTAR_BGR);

void setupLED(){  
  rgbled.begin();
  rgbled.setPixelColor(0, INITIAL_COLOR);
  rgbled.show();
}
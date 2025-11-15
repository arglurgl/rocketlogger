#include <led.h>

#define NUMPIXELS  1 // Number of LEDs in strip
#define DATAPIN    8
#define CLOCKPIN   6
#define INITIAL_COLOR 0,16,0 //color after setup
#define STANDBY_COLOR 0,0,255
#define ARM_COLOR 0,255,0
#define RECORD_COLOR 255,0,0


Adafruit_DotStar rgbled(NUMPIXELS, DATAPIN, CLOCKPIN, DOTSTAR_BGR);

/// @brief Intialize RGB LED and set to start color
void setupLED(){  
  rgbled.begin();
  rgbled.setPixelColor(0, INITIAL_COLOR);
  rgbled.show();
}

/// @brief Set LED color based on flight state
void updateLED(FlightState flightState){
  switch(flightState){
    case STANDBY:
      rgbled.setPixelColor(0, STANDBY_COLOR);
      break;
    case ARMED:
      rgbled.setPixelColor(0, ARM_COLOR);
      break;
    case RECORDING:
      rgbled.setPixelColor(0, RECORD_COLOR);
      break;
    default:
      rgbled.setPixelColor(0, INITIAL_COLOR);
      break;
  }
  rgbled.show();
}
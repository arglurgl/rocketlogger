//RGB LED
#ifndef _LED_H_GUARD
#define _LED_H_GUARD

#include <Adafruit_DotStar.h>
#include <flightstate.h>

extern Adafruit_DotStar rgbled;

void setupLED();
void updateLED(FlightState flightState);


#endif
#ifndef _PRESSURE_H_GUARD
#define _PRESSURE_H_GUARD

#include <Adafruit_BMP280.h>

#define BMP_CS A0 //chip select line

extern Adafruit_BMP280 pressureSensor;

void setupPressureSensor(float refHeight);
void calibratePressureSensor(float refHeight);
float relativeAltitude();
float sealevelAltitude();

#endif
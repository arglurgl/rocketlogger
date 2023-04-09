#include <pressure.h>

float sealevelPressure = 1023.8;
float referenceHeight = 50; // known height at point of calibration

Adafruit_BMP280 pressureSensor(BMP_CS); // hardware SPI

/// @brief Initialize and calibrate the pressure sensor. Blocks on error.
/// @param refHeight Current reference height in meters
void setupPressureSensor(float refHeight){
    if (!pressureSensor.begin()) {
    Serial.println(F("Could not find a valid BMP280 sensor, check wiring or "
                        "try a different address!"));
    Serial.print(F("SensorID was: 0x")); Serial.println(pressureSensor.sensorID(),16);
    Serial.print(F("        ID of 0xFF probably means a bad address, a BMP 180 or BMP 085\n"));
    Serial.print(F("   ID of 0x56-0x58 represents a BMP 280,\n"));
    Serial.print(F("        ID of 0x60 represents a BME 280.\n"));
    Serial.print(F("        ID of 0x61 represents a BME 680.\n"));
    while (1) delay(10);
    }

    // set settings
    pressureSensor.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                    Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                    Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                    Adafruit_BMP280::FILTER_OFF,      /* Filtering. */
                    Adafruit_BMP280::STANDBY_MS_1); /* Standby time. */

    //calibrate
    calibratePressureSensor(refHeight);
}

/// @brief Calibrate the sensor altitude readout using a known height. Also sets the reference/start height.
void calibratePressureSensor(float refHeight){
    //calibrate sealevel pressure using current pressure and known height
    float currPressure = pressureSensor.readPressure()/100.0;
    sealevelPressure = pressureSensor.seaLevelForAltitude(refHeight,currPressure);
    referenceHeight = refHeight;
}

/// @brief Get the height relative to the calibration point
/// @return Height above reference height in meters
float relativeAltitude(){
    float relHeight =(pressureSensor.readAltitude(sealevelPressure)-referenceHeight);
    return relHeight;
}

/// @brief Return the absolute calibrated height above sealevel
/// @return Absolute height in meters 
float sealevelAltitude(){
    return pressureSensor.readAltitude(sealevelPressure);
}
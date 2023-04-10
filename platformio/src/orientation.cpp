
//orientation/acceleration sensor BN0055
#include <orientation.h>
#include <Arduino.h>
#include <Adafruit_BNO055.h>

Adafruit_BNO055 orientationSensor = Adafruit_BNO055(55,0x29); // you may need to adapt the address

/// @brief Initialize and setup the BNO055 orientation/acceleration sensor. Blocks on error.
void setupOrientationSensor(){
    /* Initialise the sensor */
    if (!orientationSensor.begin())
    {
        /* There was a problem detecting the BNO055 ... check your connections */
        Serial.println("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
        while (1);
    }
    delay(1000);
    orientationSensor.setExtCrystalUse(true);
}
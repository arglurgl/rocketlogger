/*********************************************************************
 This is an example for our nRF52 based Bluefruit LE modules

 Pick one up today in the adafruit shop!

 Adafruit invests time and resources providing this open source code,
 please support Adafruit and open-source hardware by purchasing
 products from Adafruit!

 MIT license, check LICENSE for more information
 All text above, and the splash screen below must be included in
 any redistribution
*********************************************************************/
#include <Arduino.h>
#include <Adafruit_DotStar.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <utility/imumaths.h>
#include <packetCreator.h>
#include <commandline.h>
#include <flash.h>
#include <bluetooth.h>
#include <pressure.h>
#include <orientation.h>

#define INITIAL_REF_HEIGHT 50.0 // height to use in initial setup //TODO save in persistent memory

// Function prototypes for packetparser.cpp
uint8_t readPacket (BLEUart *ble_uart, uint16_t timeout);
float   parsefloat (uint8_t *buffer);
void    printHex   (const uint8_t * data, const uint32_t numBytes);

// Packet buffer
extern uint8_t packetbuffer_receive[];

//RGB LED
#define NUMPIXELS  1 // Number of LEDs in strip
#define DATAPIN    8
#define CLOCKPIN   6
Adafruit_DotStar rgbled(NUMPIXELS, DATAPIN, CLOCKPIN, DOTSTAR_BGR);

//for testing BLE data stream
bool ble_stream = false;

void printBMP280Values(){
    Serial.print(F("Temperature = "));
    Serial.print(pressureSensor.readTemperature());
    Serial.println(" *C");

    Serial.print(F("Pressure = "));
    Serial.print(pressureSensor.readPressure());
    Serial.println(" Pa");

    Serial.print(F("appr. relative height = "));
    float rel_height =relativeAltitude();
    Serial.print(rel_height, 1); /* Adjusted to local forecast! */
    Serial.println(" m");

    //LED for speed test
    if  (rel_height > 0) rgbled.setPixelColor(0,0,64,0);
    else rgbled.setPixelColor(0,64,0,0);
    rgbled.show();

    Serial.println();
}

void sendSensorValues(){
    sensors_event_t event; 

    //get the sensor data
    bno.getEvent(&event,Adafruit_BNO055::VECTOR_GRAVITY);
    float temp = pressureSensor.readTemperature();
    float press = pressureSensor.readPressure();
    float rel_height =relativeAltitude();

    Serial.print("Gravity ");
    Serial.print("X: ");
    Serial.print(event.acceleration.x, 4);
    Serial.print("\tY: ");
    Serial.print(event.acceleration.y, 4);
    Serial.print("\tZ: ");
    Serial.print(event.acceleration.z, 4);
    Serial.println("");
    
    Serial.print(F("Temperature = "));
    Serial.print(temp);
    Serial.println(" *C");

    Serial.print(F("Pressure = "));
    Serial.print(press);
    Serial.println(" Pa");

    Serial.print(F("appr. relative height = "));    
    Serial.print(rel_height, 1); 
    Serial.println(" m");

    // send the data
    sendVector3(&bleuart, PACKET_DATA_ACC_HEADER, event.acceleration.x, event.acceleration.y, event.acceleration.z);
    sendVector3(&bleuart, PACKET_DATA_ALT_HEADER, temp, press, rel_height);

}


void setup(void)
{
  Serial.begin(115200);
  
  // for nrf52840 with native usb, wait for Serial connection unless timeout happens
  uint8_t tries = 0;
  while ( (!Serial) && (tries < 200) ){
    delay(10);   
    tries++;
  }
  // flush any remaining incoming data from e.g. upload process
  while(Serial.available() > 0) Serial.read();

  Serial.println("Starting setup");

  Serial.print(F("  Flash/filesystem..."));
  //uncomment this next line and follow instructions on monitor to format a new device's flash, this will erase all data!
  //format_flash();
  setupFlash();
  Serial.println(F("OK"));
  //test_fatfs(); //TODO for testing, remove

  Serial.print(F("  Bluetooth..."));
  setupBluetooth();
  Serial.println(F("OK"));

  Serial.print(F("  RGB LED..."));
  rgbled.begin();
  rgbled.setPixelColor(0, 0, 16, 0);
  rgbled.show();
  Serial.println(F("OK"));

  Serial.print(F("  Pressure sensor..."));
  setupPressureSensor(INITIAL_REF_HEIGHT);
  Serial.println(F("OK"));

  Serial.print(F("  Orientation sensor..."));
  setupOrientationSensor();
  Serial.println(F("OK"));

  Serial.println("Setup done");  
}

void loop(void)
{
  getCommandChunk();
  doCommands();
  //poll pressure sensor and print
  //printBMP280Values();

  //test orientation sensor
  /* Get a new sensor event */ 
  // sensors_event_t event; 
  // bno.getEvent(&event);
  
  // /* Display the floating point data */
  // Serial.print("Orientation");
  // Serial.print("X: ");
  // Serial.print(event.orientation.x, 4);
  // Serial.print("\tY: ");
  // Serial.print(event.orientation.y, 4);
  // Serial.print("\tZ: ");
  // Serial.print(event.orientation.z, 4);
  // Serial.println("");

  // bno.getEvent(&event,Adafruit_BNO055::VECTOR_GRAVITY);
  // Serial.print("Gravity ");
  // Serial.print("X: ");
  // Serial.print(event.acceleration.x, 4);
  // Serial.print("\tY: ");
  // Serial.print(event.acceleration.y, 4);
  // Serial.print("\tZ: ");
  // Serial.print(event.acceleration.z, 4);
  // Serial.println("");

  //bno.getSystemStatus
  
  // delay(100);

  //LED for speed test
  float rel_height =relativeAltitude();
  if  (rel_height > 0) rgbled.setPixelColor(0,0,64,0);
  else rgbled.setPixelColor(0,64,0,0);
  rgbled.show();

  if (ble_stream) {
    sendSensorValues();
  }
  
  // Wait for new BLE data to arrive
  uint8_t len = readPacket(&bleuart, 10); // was 500 before speed test
  if (len == 0) return;

  // Got a packet!
  // printHex(packetbuffer_receive, len);

  // Task
  if (packetbuffer_receive[1] == 'T') {
    char task = packetbuffer_receive[2];
    char subtask = packetbuffer_receive[3];
    Serial.print ("Task "); Serial.print(task);
    Serial.print ("-"); Serial.println(subtask);
    if (task == 'S'){ // streaming
      if (subtask == 'B') ble_stream = true; //begin
      else if (subtask == 'E') ble_stream = false; //end
      else if (subtask == 'P') sendSensorValues(); //send single Point for each value (for polling mode)
    }
  }

  // Color
  if (packetbuffer_receive[1] == 'C') {
    uint8_t red = packetbuffer_receive[2];
    uint8_t green = packetbuffer_receive[3];
    uint8_t blue = packetbuffer_receive[4];

    //set led
    rgbled.setPixelColor(0,red,green,blue);
    rgbled.show();

    Serial.print ("RGB #");
    if (red < 0x10) Serial.print("0");
    Serial.print(red, HEX);
    if (green < 0x10) Serial.print("0");
    Serial.print(green, HEX);
    if (blue < 0x10) Serial.print("0");
    Serial.println(blue, HEX);

  }

  // Buttons
  if (packetbuffer_receive[1] == 'B') {
    uint8_t buttnum = packetbuffer_receive[2] - '0';
    boolean pressed = packetbuffer_receive[3] - '0';
    Serial.print ("Button "); Serial.print(buttnum);
    if (pressed) {
      Serial.println(" pressed");
    } else {
      Serial.println(" released");
    }
  }

  // GPS Location
  if (packetbuffer_receive[1] == 'L') {
    float lat, lon, alt;
    lat = parsefloat(packetbuffer_receive+2);
    lon = parsefloat(packetbuffer_receive+6);
    alt = parsefloat(packetbuffer_receive+10);
    Serial.print("GPS Location\t");
    Serial.print("Lat: "); Serial.print(lat, 4); // 4 digits of precision!
    Serial.print('\t');
    Serial.print("Lon: "); Serial.print(lon, 4); // 4 digits of precision!
    Serial.print('\t');
    Serial.print(alt, 4); Serial.println(" meters");
  }

  // Accelerometer
  if (packetbuffer_receive[1] == 'A') {
    float x, y, z;
    x = parsefloat(packetbuffer_receive+2);
    y = parsefloat(packetbuffer_receive+6);
    z = parsefloat(packetbuffer_receive+10);
    Serial.print("Accel\t");
    Serial.print(x); Serial.print('\t');
    Serial.print(y); Serial.print('\t');
    Serial.print(z); Serial.println();
  }

  // Magnetometer
  if (packetbuffer_receive[1] == 'M') {
    float x, y, z;
    x = parsefloat(packetbuffer_receive+2);
    y = parsefloat(packetbuffer_receive+6);
    z = parsefloat(packetbuffer_receive+10);
    Serial.print("Mag\t");
    Serial.print(x); Serial.print('\t');
    Serial.print(y); Serial.print('\t');
    Serial.print(z); Serial.println();
  }

  // Gyroscope
  if (packetbuffer_receive[1] == 'G') {
    float x, y, z;
    x = parsefloat(packetbuffer_receive+2);
    y = parsefloat(packetbuffer_receive+6);
    z = parsefloat(packetbuffer_receive+10);
    Serial.print("Gyro\t");
    Serial.print(x); Serial.print('\t');
    Serial.print(y); Serial.print('\t');
    Serial.print(z); Serial.println();
  }

  // Quaternions
  if (packetbuffer_receive[1] == 'Q') {
    float x, y, z, w;
    x = parsefloat(packetbuffer_receive+2);
    y = parsefloat(packetbuffer_receive+6);
    z = parsefloat(packetbuffer_receive+10);
    w = parsefloat(packetbuffer_receive+14);
    Serial.print("Quat\t");
    Serial.print(x); Serial.print('\t');
    Serial.print(y); Serial.print('\t');
    Serial.print(z); Serial.print('\t');
    Serial.print(w); Serial.println();
  }
}

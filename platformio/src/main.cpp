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
#define BLE_NAME "rocketlogger" 

#include <bluefruit.h>
#include <Arduino.h>
#include <Adafruit_DotStar.h>
#include <SPI.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <SdFat.h>
#include <Adafruit_SPIFlash.h>
#include <packetCreator.h>
#include <commandline.h>

// flash/filesystem
#include "flash_config.h" // for flashTransport definition
Adafruit_SPIFlash flash(&flashTransport);
FatVolume fatfs;
void format_flash(); // declare fomrat flash, defined in format_flash.cpp
#define F_TEST_TEST_TXT       "/test/test.txt"
#define D_TEST                "/test"

// OTA DFU service
BLEDfu bledfu;

// Uart over BLE service
BLEUart bleuart;

// Function prototypes for packetparser.cpp
uint8_t readPacket (BLEUart *ble_uart, uint16_t timeout);
float   parsefloat (uint8_t *buffer);
void    printHex   (const uint8_t * data, const uint32_t numBytes);

// Packet buffer
extern uint8_t packetbuffer_receive[];

// function declarations
void startAdv(void);

//RGB LED
#define NUMPIXELS  1 // Number of LEDs in strip
#define DATAPIN    8
#define CLOCKPIN   6
Adafruit_DotStar rgbled(NUMPIXELS, DATAPIN, CLOCKPIN, DOTSTAR_BGR);

//pressure sensor 
#define BMP_CS A0
float sealevel_pressure = 1023.8;
float current_height = 50;
Adafruit_BMP280 bmp(BMP_CS); // hardware SPI

//orientation/acceleration sensor BN0055
Adafruit_BNO055 bno = Adafruit_BNO055(55,0x29); // you may need to adapt the address

//for testing BLE data stream
bool ble_stream = false;

void printBMP280Values(){
    Serial.print(F("Temperature = "));
    Serial.print(bmp.readTemperature());
    Serial.println(" *C");

    Serial.print(F("Pressure = "));
    Serial.print(bmp.readPressure());
    Serial.println(" Pa");

    Serial.print(F("appr. relative height = "));
    float rel_height =(bmp.readAltitude(sealevel_pressure)-current_height);
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
    float temp = bmp.readTemperature();
    float press = bmp.readPressure();
    float rel_height =(bmp.readAltitude(sealevel_pressure)-current_height);

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

void test_fatfs(){
  // Check if a directory called 'test' exists and create it if not there.
  // Note you should _not_ add a trailing slash (like '/test/') to directory names!
  // You can use the same exists function to check for the existance of a file too.
  if (!fatfs.exists(D_TEST)) {
    Serial.println(F("Test directory not found, creating..."));
    
    // Use mkdir to create directory (note you should _not_ have a trailing slash).
    fatfs.mkdir(D_TEST);
    
    if ( !fatfs.exists(D_TEST) ) {
      Serial.println(F("Error, failed to create directory!"));
      while(1) yield();
    }else {
      Serial.println(F("Created directory!"));
    }
  }

  File32 writeFile = fatfs.open(F_TEST_TEST_TXT, FILE_WRITE);
  if (!writeFile) {
    Serial.println(F("Error, failed to open " F_TEST_TEST_TXT " for writing!"));
    while(1) yield();
  }
  Serial.println(F("Opened file " F_TEST_TEST_TXT " for writing/appending..."));

  // Once open for writing you can print to the file as if you're printing
  // to the serial terminal, the same functions are available.
  writeFile.println("Hello world!");
  writeFile.print("Hello number: "); writeFile.println(123, DEC);
  writeFile.print("Hello hex number: 0x"); writeFile.println(123, HEX);

  // Close the file when finished writing.
  writeFile.close();
  Serial.println(F("Wrote to file " F_TEST_TEST_TXT "!"));

  // Now open the same file but for reading.
  File32 readFile = fatfs.open(F_TEST_TEST_TXT, FILE_READ);
  if (!readFile) {
    Serial.println(F("Error, failed to open " F_TEST_TEST_TXT " for reading!"));
    while(1) yield();
  }

  // Read data using the same read, find, readString, etc. functions as when using
  // the serial class.  See SD library File class for more documentation:
  //   https://www.arduino.cc/en/reference/SD
  // Read a line of data:
  String line = readFile.readStringUntil('\n');
  Serial.print(F("First line of test.txt: ")); Serial.println(line);

  // You can get the current position, remaining data, and total size of the file:
  Serial.print(F("Total size of test.txt (bytes): ")); Serial.println(readFile.size(), DEC);
  Serial.print(F("Current position in test.txt: ")); Serial.println(readFile.position(), DEC);
  Serial.print(F("Available data to read in test.txt: ")); Serial.println(readFile.available(), DEC);

  // And a few other interesting attributes of a file:
  char readName[64];
  readFile.getName(readName, sizeof(readName));
  Serial.print(F("File name: ")); Serial.println(readName);
  Serial.print(F("Is file a directory? ")); Serial.println(readFile.isDirectory() ? F("Yes") : F("No"));

  // You can seek around inside the file relative to the start of the file.
  // For example to skip back to the start (position 0):
  if (!readFile.seek(0)) {
    Serial.println(F("Error, failed to seek back to start of file!"));
    while(1) yield();
  }

  // And finally to read all the data and print it out a character at a time
  // (stopping when end of file is reached):
  Serial.println(F("Entire contents of test.txt:"));
  while (readFile.available()) {
    char c = readFile.read();
    Serial.print(c);
  }

  // Close the file when finished reading.
  readFile.close();
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

  delay(5000); // add some time for platformio monitor to start and actually see setup messages

  //uncomment this next line to format a new device's flash, this will erase all data on the flash!
  //format_flash();
  // initialize 2MB external QSPI flash on the itsybitsy board
  Serial.println("Initializing flash...");
  flash.begin();
  Serial.print(F("JEDEC ID: 0x"));
  Serial.println(flash.getJEDECID(), HEX);
  Serial.print(F("Flash size: "));
  Serial.print(flash.size() / 1024);
  Serial.println(F(" KB"));
  if ( !fatfs.begin(&flash) ) {
  Serial.println(F("Error: filesystem is not existent on the flash device. Please run the SdFat_format example of the Adafruit flash library to make one."));
  while(1)
    {
      yield();
      delay(1);
    }
  }
  Serial.println(F("Flash/FAT init done"));

  //test_fatfs(); //TODO for testing, remove

  Serial.println(F("Initializing Bluetooth..."));

  Bluefruit.begin();
  Bluefruit.setTxPower(4);    // Check bluefruit.h for supported values

  // To be consistent OTA DFU should be added first if it exists
  bledfu.begin();

  // Configure and start the BLE Uart service
  bleuart.begin();

  // Set up and start advertising
  startAdv();

  Serial.println(F("Bluetooth initialization done"));

  //set up RGB LED
  rgbled.begin();
  rgbled.show(); // Initialize all pixels to 'off'

  //turn it on
  rgbled.setPixelColor(0, 0, 16, 0);
  rgbled.show();

  //initialize pressure sensor
  unsigned status;
  //status = bmp.begin(BMP280_ADDRESS_ALT, BMP280_CHIPID);
  status = bmp.begin();
  if (!status) {
    Serial.println(F("Could not find a valid BMP280 sensor, check wiring or "
                      "try a different address!"));
    Serial.print("SensorID was: 0x"); Serial.println(bmp.sensorID(),16);
    Serial.print("        ID of 0xFF probably means a bad address, a BMP 180 or BMP 085\n");
    Serial.print("   ID of 0x56-0x58 represents a BMP 280,\n");
    Serial.print("        ID of 0x60 represents a BME 280.\n");
    Serial.print("        ID of 0x61 represents a BME 680.\n");
    while (1) delay(10);
  }

  /* Default settings from datasheet. */
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_OFF,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_1); /* Standby time. */

  //calibrate sealevel pressure using current pressure and known height
  float current_pressure = bmp.readPressure()/100.0;
  sealevel_pressure = bmp.seaLevelForAltitude(current_height,current_pressure);

  //orientation sensor
  /* Initialise the sensor */
  Serial.println("Setup orientation sensor");
  if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.println("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }  
  delay(1000);    
  bno.setExtCrystalUse(true);

  Serial.println("Setup done");

  bleuart.bufferTXD(true);
}

void startAdv(void)
{
  // Advertising packet
  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addTxPower();
  
  // Include the BLE UART (AKA 'NUS') 128-bit UUID
  Bluefruit.Advertising.addService(bleuart);

  // Secondary Scan Response packet (optional)
  // Since there is no room for 'Name' in Advertising packet
  Bluefruit.setName(BLE_NAME);
  Bluefruit.ScanResponse.addName();

  /* Start Advertising
   * - Enable auto advertising if disconnected
   * - Interval:  fast mode = 20 ms, slow mode = 152.5 ms
   * - Timeout for fast mode is 30 seconds
   * - Start(timeout) with timeout = 0 will advertise forever (until connected)
   * 
   * For recommended advertising interval
   * https://developer.apple.com/library/content/qa/qa1931/_index.html   
   */
  Bluefruit.Advertising.restartOnDisconnect(true);
  Bluefruit.Advertising.setInterval(32, 244);    // in unit of 0.625 ms
  Bluefruit.Advertising.setFastTimeout(30);      // number of seconds in fast mode
  Bluefruit.Advertising.start(0);                // 0 = Don't stop advertising after n seconds  
}

/**************************************************************************/
/*!
    @brief  Constantly poll for new command or response data
*/
/**************************************************************************/
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
  float rel_height =(bmp.readAltitude(sealevel_pressure)-current_height);
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

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
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <utility/imumaths.h>
#include <flash.h>
#include <bluetooth.h>
#include <pressure.h>
#include <orientation.h>
#include <led.h>

#define INITIAL_REF_HEIGHT 50.0 // height to use in initial setup //TODO save in persistent memory

// New: flight logging / state machine constants and storage
#define MAX_SAMPLES 12000
#define SAMPLE_INTERVAL_MS 100        
#define LAUNCH_DELTA 1.0             // meters above armed reference to detect launch
#define LAND_ALT_THRESHOLD 0.3       // meters to consider landed
#define MIN_RECORD_TIME_MS 2000      // min recording time before landing is recognized

enum FlightState { STANDBY = 0, ARMED, RECORDING };
static FlightState flightState = STANDBY;

static size_t log_count = 0;
static unsigned long lastSampleMillis = 0;
static unsigned long recordingStartMillis = 0;
static float armed_ref_height = 0.0f;

// BLE packet buffer
//    READ_BUFSIZE            Size of the read buffer for incoming packets
#define READ_BUFSIZE                    (20)
uint8_t packetbuffer_receive[READ_BUFSIZE+1];

// Function prototypes
uint8_t readPacket(BLEUart *ble_uart, uint16_t timeout);
void startArming();
void startRecording();
void stopRecording();
void sampleIfNeeded();
void transferLog();

// New: helper to parse remote_logger packets (expected: 0x00, 'T', sub)
static bool handleRemoteCommand(uint8_t *pkt, uint8_t len)
{
  if (len < 3) return false; // need at least 3 bytes
  // remote_logger sends: [0]=0x00, [1]='T', [2]=sub
  // be tolerant: require pkt[1]=='T'
  if (pkt[1] != 'T') return false;

  char sub = pkt[2];
  if (sub == 'A') // Arm
  {
    if (flightState == STANDBY)
      startArming();
    else
      Serial.println("Cannot arm: not in STANDBY");
  }
  else if (sub == 'T') // Transfer
  {
    if (flightState == STANDBY)
      transferLog();
    else
      Serial.println("Cannot transfer: must be in STANDBY");
  }
  else if (sub == 'S') // Stop / abort
  {
    stopRecording();
  }
  else
  {
    Serial.print("Unknown remote subcommand: ");
    Serial.println(sub);
    return false;
  }
  return true;
}

// Packet buffer
extern uint8_t packetbuffer_receive[];

// for testing BLE data stream
bool ble_stream = false;

void printBMP280Values()
{
  Serial.print(F("Temperature = "));
  Serial.print(pressureSensor.readTemperature());
  Serial.println(" *C");

  Serial.print(F("Pressure = "));
  Serial.print(pressureSensor.readPressure());
  Serial.println(" Pa");

  Serial.print(F("appr. relative height = "));
  float rel_height = relativeAltitude();
  Serial.print(rel_height, 1); /* Adjusted to local forecast! */
  Serial.println(" m");

  // LED for speed test
  if (rel_height > 0)
    rgbled.setPixelColor(0, 0, 64, 0);
  else
    rgbled.setPixelColor(0, 64, 0, 0);
  rgbled.show();

  Serial.println();
}

void sendSensorValues()
{
  sensors_event_t event;

  // get the sensor data
  orientationSensor.getEvent(&event, Adafruit_BNO055::VECTOR_GRAVITY);
  float temp = pressureSensor.readTemperature();
  float press = pressureSensor.readPressure();
  float rel_height = relativeAltitude();

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

}

void startArming()
{
  armed_ref_height = relativeAltitude();
  flightState = ARMED;
  Serial.print("State: ARMED (ref ");
  Serial.print(armed_ref_height, 3);
  Serial.println(" m)");
}

void startRecording()
{
  log_count = 0;
  recordingStartMillis = millis();
  lastSampleMillis = 0;
  // start fresh persistent log for this flight
  clearLog();
  flightState = RECORDING;
  Serial.println("Recording started");
}

void stopRecording()
{
  flightState = STANDBY;
  Serial.print("Recording stopped, samples: ");
  Serial.println(log_count);
}

void sampleIfNeeded()
{
  if (flightState != RECORDING)
    return;
  unsigned long now = millis();
  if ((now - lastSampleMillis) < SAMPLE_INTERVAL_MS)
    return;
  lastSampleMillis = now;
  float relh = relativeAltitude();
  if (log_count < MAX_SAMPLES)
  {
    float t = (now - recordingStartMillis) / 1000.0f; // seconds
    // append to persistent storage 
    appendLogSample(t, relh);
    log_count++;
  }
  // landing detection: returned near ground after some recording time
  if ((relh <= LAND_ALT_THRESHOLD) && ((now - recordingStartMillis) > MIN_RECORD_TIME_MS))
  {
    stopRecording();
  }
}

void transferLog()
{
  Serial.print("Transferring log via BLE, samples=");
  uint32_t persistent_count = getLogSampleCount();
  if (persistent_count > 0)
  {
    Serial.println(persistent_count);
    // transfer persistent log (preferred)
    transferLogViaBLE(&bleuart);
    Serial.println("Persistent transfer complete");
    return;
  }

  Serial.println(F("No data stored, returning..."));

}

void setup(void)
{
  Serial.begin(115200);

  // for nrf52840 with native usb, wait for Serial connection unless timeout happens
  uint8_t tries = 0;
  while ((!Serial) && (tries < 200))
  {
    delay(10);
    tries++;
  }
  // flush any remaining incoming data from e.g. upload process
  while (Serial.available() > 0)
    Serial.read();

  Serial.println("Starting setup");

  Serial.print(F("  RGB LED..."));
  setupLED();
  Serial.println(F("OK"));

  Serial.print(F("  Flash/filesystem..."));
  // uncomment this next line and follow instructions on monitor to format a new device's flash, this will erase all data!
  // format_flash();
  setupFlash();
  Serial.println(F("OK"));
  // test_fatfs(); //TODO for testing, remove

  Serial.print(F("  Bluetooth..."));
  setupBluetooth();
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
  while (true){

    // State machine: check sensors and sample if needed
    // State-specific behavior
    if (flightState == ARMED)
    {
      // detect launch by altitude increase above armed reference
      float rel_height = relativeAltitude();
      if ((rel_height - armed_ref_height) > LAUNCH_DELTA)
      {
        startRecording();
      }
    }
    else if (flightState == RECORDING)
    {
      sampleIfNeeded();
    }


    // Wait for new BLE data to arrive
    uint8_t len = readPacket(&bleuart, 10); // was 500 before speed test
    if (len >= 3){
      // Try to handle as remote_logger packet
      handleRemoteCommand(packetbuffer_receive, len);
    }

  }  
}

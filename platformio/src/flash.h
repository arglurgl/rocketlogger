// flash/filesystem
#ifndef _FLASH_H_GUARD
#define _FLASH_H_GUARD

#include <Adafruit_SPIFlash.h>
#include <SdFat.h>
#include <bluetooth.h>

#define F_TEST_TEST_TXT       "/test/test.txt"
#define D_TEST                "/test"

// New: logging file definitions and API
#define LOG_DIR               "/logs"
#define LOG_FILE              "/logs/flight.bin"

extern Adafruit_SPIFlash flash;
extern FatVolume fatfs;

void format_flash();
void setupFlash();
void listfiles();
void printfile(const char* filename);


// Append a sample (time in seconds, altitude in meters) to the persistent log.
// Returns true on success.
bool appendLogSample(float time_s, float altitude_m);

// Remove existing persistent log (start fresh).
bool clearLog();

// Return number of samples currently stored in the log.
uint32_t getLogSampleCount();

// Read the log and call the provided callback for each sample.
// Callback signature: void cb(float time_s, float altitude_m)
typedef void (*LogSampleCallback)(float, float);
bool readLog(LogSampleCallback cb);

// Transfer the stored log over BLE (uses sendVector3 with PACKET_DATA_ALT_HEADER).
// Returns true on success.
bool transferLogViaBLE(BLEUart *bleuart);

#endif
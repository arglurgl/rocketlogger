#include <flash.h>
#include "flash_config.h" // for flashTransport definition

// New includes needed for BLE transfer
#include <bluetooth.h>
#include <stdio.h>

Adafruit_SPIFlash flash(&flashTransport);
FatVolume fatfs;

/// @brief Initialize 2MB external QSPI flash on the itsybitsy board. Blocks on error.
void setupFlash(){
  flash.begin();
  if ( !fatfs.begin(&flash) ) {
    Serial.println(F("Error: filesystem is not existent on the flash device. Please uncomment format_flash() to make one."));
    while(1){
        yield();
        delay(1);
    }
  }
}

/// @brief Test the filesystem by writing and reading a test file
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

/// @brief List the filenames in the root directory
void listfiles(){
  fatfs.ls();
}
/// @brief Print the contents of the passed in filename
void printfile(const char* filename){
  File32 readFile = fatfs.open(filename, FILE_READ);
  while (readFile.available()) {
    char c = readFile.read();
    Serial.print(c);
  }
  // Close the file when finished reading.
  readFile.close();
// New: persistent logging implementation
}

static bool ensureLogDir()
{
  if (!fatfs.exists(LOG_DIR))
  {
    if (!fatfs.mkdir(LOG_DIR))
    {
      Serial.println(F("Error: failed to create log directory"));
      return false;
    }
  }
  return true;
}

// Change: appendLogSample now writes human-readable CSV lines "time,alt\n"
bool appendLogSample(float time_s, float altitude_m)
{
  if (!ensureLogDir()) return false;
  File32 f = fatfs.open(LOG_FILE, FILE_WRITE);
  if (!f) {
    Serial.println(F("Error: failed to open log file for append"));
    return false;
  }
  char buf[64];
  int n = snprintf(buf, sizeof(buf), "%.3f,%.3f\n", time_s, altitude_m);
  if (n <= 0) {
    f.close();
    return false;
  }
  size_t written = f.write((const uint8_t *)buf, (size_t)n);
  f.close();
  return (written == (size_t)n);
}

bool clearLog()
{
  // Remove existing file if present
  if (fatfs.exists(LOG_FILE))
  {
    if (!fatfs.remove(LOG_FILE))
    {
      Serial.println(F("Error: failed to remove existing log file"));
      return false;
    }
  }
  return true;
}

// Change: count lines to determine number of samples
uint32_t getLogSampleCount()
{
  if (!fatfs.exists(LOG_FILE)) return 0;
  File32 f = fatfs.open(LOG_FILE, FILE_READ);
  if (!f) return 0;
  uint32_t count = 0;
  while (f.available()) {
    int c = f.read();
    if (c == '\n') count++;
  }
  f.close();
  return count;
}

// Change: read CSV lines and call callback with parsed floats
bool readLog(LogSampleCallback cb)
{
  if (!fatfs.exists(LOG_FILE)) return false;
  File32 f = fatfs.open(LOG_FILE, FILE_READ);
  if (!f) return false;
  while (f.available()) {
    String line = f.readStringUntil('\n');
    if (line.length() == 0) continue;
    int idx = line.indexOf(',');
    if (idx < 0) continue;
    String tstr = line.substring(0, idx);
    String astr = line.substring(idx + 1);
    float t = tstr.toFloat();
    float alt = astr.toFloat();
    if (cb) cb(t, alt);
  }
  f.close();
  return true;
}

// File-scope BLE pointer and transfer callback (simple CSV text framing)
static BLEUart *s_bleuart = nullptr;

// Send one CSV text line (with trailing newline) over BLE 
static void transferLogCallback(float t, float alt)
{
  if (!s_bleuart) return;

  // produce CSV line with trailing newline
  char buf[64];
  int n = snprintf(buf, sizeof(buf), "%.3f,%.3f\n", t, alt);
  if (n <= 0) return;

  if (n >= 20){
    Serial.println(F("Warning: log line probably too long for BLE transfer (20 bytes max)"));
  }

  // Debug print to serial
  Serial.print(F("[BLE TX] "));
  Serial.write((const uint8_t*)buf, (size_t)n);
  Serial.println();

  // Buffer and flush via Adafruit BLE API (use boolean arg)
  s_bleuart->write((const uint8_t*)buf, (size_t)n);
  s_bleuart->flushTXD();

  // pacing to reduce fragmentation/loss
  delay(20);
}

// Modified transfer: emits LOG_START/LOG_END and streams CSV text lines (one per notification)
bool transferLogViaBLE(BLEUart *bleuart)
{
  if (!fatfs.exists(LOG_FILE)) return false;
  s_bleuart = bleuart;

  // send start marker
  const char start[] = "LOG_START\n";
  Serial.print(F("[BLE TX] "));
  Serial.write((const uint8_t*)start, (size_t)sizeof(start)-1);
  Serial.println();

  s_bleuart->write((const uint8_t*)start, (size_t)sizeof(start)-1);
  s_bleuart->flushTXD();

  // stream CSV lines via readLog + transferLogCallback
  Serial.println(F("[BLE] Reading and streaming log samples..."));
  bool ok = readLog(&transferLogCallback);

  // send end marker
  const char end[] = "LOG_END\n";
  Serial.print(F("[BLE TX] "));
  Serial.write((const uint8_t*)end, sizeof(end)-1);
  Serial.println();

  s_bleuart->write((const uint8_t*)end, (size_t)sizeof(end)-1);
  s_bleuart->flushTXD(); // flush buffer on finishing

  Serial.println(F("[BLE] Transfer complete"));
  s_bleuart = nullptr;
  return ok;
}
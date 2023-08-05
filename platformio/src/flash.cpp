#include <flash.h>
#include "flash_config.h" // for flashTransport definition

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
}
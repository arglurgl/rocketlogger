// flash/filesystem
#include <Adafruit_SPIFlash.h>
#include <SdFat.h>

#define F_TEST_TEST_TXT       "/test/test.txt"
#define D_TEST                "/test"

extern Adafruit_SPIFlash flash;
extern FatVolume fatfs;

void format_flash();
void setupFlash();

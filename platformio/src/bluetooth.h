#ifndef _BLUETOOTH_H_GUARD
#define _BLUETOOTH_H_GUARD

#include <bluefruit.h>

#define BLE_NAME "rocketlogger" // name device appears under in bluetooth scans

extern BLEDfu bledfu;
extern BLEUart bleuart;

void setupBluetooth();
void startAdvertising();

#endif
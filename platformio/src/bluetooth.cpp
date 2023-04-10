#include <bluetooth.h>

BLEDfu bledfu; // OTA DFU service
BLEUart bleuart; // Uart over BLE service

/// @brief Configure bluetooth and start advertising
void setupBluetooth(){       
    Bluefruit.begin();
    Bluefruit.setTxPower(4);    // Check bluefruit.h for supported values

    // To be consistent OTA DFU should be added first if it exists
    bledfu.begin();

    // Configure and start the BLE Uart service
    bleuart.begin();

    // Set up and start advertising
    startAdvertising();

    bleuart.bufferTXD(true);
}

void startAdvertising(){
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

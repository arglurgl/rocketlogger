#include <bluetooth.h>
#include <Arduino.h>

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

/// @brief Read a packet from the BLE UART into packetbuffer_receive[]
/// Wait up to timeout_ms for the first byte, then read until no new bytes
/// arrive for a short inter-byte timeout. Returns number of bytes read.
uint8_t readPacket(BLEUart *ble_uart, uint16_t timeout_ms)
{
  extern uint8_t packetbuffer_receive[]; // provided by main.cpp
  uint8_t len = 0;

  unsigned long start = millis();
  // Wait for first byte up to timeout_ms
  while ((millis() - start) < (unsigned long)timeout_ms) {
    if (ble_uart->available()) break;
    delay(1);
  }
  if (!ble_uart->available()) return 0; // no data

  // Read bytes until no new byte arrives for INTER_BYTE_TIMEOUT_MS
  const unsigned long INTER_BYTE_TIMEOUT_MS = 100;
  unsigned long last_byte_time = millis();

  while ((millis() - last_byte_time) < INTER_BYTE_TIMEOUT_MS) {
    while (ble_uart->available()) {
      int c = ble_uart->read(); // returns -1 if none
      if (c < 0) break;
      // store if space (assume external buffer is sized appropriately)
      packetbuffer_receive[len++] = (uint8_t)c;
      last_byte_time = millis();
    }
    delay(1);
  }

  return len;
}

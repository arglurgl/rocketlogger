#include <string.h>
#include <Arduino.h>
#include <bluefruit.h>

#define PACKET_DATA_TYPE_ACC                 (1)

#define PACKET_DATA_ACC_LEN                  (15)

#define PACKET_DATA_ACC_HEADER               'A'


//    SEND_BUFSIZE            Size of the read buffer for assembling packets
#define SEND_BUFSIZE                    (20)


/* Buffer to hold incoming characters */
uint8_t packetbuffer_send[SEND_BUFSIZE+1];

/**************************************************************************/
/*!
    @brief  Casts 3 float vector components to three times four bytes at the specified buffer
*/
/**************************************************************************/
void vectorbytes(float x, float y, float z, uint8_t *buffer) 
{
  memcpy(buffer   , &x, 4);
  memcpy(buffer+4 , &y, 4);
  memcpy(buffer+8 , &z, 4);
}

/**************************************************************************/
/*!
    @brief  Waits for incoming data and parses it
*/
/**************************************************************************/
bool sendVector(BLEUart *ble_uart, uint8_t type, float x, float y, float z) 
{
  ble_uart->flushTXD(); // make sure buffer is empty so we fit in one MTU
  memset(packetbuffer_send, 0, SEND_BUFSIZE);
  packetbuffer_send[0] = '!';
  uint8_t packetlength = 0;

  if (type == PACKET_DATA_TYPE_ACC) {
    //prepare buffer    
    packetbuffer_send[1] = PACKET_DATA_ACC_HEADER;
    vectorbytes(x, y, z, packetbuffer_send+2);
    packetlength = PACKET_DATA_ACC_LEN;  
  }
  //more types here

  //check if data was set
  if (packetlength){
    //calc checksum
    uint8_t checksum = 0;
    for (uint8_t i=0; i<packetlength-1; i++) {
      checksum += packetbuffer_send[i];
    }
    checksum = ~checksum;
    packetbuffer_send[packetlength-1] = checksum;

    //send the buffer
    ble_uart->write(packetbuffer_send, PACKET_DATA_ACC_LEN);
    ble_uart->flushTXD();
    return true;
  }

  //invalid type, no data was sent
  Serial.println("unknown type");
  return false;
}


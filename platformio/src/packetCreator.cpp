#include <packetCreator.h>


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
    @brief  Sends data for a three component vector
*/
/**************************************************************************/
bool sendVector3(BLEUart *ble_uart, char type_header, float x, float y, float z) 
{
  memset(packetbuffer_send, 0, SEND_BUFSIZE);
  packetbuffer_send[0] = '!';
  uint8_t packetlength = 0;

  switch (type_header){
    case PACKET_DATA_ACC_HEADER:
    case PACKET_DATA_ALT_HEADER:
      break; //header is OK, continue
    default:
      Serial.println("unknown type");
      return false;
  }
  
  //prepare buffer    
  packetbuffer_send[1] = type_header;
  vectorbytes(x, y, z, packetbuffer_send+2);
  packetlength = PACKET_DATA_VECTOR3_LEN;  

  //calc checksum
  uint8_t checksum = 0;
  for (uint8_t i=0; i<packetlength-1; i++) {
    checksum += packetbuffer_send[i];
  }
  checksum = ~checksum;
  packetbuffer_send[packetlength-1] = checksum;

  //send the buffer
  ble_uart->flushTXD(); // make sure buffer is empty so we fit in one MTU
  ble_uart->write(packetbuffer_send, PACKET_DATA_VECTOR3_LEN);
  ble_uart->flushTXD();
  return true;
}


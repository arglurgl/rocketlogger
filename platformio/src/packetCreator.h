#include <string.h>
#include <Arduino.h>
#include <bluefruit.h>

#define PACKET_DATA_VECTOR3_LEN             (15)
#define PACKET_DATA_ACC_HEADER              'A'
#define PACKET_DATA_ALT_HEADER              'H'


//    SEND_BUFSIZE            Size of the read buffer for assembling packets
#define SEND_BUFSIZE                    (20)

//Function prototypes for packetcreator
bool sendVector3(BLEUart *ble_uart, char type_header, float x, float y, float z);
#define PACKET_DATA_TYPE_ACC                 (1) //do proper header later
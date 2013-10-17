

#ifndef _QSLIB_H_
#define _QSLIB_H_

#include <Arduino.h>
#include "MPU9150Lib.h"
/*
 * 
 * Error handling includes:
 *
 *
 */

// Enable/disable DEBUG print
//#define DEBUG 1

// PacketTypes
#define ACCEL_PACKET_TYPE 1
#define VELOCITY_PACKET_TYPE 2
#define REP_PACKET_TYPE 3
#define EXERCISE_PACKET_TYPE 4

// Packet constants
#define ACCEL_FIELD_LENGTH 5
#define ACCEL_FIELD_COUNT 3
#define VELOCITY_FIELD_LENGTH 5
#define VELOCITY_FIELD_COUNT 3

#define REP_FIELD_LENGTH 2 // 2 bytes for rep count

#define EXERCISE_FIELD_LENGTH 2

// Error codes
#define NO_ERROR 0
// sendPacket errors
#define ERROR_SENDPACKET_INVALID_PACKET_TYPE 1

// floatToHex errors
#define ERROR_FLOATTOHEX_NUMBER_TOO_BIG 1
#define ERROR_FLOATTOHEX_NEGATIVE_NUMBER_TOO_BIG 2

class QSLib {
  public:
    QSLib(HardwareSerial *dataSerial, HardwareSerial *outputSerial);
    // Returns 0 on sucess, error code otherwise
    uint8_t sendAcceleration(MPUVector3 accelVector3);

  private:
    HardwareSerial *dataSerial;
    HardwareSerial *outputSerial;
    // Returns 0 on sucess, error code otherwise
    // Arguments are the float to convert, a buffer to write the characters to and the buffer's length
    uint8_t floatToHex(float f, uint8_t *buf, uint8_t bufLen);
    // Returns 'X' on failure, valid hex char otherwise
    char mod16ToHex(int16_t i);
    // Returns 'X' on failure, valid hex char otherwise
    char add8ToHex(char c);
    // Returns 0 on sucess, error code otherwise
    uint8_t sendPacket(const char packetType, const void *const data);
};

#endif // _QSLIB_H_

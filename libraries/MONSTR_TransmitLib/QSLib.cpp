

#include "QSLib.h"

/* Constructor sets ups serial instance variables */
QSLib::QSLib(HardwareSerial *dataSerial, HardwareSerial *outputSerial) {
  this->dataSerial = dataSerial;
  this->outputSerial = outputSerial;
}
    
/* Returns 0 on sucess, error code otherwise */
uint8_t QSLib::sendAcceleration(MPUVector3 accelVector3) {
  // Call out to sendPacket with proper packetType
  return sendPacket(ACCEL_PACKET_TYPE, (void *) accelVector3);
}

/* Returns 0 on sucess, error code otherwise */
uint8_t QSLib::floatToHex(float float_number, uint8_t *buf, uint8_t bufLen) {
  int16_t int_number = float_number;
  int16_t digit;
  char hexChar;
  uint8_t *bufPtr = buf+bufLen-1;
  uint16_t counter = 0;
  bool negative_number_flag;

  // Handle negative numbers
  if(int_number < 0) {
    // Change to positive
    int_number *= -1;
    negative_number_flag = true;
  }
  while(int_number > 0) {
    if(counter >= bufLen) {
      // Perhaps set number to 7FFFF or FFFFF (if negative) before bailing? 
      // That way it gracefully handles the problem...
      return ERROR_FLOATTOHEX_NUMBER_TOO_BIG;
    }
    digit = int_number % 16;
    int_number /= 16;
    hexChar = mod16ToHex(digit);
    *bufPtr = hexChar;
    bufPtr--;
    counter++;
  }
  // Fill remainding MSB digits with 0s
  for(;counter < bufLen; ++counter) {
    *bufPtr = '0';
    bufPtr--;
  }
  // Change MSB if number was negative
  if(negative_number_flag) {
    bufPtr++;
    // Check if MSB 8-F already (AKA number too big!)
    if(!(*bufPtr >= '0' && *bufPtr <= '9')) {
      return ERROR_FLOATTOHEX_NEGATIVE_NUMBER_TOO_BIG;
    }
    *bufPtr = add8ToHex(*bufPtr);
  }
  return 0;
}

/* Returns 'X' on failure, valid hex char otherwise */
char QSLib::mod16ToHex(int16_t i) {

  if(i > 15 || i < 0) {
      outputSerial->println("Bad value passed to mod16ToHex");
      return 'X';
  }
  if(i < 10) {
    return '0' + i;
  } else {
    return 'A' + (i-10);
  }
}
/* Returns 'X' on failure, valid hex char otherwise */
char QSLib::add8ToHex(char c) {
  if(!(c >= '0' && c <= '7')) {
    outputSerial->println("Bad value passed to add8ToHex");
    return 'X';
  } else if(c == '0' || c == '1') {
    return c + 8;
  } else {
    return (c - '2') + 'A';
  }
}

/* Returns 0 on success, positive value on failure */
uint8_t QSLib::sendPacket(const char packetType, const void *const data) {

  // Data general to all packets
  uint16_t packetFieldLength;
  uint16_t packetFieldCount;
  char packetTypeSymbol;
  uint16_t packetDataLength;
  uint8_t* packetData;
  // Packet-type specific data
  MPUVector3*  accelData;

  // Ascertain packet specific values
  switch (packetType) {
    case ACCEL_PACKET_TYPE:
      packetTypeSymbol = '~';
      packetFieldLength = ACCEL_FIELD_LENGTH;
      packetFieldCount = ACCEL_FIELD_COUNT;
      accelData = (MPUVector3 *) data;
      packetData = (uint8_t*) malloc(packetFieldCount * packetFieldLength * sizeof(uint8_t));
      for(int i = 0; i < packetFieldCount; ++i) {
        floatToHex((*accelData)[i], packetData + (i*packetFieldLength), packetFieldLength);
        //outputSerial->print((char)*packetData);
        //outputSerial->print(" ");
      }
      break;
    case VELOCITY_PACKET_TYPE:

      break;
    default:
      packetFieldLength = 0;
      packetFieldCount = 0;
      outputSerial->println("Error, invalid packetType received in sendPacket!");
      return ERROR_SENDPACKET_INVALID_PACKET_TYPE;
      break;
  }
  // Form and send packet
  packetDataLength = packetFieldLength * packetFieldCount;
  // Write symbol
  dataSerial->write(packetTypeSymbol);

  // Write data
  dataSerial->write(packetData, packetDataLength);
  #ifdef DEBUG
    outputSerial->print(packetTypeSymbol);
    for(int i = 0; i < packetDataLength; ++i) {
      outputSerial->print((char)*(packetData+i));
    }
    outputSerial->print("\n\n");
  #endif
  free(packetData);
  return NO_ERROR;
}
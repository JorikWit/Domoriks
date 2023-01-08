#include <stdio.h>
#include <string.h>

#define MODBUS_ASCII_START_BYTE 0x3A  // ASCII ':'
#define MODBUS_ASCII_END_BYTE 0x0D  // ASCII CR
#define MODBUS_ASCII_LRC_BYTE 0x2A  // ASCII '*'

// Structure to hold decoded Modbus ASCII message
typedef struct {
  int slave_address;
  int function_code;
  int data[256];
  int data_length;
  int lrc;
} ModbusASCIIMessage;

// Calculate LRC of Modbus ASCII message
int calculateLRC(char *message) {
  int lrc = 0;
  for (int i = 1; i < strlen(message)-1; i++) {
    lrc += message[i];
  }
  return (lrc & 0xFF) ^ 0xFF;
}

// Decode Modbus ASCII message
int decodeModbusASCII(char *message, ModbusASCIIMessage *decoded) {
  // Check for start and end bytes
  if (message[0] != MODBUS_ASCII_START_BYTE || message[strlen(message)-1] != MODBUS_ASCII_END_BYTE) {
    return -1;  // Invalid message
  }

  // Extract slave address, function code, and data
  int num_bytes = sscanf(message+1, "%2x%2x%s", &decoded->slave_address, &decoded->function_code, decoded->data);
  if (num_bytes < 2) {
    return -1;  // Invalid message
  }

  // Calculate length of data
  decoded->data_length = (num_bytes - 2) / 2;

  // Extract LRC byte
  if (message[strlen(message)-2] == MODBUS_ASCII_LRC_BYTE) {
    sscanf(message+strlen(message)-3, "%2x", &decoded->lrc);
  } else {
    decoded->lrc = 0;
  }

  // Check LRC
  int calculated_lrc = calculateLRC(message);
  if (decoded->lrc != calculated_lrc) {
    return -1;  // LRC check failed
  }

  return 0;  // Success
}

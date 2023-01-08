

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

int decodeModbusASCII(char *message, ModbusASCIIMessage *decoded);

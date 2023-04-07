#include "modbus_ascii.h"

//char mesg[100] = ":02010020000cd1\r";
//char mesg[100] = ":0401000A000DE4\r";
//char mesg[100] = ":1103006B00037E\r";

int main() {
  uint8_t message[100] = ":02010020000cd1\r";  // Modbus ASCII message to decode

  ModbusASCIIMessage decoded;
  int result = decode_modbus_ASCII(message, &decoded);

  if (result == 0) {

    printf("Slave address: %d\n", decoded.slave_address);
    printf("Function code: %d\n", decoded.function_code);
    printf("Data length: %d\n", decoded.data_length);
    printf("Data: ");
    for (int i = 0; i < decoded.data_length; i++) {
      printf("%d ", decoded.data[i]);
    }
    printf("\n");
    printf("LRC: %2x\n", decoded.lrc);
  }
  else {
    printf("Invalid or corrupted message\n");
  }
  return 0;
}
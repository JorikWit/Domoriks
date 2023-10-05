#include "modbus_ascii.h"
#include "modbus_rtu.h"
#include "modbusm.h"

//char message[100] = ":02010020000cd1\r";
char message[100] = ":0401000A000DE4\r";
uint8_t message_rtu[8] = { 0x04, 0x01, 0x00, 0x0A, 0x00, 0x0D, 0xDD, 0x98 };
//char message[100] = ":1103006B00037E\r";

int main() {
  ModbusMessage decoded;
  uint8_t encoded[100];
  size_t length;
  int result;

  result = decode_modbus_ascii(message, &decoded);
  printf("ASCII\n");
  if (result == 0) {
    printf("Slave address: %d\n", decoded.slave_address);
    printf("Function code: %d\n", decoded.function_code);
    printf("Data length: %d\n", decoded.data_length);
    printf("Data: ");
    for (int i = 0; i < decoded.data_length; i++) {
      printf("%d ", decoded.data[i]);
    }
    printf("\n\n");
  }
  else {
    printf("Invalid or corrupted message\n");
  }

  encode_modbus_ascii(encoded, &decoded);
  printf("%s\n", encoded);

  result = decode_modbus_rtu(message_rtu, 8, &decoded);
  printf("RTU\n");
  if (result == 0) {
    printf("Slave address: %d\n", decoded.slave_address);
    printf("Function code: %d\n", decoded.function_code);
    printf("Data length: %d\n", decoded.data_length);
    printf("Data: ");
    for (int i = 0; i < decoded.data_length; i++) {
      printf("%d ", decoded.data[i]);
    }
    printf("\n\n");
  }
  else {
    printf("Invalid or corrupted message\n\n");
  }

  encode_modbus_rtu(encoded, &length, &decoded);
  for (int i = 0; i < length; i++)
    printf("0x%02X ", encoded[i]);
  printf("\n\n");
  return 0;
}


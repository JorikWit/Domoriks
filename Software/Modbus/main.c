#include "modbus_ascii.h"
#include "modbus_rtu.h"
#include "modbusm_handler.h"
#include "modbusm.h"

//1
//char message[100] = ":010100000008F6\r";
//uint8_t message_rtu[8] = { 0x01, 0x01, 0x00, 0x00, 0x00, 0x08, 0x3D, 0xCC };
// char message[100] = ":010200010010EC\r";
// uint8_t message_rtu[8] = { 0x01, 0x01, 0x00, 0x20, 0x00, 0x0C, 0x3D, 0xC5 };
// char message[100] = ":0401000A000DE4\r";
// uint8_t message_rtu[8] = { 0x04, 0x01, 0x00, 0x0A, 0x00, 0x0D, 0xDD, 0x98 };

//2
// char message[100] = ":010200200008D5\r";
// uint8_t message_rtu[8] = { 0x01, 0x02, 0x00, 0x20, 0x00, 0x08, 0x78, 0x06 };
// char message[100] = ":010200200004D9\r";
// uint8_t message_rtu[8] = { 0x01, 0x02, 0x00, 0x20, 0x00, 0x04, 0x78, 0x03 };
// char message[100] = ":010200200010CD\r";
// uint8_t message_rtu[8] = { 0x01, 0x02, 0x00, 0x20, 0x00, 0x10, 0x78, 0x0C };
// char message[100] = ":010200040014E5\r";
// uint8_t message_rtu[8] = { 0x01, 0x02, 0x00, 0x04, 0x00, 0x14, 0x39, 0xC4 };
// char message[100] = ":010200010010EC\r";
// uint8_t message_rtu[8] = { 0x01, 0x02, 0x00, 0x01, 0x00, 0x10, 0x28, 0x06 };

//3
// char message[100] = ":010300000002FA\r";
// uint8_t message_rtu[8] = { 0x01, 0x03, 0x00, 0x00, 0x00, 0x02, 0xC4, 0x0B };
// char message[100] = ":010300010003F8\r";
// uint8_t message_rtu[8] = { 0x01, 0x03, 0x00, 0x01, 0x00, 0x03, 0x54, 0x0B };
// char message[100] = ":1103006B00037E\r";
// uint8_t message_rtu[8] = { 0x11, 0x03, 0x00, 0x01, 0x00, 0x03, 0x76, 0x87 };

//4
// char message[100] = ":010400020004F5\r";
// uint8_t message_rtu[8] = { 0x01, 0x04, 0x00, 0x02, 0x00, 0x04, 0x50, 0x09 };

//5
// char message[100] = ":01050000FF00FB\r";
// uint8_t message_rtu[8] = { 0x01, 0x05, 0x00, 0x00, 0xFF, 0x00, 0x8C, 0x3A };
char message[100] = ":010500030000F7\r";
uint8_t message_rtu[8] = { 0x01, 0x05, 0x00, 0x03, 0x00, 0x00, 0x3D, 0xCA };
// char message[100] = ":01050005FF00F6\r";
// uint8_t message_rtu[8] = { 0x01, 0x05, 0x00, 0x05, 0xFF, 0x00, 0x9C, 0x3B };

uint8_t coils[16] = { 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
uint8_t inputs[16] = { 0x00, 0xFF, 0x00, 0x00, 0x9A, 0xBC, 0xCE, 0xF0 };
uint16_t hold_reg[16] = { 0x1234, 0x5678, 0x9ABC, 0xDEF0, 0x1122, 0x3344, 0x5566, 0x7788 };
uint16_t i_reg[16] = { 0x1234, 0x5678, 0x9ABC, 0xDEF0, 0x1122, 0x3344, 0x5566, 0x7788 };

int main() {
  modbusCoils = coils;
  modbusInputs = inputs;
  modbusHReg = hold_reg;
  modbusIReg = i_reg;

  //new message 
  printf("NEW MESSAGE\n");
  ModbusMessage newMessage;
  newMessage.slave_address = DEVICE_ID;
  newMessage.function_code = 5;
  newMessage.data_length = 4;
  uint8_t d[4] = { 0x00, 0x05, 0xFF, 0x00 };
  for (int i = 0; i < 4; i++) newMessage.data[i] = d[i];
  //print
  print_modbus_rtu(&newMessage);
  print_modbus_ascii(&newMessage);
  printf("\n");

  int result;
  ModbusMessage decoded;

  //ascii
  printf("ASCII decode encode\n");
  result = decode_modbus_ascii(message, &decoded);
  if (result == 0) {
    print_modbus_rtu(&decoded);
    print_modbus_ascii(&decoded);
    printf("\n");
  }
  else {
    printf("Invalid or corrupted message\n");
  }

  //rtu
  printf("RTU decode encode\n");
  result = decode_modbus_rtu(message_rtu, 8, &decoded);
  if (result == 0) {
    print_modbus_rtu(&decoded);
    print_modbus_ascii(&decoded);
    printf("\n");
  }
  else {
    printf("Invalid or corrupted message\n\n");
  }

  //reply
  printf("Reply\n");
  result = modbusm_handle(&decoded);
  if (result == 0) {
    print_modbus_rtu(&decoded);
    print_modbus_ascii(&decoded);
    printf("\n");
  }
  else if (ID_MISMATCH) {
    printf("Wrong slave address\n\n");
  }
  else if (INVALID_DATA_LENGHT) {
    printf("Wrong message size\n\n");
  }
  else if (INVALID_COIL_VALUE) {
    printf("Wrong coilvalue size\n\n");
  }


  printf("COILS: %02X", *coils);
  return 0;
}


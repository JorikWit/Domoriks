/*
 * File:   modbus_decode.h
 * Author: Jorik Wittevrongel
 *
 * Created on December 20 2022, 19:02
 */

#ifndef MODBUS_DECODE_H
#define MODBUS_DECODE_H

#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <inttypes.h>

#define MODBUS_ASCII_START_BYTE 0x3A  // ASCII ':'
#define MODBUS_ASCII_END_BYTE 0x0D  // ASCII CR
#define MODBUS_ASCII_LRC_BYTE 0x2A  // ASCII '*'

 // Structure to hold decoded Modbus ASCII message
typedef struct {
	uint8_t slave_address;
	uint8_t function_code;
	uint8_t data[100];
	uint8_t data_length;
	uint8_t lrc;
} ModbusASCIIMessage;

uint8_t decode_modbus_ASCII(char* message, ModbusASCIIMessage* decoded);

#endif /* MODBUS_DECODE_H */

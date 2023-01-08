#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <ctype.h>
#include <Modbus/modbus_handler.h>

#include "main.h"

uint8_t calculate_lrc(const uint8_t* data, size_t length)
{
    uint8_t lrc = 0;
    for (size_t i = 0; i < length; i++)
    {
        lrc ^= data[i];
    }
    return lrc;
}

uint16_t chars_to_int(uint8_t *chars) {
	int i = 0;
	for (int j = 0; chars[j] != '\0'; j++) {
		if (!isdigit(chars[j])) {
			return 0;
		}
		i = i * 10 + (chars[j] - '0');
	}
  return i;
}

uint16_t chars_hex_to_int(uint8_t *chars) {
	return (uint8_t)strtol(chars, NULL, 16);
}

uint8_t cmd_coil_read(uint16_t addr_data, uint16_t data) {
	for (uint16_t i = 0; i++; i<data){
		char response[20];
		//my_slave_id + funct + length (voorlopig altijd 1?)
		sprintf(response, ":%d%d0101", slave_high_char, slave_low_char);
	}
	return 0;
}

uint8_t cmd_input_read(uint16_t addr_data, uint16_t data) {

	return 0;
}

uint8_t cmd_hold_reg_read(uint16_t addr_data, uint16_t data) {

	return 0;
}

uint8_t cmd_input_reg_read(uint16_t addr_data, uint16_t data) {

	return 0;
}

uint8_t device_id = 02;
uint8_t part_of_buffer[4];

uint8_t cmd_parser(uint8_t *buffer) {
	part_of_buffer[0] = 0;
	part_of_buffer[1] = 0;
	part_of_buffer[2] = buffer[0];
	part_of_buffer[3] = buffer[1];
	if (chars_hex_to_int(part_of_buffer) ==  device_id) {
		part_of_buffer[0] = buffer[2];
		part_of_buffer[1] = buffer[3];
		part_of_buffer[2] = buffer[4];
		part_of_buffer[3] = buffer[5];
		uint16_t function = chars_hex_to_int(part_of_buffer);
		part_of_buffer[0] = buffer[6];
		part_of_buffer[1] = buffer[7];
		part_of_buffer[2] = buffer[8];
		part_of_buffer[3] = buffer[9];
		uint16_t addr_data = chars_hex_to_int(part_of_buffer);
		part_of_buffer[0] = buffer[10];
		part_of_buffer[1] = buffer[11];
		part_of_buffer[2] = buffer[12];
		part_of_buffer[3] = buffer[13];
		uint16_t data = chars_hex_to_int(part_of_buffer);
		switch (function) {
			case 1:
				//read coil
				cmd_coil_read(addr_data, data);
				break;
			case 2:
				//read input
				break;
			case 3:
				//read holding reg
				break;
			case 4:
				//read input reg
				break;
			case 5:
				//write coil
				break;
			case 6:
				//write reg
				break;
			case 15:
				//write multiple coils
				break;
			case 16:
				//write multiple regs
				break;
			default:
				//wrong cmd
				return 2;
				break;
		}
	} else {
		return 1; //wrong slave_id: message is for a other slave
	}
	return 0;
}

//uint8_t func;
//typedef enum {
//	off,
//	on,
//	toggle,
//	pwm
//} cmd;
//uint8_t handle_cmd() {
//	switch (func) {
//		case off:
//			break;
//		case on:
//			break;
//		case toggle:
//			break;
//		case pwm:
//			break;
//		default:
//			break;
//	}
//	return 0;
//}

#define BUFFERSIZE 50
uint8_t buffer[BUFFERSIZE] = {0};

uint8_t read_modbus( void ) {
	//(9600bit/s / (start + 8 data+ stop + ?2wait?) = 800char/s = 1.25ms/char
	//read 1 char Timeout after 3ms
	uint8_t start_char;
	uint8_t lenght = 1;
	if (HAL_UART_Receive(&huart1, &start_char, lenght, (uint16_t) (lenght * 2) / (huart1.Init.BaudRate) ) == HAL_OK) { //check if baudrate is accesable
		//read 50 char Timeout after 125ms
		if (start_char == ':') {
			int i = 0;
			//Read 1 char at the time
			while ((HAL_UART_Receive(&huart1, buffer+i, 1, (uint16_t) (1 * 2) / (huart1.Init.BaudRate) ) == HAL_OK) &&
					i < BUFFERSIZE) {
				if (buffer[i-1] == 0x0D && buffer[i] == 0x0A) {
					const char *mesg = "Valid cmd";
					HAL_UART_Transmit(&huart1, (uint8_t*)mesg, 9, (uint16_t) (9 * 2) / (huart1.Init.BaudRate));
					uint8_t enter = '\n';
					HAL_UART_Transmit(&huart1, &enter, 1, (uint16_t) (1 * 2) / (huart1.Init.BaudRate));
					//cmd_lcr_check(buffer, i);
					cmd_parser(buffer);
					//valid command
					return 0;
				}
				i++;
			}

			if (i <= BUFFERSIZE) {
				//Buffer overflow
				const char *error = "Error overflow: ";
				HAL_UART_Transmit(&huart1, (uint8_t*)error, 16, (uint16_t) (16 * 2) / (huart1.Init.BaudRate));
				HAL_UART_Transmit(&huart1, buffer, i, (uint16_t) (i * 2) / (huart1.Init.BaudRate));
				uint8_t enter = '\n';
				HAL_UART_Transmit(&huart1, &enter, 1, (uint16_t) (1 * 2) / (huart1.Init.BaudRate));
				return 2;
			} else {
				//Uart timeout
				const char *error = "Error timeout UART: ";
				HAL_UART_Transmit(&huart1, (uint8_t*)error, 20, (uint16_t) (20 * 2) / (huart1.Init.BaudRate));
				HAL_UART_Transmit(&huart1, buffer, i, (uint16_t) (i * 2) / (huart1.Init.BaudRate));
				uint8_t enter = '\n';
				HAL_UART_Transmit(&huart1, &enter, 1, (uint16_t) (1 * 2) / (huart1.Init.BaudRate));
				return 3;
			}

		}
	}
	// Not a start bit
	return 1;
}

uint8_t modbus_handler( void ) {
	//explore dma options
	read_modbus();
	//write_modbus();
	return 0;
}

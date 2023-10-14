/*
 * File:   uart.c
 * Author: Jorik Wittevrongel
 *
 * Created on October 14 2023
 */

#include "main.h"
#include "IO/uart.h"

 uint8_t read_uart(uint8_t *encodedmessage, size_t *length) {
	 HAL_UART_Receive (&huart1, encodedmessage, *length, (uint16_t) (*length * 2) / (huart1.Init.BaudRate));
	 return 0;
 }

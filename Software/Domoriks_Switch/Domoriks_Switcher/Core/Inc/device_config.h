/*
 * File: device_config.h
 * Author: Jorik Wittevrongel
 *
 * Created on October 5 2023
 */

#ifndef DEVICE_CONFIG_H
#define DEVICE_CONFIG_H

//#include "IO/inputs.h"
//#include "IO/outputs.h"

#define DEVICE_ID 20

#define BAUDRATE 115200

#define US_IN_S 1000000
#define UARTPACKET_SIZE 11 //extra pauze bit + start + 8 data + stop
#define UART_BYTE_TIME_US() ((BAUDRATE * UARTPACKET_SIZE / US_IN_S))
#define UART_BYTE_TIME_MS() ((UART_BYTE_TIME_US() / 1000) > 1 ? (UART_BYTE_TIME_US() / 1000) : 1)

#endif //DEVICE_CONFIG_H

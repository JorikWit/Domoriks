/*
 * File: device_config.h
 * Author: Jorik Wittevrongel
 *
 * Created on October 5 2023
 */

#ifndef DEVICE_CONFIG_H
#define DEVICE_CONFIG_H

#define MODBUS_ID 64

#define BAUDRATE 115200
#define US_IN_S 1000000
#define UARTPACKET_SIZE 11 //extra pauze bit + start + 8 data + stop
#define UART_BYTE_TIME_US ((UARTPACKET_SIZE * 1000000UL) / BAUDRATE)
#define UART_BYTE_TIME_MS ((UART_BYTE_TIME_US() / 1000) > 1 ? (UART_BYTE_TIME_US() / 1000) : 1)
#define IDLE_TIMEOUT (UART_BYTE_TIME_US * 3.5)

//#define HOLD_REGS_SIZE (OUTPUTS_SIZE + (INPUTS_SIZE * 10)) //
//#define INPUTS_REGS_SIZE 1 //not used I think

#endif //DEVICE_CONFIG_H

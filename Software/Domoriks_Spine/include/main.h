/*
 * File:   main.h
 * Author: Jorik Wittevrongel
 *
 * Created on Aug 15, 2025
*/

#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
#include <Controllino.h>
#include <Arduino.h>
#endif

#define UART_BUFFER_SIZE  128

#include <stdint.h>
#include <stdbool.h>

extern uint8_t uart_rxBuffer[UART_BUFFER_SIZE];
extern uint8_t new_rxdata;
extern uint16_t rxDataLen;

#endif /* __MAIN_H */

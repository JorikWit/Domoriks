/*
 * File:   modbus.c
 * Author: Jorik Wittevrongel
 *
 * Created on 30 December 2023
 */

/*
 * The modbus implentations is designed to write directly to the Output/Actions Structs
 * This way is is possible to make the main program as compact as possible
 * Maybe I can implement a pointers to the structs to make it easier to program / read
 */

#include "main.h"
#include "timer.h"

#include "device_config.h"

#include "Actions/actions.h"
#include "IO/outputs.h"

#include "Modbus/modbus.h"
#include "Modbus/modbus_rtu.h"
#include "Modbus/modbusm.h"
#include "Modbus/modbusm_handler.h"

#include "Flash/flash.h"

#include <stdio.h>
#include <stdlib.h>

#define REPLY_BUFFER_SIZE 100
uint8_t reply_buffer[REPLY_BUFFER_SIZE];

ModbusMessage recieved_message;
size_t length;

uint8_t wait_for_response = false;

uint32_t resend_timer;
uint8_t resend_count = 0;

void recieve() {
	memset(reply_buffer, 0, sizeof(reply_buffer));
	if (decode_modbus_rtu(received_buffer, uart_index, &recieved_message) == 0) {
		//handle message
		if (modbusm_handle(&recieved_message) != 1) { //TODO check more then ID mismatch

			//send reply
			length = 0;
			encode_modbus_rtu(reply_buffer, &length, &recieved_message);
			HAL_GPIO_WritePin(W_RS485_GPIO_Port, W_RS485_Pin, 1); //Set RS485 in write mode
			HAL_UART_Transmit(&huart1, reply_buffer, length, 2 * length * UART_BYTE_TIME_MS());
			HAL_GPIO_WritePin(W_RS485_GPIO_Port, W_RS485_Pin, 0); //Set RS485 in read mode
		}
	} else {
		//message not modbus rtu;
	}
	//reset for new message
	new_uartstream = false;
	uart_index = 0;
	memset(received_buffer, 0, sizeof(received_buffer));
}

#define SEND_BUFFER_SIZE 100
uint8_t send_buffer[REPLY_BUFFER_SIZE];
ModbusMessage send_message;

void send(){
	for (int i = 0; i < INPUTS_SIZE; i++) {
		EventAction* event = NULL;

		if (inputActions[i].singlePress.send) {
			event = &inputActions[i].singlePress;
		} else if (inputActions[i].doublePress.send) {
			event = &inputActions[i].doublePress;
		} else if (inputActions[i].longPress.send) {
			event = &inputActions[i].longPress;
		} else if (inputActions[i].switchOn.send) {
			event = &inputActions[i].switchOn;
		} else if (inputActions[i].switchOff.send) {
			event = &inputActions[i].switchOff;
		}

		if (event != NULL) {
			if (event->send == 1) {
				if (event->delayAction != nop && event->delay != 0){
					event->send = 2;
				} else if (event->extraEventId != 0){
					event->send = 3;
				} else {
					event->send = 0;
				}

				uint16_t coil_data = 0x1111; //invalid data
				// Handle the action
				if (event->action == toggle) {
					coil_data = 0x5555;				//Domoriks only less data on the bus + faster execution
				} else if (event->action == on) {
					coil_data = 0xFF00;
				} else if (event->action == off) {
					coil_data = 0x0000;
				} else {

				}

				if (event->action != nop) {
					uint16_t coil_address = event->output-1;
					// Construct and send a Modbus message to the active address
					send_message.slave_address = event->id;
					send_message.function_code = WRITE_SINGLE_COIL;
					send_message.data_length = 4; // Data length for writing a single coil
					// Set the coil address and value based on your requirements
					send_message.data[0] = (coil_address >> 8) & 0xFF; // High byte of coil address
					send_message.data[1] = coil_address & 0xFF; // Low byte of coil address
					send_message.data[2] = (coil_data >> 8) & 0xFF;
					send_message.data[3] = coil_data & 0xFF;

					// Set RS485 in write mode and transmit the message
					encode_modbus_rtu(send_buffer, &length, &send_message);
					HAL_GPIO_WritePin(W_RS485_GPIO_Port, W_RS485_Pin, GPIO_PIN_SET);
					HAL_UART_Transmit(&huart1, send_buffer, length, 2 * length * UART_BYTE_TIME_MS());
					HAL_GPIO_WritePin(W_RS485_GPIO_Port, W_RS485_Pin, GPIO_PIN_RESET); // Set RS485 in read mode
					HAL_Delay(UART_BYTE_TIME_MS() * 2);
					resend_timer = TIMER_SET();
					wait_for_response = true;
					return;
				}
			} else if (event->send == 2){
				//send delay

				// Construct and send a Modbus message to the active address
				send_message.slave_address = event->id;
				send_message.function_code = WRITE_MULTI_REGS;
				uint16_t starting_address = 0x0000;
				send_message.data[0] = (starting_address >> 8) & 0xFF;
				send_message.data[1] = starting_address & 0xFF;
				uint16_t quantity_of_registers = 4;
				send_message.data[2] = (quantity_of_registers >> 8) & 0xFF;
				send_message.data[3] = quantity_of_registers & 0xFF;
				send_message.data[4] = 8; // Number of data bytes

				uint8_t coil_address = event->output - 1;
				uint16_t coil_data = 0x1111; //invalid data
				if (event->delayAction == toggle) {
					coil_data = 0x5555;				//Domoriks only less data on the bus + faster execution
				} else if (event->delayAction == on) {
					coil_data = 0xFF00;
				} else if (event->delayAction == off) {
					coil_data = 0x0000;
				} else if (event->delayAction == ondelayoff) {
					coil_data = 0x0F00;
				} else if (event->delayAction == offdelayon) {
					coil_data = 0x00F0;
				}
				send_message.data[5] = (coil_address >> 8) & 0xFF;
				send_message.data[6] = coil_address && 0xFF;
				send_message.data[7] = (coil_data >> 8) & 0xFF;
				send_message.data[8] = coil_data & 0xFF;

				send_message.data[9] = (event->delay >> 8) & 0xFF;
				send_message.data[10] = event->delay & 0xFF;

				send_message.data[11] = event->pwm; // Brightness
				send_message.data[12] = 0; // Brightness

				send_message.data_length = 13; // Data length (address + quantity + byte count + data)

				// Set RS485 in write mode and transmit the message
				encode_modbus_rtu(send_buffer, &length, &send_message);
				HAL_GPIO_WritePin(W_RS485_GPIO_Port, W_RS485_Pin, GPIO_PIN_SET);
				HAL_UART_Transmit(&huart1, send_buffer, length, 2 * length * UART_BYTE_TIME_MS());
				HAL_GPIO_WritePin(W_RS485_GPIO_Port, W_RS485_Pin, GPIO_PIN_RESET); // Set RS485 in read mode
				HAL_Delay(UART_BYTE_TIME_MS() * 2);
				resend_timer = TIMER_SET();
				wait_for_response = true;

				if (event->extraEventId != 0){
					event->send = 3;
				} else {
					event->send = 0;
				}
				return;
			} else if (event->send == 3) {
				//send extra action
				if (event->extraEventId == 255) {
					//Flash_ReadInputActions(inputActions, FLASH_INPUTACTIONS_SIZE); //restore original/start action
					event->send = 0;
				} else {
					//copyEventAction(&extraActions[event->extraEventId], event);
					event->send = 1;
				}
				return;
			} else {
				event->send = 0;
				return;
			}
		}
	}
}


void response() {
	memset(reply_buffer, 0, sizeof(reply_buffer));

	if (new_uartstream) {
		if (decode_modbus_rtu(received_buffer, uart_index, &recieved_message) == 0) {

			//check if message is equal to message send
			wait_for_response = false;
			resend_count = 0;
		} else {
			//message not modbus rtu;
			wait_for_response = false;
			resend_count = 0;
		}
		//reset for new message
		new_uartstream = false;
		uart_index = 0;
		memset(received_buffer, 0, sizeof(received_buffer));
	}

	if (TIMER_ELAPSED_MS(resend_timer, 100) && wait_for_response) {  //resend
		HAL_GPIO_WritePin(W_RS485_GPIO_Port, W_RS485_Pin, GPIO_PIN_SET);
		HAL_UART_Transmit(&huart1, send_buffer, length, 2 * length * UART_BYTE_TIME_MS());
		HAL_GPIO_WritePin(W_RS485_GPIO_Port, W_RS485_Pin, GPIO_PIN_RESET); // Set RS485 in read mode
		resend_timer = TIMER_SET();
		wait_for_response = true;
		resend_count++;
	}

	if (resend_count == 5) {
		wait_for_response = false;
		resend_count = 0;
	}
}

void modbus() {
	if (TIMER_ELAPSED_MS(timer_lastbyte,  UART_BYTE_TIME_MS()) && wait_for_response) {
		response();
		return;
	} else if (TIMER_ELAPSED_MS(timer_lastbyte,  UART_BYTE_TIME_MS()) && new_uartstream && !wait_for_response) { //timer_lastbyte define in main
		recieve();
		return;
	} else if (TIMER_ELAPSED_MS(timer_lastbyte, DEVICE_ID * UART_BYTE_TIME_MS()) && !wait_for_response){ //timer_lastbyte define in main
		send();
		return;
	}
}



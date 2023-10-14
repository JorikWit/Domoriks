/*
 * File:   input_handler.c
 * Author: Jorik Wittevrongel
 *
 * Created on October 14 2023
 */

#include "main.h"
#include "IO/input_handler.h"

uint8_t handle_inputs_change( void ) {
//	for (int i = 0; i < INPUTS_SIZE; i++) {
//	  if (inputs->param->changed){
//		  //detect single press
//		  //detect dubble press
//		  //detect long press
//
//		  //parse holdreg for action to send
//
//		  //send
//		  if (!HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin)) {
//			  b1_pushed = true;
//			  HAL_Delay(300);
//		  } else {
//			  if (b1_pushed) {
//				b1_pushed = false;
//				const char *mesg = ":1\n";
//				HAL_UART_Transmit(&huart1, (uint8_t*)mesg, mesg-S, (uint16_t) (3 * 2) / (huart1.Init.BaudRate));
//				HAL_Delay(300);
//			  }
//		  }
//	  }
//	}
	return 0;
}


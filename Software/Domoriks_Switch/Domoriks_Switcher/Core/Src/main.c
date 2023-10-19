/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
  /* USER CODE END Header */
  /* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>
#include <stdint.h>

#include "timer.h"

#include "IO/uart.h"
#include "IO/inputs.h"
#include "IO/input_handler.h"

#include "IO/outputs.h"

#include "Modbus/modbusm.h"
#include "Modbus/modbus_ascii.h"
#include "Modbus/modbus_rtu.h"
#include "Modbus/modbusm_handler.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim14;
UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

uint8_t uart_recived_byte;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM14_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

//read uart
uint8_t uart_recived_byte;
uint32_t timer_lastbyte;
uint8_t new_uart = false;
uint8_t uart_index = 0;

//parse message
uint8_t encodedmessage[50];
const uint8_t empty_array[50];
size_t length;

//decode message
ModbusMessage decodedmessage;

//modbus registers
uint8_t m_inputs[INPUTS_SIZE / 8 + 1];
uint8_t m_coils[OUTPUTS_SIZE / 8 + 1];
uint16_t m_h_regs[HOLD_REGS_SIZE];
uint16_t m_i_regs[INPUTS_REGS_SIZE];

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  MX_TIM14_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(&htim14);
  HAL_UART_Receive_IT(&huart1, &uart_recived_byte, 1);

  uint32_t timer_blink = TIMER_SET();
  uint32_t timer_debounce = TIMER_SET();

  uint8_t timed_doublepress_index = 255;
  uint32_t timer_doublePress = TIMER_SET();

  uint8_t timed_longpress_index = 255;
  uint32_t timer_longPress = TIMER_SET();

  HAL_GPIO_WritePin(W_RS485_GPIO_Port, W_RS485_Pin, 0); //Set RS485 in read mode

  uint8_t skip_next_release = false;

  uint32_t timer_send_modbus = TIMER_SET();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    //Call every 10ms
    //GLOBAL_TIMER_TICK(); -> interrupt (done in stm32g0xxit.c)

    if (TIMER_ELAPSED_S(timer_blink, 60)) {
      timer_blink = TIMER_SET();
    }

    //read inputs
    update_inputs();

    //BUTTONS
    for (int i = 0; i < INPUTS_SIZE; i++) {
      if (!TIMER_ELAPSED_MS(timer_debounce, 50) && inputs[i].param.button_type == type_pushbutton) {
        inputs[i].param.changed = false;						//ignore changes for 50ms (pushbutton debounce)
      }
      if (skip_next_release && inputs[i].param.changed) {			//if there is a release after long press
        timer_debounce = TIMER_SET(); 							//reset debounce timer
        inputs[i].param.changed = false;						//set change false to skip
        skip_next_release = false;								//only skip one release
      }

      //Pushbutton
      //Single press
      if (inputs[i].param.button_type == type_pushbutton &&		//check if there is a pushbutton on the input
        inputs[i].param.changed &&								//is the value update
        !inputs[i].param.value) {								//is the button released

        //Single press ACTION
        outputs[i].param.value = !outputs[i].param.value;		//toggle same output

        //Double press
        if (TIMER_ELAPSED_MS(timer_doublePress, 500)) {			//if timer is older the
          timer_doublePress = TIMER_SET();					//reset timer
          timed_doublepress_index = i;					    //register that this input is currently timed
        }
        else {												//if pressed again within 500ms of previous press
          if (timed_doublepress_index == i) {					//check if this input is timed

            //Double press ACTION
            for (int j = 0; j < OUTPUTS_SIZE; j++)
              outputs[j].param.value = 1;

          }
        }

      }

      //Long press
      if (inputs[i].param.button_type == type_pushbutton &&		//check if there is a pushbutton on the input
        inputs[i].param.changed &&								//is the value update
        inputs[i].param.value) {								//is button pressed?

        timer_longPress = TIMER_SET();							//start timer
        timed_longpress_index = i;								//register that this input is currently timed
      }
      if (inputs[i].param.button_type == type_pushbutton &&		//check if there is a pushbutton on the input
        inputs[i].param.value &&								//is the button preset down
        timed_longpress_index == i &&							//check if timer is running for this input
        TIMER_ELAPSED_S(timer_longPress, 2)) {					//long press timer if elapsed & button is still pressed -> long press

        //Long press ACTION
        for (int j = 0; j < OUTPUTS_SIZE; j++)
          outputs[j].param.value = false;

        skip_next_release = true;								//next release action must be ignored after long press
      }

      //Switch
      if (inputs[i].param.button_type == type_switch &&			//check if there is a switch on the input
        inputs[i].param.changed) {								//is the value update

        //switch changed ACTION
        outputs[i].param.value = !outputs[i].param.value;		//toggle same output as input

        if (inputs[i].param.value) {
          //switch on ACTION

        }
        else {
          //switch off ACTION

        }
      }

      if (inputs[i].param.changed) {
        timer_debounce = TIMER_SET(); 							//reset debounce timer
        inputs[i].param.changed = false;						//set changed flag to false (input actions handled)
      }
    }

    //MODBUS
    //clear prev message
    //memcpy(&encodedmessage, &empty_array, sizeof(encodedmessage));
    //length = 50;

  //If last uart bit is received more than 100ms && message isnt handled yet; //change with shorter time after tests
    if (TIMER_ELAPSED_MS(timer_lastbyte, 100) && new_uart) {
      if (decode_modbus_rtu(encodedmessage, uart_index, &decodedmessage) == 0) {
        modbusm_handle(&decodedmessage);
        //match pointers
        outputs[0].param.value = !outputs[0].param.value;		//toggle same output

        //send replay
        length = 0;
        encode_modbus_rtu(encodedmessage, &length, &decodedmessage);
        //HAL_GPIO_WritePin(W_RS485_GPIO_Port, W_RS485_Pin, 1); //Set RS485 in write mode
        //HAL_UART_Transmit(&huart1, encodedmessage, length, ((uint16_t)(length * 2) / (huart1.Init.BaudRate)));
        //HAL_GPIO_WritePin(W_RS485_GPIO_Port, W_RS485_Pin, 0); //Set RS485 in read mode
      }
      else if (decode_modbus_ascii((char*)encodedmessage, &decodedmessage) == 0) {
        modbusm_handle(&decodedmessage);
        //toggle W pin
        outputs[1].param.value = !outputs[1].param.value;		//toggle same output

        //send replay
        length = 0;
        encode_modbus_ascii((char*)encodedmessage, &length, &decodedmessage);
        //HAL_GPIO_WritePin(W_RS485_GPIO_Port, W_RS485_Pin, 1); //Set RS485 in write mode
        //HAL_UART_Transmit(&huart1, encodedmessage, length, ((uint16_t)(length * 2) / (huart1.Init.BaudRate)));
        //HAL_GPIO_WritePin(W_RS485_GPIO_Port, W_RS485_Pin, 0); //Set RS485 in read mode
      }
      else {
        //message not modbus;
        __NOP();
      }

      //reset for new message
      new_uart = false;
      memcpy(&encodedmessage, &empty_array, sizeof(encodedmessage));
      uart_index = 0;
    }

    if (TIMER_ELAPSED_S(timer_send_modbus, 1)) {
      timer_send_modbus = TIMER_SET();

      ModbusMessage newMessage;
      newMessage.slave_address = DEVICE_ID;
      newMessage.function_code = 6;
      newMessage.data_length = 4;
      uint8_t d[4] = { 0x00, 0x02, 0xFF, 0x00 };
      for (int i = 0; i < 4; i++) newMessage.data[i] = d[i];
      memcpy(&encodedmessage, &empty_array, sizeof(encodedmessage));
      encode_modbus_rtu(encodedmessage, &length, &newMessage);
      HAL_GPIO_WritePin(W_RS485_GPIO_Port, W_RS485_Pin, 1); //Set RS485 in write mode
      HAL_Delay(1);
      HAL_UART_Transmit(&huart1, encodedmessage, length, ((uint16_t)(length * 2) / (huart1.Init.BaudRate)));
      //while (HAL_UART_GetState(&huart1) == HAL)
      HAL_Delay(1);
      HAL_GPIO_WritePin(W_RS485_GPIO_Port, W_RS485_Pin, 0); //Set RS485 in read mode
    }

    //write outputs
    update_outputs();


    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
  RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
    | RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM14 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM14_Init(void)
{

  /* USER CODE BEGIN TIM14_Init 0 */

  /* USER CODE END TIM14_Init 0 */

  /* USER CODE BEGIN TIM14_Init 1 */

  /* USER CODE END TIM14_Init 1 */
  htim14.Instance = TIM14;
  htim14.Init.Prescaler = 159;
  htim14.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim14.Init.Period = 1000;
  htim14.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim14.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim14) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM14_Init 2 */

  /* USER CODE END TIM14_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = { 0 };

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, W_RS485_Pin | L6_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, L1_Pin | L2_Pin | L3_Pin | L4_Pin
    | L5_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : W_RS485_Pin L6_Pin */
  GPIO_InitStruct.Pin = W_RS485_Pin | L6_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : B1_Pin B2_Pin B3_Pin B4_Pin
                           B5_Pin B6_Pin */
  GPIO_InitStruct.Pin = B1_Pin | B2_Pin | B3_Pin | B4_Pin
    | B5_Pin | B6_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : L1_Pin L2_Pin L3_Pin L4_Pin
                           L5_Pin */
  GPIO_InitStruct.Pin = L1_Pin | L2_Pin | L3_Pin | L4_Pin
    | L5_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

void HAL_UART_RxCpltCallback(UART_HandleTypeDef* huart)
{
  timer_lastbyte = TIMER_SET();						 //reset timer (time passed since last recieved byte)
  new_uart = true;									 //indicate new message

  encodedmessage[uart_index++] = uart_recived_byte;    //copy received byte and shift index

  HAL_UART_Receive_IT(&huart1, &uart_recived_byte, 1); //Set new interrupt
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
     /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

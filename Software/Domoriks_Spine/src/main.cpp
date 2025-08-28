#include "main.h"
#include "outputs.h"
#include "modbus.h"
#include "modbusm_handler.h"
#include "timer.h"
#include "device_config.h"

// uint8_t uart_rxBuffer[UART_BUFFER_SIZE];
// uint8_t new_rxdata;
// uint16_t rxDataLen;

// Temporary buffers for each serial
uint8_t serial_buffer[UART_BUFFER_SIZE];
uint8_t serial3_buffer[UART_BUFFER_SIZE];

// Merged buffer for Modbus processing
uint8_t uart_rxBuffer[UART_BUFFER_SIZE];

volatile uint16_t serial_index = 0;
volatile uint16_t serial3_index = 0;

uint16_t rxDataLen = 0;   // length of merged data
uint8_t new_rxdata = 0;   // flag to signal idle

unsigned long lastByteTime = 0;

#include <Controllino.h>

void serialRead();

void setup() {
  init_outputs();

  Controllino_RS485Init(115200);
  Controllino_RS485RxEnable();
  Serial.begin(115200);
}

void loop() {
  //blink
  static uint32_t timer_blink = 0;
  if (TIMER_ELAPSED_MS(timer_blink, 750)) {
    timer_blink = TIMER_SET();
    digitalWrite(CONTROLLINO_D0, !digitalRead(CONTROLLINO_D0));
  }

  serialRead();

  //prep registers
  modbus_get_outputs();

  //modbus RS485
  modbus();

  //Parse single output
  modbus_set_outputs();
  modbus_parse_register(); // always after set. this write outputs directly
  //modbus_parse_action_update();

  //write outputs
  update_outputs();
}

unsigned long lastByteTimeSerial = 0;
unsigned long lastByteTimeSerial3 = 0;

void serialRead() {
    // Read USB Serial
    while (Serial.available() > 0 && serial_index < UART_BUFFER_SIZE) {
        serial_buffer[serial_index++] = Serial.read();
        lastByteTimeSerial = micros();
    }

    // Read RS485 Serial3
    while (Serial3.available() > 0 && serial3_index < UART_BUFFER_SIZE) {
        serial3_buffer[serial3_index++] = Serial3.read();
        lastByteTimeSerial3 = micros();
    }

    // Check idle timeout for Serial
    if (serial_index > 0 && (micros() - lastByteTimeSerial > IDLE_TIMEOUT) && !new_rxdata) {
        rxDataLen = serial_index;
        if (rxDataLen > UART_BUFFER_SIZE) rxDataLen = UART_BUFFER_SIZE;

        memcpy(uart_rxBuffer, serial_buffer, rxDataLen);
        serial_index = 0;
        new_rxdata = 1;  // signal packet ready
    }

    // Check idle timeout for Serial3
    if (serial3_index > 0 && (micros() - lastByteTimeSerial3 > IDLE_TIMEOUT) && !new_rxdata) {
        rxDataLen = serial3_index;
        if (rxDataLen > UART_BUFFER_SIZE) rxDataLen = UART_BUFFER_SIZE;

        memcpy(uart_rxBuffer, serial3_buffer, rxDataLen);
        serial3_index = 0;
        new_rxdata = 1;  // signal packet ready
    }
}

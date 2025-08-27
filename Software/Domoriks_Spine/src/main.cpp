#include "main.h"
#include "outputs.h"
#include "modbus.h"
#include "modbusm_handler.h"
#include "timer.h"
#include "device_config.h"

uint8_t uart_rxBuffer[UART_BUFFER_SIZE];
uint8_t new_rxdata;
uint16_t rxDataLen;


#include <Controllino.h>

void serialRead();

void setup() {
  init_outputs();

  Controllino_RS485Init(115200);
  Controllino_RS485RxEnable();
  
  Serial.begin(115200);
  Serial.println("Controllino Booted");
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

volatile uint8_t rxIndex = 0;
unsigned long lastByteTime = 0;

void serialRead() {
  // Check for idle condition
  if (rxIndex > 0 && (micros() - lastByteTime > IDLE_TIMEOUT) && !new_rxdata) {
    rxDataLen = rxIndex;   // store received length
    rxIndex = 0;           // reset index for next frame
    new_rxdata = true;     // signal new packet ready
    return;                // exit, don't read more
  }

  // Read incoming data while no unprocessed packet exists
  while (Serial3.available() > 0 && !new_rxdata) {
    uint8_t incomingByte = Serial3.read();

    if (rxIndex < UART_BUFFER_SIZE - 1) {
      uart_rxBuffer[rxIndex++] = incomingByte;
    }

    lastByteTime = micros();  // Update timestamp in µs
  }
}
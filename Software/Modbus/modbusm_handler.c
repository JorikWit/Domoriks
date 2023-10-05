/*
 * File:   modbus_function_handler.c
 * Author: Jorik Wittevrongel
 *
 * Created on October 5 2023
 */

#include "modbusm_handler.h"

 /*

make functionhandler template with switch case struct for all modbus functions

01 (0x01) Read Coils
02 (0x02) Read Discrete Inputs
03 (0x03) Read Holding Registers
04 (0x04) Read Input Registers
05 (0x05) Write Single Coil
06 (0x06) Write Single Register
08 (0x08) Diagnostics (Serial Line only)
11 (0x0B) Get Comm Event Counter (Serial Line only)
15 (0x0F) Write Multiple Coils
16 (0x10) Write Multiple Registers
17 (0x11) Report Server ID (Serial Line only)
22 (0x16) Mask Write Register
23 (0x17) Read/Write Multiple Registers

43 / 14 (0x2B / 0x0E) Read Device Identification

 */

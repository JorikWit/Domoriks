/*
 * File:   modbus_function_handler.c
 * Author: Jorik Wittevrongel
 *
 * Created on October 5 2023
 */

#include "Modbus/modbusm_handler.h"

 /*

make functionhandler template with switch case struct for all modbus functions

01 (0x01) Read Coils
02 (0x02) Read Discrete Inputs
03 (0x03) Read Holding Registers
04 (0x04) Read Input Registers
05 (0x05) Write Single Coil
06 (0x06) Write Single Register
*08 (0x08) Diagnostics (Serial Line only)
*11 (0x0B) Get Comm Event Counter (Serial Line only)
15 (0x0F) Write Multiple Coils
16 (0x10) Write Multiple Registers
*17 (0x11) Report Server ID (Serial Line only)
*22 (0x16) Mask Write Register
*23 (0x17) Read/Write Multiple Registers
*43 / 14 (0x2B / 0x0E) Read Device Identification

 */

uint8_t* modbusCoils;
uint8_t* modbusInputs;
uint16_t* modbusHReg;
uint16_t* modbusIReg;

uint8_t modbusm_handle(ModbusMessage* message) {
    //check if device ID matches
    if (message->slave_address != DEVICE_ID) {
        message = NULL;
        return ID_MISMATCH;
    }

    //handle message
    switch (message->function_code) {
    case READ_COILS: ;
        //parse
        uint16_t first_coil = 0;
        uint16_t amount_coils = 0;
        if (message->data_length == 4) {
            first_coil = (message->data[0] << 8 | message->data[1]);
            amount_coils = (message->data[2] << 8 | message->data[3]);
        }
        else {
            message = NULL;
            return INVALID_DATA_LENGHT;
        }
        //ADD MAX VALUE ERROR
        /*
        pressent in device_config
        #define MAX_COILS 256
        #define MAX_INPUTS 256
        #define MAX_HOLD_REGS 64
        #define MAX_INPUTS_REGS 64
        */

        //reply
        message->slave_address = DEVICE_ID;
        message->function_code = READ_COILS;
        message->data_length = amount_coils % 8 ? 2 + (amount_coils / 8) : 1 + (amount_coils / 8);
        message->data[0] = message->data_length - 1;
        for (int i = 1; i < message->data_length; i++) {
            uint8_t currentB = *(modbusCoils + ((first_coil / 8) + (i - 1)));
            uint8_t nextB = *(modbusCoils + ((first_coil / 8) + (i)));
            *(message->data + i) = (nextB << (8 - (first_coil % 8)) | currentB >> (first_coil % 8));
        }
        break;
    case READ_DISC_INPUTS: ;
        //parse
        uint16_t first_input = 0;
        uint16_t amount_inputs = 0;
        if (message->data_length == 4) {
            first_input = (message->data[0] << 8 | message->data[1]);
            amount_inputs = (message->data[2] << 8 | message->data[3]);
        }
        else {
            message = NULL;
            return INVALID_DATA_LENGHT;
        }
        //ADD MAX VALUE ERROR

        //reply
        message->slave_address = DEVICE_ID;
        message->function_code = READ_DISC_INPUTS;
        message->data_length = amount_inputs % 8 ? 2 + (amount_inputs / 8) : 1 + (amount_inputs / 8);
        *message->data = message->data_length - 1;
        for (int i = 1; i < message->data_length; i++) {
            uint8_t currentB = *(modbusInputs + ((first_input / 8) + (i - 1)));
            uint8_t nextB = *(modbusInputs + ((first_input / 8) + (i)));
            *(message->data + i) = ( ((nextB << 8) - (first_input % 8)) | (currentB >> (first_input % 8)) );
        }
        break;
    case READ_HOLD_REGS: ;
        //parse
        uint16_t first_hreg = 0;
        uint16_t amount_hregs = 0;
        if (message->data_length == 4) {
            first_hreg = (message->data[0] << 8 | message->data[1]);
            amount_hregs = (message->data[2] << 8 | message->data[3]);
        }
        else {
            message = NULL;
            return INVALID_DATA_LENGHT;
        }
        //ADD MAX VALUE ERROR

        //reply
        message->slave_address = DEVICE_ID;
        message->function_code = READ_HOLD_REGS;
        message->data_length = 1 + (amount_hregs * 2);
        *message->data = message->data_length - 1;
        for (int i = 1; i < message->data_length; i++) {
            *(message->data + (i * 2)) = *(modbusHReg + i - 1 + first_hreg) >> 8;
            *(message->data + (i * 2) - 1) = *(modbusHReg + i - 1 + first_hreg);
        }
        break;
    case READ_INPUT_REGS: ;
        //parse
        uint16_t first_ireg = 0;
        uint16_t amount_iregs = 0;
        if (message->data_length == 4) {
            first_ireg = (message->data[0] << 8 | message->data[1]);
            amount_iregs = (message->data[2] << 8 | message->data[3]);
        }
        else {
            //invalid length
            return 2;
        }
        //reply
        message->slave_address = DEVICE_ID;
        message->function_code = READ_INPUT_REGS;
        message->data_length = 1 + (amount_iregs * 2);
        *message->data = message->data_length - 1;
        for (int i = 1; i < message->data_length; i++) {
            *(message->data + (i * 2)) = *(modbusIReg + i - 1 + first_ireg) >> 8;
            *(message->data + (i * 2) - 1) = *(modbusIReg + i - 1 + first_ireg);
        }
        break;
    case WRITE_SINGLE_COIL: ;
        //parse
        uint16_t coil_adress = 0;
        uint16_t value = 0;
        if (message->data_length == 4) {
            coil_adress = (message->data[0] << 8 | message->data[1]);
            value = (message->data[2] << 8 | message->data[3]);
        }
        else {
            return INVALID_DATA_LENGHT;
        }
        //ADD MAX VALUE ERROR

        //write coil;
        printf("%02X, %02X\n", coil_adress, value);
        if (value == 0xFF00) {
            modbusCoils[coil_adress / 8] |= (1 << (coil_adress % 8));
        }
        else if (value == 0x0000) {
            modbusCoils[coil_adress / 8] = modbusCoils[coil_adress / 8] & ~(1 << (coil_adress % 8));
        }
        else {
            message = NULL;
            return INVALID_COIL_VALUE;
        }
        message = message; //echo message
        break;
    case WRITE_SINGLE_REG: ;      //   <- Tis only apply for holding regs
        //parse
        uint16_t reg_adress = 0;
        if (message->data_length == 4) {
            reg_adress = (message->data[0] << 8 | message->data[1]);
        }
        else {
            return INVALID_DATA_LENGHT;
        }
        //write regs
        modbusHReg[reg_adress] = message->data[2] << 8 | message->data[3];
        //reply
        message = message; //echo message
        break;
    case READ_EXCEPTION_STATUS:
    case DIAGNOSE_SERIAL:
    case COMM_EVENT_COUNT:
    case WRITE_MULTI_COILS:
    case WRITE_MULTI_REGS:
    case REPORT_SERVER_ID:
    case READ_DEVICE_ID:
    case MASK_WRITE_REG:
    case RW_MULTI_REGS:
    case READ_DEVICE_ID_:
        return NOT_IMPLEMENTED;
        break;
    default:
        return INVALID_FUNCTION;
    }

    return HANDLED_OK;
}

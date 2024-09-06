/*
 * flash.c
 *
 *  Created on: Jul 16, 2024
 *      Author: Jorik Wittevrongel
 */

#include "Flash/flash.h"

// Generic function to write data to 5las7 in chunks
HAL_StatusTypeDef Flash_Write(uint32_t addr, void *data, uint32_t size) {
    HAL_StatusTypeDef status = HAL_OK;
    uint32_t remainingSize = size;
    uint8_t *pData = (uint8_t *)data;

    HAL_FLASH_Unlock();

    while (remainingSize > 0) {
        uint64_t data64 = 0;

        // Prepare 64-bit data from pData (little-endian order for STM32)
        for (int i = 0; i < sizeof(uint64_t) && i < remainingSize; i++) {
            data64 |= ((uint64_t)pData[i]) << (i * 8);
        }

        // Program double word (64 bits) at a time
        status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, addr, data64);
        if (status != HAL_OK) {
            break;
        }

        addr += sizeof(uint64_t);
        pData += sizeof(uint64_t);
        remainingSize -= sizeof(uint64_t);
    }

    HAL_FLASH_Lock();
    return status;
}

// Function to read data from flash
HAL_StatusTypeDef Flash_Read(uint32_t addr, void *data, uint32_t size) {
    HAL_StatusTypeDef status = HAL_OK;
    memcpy(data, (void *)addr, size);
    return status;
}

// Write and read functions for inputs
HAL_StatusTypeDef Flash_WriteInputs(Input *inputs, uint32_t size) {
    return Flash_Write(FLASH_INPUTS_ADDR, (void *)inputs, size * sizeof(Input));
}

HAL_StatusTypeDef Flash_ReadInputs(Input *inputs, uint32_t size) {
    return Flash_Read(FLASH_INPUTS_ADDR, (void *)inputs, size * sizeof(Input));
}

// Write and read functions for inputActions
HAL_StatusTypeDef Flash_WriteInputActions(InputAction *inputActions, uint32_t size) {
    return Flash_Write(FLASH_INPUTACTIONS_ADDR, (void *)inputActions, size * sizeof(InputAction));
}

HAL_StatusTypeDef Flash_ReadInputActions(InputAction *inputActions, uint32_t size) {
    return Flash_Read(FLASH_INPUTACTIONS_ADDR, (void *)inputActions, size * sizeof(InputAction));
}

// Write and read functions for extraActions
HAL_StatusTypeDef Flash_WriteExtraActions(EventAction *extraActions, uint32_t size) {
    return Flash_Write(FLASH_EXTRAACTIONS_ADDR, (void *)extraActions, size * sizeof(EventAction));
}

HAL_StatusTypeDef Flash_ReadExtraActions(EventAction *extraActions, uint32_t size) {
    return Flash_Read(FLASH_EXTRAACTIONS_ADDR, (void *)extraActions, size * sizeof(EventAction));
}

// Write and read functions for outputs
HAL_StatusTypeDef Flash_WriteOutputs(Output *outputs, uint32_t size) {
    return Flash_Write(FLASH_OUTPUTS_ADDR, (void *)outputs, size * sizeof(Output));
}

HAL_StatusTypeDef Flash_ReadOutputs(Output *outputs, uint32_t size) {
    return Flash_Read(FLASH_OUTPUTS_ADDR, (void *)outputs, size * sizeof(Output));
}


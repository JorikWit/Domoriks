/*
 * File: device_config.h
 * Author: Jorik Wittevrongel
 *
 * Created on October 5 2023
 */

#ifndef DEVICE_CONFIG_H
#define DEVICE_CONFIG_H

#include "IO/inputs.h"
#include "IO/outputs.h"

#define DEVICE_ID 1

#define HOLD_REGS_SIZE (OUTPUTS_SIZE + (INPUTS_SIZE * 10)) //
#define INPUTS_REGS_SIZE 1 //not used I think

#endif //DEVICE_CONFIG_H

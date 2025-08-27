/*
 * File:   outputs.h
 * Author: Jorik Wittevrongel
 *
 * Created on December 20 2022
 */

#ifndef OUTPUTS_H
#define	OUTPUTS_H

#include <Arduino.h>
#include <Controllino.h>
#include "timer.h"

#include <stdbool.h>
#include <stdint.h>

#include "main.h"
#include "timer.h"

#define OUTPUTS_SIZE 16

typedef struct  __attribute__((packed)){             //Combine with Inputparam in IO param
	  uint16_t pin;
  	uint8_t value;
  	uint8_t invert;
  	uint8_t min;
  	uint8_t max;
  	uint16_t delay;
  	uint8_t delay_value;
  	uint32_t startTimer;
} OutputParam;

typedef uint8_t (*pfnOutputInit)(OutputParam*);
typedef uint8_t (*pfnOutputUpdate)(OutputParam*);

typedef struct {
	OutputParam param;
	pfnOutputInit initFunction;
	pfnOutputUpdate updateFunction;
} Output;

extern Output outputs[OUTPUTS_SIZE];

uint8_t init_outputs(void); 
uint8_t update_outputs(void);

#endif	/* OUTPUTS_H */

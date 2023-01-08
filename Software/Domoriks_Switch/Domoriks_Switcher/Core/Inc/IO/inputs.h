/*
 * File:   inputs.h
 * Author: Jorik Wittevrongel
 *
 * Created on December 20 2022, 17:40
 */

#ifndef INPUTS_H
#define	INPUTS_H

typedef struct {             //Combine with Outputparam in IO param
	uint16_t pin;
  	const int port;
  	uint8_t value;
  	uint8_t min;
  	uint8_t max;
} InputParam;

typedef uint8_t (*pfnInputInit)(InputParam*);
typedef uint8_t (*pfnInputUpdate)(InputParam*);

typedef struct {
	InputParam param;
	pfnInputInit initFunction;
	pfnInputUpdate updateFunction;
} Input;

#define INPUTS_SIZE 6
extern Input inputs[INPUTS_SIZE];

uint8_t update_inputs( void );

#endif	/* INPUTS_H */

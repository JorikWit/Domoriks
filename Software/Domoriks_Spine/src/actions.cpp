/*
 * File:   actions.c
 * Author: Jorik Wittevrongel
 *
 * Created on December 20 2022, 17:40
 */

#include "device_config.h"
#include "actions.h"
#include "main.h"

#define NO_ACTION {nop, nop, 0, 100, 0, 0, 0, 0}

InputAction inputActions[1] = {
	{NO_ACTION, NO_ACTION, NO_ACTION, NO_ACTION, NO_ACTION}
};

EventAction extraActions[1] = {
		NO_ACTION
};

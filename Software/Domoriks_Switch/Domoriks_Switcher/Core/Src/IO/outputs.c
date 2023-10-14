/*
 * File:   outputs.c
 * Author: Jorik Wittevrongel
 *
 * Created on December 20 2022, 19:02
 */

#include "IO/outputs.h"

uint8_t init_do(OutputParam *param) {
	//Not needed done by generated code
	return 0;
}

uint8_t write_do(OutputParam *param) {
	HAL_GPIO_WritePin((GPIO_TypeDef *)param->port, param->pin, param->value);
	return 0;
}

uint8_t write_pwm(OutputParam *param) {
	return 1; //TODO
}

Output outputs[OUTPUTS_SIZE] = {
//		{{pin,    port,             value,min,max} init_func, update_func}
		{{L1_Pin,(const int)L1_GPIO_Port,0,0,1}, &init_do, &write_do},
		{{L2_Pin,(const int)L2_GPIO_Port,0,0,1}, &init_do, &write_do},
		{{L3_Pin,(const int)L3_GPIO_Port,0,0,1}, &init_do, &write_do},
//		{{pin,    port,             value,min,max} init_func, update_func}
		{{L4_Pin,(const int)L4_GPIO_Port,0,0,1}, &init_do, &write_do},
		{{L5_Pin,(const int)L5_GPIO_Port,0,0,1}, &init_do, &write_do},
		{{L6_Pin,(const int)L6_GPIO_Port,0,0,1}, &init_do, &write_do}
//		{{pin,    port,             value,min,max} init_func, update_func}
};

uint8_t update_outputs() {
    uint8_t nok = 0;
    Output *ptr = outputs;
    for (int i = 0; i < OUTPUTS_SIZE; i++) {
        if (!ptr->updateFunction(&ptr->param))
            nok = 1;
        ptr++;
    }
    return nok;
}

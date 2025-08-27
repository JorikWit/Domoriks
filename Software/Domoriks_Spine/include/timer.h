/* 
 * File:   timer.h
 * Author: Jorik Wittevrongel
 *
 * Created on May 11, 2022
 */

#ifndef TIMER_H
#define	TIMER_H

#include <stdint.h>
#include <stdbool.h>
#include <Arduino.h>

#define TIMER_SET() (millis())
#define TIMER_ELAPSED_MS(t, ms) ((uint32_t)(millis() - (t)) > (uint32_t)(ms))
#define TIMER_ELAPSED_S(t, s)   ((uint32_t)(millis() - (t)) > ((uint32_t)(s) * 1000))

#endif	/* TIMER_H */

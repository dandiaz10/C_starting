/*
 * debounce.h
 *
 *  Created on: Apr 10, 2020
 *      Author: Dann
 */

#ifndef SRC_DEBOUNCE_H_
#define SRC_DEBOUNCE_H_

#include <stdint.h>
#include "stm32l4xx_hal.h"

// Declaring functions

void debounceInit();
void deBounceInit(uint16_t pin, char port, int8_t mode);
int8_t deBounceReadPin(uint16_t pin, char port, int8_t mode);

#endif /* SRC_DEBOUNCE_H_ */

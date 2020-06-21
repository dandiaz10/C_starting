/*
 * debounce.c
 *
 *  Created on: Apr 19, 2020
 *      Author: Dann
 */
#include "debounce.h"
#include <stdint.h>
#include "stm32l4xx_hal.h"
// Defining functions

void debounceInit()
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = GPIO_PIN_0;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}


void deBounceInit(uint16_t pin, char port, int8_t mode)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0}; // Declaring GPIO_initStruct as GPIO_InitTypeDef Struc

	GPIO_InitStruct.Pin = pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;

	switch(mode)
	{
		case 0:
			GPIO_InitStruct.Pull = GPIO_PULLUP;
			break;
		case 1:
			GPIO_InitStruct.Pull = GPIO_PULLDOWN;
			break;
	}

	switch(port)
	{
		case 'A':
			HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
			break;
		case 'B':
				HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
				break;
		case 'C':
				HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
				break;
	}

}

int8_t deBounceReadPin(uint16_t pin, char port, int8_t mode)
{


	GPIO_PinState pinState = GPIO_PIN_RESET; // pin status

	int8_t pinStateWeAreLookingFor;
	int8_t stableInterval = 50; // stable interval

	int32_t msTimeStamp=HAL_GetTick();

	switch(port)
		{
			case 'A':

				pinState = HAL_GPIO_ReadPin(GPIOA, pin);
				break;
			case 'B':
				pinState = HAL_GPIO_ReadPin(GPIOB, pin);
					break;
			case 'C':
				pinState = HAL_GPIO_ReadPin(GPIOC, pin);
					break;
		}

	if(pinState == GPIO_PIN_RESET) //check if the button  is pressed
	{
		pinStateWeAreLookingFor = 0;
	}
	else
	{
		pinStateWeAreLookingFor = 1;
	}

	while(HAL_GetTick() < (msTimeStamp + stableInterval))
	{

		switch(port)
		{
			case 'A':

				pinState = HAL_GPIO_ReadPin(GPIOA, pin);
				break;
			case 'B':
				pinState = HAL_GPIO_ReadPin(GPIOB, pin);
					break;
			case 'C':
				pinState = HAL_GPIO_ReadPin(GPIOC, pin);
					break;
		}
		if (pinState != pinStateWeAreLookingFor)
		{
			pinStateWeAreLookingFor =! pinStateWeAreLookingFor;
			msTimeStamp = HAL_GetTick();
		}
	}

	return (pinStateWeAreLookingFor);
}



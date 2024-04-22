/*
 * stm32f4xx_rcc_driver.h
 *
 *  Created on: Apr 19, 2024
 *      Author: leo
 */

#ifndef INC_STM32F4XX_RCC_DRIVER_H_
#define INC_STM32F4XX_RCC_DRIVER_H_


#include "stm32f407xx.h"

uint32_t RCC_GetPLLOutputClock();
uint32_t RCC_GetPCLK1Value(void);
uint32_t RCC_GetPCLK2Value(void);

#endif /* INC_STM32F4XX_RCC_DRIVER_H_ */

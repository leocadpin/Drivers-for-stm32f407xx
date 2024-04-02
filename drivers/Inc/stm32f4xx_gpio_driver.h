
#ifndef INC_STM32F407XX_GPIO_DRIVER_H_
#define INC_STM32F407XX_GPIO_DRIVER_H_

#include "stm32f407xx.h"



//Configuration structure for GPIO pin

typedef struct
{
	uint8_t GPIO_PinNumber;
	uint8_t GPIO_PinMode;
	uint8_t GPIO_PinSpeed;
	uint8_t GPIO_PinPuPdControl;
	uint8_t GPIO_PinOPType;
	uint8_t GPIO_PinAltFunMode;
}GPIO_PinConfig_t;

//Handle structure for GPIO

typedef struct
{

	GPIO_RegDef_t *pGPIOx; //Hold the base addres of the GPIO port
	GPIO_PinConfig_t GPIO_PinConfig; //Hold pin configuration settings

}GPIO_Handle_t;

#endif

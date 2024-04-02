
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


//Prototipes for API


void GPIO_PeriClock_Control(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi);

void GPIO_Init(GPIO_Handle_t *pGPIOx); //Initialize GPIO port and pin
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx);



uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx);
void GPIO_write_to_output_pin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value);
void GPIO_write_to_output_port(GPIO_RegDef_t *pGPIOx, uint16_t Value);
void GPIO_toggle_output_pin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);


void GPIO_IRQConfig(uint8_t IRQNumber, uint8_t IRQPriority, uint8_t EnorDi);
void GPIO_IRQHandling(uint8_t PinNumber);




#endif

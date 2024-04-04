
#ifndef INC_STM32F407XX_GPIO_DRIVER_H_
#define INC_STM32F407XX_GPIO_DRIVER_H_

#include "stm32f407xx.h"



//Configuration structure for GPIO pin

typedef struct
{
	uint8_t GPIO_PinNumber;
	uint8_t GPIO_PinMode;           //Possible values from @GPIO_PIN_MODES
	uint8_t GPIO_PinSpeed;			//Possible values from @GPIO_PIN_SPEEDS
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

/*
 * @GPIO_PIN_NUMBERS
 * GPIO pin numbers
 */
#define GPIO_PIN_NO_0    0
#define GPIO_PIN_NO_1    1
#define GPIO_PIN_NO_2    2
#define GPIO_PIN_NO_3    3
#define GPIO_PIN_NO_4    4
#define GPIO_PIN_NO_5    5
#define GPIO_PIN_NO_6    6
#define GPIO_PIN_NO_7    7
#define GPIO_PIN_NO_8    8
#define GPIO_PIN_NO_9    9
#define GPIO_PIN_NO_10  10
#define GPIO_PIN_NO_11  11
#define GPIO_PIN_NO_12  12
#define GPIO_PIN_NO_13  13
#define GPIO_PIN_NO_14  14
#define GPIO_PIN_NO_15  15


//@GPIO pin possible modes

#define GPIO_MODE_INPUT  0
#define GPIO_MODE_OUTPUT 1
#define GPIO_MODE_ALTFN  2
#define GPIO_MODE_ANALOG 3
#define GPIO_MODE_IT_FT  4
#define GPIO_MODE_IT_RT  5
#define GPIO_MODE_IT_RFT 6

//GPIO pin possible output types
#define GPIO_OP_TYPE_PP 0
#define GPIO_OP_TYPE_OD 1

// @GPIO_PIN_SPEEDS
//GPIO pin possible output types
#define GPIO_SPEED_LOW    0
#define GPIO_SPEED_MEDIUM 1
#define GPIO_SPEED_FAST   2
#define GPIO_SPEED_HIGH   3

//Pull-up or pull-down configurations

#define GPIO_NO_PUPD      0
#define GPIO_PIN_PU       1
#define GPIO_PIN_PD       2

//Prototipes for API


void GPIO_PeriClock_Control(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi);

void GPIO_Init(GPIO_Handle_t *); //Initialize GPIO port and pin
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx);



uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx);
void GPIO_write_to_output_pin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value);
void GPIO_write_to_output_port(GPIO_RegDef_t *pGPIOx, uint16_t Value);
void GPIO_toggle_output_pin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);


void GPIO_IRQConfig(uint8_t IRQNumber, uint8_t IRQPriority, uint8_t EnorDi);
void GPIO_IRQHandling(uint8_t PinNumber);




#endif

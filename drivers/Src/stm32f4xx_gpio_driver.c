#include "stm32f4xx_gpio_driver.h"
/**************************************************
 * @fn             -
 *
 * @brief          -
 *
 * @param[in]      -
 * @param[in]      -
 * @param[in]      -
 *
 * @return         - none
 *
 * @Note           - none
 */

/*
 * Peripheral Clock Setup
 */

/**************************************************
 * @fn             - GPIO_PeriClock_Control
 *
 * @brief          - Enables or disable periheral clock for the giben gpio port
 *
 * @param[in]      - base address of the peripheral
 * @param[in]      - ENABLE or disable macros
 *
 *
 * @return         - none
 *
 * @Note           - none
 */
void GPIO_PeriClock_Control(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi)
{

	if(EnorDi == ENABLE)
	{
		if(pGPIOx == GPIOA)
		{
			GPIOA_PCLK_EN();
		}
		else if(pGPIOx == GPIOB)
		{
			GPIOB_PCLK_EN();
		}
		else if(pGPIOx == GPIOC)
		{
			GPIOC_PCLK_EN();
		}
		else if(pGPIOx == GPIOD)
		{
			GPIOD_PCLK_EN();
		}
		else if(pGPIOx == GPIOE)
		{
			GPIOE_PCLK_EN();
		}
		else if(pGPIOx == GPIOF)
		{
			GPIOF_PCLK_EN();
		}
		else if(pGPIOx == GPIOG)
		{
			GPIOG_PCLK_EN();
		}
		else if(pGPIOx == GPIOH)
		{
			GPIOH_PCLK_EN();
		}
		else if(pGPIOx == GPIOI)
		{
			GPIOI_PCLK_EN();
		}

	}
	else
	{
		if(pGPIOx == GPIOA)
		{
			GPIOA_PCLK_DIS();
		}
		else if(pGPIOx == GPIOB)
		{
			GPIOB_PCLK_DIS();
		}
		else if(pGPIOx == GPIOC)
		{
			GPIOC_PCLK_DIS();
		}
		else if(pGPIOx == GPIOD)
		{
			GPIOD_PCLK_DIS();
		}
		else if(pGPIOx == GPIOE)
		{
			GPIOE_PCLK_DIS();
		}
		else if(pGPIOx == GPIOF)
		{
			GPIOF_PCLK_DIS();
		}
		else if(pGPIOx == GPIOG)
		{
			GPIOG_PCLK_DIS();
		}
		else if(pGPIOx == GPIOH)
		{
			GPIOH_PCLK_DIS();
		}
		else if(pGPIOx == GPIOI)
		{
			GPIOI_PCLK_DIS();
		}
	}

}


/**************************************************
 * @fn             - GPIO_Init
 *
 * @brief          - Initialization of a GPIO pin
 *
 * @param[in]      - Structure gpio handle with config atributes
 *
 * @return         - none
 *
 * @Note           - none
 */

void GPIO_Init(GPIO_Handle_t *pGPIOHandle)
{
	uint32_t temp = 0; //temp. register

	//1-configure the mode of gpio pin

	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG)
	{
		temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber) );

		pGPIOHandle->pGPIOx->MODER &= ~(0x3 << (2*pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
		pGPIOHandle->pGPIOx->MODER |= temp;
		temp = 0;

	}
	else
	{
		if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_FT)
		{
			EXTI->EXTI_FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

			//clear rtsr bit

			EXTI->EXTI_RTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}
		else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RT)
		{
			EXTI->EXTI_RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

			//clear ftsr bit

			EXTI->EXTI_FTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}
		else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RFT)
		{
			EXTI->EXTI_FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			EXTI->EXTI_RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}

		//Configure the gpio port selection in SYSCFG_EXTICR
		uint8_t temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 4;
		uint8_t temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 4;
		uint8_t portcode = GPIO_BASEADDR_TO_CODE(pGPIOHandle->pGPIOx);
		SYSCFG_PCLK_EN();
		SYSCFG->SYSCFG_EXTICR[temp1] |= portcode << ( temp2 * 4);

		//Enable the exti interrupt delivery using IMR
		EXTI->EXTI_IMR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

	}

	temp = 0;
   //2-Configure the speed
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << (2 *pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed));
	pGPIOHandle->pGPIOx->OSPEEDR &= ~(0x3 << (2*pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OSPEEDR |= temp;

	temp = 0;
   //3-Configure the pupd settings
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl << (2 *pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl));
	pGPIOHandle->pGPIOx->PUPDR &= ~(0x3 << (2*pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->PUPDR |= temp;

	temp = 0;
   //4-configure the op type
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType << pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType);
	pGPIOHandle->pGPIOx->OTYPER &= ~(0x1 << (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OTYPER |= temp;

	temp = 0;

	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ALTFN)
	{
		//Configure the alt function
		uint32_t temp1;
		uint32_t temp2;

		temp1 = (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber) / 8 ;
		temp2 = (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber) % 8 ;
		pGPIOHandle->pGPIOx->AFR[temp1] &= ~(0xF << ( 4 * temp2 ));
		pGPIOHandle->pGPIOx->AFR[temp1] |= (pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << ( 4 * temp2 ));
	}
}

/**************************************************
 * @fn             - GPIO_DeInit()
 *
 * @brief          - Deinitialization of a GPIO pin
 *
 * @param[in]      - Structure gpio handle with config atributes
 *
 * @return         - none
 *
 * @Note           - none
 */
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx)
{
	if(pGPIOx == GPIOA)
	{
		GPIOA_REG_RESET();
	}
	else if(pGPIOx == GPIOB)
	{
		GPIOB_REG_RESET();
	}
	else if(pGPIOx == GPIOC)
	{
		GPIOC_REG_RESET();
	}
	else if(pGPIOx == GPIOD)
	{
		GPIOD_REG_RESET();
	}
	else if(pGPIOx == GPIOE)
	{
		GPIOE_REG_RESET();
	}
	else if(pGPIOx == GPIOF)
	{
		GPIOF_REG_RESET();
	}
	else if(pGPIOx == GPIOG)
	{
		GPIOG_REG_RESET();
	}
	else if(pGPIOx == GPIOH)
	{
		GPIOH_REG_RESET();
	}
	else if(pGPIOx == GPIOI)
	{
		GPIOI_REG_RESET();
	}
}
/**************************************************
 * @fn             - GPIO_ReadFromInputPin
 *
 * @brief          - read data received on a GPIO pin
 *
 * @param[in]      - Structure with gpio registers
 * @param[in]      - Pin number
 *
 * @return         - 1 or 0
 *
 * @Note           - none
 */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	uint8_t value;
	value = (uint8_t)((pGPIOx->IDR >> PinNumber) && 0x00000001); //Shift pinnumber times the read value and mask it
	return value;
}

/**************************************************
 * @fn             - GPIO_ReadFromInputPort
 *
 * @brief          - read data received on a GPIO port
 *
 * @param[in]      - Structure with gpio registers
 * @param[in]      - Pin number
 *
 * @return         - idr register 16 bit value
 *
 * @Note           - none
 */
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx)
{
	uint16_t value;
	value = (uint16_t)(pGPIOx->IDR ); //Shift pinnumber times the read value and mask it
	return value;
}
/**************************************************
 * @fn             - GPIO_write_to_output_pin
 *
 * @brief          - write data to GPIO pin
 *
 * @param[in]      - Structure with gpio registers
 * @param[in]      - Pin number
 * @param[in]      - Value to write on pin
 * @return         - none
 *
 * @Note           - none
 */
void GPIO_write_to_output_pin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value)
{
	if(Value == GPIO_PIN_SET)
	{
		pGPIOx->ODR |= (1 << PinNumber);
	}
	else
	{
		pGPIOx->ODR &= ~(1 << PinNumber);
	}

}
/**************************************************
 * @fn             - GPIO_write_to_output_port
 *
 * @brief          - read data received on a GPIO pin
 *
 * @param[in]      - Structure with gpio registers
 * @param[in]      - Value to write on port
 *
 * @return         - none
 *
 * @Note           - none
 */
void GPIO_write_to_output_port(GPIO_RegDef_t *pGPIOx, uint16_t Value)
{
	pGPIOx->ODR = Value;
}
/**************************************************
 * @fn             - GPIO_toggle_output_pin
 *
 * @brief          - Changes gpio pin state from 1 to 0 or from 0 to 1
 *
 * @param[in]      - Structure with gpio registers
 * @param[in]      - Pin number
 *
 * @return         - none
 *
 * @Note           - none
 */
void GPIO_toggle_output_pin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	//way 1
	uint8_t value;
	value = GPIO_ReadFromInputPin(pGPIOx, PinNumber);
	value = ~value;
	GPIO_write_to_output_pin(pGPIOx, PinNumber, value);

	//way 2
	// pGPIOx->ODR ^= (1 << PinNumber);

}

/**************************************************
 * @fn             - GPIO_IRQConfig()
 *
 * @brief          -
 *
 * @param[in]      -
 *
 * @return         - none
 *
 * @Note           - none
 */
void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		if(IRQNumber <= 31)
		{
			*NVIC_ISER0 |= (1 << IRQNumber);
		}
		else if(IRQNumber > 31 && IRQNumber <= 64)
		{
			*NVIC_ISER1 |= (1 << IRQNumber % 32);
		}
		else if(IRQNumber > 64 && IRQNumber <= 96)
		{
			*NVIC_ISER2 |= (1 << IRQNumber % 64);
		}
	}
	else
	{
		if(IRQNumber <= 31)
		{
			*NVIC_ICER0 |= (1 << IRQNumber);
		}
		else if(IRQNumber > 31 && IRQNumber <= 64)
		{
			*NVIC_ICER1 |= (1 << IRQNumber % 32);
		}
		else if(IRQNumber > 64 && IRQNumber <= 96)
		{
			*NVIC_ICER2 |= (1 << IRQNumber % 64);
		}
	}


}

/**************************************************
 * @fn             - GPIO_IRQPriorityConfig()
 *
 * @brief          -
 *
 * @param[in]      -
 *
 * @return         - none
 *
 * @Note           - none
 */
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
{
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section = IRQNumber % 4;

	uint8_t shift_amount = ( 8 * iprx_section) + (8 - NO_PR_BITS_IMPLEMENTED);
	*(NVIC_PR_BASE_ADDR + (iprx*4)) |= (IRQPriority << shift_amount);
}

/**************************************************
 * @fn             - GPIO_IRQConfig()
 *
 * @brief          -
 *
 * @param[in]      -
 *
 * @return         - none
 *
 * @Note           - none
 */
void GPIO_IRQHandling(uint8_t PinNumber)
{
	//clear the exti pr register corresponding to the pin number

	if(EXTI->EXTI_PR & (1 << PinNumber))
	{
		//clear
		EXTI->EXTI_PR |= (1 << PinNumber);
	}
}

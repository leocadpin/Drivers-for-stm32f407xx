/*
 * stm32f4xx_usart_drivers.c
 *
 *  Created on: Apr 19, 2024
 *      Author: leo
 */

#include "stm32f4xx_usart_drivers.h"




void USART_Init(USART_Handle_t *pUSARTHandle)
{

}

/**************************************************
 * @fn             - USART_Deinit()
 *
 * @brief          - Initialize USART with config values
 *
 * @param[in]      - USART Handle structure pointer
 * @param[in]      -
 * @param[in]      -
 *
 * @return         - none
 *
 * @Note           - none
 */
void USART_Deinit(USART_RegDef_t *pUSARTx)
{
	if(pUSARTx == USART1)
	{
		USART1_REG_RESET();
	}
	else if(pUSARTx == USART2)
	{
		USART2_REG_RESET();
	}
	else if(pUSARTx == USART3)
	{
		USART3_REG_RESET();
	}
	else if(pUSARTx == UART4)
	{
		UART4_REG_RESET();
	}
	else if(pUSARTx == UART5)
	{
		UART5_REG_RESET();
	}
	else if(pUSARTx == USART6)
	{
		USART6_REG_RESET();
	}

}
 /**************************************************
  * @fn             - USART_PeriClockControl()
  *
  * @brief          - enable cloch for USART peripheral device
  *
  * @param[in]      - USART register definition structure
  * @param[in]      - enable or disable option
  * @param[in]      -
  *
  * @return         - none
  *
  * @Note           - none
  */
 void USART_PeriClockControl(USART_RegDef_t *pUSARTx, uint8_t en_or_di)
 {

 	if(en_or_di == ENABLE)
 	{
 		if(pUSARTx == USART1)
 		{
 			USART1_PCLK_EN();
 		}
 		else if(pUSARTx == USART2)
 		{
 			USART2_PCLK_EN();
 		}
 		else if(pUSARTx == USART3)
 		{
 			USART3_PCLK_EN();
 		}
 		else if(pUSARTx == UART4)
 		{
 			UART4_PCLK_EN();
 		}
 		else if(pUSARTx == UART5)
 		{
 			UART5_PCLK_EN();
 		}
 		else if(pUSARTx == USART6)
 		{
 			USART6_PCLK_EN();
 		}
 	}
 	else
 	{
 		if(pUSARTx == USART1)
 		{
 			USART1_PCLK_DIS();
 		}
 		else if(pUSARTx == USART2)
 		{
 			USART2_PCLK_DIS();
 		}
 		else if(pUSARTx == USART3)
 		{
 			USART3_PCLK_DIS();
 		}
 		else if(pUSARTx == UART4)
 		{
 			UART4_PCLK_DIS();
 		}
 		else if(pUSARTx == UART5)
 		{
 			UART5_PCLK_DIS();
 		}
 		else if(pUSARTx == USART6)
 		{
 			USART6_PCLK_DIS();
 		}

 	}

 }


 /**************************************************
  * @fn             - USART_PeripheralControl()
  *
  * @brief          - Enable or disable USART peripheral
  *
  * @param[in]      - Structure USART handle with config atributes
  * @param[in]      - enable or disable macro

  *
  * @return         - none
  *
  * @Note           -
  */
 void USART_PeripheralControl(USART_RegDef_t *pUSARTx, uint8_t EnOrDi)
 {
	 if((pUSARTx != UART4) && (pUSARTx != UART5))
	 {
			if(EnOrDi == ENABLE)
			{
				pUSARTx->USART_CR2 |= (1 << USART_CR2_CLKEN);
			}
			else
			{
				pUSARTx->USART_CR2 &= ~(1 << USART_CR2_CLKEN);
			}
	 }

 }




/**************************************************
 * @fn             - USART_GetFlagStatus
 *
 * @brief          - returns the value of the given flag
 *
 * @param[in]      - Structure USART with register definitions
 * @param[in]      - flag name
 *
 * @return         - none
 *
 * @Note           - none
 */
 uint8_t USART_Get_Flag_status(USART_RegDef_t *pUSARTx, uint8_t status_flag_name)
{
	if(pUSARTx->USART_SR & status_flag_name)
	{
		return FLAG_SET;
	}

	return FLAG_RESET;
}


void USART_Clear_Flag(USART_RegDef_t *pUSARTx, uint16_t status_flag_name);

/*
 * Interrupt related functions
 */



/**************************************************
 * @fn             - USART_IRQInterruptConfig()
 *
 * @brief          -	CPU config for irqs in the NVIC
 *
 * @param[in]      -	IRQ number
 * @param[in]      - 	Enable or disable macro
 *
 * @return         - none
 *
 * @Note           - none
 */
void USART_IRQ_interrupt_config(uint8_t IRQ_number, uint8_t en_or_di)
{
	if(en_or_di == ENABLE)
	{
		if(IRQ_number <= 31)
		{
			*NVIC_ISER0 |= (1 << IRQ_number);
		}
		else if(IRQ_number > 31 && IRQ_number <= 64)
		{
			*NVIC_ISER1 |= (1 << IRQ_number % 32);
		}
		else if(IRQ_number > 64 && IRQ_number <= 96)
		{
			*NVIC_ISER2 |= (1 << IRQ_number % 64);
		}
	}
	else
	{
		if(IRQ_number <= 31)
		{
			*NVIC_ICER0 |= (1 << IRQ_number);
		}
		else if(IRQ_number > 31 && IRQ_number <= 64)
		{
			*NVIC_ICER1 |= (1 << IRQ_number % 32);
		}
		else if(IRQ_number > 64 && IRQ_number <= 96)
		{
			*NVIC_ICER2 |= (1 << IRQ_number % 64);
		}
	}


}

/**************************************************
 * @fn             - USART_IRQPriorityConfig()
 *
 * @brief          -	priority assignment for the requested interrupt
 *
 * @param[in]      -	IRQ number
 * @param[in]      - 	Priority value
 *
 * @return         - none
 *
 * @Note           - none
 */
void USART_IRQ_priority_config(uint8_t IRQ_number, uint32_t IRQ_priority)
{
	uint8_t iprx = IRQ_number / 4;
	uint8_t iprx_section = IRQ_number % 4;

	uint8_t shift_amount = ( 8 * iprx_section) + (8 - NO_PR_BITS_IMPLEMENTED);
	*(NVIC_PR_BASE_ADDR + (iprx*4)) |= (IRQ_priority << shift_amount);
}


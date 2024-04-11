/*
 * stm32f4xx_i2c_driver.c
 *
 *  Created on: Apr 11, 2024
 *      Author: leo
 */

#include"stm32f4xx_i2c_driver.h"


/**************************************************
 * @fn             - RCC_Get_PCLK_val()
 *
 * @brief          - Initialize i2c with config values
 *
 * @param[in]      - i2c Handle structure pointer
 * @param[in]      -
 * @param[in]      -
 *
 * @return         - none
 *
 * @Note           - none
 */

uint16_t AHB_PreScaler[9] = {2,4,8,16,32,64,128,256,512};
uint16_t APB1_PreScaler[4] = {2,4,8,16};

uint32_t RCC_Get_PCLK_val(void)
{
	uint32_t pclk1;
	uint32_t Systemclk;

	uint8_t clksrc;
	uint8_t temp;
	uint8_t ahb_p;
	uint8_t apb1_p;

	clksrc = ((RCC->RCC_CFGR >> 2) & 0x3);

	//Clock source calculations
	if(clksrc == 0)
	{
		Systemclk = 16000000;
	}
	if(clksrc == 1)
	{
		Systemclk = 8000000;
	}
	if(clksrc == 2)
	{
		//value would come from pll;
	}

	//Prescaler calculations
	temp = ( (RCC->RCC_CFGR >> 4) & 0XF );

	if(temp < 8)
	{
		ahb_p = 1;
	}
	else
	{
		ahb_p = AHB_PreScaler[temp-8];
	}

	temp = ( (RCC->RCC_CFGR >> 10) & 0X7 );

	if(temp < 4)
	{
		apb1_p = 1;
	}
	else
	{
		apb1_p = APB1_PreScaler[temp-4];
	}

	pclk1 = (Systemclk / ahb_p) / apb1_p;

	return pclk1;
}

/**************************************************
 * @fn             - I2C_Init()
 *
 * @brief          - Initialize i2c with config values
 *
 * @param[in]      - i2c Handle structure pointer
 * @param[in]      -
 * @param[in]      -
 *
 * @return         - none
 *
 * @Note           - none
 */
void I2C_Init(I2C_Handle_t* pI2CHandle)
{
	uint32_t tempreg = 0;

	tempreg |= pI2CHandle->I2C_config.I2C_ACKControl << I2C_CR1_ACK;
	pI2CHandle->pI2Cx->I2C_CR1 = tempreg;

	//configure the freq field of cr2

	tempreg = 0;

	tempreg |= RCC_Get_PCLK_val() / 1000000U;

	pI2CHandle->pI2Cx->I2C_CR2 = (tempreg & 0x3F);

	//program the device own address
	tempreg = 0;
	tempreg |= pI2CHandle->I2C_config.I2C_DeviceAddress << 1;
	tempreg |= (1 << 14);
	pI2CHandle->pI2Cx->I2C_OAR1 = tempreg;

	//CCR calculations
	uint16_t ccr_value = 0;
	tempreg = 0;

	if(pI2CHandle->I2C_config.I2C_SCL_Speed <= I2C_SCL_SPEED_SM)
	{
		ccr_value = RCC_Get_PCLK_val() / (2 * pI2CHandle->I2C_config.I2C_SCL_Speed);
		tempreg |= (ccr_value & 0xFFF);
	}
	else
	{
		tempreg |= (1 << 15);
		tempreg |= (pI2CHandle->I2C_config.I2C_FMDutyCycle << 14);

		if(pI2CHandle->I2C_config.I2C_FMDutyCycle == I2C_FM_DUTY_2)
		{
			ccr_value = RCC_Get_PCLK_val() / (3 * pI2CHandle->I2C_config.I2C_SCL_Speed);
		}
		else
		{
			ccr_value = RCC_Get_PCLK_val() / (25 * pI2CHandle->I2C_config.I2C_SCL_Speed);
		}
		tempreg |= (ccr_value & 0xFFF);

	}

	pI2CHandle->pI2Cx->I2C_CCR = tempreg;

}
/**************************************************
 * @fn             - I2C_DeInit()
 *
 * @brief          - Initialize i2c with config values
 *
 * @param[in]      - i2c Handle structure pointer
 * @param[in]      -
 * @param[in]      -
 *
 * @return         - none
 *
 * @Note           - none
 */
void I2C_DeInit(I2C_RegDef_t *pI2Cx)
{
	if(pI2Cx == I2C1)
	{
		I2C1_REG_RESET();
	}
	else if(pI2Cx == I2C2)
	{
		I2C2_REG_RESET();
	}
	else if(pI2Cx == I2C3)
	{
		I2C3_REG_RESET();
	}
}

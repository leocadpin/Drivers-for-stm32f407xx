/*
 * stm32f4xx_i2c_driver.c
 *
 *  Created on: Apr 11, 2024
 *      Author: leo
 */

#include"stm32f4xx_i2c_driver.h"

 static void I2C_start_condition(I2C_RegDef_t *pI2Cx);
 static void I2C_stop_condition(I2C_RegDef_t *pI2Cx);
 static void I2C_address_phase(I2C_RegDef_t* pI2Cx, uint8_t slave_addr);
 static void clear_ADDR_flag(I2C_RegDef_t* pI2Cx);


 /**************************************************
  * @fn             - I2C_PeriClock_Control()
  *
  * @brief          - enable cloch for i2c peripheral device
  *
  * @param[in]      - i2c register definition structure
  * @param[in]      - enable or disable option
  * @param[in]      -
  *
  * @return         - none
  *
  * @Note           - none
  */
 void I2C_PeriClock_Control(I2C_RegDef_t *pI2Cx, uint8_t EnorDi)
 {

 	if(EnorDi == ENABLE)
 	{
 		if(pI2Cx == I2C1)
 		{
 			I2C1_PCLK_EN();
 		}
 		else if(pI2Cx == I2C2)
 		{
 			I2C2_PCLK_EN();
 		}
 		else if(pI2Cx == I2C3)
 		{
 			I2C3_PCLK_EN();
 		}


 	}
 	else
 	{
 		if(pI2Cx == I2C1)
 		{
 			I2C1_PCLK_DIS();
 		}
 		else if(pI2Cx == I2C2)
 		{
 			I2C2_PCLK_DIS();
 		}
 		else if(pI2Cx == I2C3)
 		{
 			I2C3_PCLK_DIS();
 		}

 	}

 }

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

	//enable clock for i2c peripheral
	I2C_PeriClock_Control(pI2CHandle->pI2Cx, ENABLE);

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


	tempreg = 0;

	if(pI2CHandle->I2C_config.I2C_SCL_Speed <= I2C_SCL_SPEED_SM)
	{
		tempreg = (RCC_Get_PCLK_val() / 1000000U) + 1;
	}
	else
	{
		tempreg = ((RCC_Get_PCLK_val() * 300U) / 1000000000 ) + 1;
	}

	pI2CHandle->pI2Cx->I2C_TRISE = (tempreg & 0x3F);
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
/**************************************************
 * @fn             - I2C_MasterSendData()
 *
 * @brief          - Initialize i2c with config values
 *
 * @param[in]      - i2c Handle structure pointer
 * @param[in]      - buffer with data
 * @param[in]      - data length
 * @param[in]      - slave address
 * @return         - none
 *
 * @Note           - none
 */
void I2C_MasterSendData(I2C_Handle_t *pI2CHandle, uint8_t *pTXbuffer, uint32_t len, uint8_t slave_addr)
{
	//Start condition
	I2C_start_condition(pI2CHandle->pI2Cx);

	//Confirm start generation is completed checking SB flasg inte sr1
	while(I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_SB));

	//send addres to slave with r/w bit to 0 (write)
	I2C_address_phase(pI2CHandle->pI2Cx, slave_addr);

	//confirm that address phase is completed by checking the addr flag in sr1
	while( ! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_ADDR));

	//CLEAR ADDR FLAG
	clear_ADDR_flag(pI2CHandle->pI2Cx);

	//send data until len gets to 0
	while(len > 0)
	{
		while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_TXE)); //wait till txe is set
		pI2CHandle->pI2Cx->I2C_DR = *pTXbuffer;
		pTXbuffer++;
		len--;
	}

	//wait for TXE and BTF are set
	while( ! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_TXE));

	while( ! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_BTF));

	//stop condition
	I2C_stop_condition(pI2CHandle->pI2Cx);
}





static void I2C_start_condition(I2C_RegDef_t *pI2Cx)
{
	pI2Cx->I2C_CR1 |= (1 << I2C_CR1_START);
}



/**************************************************
 * @fn             - I2C_GetFlagStatus
 *
 * @brief          - returns the value of the given flag
 *
 * @param[in]      - Structure I2C with register definitions
 * @param[in]      - flag name
 *
 * @return         - none
 *
 * @Note           - none
 */
uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx, uint32_t FlagName)
{
	if(pI2Cx->I2C_SR1 & FlagName)
	{
		return FLAG_SET;
	}

	return FLAG_RESET;
}

/**************************************************
 * @fn             - I2C_GetFlagStatus
 *
 * @brief          - executes the addres phase in the i2c protocol
 *
 * @param[in]      - Structure I2C with register definitions
 * @param[in]      - slave addres
 *
 * @return         - none
 *
 * @Note           - none
 */
static void I2C_address_phase(I2C_RegDef_t* pI2Cx, uint8_t slave_addr)
{
	slave_addr = slave_addr << 1;
	slave_addr &= ~(1);
	pI2Cx->I2C_DR = slave_addr;
}

/**************************************************
 * @fn             - clear_ADDR_flag
 *
 * @brief          - executes the addres phase in the i2c protocol
 *
 * @param[in]      - Structure I2C with register definitions

 *
 * @return         - none
 *
 * @Note           - none
 */
static void clear_ADDR_flag(I2C_RegDef_t* pI2Cx)
{
	uint16_t dummy_read;

	dummy_read = pI2Cx->I2C_SR1;
	dummy_read = pI2Cx->I2C_SR2;
	(void)dummy_read;
}

/**************************************************
 * @fn             - I2C_stop_condition
 *
 * @brief          - executes the stop condition in the i2c protocol
 *
 * @param[in]      - Structure I2C with register definitions

 *
 * @return         - none
 *
 * @Note           - none
 */

static void I2C_stop_condition(I2C_RegDef_t *pI2Cx)
{
	pI2Cx->I2C_CR1 |= (1 << I2C_CR1_STOP);
}

























/*
 * stm32f407xx.h
 *
 *  Created on: Mar 29, 2024
 *      Author: leo
 */

#ifndef STM32F407XX_H_
#define STM32F407XX_H_

#include <stdint.h>

// base addresses of flash and SRAM memories

#define DRV_FLASH_BASEADDR 	        0x08000000U   // flash memory
#define DRV_SRAM1_BASEADDR          0x20000000U
#define DRV_SRAM2_BASEADDR          0x2001C000U   // auxiliar sram
#define DRV_ROM                     0x1FFF0000U   // System memory
//#define DRV_FLASH_BASEADDR          SRAM1_BASEADDR

// AHB and APB Bus Peripheral bases addresses

#define DRV_PERIPH_BASE 	         0x40000000U
#define DRV_APB1PERIPH_BASE          0x40000000U
#define DRV_APB2PERIPH_BASE          0x40010000U
#define DRV_AHB1PERIPH_BASE          0x40020000U
#define DRV_AHB2PERIPH_BASE          0x50050000U

// Base addresses of peripherals which are hanging on AHB1 bus

#define DRV_GPIOA_BASEADDR           (DRV_AHB1PERIPH_BASE + 0x0000)
#define DRV_GPIOB_BASEADDR           (DRV_AHB1PERIPH_BASE + 0x0400)
#define DRV_GPIOC_BASEADDR           (DRV_AHB1PERIPH_BASE + 0x0800)
#define DRV_GPIOD_BASEADDR           (DRV_AHB1PERIPH_BASE + 0x0C00)
#define DRV_GPIOE_BASEADDR           (DRV_AHB1PERIPH_BASE + 0x1000)
#define DRV_GPIOF_BASEADDR           (DRV_AHB1PERIPH_BASE + 0x1400)
#define DRV_GPIOG_BASEADDR           (DRV_AHB1PERIPH_BASE + 0x1800)
#define DRV_GPIOH_BASEADDR           (DRV_AHB1PERIPH_BASE + 0x1C00)
#define DRV_GPIOI_BASEADDR           (DRV_AHB1PERIPH_BASE + 0x2000)

// Base addresses of peripherals which are hanging on APB1 bus

#define DRV_I2C1_BASEADDR            (DRV_APB1PERIPH_BASE + 0x5400)
#define DRV_I2C2_BASEADDR            (DRV_APB1PERIPH_BASE + 0x5800)
#define DRV_I2C3_BASEADDR            (DRV_APB1PERIPH_BASE + 0x5C00)
#define DRV_SPI2_BASEADDR            (DRV_APB1PERIPH_BASE + 0x3800)
#define DRV_SPI3_BASEADDR            (DRV_APB1PERIPH_BASE + 0x3C00)
#define DRV_USART2_BASEADDR          (DRV_APB1PERIPH_BASE + 0x4400)
#define DRV_USART3_BASEADDR          (DRV_APB1PERIPH_BASE + 0x4800)
#define DRV_UART4_BASEADDR           (DRV_APB1PERIPH_BASE + 0x4C00)
#define DRV_UART5_BASEADDR           (DRV_APB1PERIPH_BASE + 0x5000)

// Base addresses of peripherals which are hanging on APB2 bus


#define DRV_SPI1_BASEADDR            (DRV_APB2PERIPH_BASE + 0x3000)
#define DRV_USART1_BASEADDR          (DRV_APB2PERIPH_BASE + 0x1000)
#define DRV_USART6_BASEADDR          (DRV_APB2PERIPH_BASE + 0x1400)
#define DRV_EXTI_BASEADDR            (DRV_APB2PERIPH_BASE + 0x3C00)
#define DRV_SYSCFG_BASEADDR          (DRV_APB2PERIPH_BASE + 0x3800)

// Base address for RCC

#define DRV_RCC_BASEADDR             (DRV_AHB1PERIPH_BASE + 0x3800)

//Peripheral register definition structures

typedef struct
{
	volatile uint32_t MODER;
	volatile uint32_t OTYPER;
	volatile uint32_t OSPEEDR;
	volatile uint32_t PUPDR;
	volatile uint32_t IDR;
	volatile uint32_t ODR;
	volatile uint32_t BSRR;
	volatile uint32_t LCKR;
	volatile uint32_t AFR[2]; // GPIO alternate function low and high registers
}GPIO_RegDef_t;

typedef struct
{
	volatile uint32_t RCC_CR;
	volatile uint32_t RCC_PLLCFGR;
	volatile uint32_t RCC_CFGR;
	volatile uint32_t RCC_CIR;
	volatile uint32_t AHB1RSTR;
	volatile uint32_t AHB2RSTR;
	volatile uint32_t AHB3RSTR;
	uint32_t RESERVED0;
	volatile uint32_t APB1RSTR;
	volatile uint32_t APB2RSTR;
	volatile uint32_t AHB1ENR;
	volatile uint32_t AHB2ENR;
	volatile uint32_t AHB3ENR;
	volatile uint32_t APB1ENR;
	volatile uint32_t APB2ENR;
	uint32_t RESERVED1[2];
	volatile uint32_t AHB1LPENR;
	volatile uint32_t AHB2LPENR;
	volatile uint32_t AHB3LPENR;
	uint32_t RESERVED2;
	volatile uint32_t APB1LPENR;
	volatile uint32_t APB2LPENR;
	uint32_t RESERVED3[2];
	volatile uint32_t RCC_BDCR;
	volatile uint32_t RCC_CSR;
	uint32_t RESERVED4[2];
	volatile uint32_t RCC_SSCGR;
	volatile uint32_t RCC_PLLI2SCFGR;

}RCC_RegDef_t;

//Peripheral register definition structure for EXTI
typedef struct
{
	volatile uint32_t EXTI_IMR;
	volatile uint32_t EXTI_EMR;
	volatile uint32_t EXTI_RSTR;
	volatile uint32_t EXTI_FTSR;
	volatile uint32_t EXTI_SWIER;
	volatile uint32_t EXTI_PR;
}EXTI_RegDef_t;

// Peripheral definitions

#define GPIOA ((GPIO_RegDef_t*) DRV_GPIOA_BASEADDR)
#define GPIOB ((GPIO_RegDef_t*) DRV_GPIOB_BASEADDR)
#define GPIOC ((GPIO_RegDef_t*) DRV_GPIOC_BASEADDR)
#define GPIOD ((GPIO_RegDef_t*) DRV_GPIOD_BASEADDR)
#define GPIOE ((GPIO_RegDef_t*) DRV_GPIOE_BASEADDR)
#define GPIOF ((GPIO_RegDef_t*) DRV_GPIOF_BASEADDR)
#define GPIOG ((GPIO_RegDef_t*) DRV_GPIOG_BASEADDR)
#define GPIOH ((GPIO_RegDef_t*) DRV_GPIOH_BASEADDR)
#define GPIOI ((GPIO_RegDef_t*) DRV_GPIOI_BASEADDR)

#define RCC ((RCC_RegDef_t*) DRV_RCC_BASEADDR)

#define EXTI ((EXTI_RegDef_t*) DRV_EXTI_BASEADDR)

// Clock Enable Macros for GPIOx peripherals

#define GPIOA_PCLK_EN()  (RCC->AHB1ENR |= (1 << 0))
#define GPIOB_PCLK_EN()  (RCC->AHB1ENR |= (1 << 1))
#define GPIOC_PCLK_EN()  (RCC->AHB1ENR |= (1 << 2))
#define GPIOD_PCLK_EN()  (RCC->AHB1ENR |= (1 << 3))
#define GPIOE_PCLK_EN()  (RCC->AHB1ENR |= (1 << 4))
#define GPIOF_PCLK_EN()  (RCC->AHB1ENR |= (1 << 5))
#define GPIOG_PCLK_EN()  (RCC->AHB1ENR |= (1 << 6))
#define GPIOH_PCLK_EN()  (RCC->AHB1ENR |= (1 << 7))
#define GPIOI_PCLK_EN()  (RCC->AHB1ENR |= (1 << 8))

//Clock enable macros for I2C peripherals

#define I2C1_PCLK_EN()   (RCC->APB1ENR |= (1 << 21))
#define I2C2_PCLK_EN()   (RCC->APB1ENR |= (1 << 22))
#define I2C3_PCLK_EN()   (RCC->APB1ENR |= (1 << 23))


//Clock enable macros for SPI peripherals

#define SPI1_PCLK_EN()   (RCC->APB2ENR |= (1 << 12))
#define SPI2_PCLK_EN()   (RCC->APB1ENR |= (1 << 14))
#define SPI3_PCLK_EN()   (RCC->APB1ENR |= (1 << 15))
#define SPI4_PCLK_EN()   (RCC->APB2ENR |= (1 << 13))

//Clock enable macros for USART peripherals

#define USART1_PCLK_EN()   (RCC->APB2ENR |= (1 << 4))
#define UART2_PCLK_EN()    (RCC->APB1ENR |= (1 << 17))
#define UART3_PCLK_EN()    (RCC->APB1ENR |= (1 << 18))
#define UART4_PCLK_EN()    (RCC->APB1ENR |= (1 << 19))
#define UART5_PCLK_EN()    (RCC->APB1ENR |= (1 << 20))
#define USART6_PCLK_EN()   (RCC->APB2ENR |= (1 << 5))

//Clock enable macros for SYSCFG peripherals
#define SYSCFG_PCLK_EN()   (RCC->APB2ENR |= (1 << 14))


//Macros to reset GPIOx peripherals

#define GPIOA_REG_RESET()   do{ (RCC->AHB1RSTR |= (1 << 0)); (RCC->AHB1RSTR &= ~(1 << 0)); }while(0)
#define GPIOB_REG_RESET()   do{ (RCC->AHB1RSTR |= (1 << 1)); (RCC->AHB1RSTR &= ~(1 << 1)); }while(0)
#define GPIOC_REG_RESET()   do{ (RCC->AHB1RSTR |= (1 << 2)); (RCC->AHB1RSTR &= ~(1 << 2)); }while(0)
#define GPIOD_REG_RESET()   do{ (RCC->AHB1RSTR |= (1 << 3)); (RCC->AHB1RSTR &= ~(1 << 3)); }while(0)
#define GPIOE_REG_RESET()   do{ (RCC->AHB1RSTR |= (1 << 4)); (RCC->AHB1RSTR &= ~(1 << 4)); }while(0)
#define GPIOF_REG_RESET()   do{ (RCC->AHB1RSTR |= (1 << 5)); (RCC->AHB1RSTR &= ~(1 << 5)); }while(0)
#define GPIOG_REG_RESET()   do{ (RCC->AHB1RSTR |= (1 << 6)); (RCC->AHB1RSTR &= ~(1 << 6)); }while(0)
#define GPIOH_REG_RESET()   do{ (RCC->AHB1RSTR |= (1 << 7)); (RCC->AHB1RSTR &= ~(1 << 7)); }while(0)
#define GPIOI_REG_RESET()   do{ (RCC->AHB1RSTR |= (1 << 8)); (RCC->AHB1RSTR &= ~(1 << 8)); }while(0)

// Clock disable Macros for GPIOx peripherals

#define GPIOA_PCLK_DIS()  (RCC->AHB1ENR &= ~(1 << 0))
#define GPIOB_PCLK_DIS()  (RCC->AHB1ENR &= ~(1 << 1))
#define GPIOC_PCLK_DIS()  (RCC->AHB1ENR &= ~(1 << 2))
#define GPIOD_PCLK_DIS()  (RCC->AHB1ENR &= ~(1 << 3))
#define GPIOE_PCLK_DIS()  (RCC->AHB1ENR &= ~(1 << 4))
#define GPIOF_PCLK_DIS()  (RCC->AHB1ENR &= ~(1 << 5))
#define GPIOG_PCLK_DIS()  (RCC->AHB1ENR &= ~(1 << 6))
#define GPIOH_PCLK_DIS()  (RCC->AHB1ENR &= ~(1 << 7))
#define GPIOI_PCLK_DIS()  (RCC->AHB1ENR &= ~(1 << 8))


//Clock disable macros for I2C peripherals

#define I2C1_PCLK_DIS()   (RCC->APB1ENR &= ~(1 << 21))
#define I2C2_PCLK_DIS()   (RCC->APB1ENR &= ~(1 << 22))
#define I2C3_PCLK_DIS()   (RCC->APB1ENR &= ~(1 << 23))

//Clock disable macros for SPI peripherals

#define SPI1_PCLK_DIS()   (RCC->APB2ENR &= ~(1 << 12))
#define SPI2_PCLK_DIS()   (RCC->APB1ENR &= ~(1 << 14))
#define SPI3_PCLK_DIS()   (RCC->APB1ENR &= ~(1 << 15))
#define SPI4_PCLK_DIS()   (RCC->APB2ENR &= ~(1 << 13))

//Clock disable macros for USART peripherals


#define USART1_PCLK_DIS()   (RCC->APB2ENR &= ~(1 << 4))
#define UART2_PCLK_DIS()    (RCC->APB1ENR &= ~(1 << 17))
#define UART3_PCLK_DIS()    (RCC->APB1ENR &= ~(1 << 18))
#define UART4_PCLK_DIS()    (RCC->APB1ENR &= ~(1 << 19))
#define UART5_PCLK_DIS()    (RCC->APB1ENR &= ~(1 << 20))
#define USART6_PCLK_DIS()   (RCC->APB2ENR &= ~(1 << 5))


//Clock disable  macros for SYSCFG peripherals
#define SYSCFG_PCLK_DIS()   (RCC->APB2ENR &= ~(1 << 14))






//Generic MACROS

#define ENABLE          1
#define DISABLE         0

#define SET             ENABLE
#define RESET           DISABLE

#define GPIO_PIN_SET    SET
#define GPIO_PIN_RESET  RESET

#endif /* STM32F407XX_H_ */


















































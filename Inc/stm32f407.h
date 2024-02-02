/*
	Made by Ruslan Abdulin
				with love.

	DRV - Driver related
	DRVF - Driver related function
	DRVV - Driver related value
	p - Pointer
*/

#ifndef INC_STM32F407_H_
#define INC_STM32F407_H_
#include <stdint.h>

/* General use macros */
#define __vo volatile
#define SET 1
#define RESET 0
#define ENABLE SET
#define DISABLE RESET

/* Base addresses of memory */
#define DRV_FLASH_BASEADDR 0x08000000U
#define DRV_SRAM1_BASEADDR 0x20000000U
#define DRV_SRAM2_BASEADDR 0x2001C000U
#define DRV_ROM_BASEADDR   0x1FFF0000U


/* Base addresses of peripheral buses */
#define DRV_APB1_BASEADDR 0x40000000U
#define DRV_APB2_BASEADDR 0x40001000U
#define DRV_AHB1_BASEADDR 0x40002000U
#define DRV_AHB2_BASEADDR 0x50000000U
#define DRV_RCC_BASEADDR  0x40003800U


/* Base addresses of  APB1 peripherals */
#define DRV_SPI2_BASEADDR   (DRV_APB1_BASEADDR + 0x3800U)
#define DRV_SPI3_BASEADDR   (DRV_APB1_BASEADDR + 0x3C00U)
#define DRV_USART2_BASEADDR (DRV_APB1_BASEADDR + 0x4800U)
#define DRV_USART3_BASEADDR (DRV_APB1_BASEADDR + 0x3C00U)
#define DRV_UART4_BASEADDR  (DRV_APB1_BASEADDR + 0x4C00U)
#define DRV_UART5_BASEADDR  (DRV_APB1_BASEADDR + 0x5000U)
#define DRV_UART7_BASEADDR  (DRV_APB1_BASEADDR + 0x7800U)
#define DRV_UART8_BASEADDR  (DRV_APB1_BASEADDR + 0x7C00U)
#define DRV_I2C1_BASEADDR   (DRV_APB1_BASEADDR + 0x5400U)
#define DRV_I2C2_BASEADDR   (DRV_APB1_BASEADDR + 0x5800U)
#define DRV_I2C3_BASEADDR   (DRV_APB1_BASEADDR + 0x5C00U)


/* Base addresses of  APB2 peripherals */
#define DRV_USART1_BASEADDR   (DRV_APB2_BASEADDR + 1000U)
#define DRV_USART6_BASEADDR   (DRV_APB2_BASEADDR + 0x1400U)
#define DRV_SPI1_BASEADDR     (DRV_APB2_BASEADDR + 0x3000U)
#define DRV_SPI4_BASEADDR     (DRV_APB2_BASEADDR + 0x3400U)
#define DRV_EXTI_BASEADDR     (DRV_APB2_BASEADDR + 0x3C00U)
#define DRV_SYSCFG_BASEADDR   (DRV_APB2_BASEADDR + 0x3800U)


/* Base addresses of  AHB1 peripherals */
#define DRV_GPIOA_BASEADDR (DRV_AHB1_BASEADDR)
#define DRV_GPIOB_BASEADDR (DRV_AHB1_BASEADDR + 0x0400U)
#define DRV_GPIOC_BASEADDR (DRV_AHB1_BASEADDR + 0x0800U)
#define DRV_GPIOD_BASEADDR (DRV_AHB1_BASEADDR + 0x0C00U)
#define DRV_GPIOE_BASEADDR (DRV_AHB1_BASEADDR + 0x1000U)
#define DRV_GPIOF_BASEADDR (DRV_AHB1_BASEADDR + 0x1400U)
#define DRV_GPIOG_BASEADDR (DRV_AHB1_BASEADDR + 0x1800U)
#define DRV_GPIOH_BASEADDR (DRV_AHB1_BASEADDR + 0x1C00U)
#define DRV_GPIOI_BASEADDR (DRV_AHB1_BASEADDR + 0x2000U)
#define DRV_GPIOJ_BASEADDR (DRV_AHB1_BASEADDR + 0x2400U)
#define DRV_GPIOK_BASEADDR (DRV_AHB1_BASEADDR + 0x2800U)


/* GPIO registers structure */
typedef struct{
	__vo uint32_t MODER; //   mode type
	__vo uint32_t OTYPER; //  output type
	__vo uint32_t OSPEEDR; // output speed
	__vo uint32_t PUPDR; //   internal pull up pull down resistors
	__vo uint32_t IDR; //     input data
	__vo uint32_t ODR; //     output data
	__vo uint32_t BSRR; //    port bit set reset
	__vo uint32_t LCKR; //    port configuration lock
	__vo uint32_t AFRL; //    alternate function low portion
	__vo uint32_t AFRH; //    alternate function top portion
} GPIO_RegDef_t;

// GPIO_RegDef_t *DRV_GPIOA_registers = (GPIO_RegDef_t*) DRV_GPIOA_BASEADDR;



/* RCC registers structure */
typedef struct{
	__vo uint32_t CR; //         clock control
	__vo uint32_t PLLCFGR; //    PLL clock output configuration
	__vo uint32_t CFGR; //       clock configuration
	__vo uint32_t CIR; //        clock interrupt
	__vo uint32_t AHB1RSTR; //   AHB1 peripheral reset
	__vo uint32_t AHB2RSTR; //   AHB1 peripheral reset
	__vo uint32_t AHB3RSTR; //   AHB3 peripheral reset
	uint32_t RESERVED0;
	__vo uint32_t APB1RSTR; //   APB1 peripheral reset
	__vo uint32_t APB2RSTR; //   APB2 peripheral reset
	uint32_t RESERVED1;
	uint32_t RESERVED2;
	__vo uint32_t AHB1ENR; //    AHB1 enable
	__vo uint32_t AHB2ENR; //    AHB2 enable
	__vo uint32_t AHB3ENR; //    AHB3 enable
	uint32_t RESERVED3;
	__vo uint32_t APB1ENR; //    APB1 enable
	__vo uint32_t APB2ENR; //    APB2 enable
	uint32_t RESERVED4;
	uint32_t RESERVED5;
	__vo uint32_t AHB1LPENR; //  AHB1 enable in low power mode
	__vo uint32_t AHB2LPENR; //  AHB2 enable in low power mode
	__vo uint32_t AHB3LPENR; //  AHB3 enable in low power mode
	uint32_t RESERVED6;
	__vo uint32_t APB1LPENR; //  APB1 enable in low power mode
	__vo uint32_t APB2LPENR; //  APB2 enable in low power mode
	uint32_t RESERVED7;
	uint32_t RESERVED8;
	__vo uint32_t BDCR; //       backup domain control
	__vo uint32_t CSR; //        clock control and status
	uint32_t RESERVED9;
	uint32_t RESERVED10;
	__vo uint32_t SSCGR; //      spread spectrum clock generation
	__vo uint32_t PLLI2SCFGR; // PLL I2C clock output configuration

} RCC_RegDef_t;

#define DRV_RCC ((RCC_RegDef_t*) DRV_RCC_BASEADDR)


/* Clock enable macros for GPIO */
#define DRVV_GPIOA_PCLK_EN ( DRV_RCC->AHB1ENR |= 0x1 );
#define DRVV_GPIOB_PCLK_EN ( DRV_RCC->AHB1ENR |= (0x1 << 1) );
#define DRVV_GPIOC_PCLK_EN ( DRV_RCC->AHB1ENR |= (0x1 << 2) );
#define DRVV_GPIOD_PCLK_EN ( DRV_RCC->AHB1ENR |= (0x1 << 3) );
#define DRVV_GPIOE_PCLK_EN ( DRV_RCC->AHB1ENR |= (0x1 << 4) );
#define DRVV_GPIOF_PCLK_EN ( DRV_RCC->AHB1ENR |= (0x1 << 5) );
#define DRVV_GPIOG_PCLK_EN ( DRV_RCC->AHB1ENR |= (0x1 << 6) );
#define DRVV_GPIOH_PCLK_EN ( DRV_RCC->AHB1ENR |= (0x1 << 7) );
#define DRVV_GPIOI_PCLK_EN ( DRV_RCC->AHB1ENR |= (0x1 << 8) );


/* Clock enable macros for I2C */
#define DRVV_I2C1_CLK_EN ( DRV_RCC->APB1ENR |= (0x1 << 21) );
#define DRVV_I2C2_CLK_EN ( DRV_RCC->APB1ENR |= (0x1 << 22) );
#define DRVV_I2C3_CLK_EN ( DRV_RCC->APB1ENR |= (0x1 << 23) );


/* Clock enable macros for SPI */
#define DRVV_SPI1_CLK_EN ( DRV_RCC->APB2ENR |= (0x1 << 12) );
#define DRVV_SPI2_CLK_EN ( DRV_RCC->APB1ENR |= (0x1 << 14) );
#define DRVV_SPI3_CLK_EN ( DRV_RCC->APB1ENR |= (0x1 << 15) );


/* Clock enable macros for USART */
#define DRVV_USART1_CLK_EN ( DRV_RCC->APB2ENR |= (0x1 << 4) );
#define DRVV_USART2_CLK_EN ( DRV_RCC->APB1ENR |= (0x1 << 17) );
#define DRVV_USART3_CLK_EN ( DRV_RCC->APB1ENR |= (0x1 << 18) );
#define DRVV_UART4_CLK_EN ( DRV_RCC->APB1ENR |= (0x1 << 19) );
#define DRVV_UART5_CLK_EN ( DRV_RCC->APB1ENR |= (0x1 << 20) );
#define DRVV_USART6_CLK_EN ( DRV_RCC->APB2ENR |= (0x1 << 5) );


/* Clock enable macros for SYSCFG */
#define DRVV_SYSCFG_CLK_EN ( DRV_RCC->APB2ENR |= (0x1 << 14) );


/* Clock disable macros for GPIO */
#define DRVV_GPIOA_PCLK_DI ( DRV_RCC->AHB1ENR &= ~0x1 );
#define DRVV_GPIOB_PCLK_DI ( DRV_RCC->AHB1ENR &= ~(0x1 << 1) );
#define DRVV_GPIOC_PCLK_DI ( DRV_RCC->AHB1ENR &= ~(0x1 << 2) );
#define DRVV_GPIOD_PCLK_DI ( DRV_RCC->AHB1ENR &= ~(0x1 << 3) );
#define DRVV_GPIOE_PCLK_DI ( DRV_RCC->AHB1ENR &= ~(0x1 << 4) );
#define DRVV_GPIOF_PCLK_DI ( DRV_RCC->AHB1ENR &= ~(0x1 << 5) );
#define DRVV_GPIOG_PCLK_DI ( DRV_RCC->AHB1ENR &= ~(0x1 << 6) );
#define DRVV_GPIOH_PCLK_DI ( DRV_RCC->AHB1ENR &= ~(0x1 << 7) );
#define DRVV_GPIOI_PCLK_DI ( DRV_RCC->AHB1ENR &= ~(0x1 << 8) );


/* Clock disable macros for I2C */
#define DRVV_I2C1_CLK_EN ( DRV_RCC->APB1ENR |= (0x1 << 21) );
#define DRVV_I2C2_CLK_EN ( DRV_RCC->APB1ENR |= (0x1 << 22) );
#define DRVV_I2C3_CLK_EN ( DRV_RCC->APB1ENR |= (0x1 << 23) );


/* Clock disable macros for SPI */
#define DRVV_SPI1_CLK_DI ( DRV_RCC->APB2ENR &= ~(0x1 << 12) );
#define DRVV_SPI2_CLK_DI ( DRV_RCC->APB1ENR &= ~(0x1 << 14) );
#define DRVV_SPI3_CLK_DI ( DRV_RCC->APB1ENR &= ~(0x1 << 15) );


/* Clock disable macros for USART */
#define DRVV_USART1_CLK_DI ( DRV_RCC->APB2ENR &= ~(0x1 << 4) );
#define DRVV_USART2_CLK_DI ( DRV_RCC->APB1ENR &= ~(0x1 << 17) );
#define DRVV_USART3_CLK_DI ( DRV_RCC->APB1ENR &= ~(0x1 << 18) );
#define DRVV_UART4_CLK_DI ( DRV_RCC->APB1ENR &= ~(0x1 << 19) );
#define DRVV_UART5_CLK_DI ( DRV_RCC->APB1ENR &= ~(0x1 << 20) );
#define DRVV_USART6_CLK_DI ( DRV_RCC->APB2ENR &= ~(0x1 << 5) );


/* Clock disable macros for SYSCFG */
#define DRVV_SYSCFG_CLK_DI ( DRV_RCC->APB2ENR &= ~(0x1 << 14) );


#endif /* INC_STM32F407_H_ */


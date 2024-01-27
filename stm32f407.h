#ifndef INC_STM32F407_H_
	#define INC_STM32F407_H_
#endif /* INC_STM32F407_H_ */

/* General use macros */
#define __vo volatile

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

GPIO_RegDef_t *DRV_GPIOA_registers = (GPIO_RegDef_t*) DRV_GPIOA_BASEADDR;
GPIO_RegDef_t *DRV_GPIOB_registers = (GPIO_RegDef_t*) DRV_GPIOB_BASEADDR;
GPIO_RegDef_t *DRV_GPIOC_registers = (GPIO_RegDef_t*) DRV_GPIOC_BASEADDR;
GPIO_RegDef_t *DRV_GPIOD_registers = (GPIO_RegDef_t*) DRV_GPIOD_BASEADDR;
GPIO_RegDef_t *DRV_GPIOE_registers = (GPIO_RegDef_t*) DRV_GPIOE_BASEADDR;
GPIO_RegDef_t *DRV_GPIOF_registers = (GPIO_RegDef_t*) DRV_GPIOF_BASEADDR;
GPIO_RegDef_t *DRV_GPIOG_registers = (GPIO_RegDef_t*) DRV_GPIOG_BASEADDR;
GPIO_RegDef_t *DRV_GPIOH_registers = (GPIO_RegDef_t*) DRV_GPIOH_BASEADDR;
GPIO_RegDef_t *DRV_GPIOI_registers = (GPIO_RegDef_t*) DRV_GPIOI_BASEADDR;
GPIO_RegDef_t *DRV_GPIOJ_registers = (GPIO_RegDef_t*) DRV_GPIOJ_BASEADDR;
GPIO_RegDef_t *DRV_GPIOK_registers = (GPIO_RegDef_t*) DRV_GPIOK_BASEADDR;







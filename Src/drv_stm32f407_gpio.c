#include "drv_stm32f407_gpio.h"

/*
 * Initialization and De-initialization
 */
void GPIO_Init(GPIO_Handle_t *p_GPIOxHandle){

	// 1. Mode
	if(p_GPIOxHandle->GPIOx_PinConfig.pinMode < 4 ){
		// non-interrupt mode
		p_GPIOxHandle->p_GPIOx->MODER &= ~(0b11 << (p_GPIOxHandle->GPIOx_PinConfig.pinNumber*2));
		p_GPIOxHandle->p_GPIOx->MODER |= p_GPIOxHandle->GPIOx_PinConfig.pinMode << (p_GPIOxHandle->GPIOx_PinConfig.pinNumber*2);

	} else {
		// interrupt mode
	}

	// 2. Output Speed
	p_GPIOxHandle->p_GPIOx->OSPEEDR &= ~(0b11 << (p_GPIOxHandle->GPIOx_PinConfig.pinNumber*2));
	p_GPIOxHandle->p_GPIOx->OSPEEDR |= p_GPIOxHandle->GPIOx_PinConfig.pinSpeed << (p_GPIOxHandle->GPIOx_PinConfig.pinNumber*2);

	// 3. Pull up Pull down resistor mode
	p_GPIOxHandle->p_GPIOx->PUPDR &= ~(0b11 << (p_GPIOxHandle->GPIOx_PinConfig.pinNumber*2));
	p_GPIOxHandle->p_GPIOx->PUPDR |= p_GPIOxHandle->GPIOx_PinConfig.pinPuPdControl << (p_GPIOxHandle->GPIOx_PinConfig.pinNumber*2);

	// 4. Output type
	p_GPIOxHandle->p_GPIOx->OTYPER &= ~(0b1 << p_GPIOxHandle->GPIOx_PinConfig.pinNumber);
	p_GPIOxHandle->p_GPIOx->OTYPER |= p_GPIOxHandle->GPIOx_PinConfig.pinOutType << (p_GPIOxHandle->GPIOx_PinConfig.pinNumber);

	// 5. Alternate functionality
	if(p_GPIOxHandle->GPIOx_PinConfig.pinMode == DRV_GPIO_MODE_ALTFN){
		uint8_t pinNum = p_GPIOxHandle->GPIOx_PinConfig.pinNumber;
		// AFR[pinNum / 8] used to figure whether to change low or high portions of ALTF
		// (pinNum % 8) * 4) used to find corresponding bit position for the pin
		p_GPIOxHandle->p_GPIOx->AFR[pinNum / 8] &= ~(0b1111 << ((pinNum % 8) * 4));
		p_GPIOxHandle->p_GPIOx->AFR[pinNum / 8] |= p_GPIOxHandle->GPIOx_PinConfig.pinAltFModeNum << ((pinNum % 8) * 4);
	}
}
void GPIO_DeInit(GPIO_RegDef_t *p_GPIOx){
	uint8_t bitPosition = ((uint32_t)p_GPIOx - (uint32_t)DRV_GPIOA_BASEADDR) / (uint16_t)0x0400;

	DRV_RCC->AHB1RSTR |= 0b1 << bitPosition;
	DRV_RCC->AHB1RSTR &= ~(0b1 << bitPosition);
}


/*
 * Peripheral clock control
 */
void GPIO_ClkControl(GPIO_RegDef_t *p_GPIOx, uint8_t ControlType){
	uint8_t bitPosition = ((uint32_t)p_GPIOx - (uint32_t)DRV_GPIOA_BASEADDR) / (uint16_t)0x0400;

	if( ControlType == ENABLE ){
		DRVF_GPIOx_PCLK_EN(bitPosition);
	} else {
		DRVF_GPIOx_PCLK_DI(bitPosition);
	}
}


// Read and Write
uint8_t GPIO_ReadInput_Pin(GPIO_RegDef_t *p_GPIOx, uint8_t pinNumber){
	return 0;
}

uint16_t GPIO_ReadInput_Port(GPIO_RegDef_t *p_GPIOx){
	return 0;
}

void GPIO_WriteOutput_Pin(GPIO_RegDef_t *p_GPIOx, uint8_t pinNumber, uint8_t data){

}

void GPIO_WriteOutput_Port(GPIO_RegDef_t *p_GPIOx, uint16_t data){

}

void GPIO_ToggleOutput_Pin(GPIO_RegDef_t *p_GPIOx, uint8_t pinNumber){

}


// IRQ configuration and ISR handling
// ControlType = 1 => enable ; ControlType = 0 => disable
void GPIO_IrqConfig(uint8_t IrqNumber, uint8_t IrqPriority, uint8_t ControlType){

}

void GPIO_IrqHandling(uint8_t pinNumber){

}




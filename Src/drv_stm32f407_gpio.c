#include "drv_stm32f407_gpio.h"

/*
 * Initialization and De-initialization
 */
void GPIO_Init(GPIO_Handle_t *p_GPIOxHandle){

}
void GPIO_DeInit(GPIO_RegDef_t *p_GPIOx){

}


/*
 * Peripheral clock control
 */
void GPIO_ClkControl(GPIO_RegDef_t *p_GPIOx, uint8_t ControlType){

}


// Read and Write
uint8_t GPIO_ReadInput_Pin(GPIO_RegDef_t *p_GPIOx, uint8_t pinNumber){

}

uint16_t GPIO_ReadInput_Port(GPIO_RegDef_t *p_GPIOx){

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



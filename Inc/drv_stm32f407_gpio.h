#ifndef INC_DRV_STM32F407_GPIO_H_
#define INC_DRV_STM32F407_GPIO_H_


#include "stm32f407.h" // target header file


typedef struct{
	uint8_t pinNumber;
	uint8_t pinMode;
	uint8_t pinSpeed;
	uint8_t pinPuPdControl; // Pull up and Pull down resistors control
	uint8_t pinOutType; //     Output type
	uint8_t pinAltFMode; //    Alternating function mode

} GPIO_PinConfig_t;


typedef struct {
	GPIO_RegDef_t *p_GPIOx; // Pointer to a certain address where the structure matches setup with the target device
	GPIO_PinConfig_t GPIOx_PinConfig;

} GPIO_Handle_t;


/* APIs supported */

// Initialization and De-initialization
void GPIO_Init(GPIO_Handle_t *p_GPIOxHandle);
void GPIO_DeInit(GPIO_RegDef_t *p_GPIOx);

// Peripheral clock control
// ControlType = 1 => enable ; ControlType = 0 => disable
void GPIO_ClkControl(GPIO_RegDef_t *p_GPIOx, uint8_t ControlType);

// Read and Write
uint8_t GPIO_ReadInput_Pin(GPIO_RegDef_t *p_GPIOx, uint8_t pinNumber);
uint16_t GPIO_ReadInput_Port(GPIO_RegDef_t *p_GPIOx);
void GPIO_WriteOutput_Pin(GPIO_RegDef_t *p_GPIOx, uint8_t pinNumber, uint8_t data);
void GPIO_WriteOutput_Port(GPIO_RegDef_t *p_GPIOx, uint16_t data);
void GPIO_ToggleOutput_Pin(GPIO_RegDef_t *p_GPIOx, uint8_t pinNumber);

// IRQ configuration and ISR handling
void GPIO_IrqConfig(uint8_t IrqNumber, uint8_t IrqPriority, uint8_t ControlType);
void GPIO_IrqHandling(uint8_t pinNumber);


#endif /* INC_DRV_STM32F407_GPIO_H_ */

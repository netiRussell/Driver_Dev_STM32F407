#include "drv_stm32f407_I2C.h"

/*
 * Initialization and De-initialization
 */
void I2C_Init(I2C_Handle_t *p_I2C_Handle_t){

}

void I2C_DeInit(I2C_Def_t *p_I2C_struct){

}


/*
 * Peripheral clock control
 */
void I2C_ClkControl(I2C_Def_t *p_I2C_struct, uint8_t ControlType){
	if(ControlType == 1){

		if(p_I2C_struct == I2C1){
			DRVF_I2C1_CLK_EN;
		} else if(p_I2C_struct == I2C2){
			DRVF_I2C2_CLK_EN;
		} else if(p_I2C_struct == I2C3){
			DRVF_I2C3_CLK_EN;
		}

	} else {

		if(p_I2C_struct == I2C1){
			DRVF_I2C1_CLK_DI;
		} else if (p_I2C_struct == I2C2) {
			DRVF_I2C2_CLK_DI;
		} else if (p_I2C_struct == I2C3) {
			DRVF_I2C3_CLK_DI;
		}

	}
}


/*
 * Get bit-position status
 */
// TODO: 
// uint8_t I2C_SR_Status( I2C_Def_t *p_I2C_struct, uint8_t bitPosition){
// 	if((p_I2C_struct->SR >> bitPosition) & 0b1){
// 		return HIGH;
// 	}

// 	return LOW;
// }


/*
 * Enable or Disable I2C peripheral
 */
void I2C_PeripheralControl( I2C_Def_t *p_I2C_struct, uint8_t ControlType ){
	if( ControlType == ENABLE ){
		p_I2C_struct->CR1 |= (0b1 << DRV_BITPOS_I2C_CR1_PE);
	} else {
		p_I2C_struct->CR1 &= ~(0b1 << DRV_BITPOS_I2C_CR1_PE);
	}
}

/*
 * IRQ configuration and ISR handling
 */
void I2C_IrqInterruptConfig(uint8_t IrqNumber, uint8_t ControlType){
	// Enable or Disable functionality
	uint8_t registerNumber = IrqNumber / 32; // corresponding number(0-7) for ISER or ICER
	IrqNumber = IrqNumber % 32; // corresponding bit position

	if (ControlType == ENABLE) {
		*( DRV_NVIC_ISER + registerNumber) |= 0b1 << IrqNumber;
	} else {
		*( DRV_NVIC_ICER + registerNumber) |= 0b1 << IrqNumber;
	}
}

void I2C_IrqPriorityConfig(uint8_t IrqNumber, uint8_t IrqPriority){
	//Priority functionality
	uint32_t registerNumber = IrqNumber / 4; // corresponding number(0-59) for IPR
	uint8_t validBitsPosition = 8 - NVIC_NUM_PRIOR_BITS;

	// IrqNumber = (# of a field) * # of bits per field = 8 + shift to the beginning of valid bits = 4
	IrqNumber = (IrqNumber % 4) * 8 + validBitsPosition; // corresponding bit position

	*( DRV_NVIC_IPR + (registerNumber)) &= ~(0b1111 << IrqNumber); // clear the bits
	*( DRV_NVIC_IPR + (registerNumber)) |= (uint32_t) IrqPriority << IrqNumber; // set the bits
}

/*
 * Event callback(a function meant to be used after some other function is used. In this case, after I2C_IrqHandling finished its execution)
 */
__weak void I2C_ApplicationEventCallback(I2C_Handle_t* p_I2C_Handle_t, uint8_t event){
	// To be overwritten by the user. This is strictly an API function.
}



#include "drv_stm32f407_spi.h"


/*
 * Initialization and De-initialization
 */
void SPI_Init(SPI_Handle_t *p_SPI_Handle_t){

}
void SPI_DeInit(SPI_Def_t *p_SPI_struct){

}

/*
 * Peripheral clock control
 */
void SPI_ClkControl(SPI_Def_t *p_SPI_struct, uint8_t ControlType){
	if(ControlType == 1){

		if(p_SPI_struct == SPI1){
			DRVF_SPI1_CLK_EN;
		} else if(p_SPI_struct == SPI2){
			DRVF_SPI2_CLK_EN;
		} else if(p_SPI_struct == SPI3){
			DRVF_SPI3_CLK_EN;
		} else if(p_SPI_struct == SPI4){
			DRVF_SPI4_CLK_EN;
		}

	} else {

		if(p_SPI_struct == SPI1){
			DRVF_SPI1_CLK_DI;
		} else if (p_SPI_struct == SPI2) {
			DRVF_SPI2_CLK_DI;
		} else if (p_SPI_struct == SPI3) {
			DRVF_SPI3_CLK_DI;
		} else if(p_SPI_struct == SPI4){
			DRVF_SPI4_CLK_DI;
		}

	}
}

/*
 * Data Sending and Receiving
 */
void SPI_SendData( SPI_Def_t *p_SPI_struct, uint8_t p_TxBuffer, uint32_t length ){

}

void SPI_ReceiveData( SPI_Def_t *p_SPI_struct, uint8_t p_RxBuffer, uint32_t length ){

}


/*
 * IRQ configuration and ISR handling
 */
uint8_t SPI_getIrqNum(uint8_t pinNumber){

	return 0;
}

void SPI_IrqInterruptConfig(uint8_t IrqNumber, uint8_t ControlType){

}

void SPI_IrqPriorityConfig(uint8_t IrqNumber, uint8_t IrqPriority){

}

void SPI_IrqHandling(SPI_Handle_t* p_SPI_Handle){

}



#include "drv_stm32f407_spi.h"


/*
 * Initialization and De-initialization
 */
void SPI_Init(SPI_Handle_t *p_SPI_Handle_t){

	uint32_t newCR1 = 0b0;

	// Set device's role
	newCR1 |= (p_SPI_Handle_t->SPI_Config.DeviceMode << BITPOS_SPI_CR1_MSTR);

	// Set SPI protocol mode( FD doesn't require any actions )
	if( p_SPI_Handle_t->SPI_Config.BusConfig == SPI_BUS_CONFIG_HD ){
		// Half Duplex
		newCR1 |= (0b1 << BITPOS_SPI_CR1_BIDIMODE);	// BIDIMODE set
	} else if( p_SPI_Handle_t->SPI_Config.BusConfig == SPI_BUS_CONFIG_RXONLY ){
		// Simplex
		newCR1 |= (0b1 << BITPOS_SPI_CR1_RXONLY);	// RXONLY set while BIDIMODE is cleared
	}

	// Set Baud Rate
	newCR1 |= (p_SPI_Handle_t->SPI_Config.SclkSpeed << BITPOS_SPI_CR1_BR);

	// Set Data Frame Format (Shift reg. = 8 or 16 bits)
	newCR1 |= (p_SPI_Handle_t->SPI_Config.DFF << BITPOS_SPI_CR1_DFF);

	// Set CPOL
	newCR1 |= (p_SPI_Handle_t->SPI_Config.CPOL << BITPOS_SPI_CR1_CPOL);

	// Set CPHA
	newCR1 |= (p_SPI_Handle_t->SPI_Config.CPHA << BITPOS_SPI_CR1_CPHA);

	// Set SSM
	newCR1 |= (p_SPI_Handle_t->SPI_Config.SSM << BITPOS_SPI_CR1_SSM);


	p_SPI_Handle_t->p_SPI_struct->CR1 = newCR1;
}


// !!! TO BE IMPLEMENTED: CR1_BIDIOE for half duplex and CR1_SSI for determing Slave status

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



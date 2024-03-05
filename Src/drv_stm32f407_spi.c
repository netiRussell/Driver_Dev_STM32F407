#include "drv_stm32f407_spi.h"


/*
 * Initialization and De-initialization
 */
void SPI_Init(SPI_Handle_t *p_SPI_Handle_t){

	// Enable the corresponding peripheral clock
	SPI_ClkControl(p_SPI_Handle_t->p_SPI_struct, ENABLE);

	// Disable the SPI peripheral
	//SPI_PeripheralControl(p_SPI_Handle_t->p_SPI_struct, DISABLE);

	uint32_t newCR1 = 0b0;

	// Set device's role
	newCR1 |= (p_SPI_Handle_t->SPI_Config.DeviceMode << DRV_BITPOS_SPI_CR1_MSTR);

	// Set SPI protocol mode( FD doesn't require any actions )
	if( p_SPI_Handle_t->SPI_Config.BusConfig == DRV_SPI_BUS_CONFIG_HD ){
		// Half Duplex
		newCR1 |= (0b1 << DRV_BITPOS_SPI_CR1_BIDIMODE);	// BIDIMODE set
	} else if( p_SPI_Handle_t->SPI_Config.BusConfig == DRV_SPI_BUS_CONFIG_RXONLY ){
		// Simplex
		newCR1 |= (0b1 << DRV_BITPOS_SPI_CR1_RXONLY);	// RXONLY set while BIDIMODE is cleared
	}

	// Set Baud Rate
	newCR1 |= (p_SPI_Handle_t->SPI_Config.SclkSpeed << DRV_BITPOS_SPI_CR1_BR);

	// Set Data Frame Format (Shift reg. = 8 or 16 bits)
	newCR1 |= (p_SPI_Handle_t->SPI_Config.DFF << DRV_BITPOS_SPI_CR1_DFF);

	// Set CPOL
	newCR1 |= (p_SPI_Handle_t->SPI_Config.CPOL << DRV_BITPOS_SPI_CR1_CPOL);

	// Set CPHA
	newCR1 |= (p_SPI_Handle_t->SPI_Config.CPHA << DRV_BITPOS_SPI_CR1_CPHA);

	// Set SSM
	newCR1 |= (p_SPI_Handle_t->SPI_Config.SSM << DRV_BITPOS_SPI_CR1_SSM);


	// Apply the configurations
	p_SPI_Handle_t->p_SPI_struct->CR1 = newCR1;

	// Enable the SPI peripheral
	//SPI_PeripheralControl(p_SPI_Handle_t->p_SPI_struct, ENABLE);
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
 * Get bit-position status
 */
uint8_t SPI_SR_Status( SPI_Def_t *p_SPI_struct, uint8_t bitPosition){
	if((p_SPI_struct->SR >> bitPosition) & 0b1){
		return HIGH;
	}

	return LOW;
}

/*
 * Data Sending and Receiving
 */
void SPI_SendData( SPI_Def_t *p_SPI_struct, uint8_t *p_TxBuffer, uint32_t length ){

	while (length > 0) {
		// Wait until TX buffer becomes empty
		while (!SPI_SR_Status(p_SPI_struct, DRV_BITPOS_SPI_SR_TXE)); // ! SUBSTITUTE WITH INTERRUPT

		if (SPI_SR_Status(p_SPI_struct, DRV_BITPOS_SPI_CR1_DFF)) {
			//16 bits DFF
			p_SPI_struct->DR = *((uint16_t*) p_TxBuffer);
			length -= 2;
			(uint16_t*) p_TxBuffer++;
		} else {
			//8 bits DFF
			p_SPI_struct->DR = *(p_TxBuffer);
			length--;
			p_TxBuffer++;
		}
	}

}

void SPI_ReceiveData( SPI_Def_t *p_SPI_struct, uint8_t *p_RxBuffer, uint32_t length ){

	while( length > 0){

		// Wait until Rx buffer becomes non-empty
		while( !SPI_SR_Status(p_SPI_struct, DRV_BITPOS_SPI_SR_RXNE) );

		if (SPI_SR_Status(p_SPI_struct, DRV_BITPOS_SPI_CR1_DFF)) {
			//16 bits DFF
			*((uint16_t*)p_RxBuffer) = p_SPI_struct->DR;
			length -= 2;
			(uint16_t*) p_RxBuffer++;
		} else {
			//8 bits DFF
			*(p_RxBuffer) = p_SPI_struct->DR;
			length--;
			p_RxBuffer++;
		}

	}

}

/*
 * Enable or Disable SPI peripheral
 */
void SPI_PeripheralControl( SPI_Def_t *p_SPI_struct, uint8_t ControlType ){
	if( ControlType == ENABLE ){
		p_SPI_struct->CR1 |= (0b1 << DRV_BITPOS_SPI_CR1_SPE);
	} else {
		p_SPI_struct->CR1 &= ~(0b1 << DRV_BITPOS_SPI_CR1_SPE);
	}
}

/*
 * SSI(value that overwrites NSS pin's value) control
 */
void SPI_SSIControl( SPI_Def_t *p_SPI_struct, uint8_t ControlType ){
	if( ControlType == ENABLE ){
		p_SPI_struct->CR1 |= (0b1 << DRV_BITPOS_SPI_CR1_SSI);
	} else {
		p_SPI_struct->CR1 &= ~(0b1 << DRV_BITPOS_SPI_CR1_SSI);
	}
}

/*
 * SSOE(enables or disables NSS output) control
 */
void SPI_SSOEControl( SPI_Def_t *p_SPI_struct, uint8_t ControlType ){
	if( ControlType == ENABLE ){
		p_SPI_struct->CR2 |= (0b1 << DRV_BITPOS_SPI_CR2_SSOE);
	} else {
		p_SPI_struct->CR2 &= ~(0b1 << DRV_BITPOS_SPI_CR2_SSOE);
	}
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



#ifndef INC_DRV_STM32F407_SPI_H_
#define INC_DRV_STM32F407_SPI_H_

#include "stm32f407.h" // target header file

typedef struct {
	uint8_t SPI_DeviceMode;
	uint8_t SPI_BusConfig;
	uint8_t SPI_SclkSpeed;
	uint8_t SPI_DFF;
	uint8_t SPI_CPOL;
	uint8_t SPI_CPHA;
	uint8_t SPI_SSM;
} SPI_Config_t;

typedef struct {
	SPI_Def_t *p_SPI_struct;
	SPI_Config_t SPI_Config;
} SPI_Handle_t;


/* ---------------------- APIs supported ---------------------- */

/*
 * Initialization and De-initialization
 */
void SPI_Init(SPI_Handle_t *p_SPI_Handle_t);
void SPI_DeInit(SPI_Def_t *p_SPI_struct);

/*
 * Peripheral clock control
 */
void SPI_ClkControl(SPI_Def_t *p_SPI_struct, uint8_t ControlType);

/*
 * Data Sending and Receiving
 */
void SPI_SendData( SPI_Def_t *p_SPI_struct, uint8_t p_TxBuffer, uint32_t length );
void SPI_ReceiveData( SPI_Def_t *p_SPI_struct, uint8_t p_RxBuffer, uint32_t length );

/*
 * IRQ configuration and ISR handling
 */
uint8_t SPI_getIrqNum(uint8_t pinNumber);
void SPI_IrqInterruptConfig(uint8_t IrqNumber, uint8_t ControlType);
void SPI_IrqPriorityConfig(uint8_t IrqNumber, uint8_t IrqPriority);
void SPI_IrqHandling(SPI_Handle_t* p_SPI_Handle);



#endif /* INC_DRV_STM32F407_SPI_H_ */

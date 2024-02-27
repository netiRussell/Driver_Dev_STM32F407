#ifndef INC_DRV_STM32F407_SPI_H_
#define INC_DRV_STM32F407_SPI_H_

#include "stm32f407.h" // target header file

typedef struct {
	uint8_t DeviceMode;	// Master or Slave (@SPI_DeviceMode)
	uint8_t BusConfig;	// Full duplex, half duples, or Simplex (@SPI_BusConfig)
	uint8_t SclkSpeed;	// SCLK = Main clock divide by 2^x (@SPI_SclkSpeed)
	uint8_t DFF;		// 8 bits shift register or 16 bits (@SPI_DFF)
	uint8_t CPOL;		// Idle state of a clock is low or high (@SPI_CPOL)
	uint8_t CPHA;		// Sample data at every 1st or 2nd edge of SCLK (@SPI_CPHA)
	uint8_t SSM;		// Slave management via software or hardware (@SPI_SSM)
} SPI_Config_t;

typedef struct {
	SPI_Def_t *p_SPI_struct;
	SPI_Config_t SPI_Config;
} SPI_Handle_t;


/*
 * Device modes (@SPI_DeviceMode)
 */
#define SPI_DEVICE_MODE_SLAVE	0
#define SPI_DEVICE_MODE_MASTER	1

/*
 * Bus configurations (@SPI_BusConfig)
 */
#define SPI_BUS_CONFIG_FD		0b00 // Full duplex
#define SPI_BUS_CONFIG_HD		0b01 // Half duplex
#define SPI_BUS_CONFIG_RXONLY	0b10 // Simplex with RX only

/*
 * SPI clock speed (@SPI_SclkSpeed)
 */
#define SPI_SCLK_SPEED_DIV2		0b000
#define SPI_SCLK_SPEED_DIV4		0b001
#define SPI_SCLK_SPEED_DIV8		0b010
#define SPI_SCLK_SPEED_DIV16	0b011
#define SPI_SCLK_SPEED_DIV32	0b100
#define SPI_SCLK_SPEED_DIV64	0b101
#define SPI_SCLK_SPEED_DIV128	0b110
#define SPI_SCLK_SPEED_DIV256	0b111

/*
 * Data Frame Format (@SPI_DFF)
 */
#define SPI_DFF_8BITS	0
#define SPI_DFF_16BITS	1

/*
 * CPOL (@SPI_CPOL)
 */
#define SPI_CPOL_LOW 0
#define SPI_CPOL_HIGH 1

/*
 * CPHA (SPI_CPHA)
 */
#define SPI_CPHA_FIRST	0
#define SPI_CPHA_SECOND	1

/*
 * Software Slave Management (SPI_CPHA)
 */
#define SPI_SSM_DISABLE	0
#define SPI_SSM_ENABLE	1

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

#ifndef INC_DRV_STM32F407_I2C_H_
#define INC_DRV_STM32F407_I2C_H_

#include "stm32f407.h" // target header file

typedef struct {
	uint8_t SCLspeed;			// Clock speed (@SCLspeed)
	uint8_t deviceAddr;		// Device address
	uint8_t enableACK;		// ACK control (@enableACK)
	uint8_t FMdutyCycle;	// Clock speed (@FMdutyCycle)
} I2C_Config_t;

typedef struct {
	I2C_Def_t *p_I2C_struct;
	I2C_Config_t I2C_Config;
} I2C_Handle_t;


/*
 * Clock speed (@SCLspeed)
 */
#define DRV_I2C_SCLSPEED_NORM		100000
#define DRV_I2C_SCLSPEED_FAST2K	200000
#define DRV_I2C_SCLSPEED_FAST4K	400000

/*
 * ACK control (@enableACK)
 */
#define DRV_I2C_ACK_ENABLE	1
#define DRV_I2C_ACK_DISABLE	0

/*
 * FM duty sycle (@FMdutyCycle)
 */
#define DRV_I2C_FMDUTY_16_9		1
#define DRV_I2C_FMDUTY_2			0

/* ---------------------- APIs supported ---------------------- */

/*
 * Initialization and De-initialization
 */
void I2C_Init(I2C_Handle_t *p_I2C_Handle_t);
void I2C_DeInit(I2C_Def_t *p_I2C_struct);


/*
 * Peripheral clock control
 */
void I2C_ClkControl(I2C_Def_t *p_I2C_struct, uint8_t ControlType);


/*
 * Get value of some I2C status at a bit position from I2C_SR register
 */
uint8_t I2C_SR_Status( I2C_Def_t *p_I2C_struct, uint8_t bitPosition);


/*
 * Data Sending and Receiving
 */



/*
 * Enable or Disable I2C peripheral
 */
void I2C_PeripheralControl( I2C_Def_t *p_I2C_struct, uint8_t ControlType );


/*
 * IRQ configuration and ISR handling
 */
uint8_t I2C_getIrqNum(uint8_t pinNumber);
void I2C_IrqInterruptConfig(uint8_t IrqNumber, uint8_t ControlType);
void I2C_IrqPriorityConfig(uint8_t IrqNumber, uint8_t IrqPriority);

/*
 * Event callback
 */
void I2C_ApplicationEventCallback(I2C_Handle_t* p_I2C_Handle, uint8_t event);



#endif /* INC_DRV_STM32F407_I2C_H_ */

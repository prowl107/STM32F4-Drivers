/*
 * stm32f407xx_i2c_driver.h
 *
 *  Created on: May 29, 2021
 *      Author: milesosborne
 */

#include "stm32f407xx_base_driver.h"

#ifndef INC_STM32F407XX_I2C_DRIVER_H_
#define INC_STM32F407XX_I2C_DRIVER_H_

/*
 * Configuration structure for I2Cx peripheral
 */
typedef struct
{
	uint32_t I2C_SCLSpeed; //Desired serial clock speed
	uint8_t I2C_DeviceAddress; //This devices address (slave mode)
	/*
	 * NOTE: Is a acking control variable necessary, does not seem so
	 *uint8_t I2C_ACKControl; 
	 * 
	*/
	uint8_t I2C_FMDutyCycle; //Duty cycle for fast mode
	uint8_t I2C_Mode; // Desired i2c mode (standard or fast mode)
} I2C_Config_t;

/*
 * handle structure for I2Cx peripheral
 */
typedef struct
{
	I2C_RegDef_t *pI2Cx;
	I2C_Config_t I2C_Config;
} I2C_Handle_t;

/*
 * Clock Enable Macros for I2Cx peripherals
 */
#define I2C1_PCLK_EN() (RCC->APB1ENR |= (1 << 21))
#define I2C2_PCLK_EN() (RCC->APB1ENR |= (1 << 22))
#define I2C3_PCLK_EN() (RCC->APB1ENR |= (1 << 23))

/*
 * Clock Disable Macros for I2Cx peripherals
 */
#define I2C1_PCLK_DI() (RCC->APB1ENR &= ~(1 << 21))
#define I2C2_PCLK_DI() (RCC->APB1ENR &= ~(1 << 22))
#define I2C3_PCLK_DI() (RCC->APB1ENR &= ~(1 << 23))

/*
 * @I2C_Mode
 */
#define I2C_MODE_SM 0	//I2C Standard Mode
#define I2C_MODE_FM 1	//I2C Fast Mode

/*
 * @I2C_SCLSSpeed
 */
#define I2C_SCL_SPEED_SM 100000	  //100KHz (STANDARD MODE)
#define I2C_SCL_SPEED_FM4k 400000 //400KHz (FAST MODE)
#define I2C_SCL_SPEED_FM2K 200000 //200KHz (FAST MODE)

/*
 * I2C Maximum rise time
 */
#define I2C_RISE_SM	1000	// 1000ns
#define I2C_RISE_FM	300		// 300ns

/*
 * @I2C_ACKControl
 * Automatic acking is diabled by default
 */
#define I2C_ACK_ENABLE 1  //Acknowledge returned after a byte is received (matched address or data)
#define I2C_ACK_DISABLE 0 //No acknowledge returned

/*
 * @I2C_FMDutyCycle
 */
#define I2C_FM_DUTY_2 0	   //Fm mode t_low/t_high = 2
#define I2C_FM_DUTY_16_9 1 //Fm mode t_low/t_high = 16/9

/*
 * I2C related status flags definition
 */
#define I2C_FLAG_TXE (1 << I2C_SR1_TXE)
#define I2C_FLAG_RXNE (1 << I2C_SR1_RXNE)
#define I2C_FLAG_ADDR (1 << I2C_SR1_ADDR)
#define I2C_FLAG_ADD10 (1 << I2C_SR1_ADD10)
#define I2C_FLAG_SB (1 << I2C_SR1_SB)
#define I2C_FLAG_BTF (1 << I2C_SR1_BTF)
#define I2C_FLAG_STOPF (1 << I2C_SR1_STOPF)
#define I2C_FLAG_BERR (1 << I2C_SR1_BERR)
#define I2C_FLAG_ARLO (1 << I2C_SR1_ARLO)
#define I2C_FLAG_AF (1 << I2C_SR1_AF)
#define I2C_FLAG_OVR (1 << I2C_SR1_OVR)
#define I2C_FLAG_TIMEOUT (1 << I2C_SR1_TIMEOUT)

#define I2C_DISABLE_SR RESET
#define I2C_ENABLE_SR SET

/*
 * I2C application states
 */
#define I2C_READY 0
#define I2C_BUSY_IN_RX 1
#define I2C_BUSY_IN_TX 2

/*
 * I2C application events
 */
#define I2C_EV_TX_COMPLETE 0
#define I2C_EV_RX_COMPLETE 1
#define I2C_EV_STOP 2
#define I2C_ERROR_BERR 3
#define I2C_ERROR_ARLO 4
#define I2C_ERROR_AF 5
#define I2C_ERROR_OVR 6
#define I2C_ERROR_TIMEOUT 7
#define I2C_EV_DATA_REQ 8
#define I2C_EV_DATA_RCV 9
#define I2C_EV_SR2_TRA 10

/*
 * Bit field definitions of I2C peripherals
 */

//I2C Control register 1 (I2C_CR1)
#define I2C_CR1_PE 0		//Peripheral enable
#define I2C_CR1_SMBUS 1		//SMBus mode
#define I2C_CR1_SMBTYPE 3	//SMBus type
#define I2C_CR1_ENARP 4		//ARP enable
#define I2C_CR1_ENPEC 5		//PEC enable
#define I2C_CR1_ENGC 6		//General call enable
#define I2C_CR1_NOSTRETCH 7 //Clock stretching disable (Slave mode)
#define I2C_CR1_START 8		//Start generation
#define I2C_CR1_STOP 9		//Stop generation
#define I2C_CR1_ACK 10		//Acknowledge enable
#define I2C_CR1_POS 11		//Acknowledge/PEC Position (for data reception)
#define I2C_CR1_PEC 12		//Packet error checking
#define I2C_CR1_ALERT 13	//SMBus alert
#define I2C_CR1_SWRST 15	//Software reset

//I2C Control register 2 (I2C_CR2)
#define I2C_CR2_FREQ 0	   //Peripheral clock frequency
#define I2C_CR2_ITERREN 8  //Error interrupt enable
#define I2C_CR2_ITEVTEN 9  //Event interrupt enable
#define I2C_CR2_ITBUFEN 10 //Buffer interrupt enable
#define I2C_CR2_DMAEN 11   //DMA requests enable
#define I2C_CR2_LAST 12	   //DMA last transfer

//I2C Own address register 1 (I2C_OAR1)
#define I2C_OAR1_ADD0 0		//Interface address
#define I2C_OAR1_ADD71 1	//Interface address
#define I2C_OAR1_ADD98 8	//Interface address
#define I2C_OAR1_ADDMODE 15 //Addressing mode (slave mode)

//I2C Own address register 2 (I2C_OAR2)
#define I2C_OAR2_ENDUAL 0 //Dual addressing mode enable
#define I2C_OAR2_ADD2 1	  //Interface address

//I2C Data register (I2C_DR)
#define I2C_DR_DR 0 //8-bit data register

//I2C Status register 1 (I2C_SR1)
#define I2C_SR1_SB 0		//Start bit (Master mode)
#define I2C_SR1_ADDR 1		//Address sent (master mode)/matched (slave mode)
#define I2C_SR1_BTF 2		//Byte transfer finished
#define I2C_SR1_ADD10 3		//10-bit header sent (Master mode)
#define I2C_SR1_STOPF 4		//Stop detection (slave mode)
#define I2C_SR1_RXNE 6		//Data register not empty (receivers)
#define I2C_SR1_TXE 7		//Data register empty (transmitters)
#define I2C_SR1_BERR 8		//Bus error
#define I2C_SR1_ARLO 9		//Arbitration lost (master mode)
#define I2C_SR1_AF 10		//Acknowledge failure
#define I2C_SR1_OVR 11		//Overrun/Underrun
#define I2C_SR1_PECERR 12	//PEC Error in reception
#define I2C_SR1_TIMEOUT 14	//Timeout or Tlow error
#define I2C_SR1_SMBALERT 15 //SMBus alert

//I2C Status register 2 (I2C_SR2)
#define I2C_SR2_MSL 0		 //Master/slave
#define I2C_SR2_BUSY 1		 //Bus busy
#define I2C_SR2_TRA 2		 //Transmitter/receiver
#define I2C_SR2_GENCALL 4	 //General call address (Slave mode)
#define I2C_SR2_SMBDEFAULT 5 //SMBus device default address (Slave mode)
#define I2C_SR2_SMBHOST 6	 //SMBus host header (Slave mode)
#define I2C_SR2_DUALF 7		 //Dual flag (Slave mode)
#define I2C_SR2_PEC 8		 //Packet error checking register

//I2C Clock control register (I2C_CCR)
#define I2C_CCR_CCR 0	//Clock control register in Fm/Sm mode (Master mode)
#define I2C_CCR_DUTY 14 //Fm mode duty cycle
#define I2C_CCR_FS 15	//I2C master mode selection

//I2C TRISE register (I2C_TRISE)
#define I2C_TRISE_TRISE 0 //Maximum rise time in Fm/Sm mode (Master mode)

//I2C FLTR register (I2C_FLTR)
#define I2C_FLTR_DNF 0	 //Digital noise filter
#define I2C_FLTR_ANOFF 4 //Analog noise filter OFF

/******************************APIs supported by this driver****************************/

/*
 * Peripheral clock control
 */
void i2c_peripheral_clock_enable(I2C_RegDef_t *pI2Cx);
void i2c_peripheral_clock_disable(I2C_RegDef_t *pI2Cx);

/*
 * Initialization / Deinitialization
 */
void i2c_init(I2C_Handle_t *pI2CHandle);
void i2c_deinit(I2C_RegDef_t *pI2Cx);
void i2c_software_reset(I2C_RegDef_t *pI2Cx);

/*
 * Data send & receive
 */
// void i2c_master_write(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer, uint8_t len, uint8_t slaveAddr);
void i2c_master_write(I2C_Handle_t* pI2CHandle, I2C_RegDef_t *pI2Cx, uint8_t slaveAddr, uint8_t command, uint8_t len, uint8_t *pData);
// void i2c_master_read(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer, uint8_t len, uint8_t slaveAddr);
void i2c_master_read(I2C_RegDef_t *pI2Cx, uint8_t slaveAddr, uint8_t command, uint8_t len, uint8_t* pRxBuffer);
void i2c_slave_write(I2C_RegDef_t *pI2C, uint8_t data);
uint8_t i2c_slave_read(I2C_RegDef_t *pI2C);

/*
 * Other peripheral control APIs
 */
uint8_t i2c_get_flag_status(I2C_RegDef_t *pI2Cx, uint32_t FlagName);
void i2c_peripheral_enable(I2C_RegDef_t *pI2Cx);
void i2c_peripheral_disable(I2C_RegDef_t *pI2Cx);
uint32_t rcc_get_pckl1_value(void);
void i2c_generate_start(I2C_RegDef_t *pI2Cx);
void i2c_generate_stop(I2C_RegDef_t *pI2Cx);
void i2c_clear_addr_flag(I2C_RegDef_t *pI2Cx);
void i2c_address_phase_write(I2C_RegDef_t *pI2Cx, uint8_t slaveAddr);
void i2c_address_phase_read(I2C_RegDef_t *pI2Cx, uint8_t slaveAddr);
void i2c_ack_enable(I2C_RegDef_t *pI2Cx);
void i2c_ack_disable(I2C_RegDef_t *pI2Cx);

/*
 * I2C_ApplicationEventCallback
 */
void I2C_ApplicationCallback(I2C_Handle_t *pI2CHandle, uint8_t handleEvent);

#endif /* INC_STM32F407XX_I2C_DRIVER_H_ */

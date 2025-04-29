/**
 * @file np0_hal.c
 *
 * @brief Source file for nPZero Hardware Abstraction Layer (HAL)
 *
 * This file contains the hardware-specific functions and configurations for
 * the I2C communication interface. Users are required to customize these
 * functions according to their target hardware and MCU setup.
 *
 * The functions in this file serve as an abstraction layer between the IPMIC
 * driver and the hardware-specific I2C implementation on the target system.
 * Users should modify these functions to match the I2C peripheral and
 * communication protocol used in their system.
 *
 * @note This file provides placeholder functions and configurations that need
 * to be adapted to the specific hardware and MCU setup. Users must replace
 * these placeholders with the actual implementations relevant to their system.
 *
 * @warning Incorrect configuration or improper implementation of these functions
 * may result in I2C communication failures or unexpected behavior.
 */

/*****************************************************************************
 * Includes
 *****************************************************************************/

#include "../Inc/np0_hal.h"
#include "stm32l0xx_hal.h"
#include <stdio.h>

/*****************************************************************************
 * Defines
 *****************************************************************************/

#define I2C_DELAY_MS 1

/*****************************************************************************
 * Data
 *****************************************************************************/

static I2C_HandleTypeDef m_hi2c1;

/*****************************************************************************
 * Private Methods
 *****************************************************************************/

/*****************************************************************************
 * Public Methods
 *****************************************************************************/

/**
 * @brief Function to read registers over I2C.
 */
np0_status_e np0_hal_read(uint8_t slave_address, uint8_t slave_register, uint8_t *pData, uint16_t size, uint32_t timeout)
{
	uint8_t transmitData[] = { slave_register};

//	if (HAL_I2C_Master_Transmit(&m_hi2c1, slave_address, transmitData, sizeof(transmitData), timeout) != HAL_OK)
//	{
//		return ERR;
//	}
//
//	HAL_Delay(I2C_DELAY_MS);
//
//	if (HAL_I2C_Master_Receive(&m_hi2c1, slave_address, pData, size, timeout) != HAL_OK)
//	{
//		return ERR;
//	}

	HAL_I2C_Mem_Read(&m_hi2c1, slave_address, transmitData[0], 1, pData, size, timeout);

	return OK;
}

/**
 * @brief Function to write to registers over I2C.
 */
np0_status_e np0_hal_write(uint8_t slave_address, uint8_t *pData, uint16_t size, uint32_t timeout)
{
	if (HAL_I2C_Master_Transmit(&m_hi2c1, slave_address, pData, size, timeout) != HAL_OK)
	{
		return ERR;
	}

	return OK;
}

/**
 * @brief Function to initialize I2C instance that will communicate with nPZero.
 */
np0_status_e np0_hal_init() {
	m_hi2c1.Instance = I2C1;
	m_hi2c1.Init.Timing = 0x00707CBB;
	m_hi2c1.Init.OwnAddress1 = 0;
	m_hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	m_hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	m_hi2c1.Init.OwnAddress2 = 0;
	m_hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
	m_hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	m_hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	if (HAL_I2C_Init(&m_hi2c1) != HAL_OK) {
		return ERR;
	}

	/* Configure Analogue filter */
	if (HAL_I2CEx_ConfigAnalogFilter(&m_hi2c1, I2C_ANALOGFILTER_DISABLE)
			!= HAL_OK) {
		return ERR;
	}

	/* Configure Digital filter */
	if (HAL_I2CEx_ConfigDigitalFilter(&m_hi2c1, 0) != HAL_OK) {
		return ERR;
	}

	return OK;
}

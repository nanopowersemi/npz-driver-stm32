/**
 * @file np0_temp_sensor.c
 * @brief Temperature sensor management for the AS622 sensor
 *
 * This file contains functions to configure and manage the AS622 temperature sensor.
 * It includes functions for setting temperature limits, configuring the sensor, and
 * handling temperature interrupts.
 *
 *@see np0_temp_sensor.h for the corresponding header file providing function prototypes and definitions.
 */

/*****************************************************************************
 * Includes
 *****************************************************************************/

#include "../../Drivers/nPZero_Driver/Inc/np0.h"
#include "../../Drivers/nPZero_Driver/Inc/np0_hal.h"
#include "np0_temp_sensor.h"
#include "stm32l0xx_hal.h"
#include <stddef.h>
#include <stdio.h>
#include <string.h>

/*****************************************************************************
 * Defines
 *****************************************************************************/

#define TMP_SENSOR_ADDR 0x49
#define TMP_SENSOR_ADDR_GND 0x48
#define TMP_SENSOR_CONFIGURATION_REG 0x01
#define TMP_SENSOR_TEMP_LOW_LIMIT_REG 0x02
#define TMP_SENSOR_TEMP_HIGH_LIMIT_REG 0x03
#define FLOAT_TO_INT_SCALE 1000
#define TEMP_LIMIT_LOW_RAW 0x0500  // 10°C in raw value
#define TEMP_LIMIT_HIGH_RAW 0x0C80 // 25°C in raw value
#define TEMP_SLEEP_LOW_RAW 0x2580  // 75°C in raw value
#define TEMP_SLEEP_HIGH_RAW 0x2800 // 80°C in raw value
/*****************************************************************************
 * Private Methods
 *****************************************************************************/

/**
 * @brief Sets the temperature limits for the sensor.
 *
 * @param [in] temperature_limit_low The low temperature limit in raw value.
 * @param [in] temperature_limit_high The high temperature limit in raw value.
 */
static void set_temperature_limits(uint16_t temperature_limit_low, uint16_t temperature_limit_high)
{
    uint8_t transmitData[3] = {0};

    // Set low temperature limit
    transmitData[0] = TMP_SENSOR_TEMP_LOW_LIMIT_REG;
    transmitData[1] = (temperature_limit_low >> 8) & 0xFF; // MSB
    transmitData[2] = temperature_limit_low & 0xFF;        // LSB

    printf("Setting low temperature limit: Reg=0x%02x, MSB=0x%02x, LSB=0x%02x\r\n", transmitData[0], transmitData[1],
            transmitData[2]);

    np0_hal_write(TMP_SENSOR_ADDR, transmitData, sizeof(transmitData), I2C_TRANSMISSION_TIMEOUT_MS);

    // Set high temperature limit
    transmitData[0] = TMP_SENSOR_TEMP_HIGH_LIMIT_REG;
    transmitData[1] = (temperature_limit_high >> 8) & 0xFF; // MSB
    transmitData[2] = temperature_limit_high & 0xFF;        // LSB

    printf("Setting high temperature limit: Reg=0x%02x, MSB=0x%02x, LSB=0x%02x\r\n", transmitData[0], transmitData[1],
            transmitData[2]);

    np0_hal_write(TMP_SENSOR_ADDR, transmitData, sizeof(transmitData), I2C_TRANSMISSION_TIMEOUT_MS);
}


/**
 * @brief Configures the temperature sensor with the specified mode.
 */
static void configure_temperature_sensor(uint8_t mode)
{
    uint8_t i2c_tmp_buffer[3];

    i2c_tmp_buffer[0] = TMP_SENSOR_CONFIGURATION_REG;
    switch (mode)
    {
        case 0:
            // (AL=1,CR[0]=0,CR[1]=1,IM=1,POL=0,SS=1)
            i2c_tmp_buffer[1] = 0x82; // MSB first
            i2c_tmp_buffer[2] = 0xA0; // LSB
            printf("Temperature configuration set to mode 0\r\n");
            break;
        case 1:
            // (AL=1,CR[0]=0,CR[1]=1,IM=1,POL=1,SS=0)
            i2c_tmp_buffer[1] = 0x06; // MSB first
            i2c_tmp_buffer[2] = 0xA0; // LSB
            printf("Temperature configuration set to mode 1\r\n");
            break;
        case 2:
            // (AL=1,CR[0]=0,CR[1]=0,IM=1,POL=1)
            i2c_tmp_buffer[1] = 0x06; // MSB first
            i2c_tmp_buffer[2] = 0x20; // LSB
            printf("Temperature configuration set to mode 2\r\n");
            break;
        case 3:
            // (AL=0,CR[0]=0,CR[1]=0,IM=0,POL=0)
            i2c_tmp_buffer[1] = 0x00; // MSB first
            i2c_tmp_buffer[2] = 0x00; // LSB
            printf("Temperature configuration set to mode 3\r\n");
            break;
        default:
        	printf("Invalid temperature configuration mode: %d\r\n", mode);
            return; // Invalid mode
    }

    np0_hal_write(TMP_SENSOR_ADDR, i2c_tmp_buffer, sizeof(i2c_tmp_buffer), I2C_TRANSMISSION_TIMEOUT_MS);
}

static uint16_t read_temperature(void)
{
    uint8_t data_rd[2];

    np0_hal_read(TMP_SENSOR_ADDR, 0x00, &data_rd[0], 2, I2C_TRANSMISSION_TIMEOUT_MS);

    HAL_Delay(2000);

    uint16_t temperature = (uint16_t) (data_rd[0] << 8) | data_rd[1];

    float temperature_f = temperature * 0.0078125;
    printf("Temperature read: Raw= 0x%02x%02x, Celsius= %d.%03d\r\n", data_rd[0], data_rd[1], (int) temperature_f,
            (int) ((temperature_f - (int) temperature_f) * FLOAT_TO_INT_SCALE));
    return temperature;
}

static void sleep_mode()
{
    uint8_t data_wr[10];
    data_wr[0] = TMP_SENSOR_CONFIGURATION_REG; // SM Sleep Mode;
    data_wr[1] = 0x01;                         // MSB;
    data_wr[2] = 0x00;                         // LSB
    np0_hal_write(TMP_SENSOR_ADDR, data_wr, sizeof(data_wr), I2C_TRANSMISSION_TIMEOUT_MS);
}

/*****************************************************************************
 * Public Methods
 *****************************************************************************/

void np0_temp_sensor_manage_mode2(void) {
	uint16_t temperature = read_temperature();

	if (temperature > TEMP_LIMIT_LOW_RAW && temperature < TEMP_LIMIT_HIGH_RAW)
	{
		set_temperature_limits(TEMP_LIMIT_LOW_RAW, TEMP_LIMIT_HIGH_RAW);
		HAL_Delay(2000);
		configure_temperature_sensor(1);
		printf("Temperature within range: INIT SET\r\n");
	}
	else
	{
		sleep_mode();
		set_temperature_limits(TEMP_SLEEP_LOW_RAW, TEMP_SLEEP_HIGH_RAW);
		configure_temperature_sensor(1);
		printf("Temperature out of range: INIT NOT SET\r\n");
	}

}

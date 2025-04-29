/**
 * @file example_peripheral_modes.h
 *
 *  @brief Header file for the nP0 peripheral modes examples.
 */

#ifndef EXAMPLE_PERIPHERAL_MODES_H_
#define EXAMPLE_PERIPHERAL_MODES_H_

#include "../../Drivers/nPZero_Driver/Inc/np0.h"

#define AS6212_LSB 0.0078125

/**
 * @brief This function is an example of how a user can implement the peripheral polling mode 0 on a I2C sensor.
 *
 * The sensor in this example is a temperature sensor from AMS, AS6212.
 *
 * @param [in] np0_device_config_s Pointer to the device configuration structure.
 */
void example_peripheral_modes_config_i2c_PM0(np0_device_config_s *np0_config);

#ifndef NP0_G0
/**
 * @brief This function is an example of how a user can implement the peripheral polling mode 1 on a I2C sensor.
 *
 * The sensor in this example is a temperature sensor from AMS, AS6212.
 *
 * @param [in] np0_device_config_s Pointer to the device configuration structure.
 */
void example_peripheral_modes_config_i2c_PM1(np0_device_config_s *np0_config);
#endif
/**
 * @brief This function is an example of how a user can implement the peripheral polling mode 2 on a I2C sensor.
 *
 * The sensor in this example is a temperature sensor from AMS, AS6212.
 *
 * @param [in] np0_device_config_s Pointer to the device configuration structure.
 */
void example_peripheral_modes_config_i2c_PM2(np0_device_config_s *np0_config);

/**
 * @brief This function is an example of how a user can implement the Peripheral Mode 3 on a I2C sensor.
 *
 * This mode can be used with simple peripherals that do not require any setup.
 * To enable the trigger for Peripheral Mode 3, please connect the interrupt pin to GND in either the PMOD 4 I2C
 * or PMOD 2 I2C connectors on the EVB.
 *
 * @param [in] np0_device_config_s Pointer to the device configuration structure.
 */
void example_peripheral_modes_config_i2c_PM3(np0_device_config_s *np0_config);

#ifndef NP0_G0
/**
 * @brief This function is an example of how a user can implement the Peripheral Mode 0 on a SPI sensor.
 *
 * The sensor in this example is a accelerometer from ST, LIS2DW12.
 *
 * @param [in] device PMIC struct which holds the value to be written to PMIC.
 * @param [in] sensor Peripheral struct which holds the value to be setTime To Wait Peripheral register that holds the
 * value to be written.
 */
//void example_peripheral_modes_config_spi_PM0(np0_device_s device, np0_peripheral_s sensor);

/**
 * @brief This function is an example of how a user can implement the Peripheral Mode 1 on a SPI sensor.
 *
 * The sensor in this example is a accelerometer from ST, LIS2DW12.
 *
 * @param [in] device PMIC struct which holds the value to be written to PMIC.
 * @param [in] sensor Peripheral struct which holds the value to be setTime To Wait Peripheral register that holds the
 * value to be written.
 */
//void example_peripheral_modes_config_spi_PM1(np0_device_s device, np0_peripheral_s sensor);

/**
 * @brief This function is an example of how a user can implement the Peripheral Mode 2 on a SPI sensor.
 *
 * The sensor in this example is a accelerometer from ST, LIS2DW12.
 *
 * @param [in] device PMIC struct which holds the value to be written to PMIC.
 * @param [in] sensor Peripheral struct which holds the value to be setTime To Wait Peripheral register that holds the
 * value to be written.
 */
//void example_peripheral_modes_config_spi_PM2(np0_device_s device, np0_peripheral_s sensor);

/**
 * @brief This function is an example of how a user can implement the Peripheral Mode 3 on a SPI sensor.
 *
 * The sensor in this example (ST LIS2DW12) does not support such a mode and therefor the example is a bare minimum to
 * implement Peripheral Mode 4.
 *
 * @param [in] device PMIC struct which holds the value to be written to PMIC.
 * @param [in] sensor Peripheral struct which holds the value to be setTime To Wait Peripheral register that holds the
 * value to be written.
 */
//void example_peripheral_modes_config_spi_PM3(np0_device_s device, np0_peripheral_s sensor);

/**
 * @brief This function is an example of how a user can implement and configures SPI interrupt on an
 * SPI sensor.
 *
 * The sensor in this example is a accelerometer from ST, LIS2DW12.
 *
 * @param [in] device PMIC struct which holds the value to be written to PMIC.
 * @param [in] sensor Peripheral struct which holds the value to be setTime To Wait Peripheral register that holds the
 * value to be written.
 */
//void example_peripheral_config_spi_interrupt(np0_device_s device, np0_peripheral_s sensor);

#endif

/**
 * @}
 */

#endif /* EXAMPLE_PERIPHERAL_MODES_H_ */

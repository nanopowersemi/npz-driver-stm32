/**
 * @file example_peripheral_modes.h
 *
 *  @brief Header file for the npz peripheral modes examples.
 */

#ifndef __EXAMPLE_PERIPHERAL_MODES_H
#define __EXAMPLE_PERIPHERAL_MODES_H

#include "../../Drivers/npz_Driver/Inc/npz.h"

#define AS6212_LSB 0.0078125

/**
 * @brief This function is an example of how a user can implement the peripheral polling mode 0 on a I2C sensor.
 *
 * The sensor in this example is a temperature sensor from AMS, AS6212.
 *
 * @param [in] npz_device_config_s Pointer to the device configuration structure.
 */
void example_peripheral_modes_config_i2c_PM0(npz_device_config_s *npz_config);

#ifndef npz_G0
/**
 * @brief This function is an example of how a user can implement the peripheral polling mode 1 on a I2C sensor.
 *
 * The sensor in this example is a temperature sensor from AMS, AS6212.
 *
 * @param [in] npz_device_config_s Pointer to the device configuration structure.
 */
void example_peripheral_modes_config_i2c_PM1(npz_device_config_s *npz_config);
#endif
/**
 * @brief This function is an example of how a user can implement the peripheral polling mode 2 on a I2C sensor.
 *
 * The sensor in this example is a temperature sensor from AMS, AS6212.
 *
 * @param [in] npz_device_config_s Pointer to the device configuration structure.
 */
void example_peripheral_modes_config_i2c_PM2(npz_device_config_s *npz_config);

/**
 * @brief This function is an example of how a user can implement the Peripheral Mode 3 on a I2C sensor.
 *
 * This mode can be used with simple peripherals that do not require any setup.
 * To enable the trigger for Peripheral Mode 3, please connect the interrupt pin to GND in either the PMOD 4 I2C
 * or PMOD 2 I2C connectors on the EVB.
 *
 * @param [in] npz_device_config_s Pointer to the device configuration structure.
 */
void example_peripheral_modes_config_i2c_PM3(npz_device_config_s *npz_config);

#ifndef npz_G0
/**
 * @brief This function is an example of how a user can implement the Peripheral Mode 0 on a SPI sensor.
 *
 * The sensor in this example is a accelerometer from ST, LIS2DW12.
 *
 * @param [in] device PMIC struct which holds the value to be written to PMIC.
 * @param [in] sensor Peripheral struct which holds the value to be setTime To Wait Peripheral register that holds the
 * value to be written.
 */
//void example_peripheral_modes_config_spi_PM0(npz_device_s device, npz_peripheral_s sensor);

/**
 * @brief This function is an example of how a user can implement the Peripheral Mode 1 on a SPI sensor.
 *
 * The sensor in this example is a accelerometer from ST, LIS2DW12.
 *
 * @param [in] device PMIC struct which holds the value to be written to PMIC.
 * @param [in] sensor Peripheral struct which holds the value to be setTime To Wait Peripheral register that holds the
 * value to be written.
 */
//void example_peripheral_modes_config_spi_PM1(npz_device_s device, npz_peripheral_s sensor);

/**
 * @brief This function is an example of how a user can implement the Peripheral Mode 2 on a SPI sensor.
 *
 * The sensor in this example is a accelerometer from ST, LIS2DW12.
 *
 * @param [in] device PMIC struct which holds the value to be written to PMIC.
 * @param [in] sensor Peripheral struct which holds the value to be setTime To Wait Peripheral register that holds the
 * value to be written.
 */
//void example_peripheral_modes_config_spi_PM2(npz_device_s device, npz_peripheral_s sensor);

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
//void example_peripheral_modes_config_spi_PM3(npz_device_s device, npz_peripheral_s sensor);

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
//void example_peripheral_config_spi_interrupt(npz_device_s device, npz_peripheral_s sensor);

#endif

/**
 * @}
 */

#endif /* __EXAMPLE_PERIPHERAL_MODES_H */

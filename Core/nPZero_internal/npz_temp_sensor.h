/**
 * @file npz_temp_sensor.h
 * @brief Temperature sensor management for the AS6212 sensor
 *
 * This header file contains functions to configure and manage the AS6212 temperature sensor.
 * It includes functions for handling temperature interrupts.
 *
 */

#ifndef __NPZ_TEMP_SENSOR_H
#define __NPZ_TEMP_SENSOR_H

/** @cond */
#include <stdbool.h>
#include <stdint.h>
/** @endcond */

/**
 * @brief Initialize I2c temp sensor for mode2.
 *
 * This function reads the current temperature from the sensor, converts it to Celsius,
 * and checks if it is within the predefined temperature limits. If the temperature is
 * within the range, it sets the appropriate configures for the sensor.
 * Otherwise, it sets the sensor to sleep mode and reconfigures it.
 */
void npz_temp_sensor_manage_mode2(void);

#endif /* _NPZ_TEMP_SENSOR_H */

/**
 * @file example_adc.h
 *
 *  @brief Header file for the nPZero ADC examples.
 */

#ifndef __EXAMPLE_ADC_H
#define __EXAMPLE_ADC_H

#include "../../Drivers/nPZero_Driver/Inc/npz.h"

/**
 * @brief This function is an example of how a user can implement the external adc.
 *
 * If ADC_IN analog pin not connected. Please connect the pin in order to enable the trigger of external adc.
 *
 * @param [in] npz_device_config_s Pointer to the device configuration structure.
 */

void example_adc_external(npz_device_config_s *npz_config);

/**
 * @brief This function is an example of how a user can implement the internal adc.
 *
 * The internal ADC channel is always enabled and operates at 3.3V VBAT. Use a power supply for VDD = 2.5 VDC.
 * If VDD falls below 2.2 VDC, the internal ADC is triggered, and the host wakes up.
 *
 * @param [in] npz_device_config_s Pointer to the device configuration structure.
 */
void example_adc_internal(npz_device_config_s *npz_config);

#endif /* __EXAMPLE_ADC_H */

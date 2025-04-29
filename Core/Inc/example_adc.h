/**
 * @file example_adc.h
 *
 *  @brief Header file for the nP0 ADC examples.
 */

#ifndef EXAMPLE_ADC_H_
#define EXAMPLE_ADC_H_

#include "../../Drivers/nPZero_Driver/Inc/np0.h"

/**
 * @brief This function is an example of how a user can implement the external adc.
 *
 * If ADC_IN analog pin not connected. Please connect the pin in order to enable the trigger of external adc.
 *
 * @param [in] np0_device_config_s Pointer to the device configuration structure.
 */

void example_adc_external(np0_device_config_s *np0_config);

/**
 * @brief This function is an example of how a user can implement the internal adc.
 *
 * The internal ADC channel is always enabled and operates at 3.3V VBAT. Use a power supply for VDD = 2.5 VDC.
 * If VDD falls below 2.2 VDC, the internal ADC is triggered, and the host wakes up.
 *
 * @param [in] np0_device_config_s Pointer to the device configuration structure.
 */
void example_adc_internal(np0_device_config_s *np0_config);

#endif /* EXAMPLE_ADC_H_ */

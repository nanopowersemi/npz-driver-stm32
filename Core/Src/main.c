/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2023 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "../../Drivers/nPZero_Driver/Inc/np0_device_control.h"
#include "../../Drivers/nPZero_Driver/Inc/np0.h"
#include "../../Drivers/nPZero_Driver/Inc/np0_hal.h"
#include "../../Drivers/nPZero_Driver/Inc/np0_logs.h"
#include "../../Drivers/nPZero_Driver/Inc/np0_registers.h"

#include "stm32l0xx_hal.h"
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
void Configure_BOR();
void Check_Reset_Cause();
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int _write(int file, char *data, int len) {
	HAL_UART_Transmit(&huart2, (uint8_t*) data, len, HAL_MAX_DELAY);
	return len;
}

np0_peripheral_config_s peripheral_3 = {
	    .power_mode = POWER_MODE_PERIODIC,
	    .communication_protocol = COM_SPI,
	    .polling_mode = POLLING_MODE_PERIODIC_READ_COMPARE_THRESHOLD,
	    .power_switch_mode = POWER_SWITCH_MODE_LOGIC_OUTPUT_HIGH,
	    .interrupt_pin_mode = INTERRUPT_PIN_MODE_INPUT_ACTIVE_HIGH,
	    .comparison_mode = COMPARISON_MODE_INSIDE_THRESHOLD,
	    .sensor_data_type = DATA_TYPE_INT16,
	    .multi_byte_transfer_enable = 0,
	    .swap_registers = 0,
	    .spi_cfg.bytes_from_sram_num = 2,
	    .spi_cfg.bytes_from_sram = {0x20, 0x10},
	    .spi_cfg.bytes_from_sram_read_num = 1,
	    .spi_cfg.bytes_from_sram_read = {0xA8},
	    .spi_cfg.mode = SPIMOD_SPI_MODE_0,
	    .polling_period = 50,
	    .pre_wait_time = PRE_WAIT_TIME_EXTEND_256,
	    .post_wait_time = POST_WAIT_TIME_EXTEND_256,
	    .time_to_wait = 156,
	    .threshold_over = 1000,
	    .threshold_under = 64536,
};

np0_peripheral_config_s peripheral_4 = {
    .communication_protocol = COM_I2C,
    .power_mode = POWER_MODE_PERIODIC,
    .polling_mode = POLLING_MODE_PERIODIC_READ_COMPARE_THRESHOLD,
    .power_switch_mode = POWER_SWITCH_MODE_LOGIC_OUTPUT_HIGH,
    .interrupt_pin_mode = INTERRUPT_PIN_MODE_INPUT_ACTIVE_HIGH,
    .comparison_mode = COMPARISON_MODE_INSIDE_THRESHOLD,
    .sensor_data_type  = DATA_TYPE_INT16,
    .multi_byte_transfer_enable  = MULTIBYTE_TRANSFER_ENABLE,
    .swap_registers = ENDIAN_BIG,
    .polling_period = 0x012C, // Wakeup peripheral every 30 seconds with 10Hz clock
    .i2c_cfg.sensor_address = 0x49,
    .i2c_cfg.command_num = 1,
    .i2c_cfg.bytes_from_sram = {0x01, 0x82, 0xA0},
    .i2c_cfg.reg_address_value = 0x00,
    .i2c_cfg.wake_on_nak = ENABLED,
    .i2c_cfg.num_of_retries_on_nak = 3,
    .time_to_wait = 0x31, /* 0x31 (49 in decimal):
                           * When multiplied by 4096: 49 * 4096 = 200704  clock cycles * 2.5�s (1 / 400000),
                           * which equals 501.76ms at 400kHz.
                           * When multiplied by 256: 49 * 256 = 12544  clock cycles * 2.5�s (1 / 400000),
                           * which equals 31.36ms at 400kHz.
                           */
    .pre_wait_time = POST_WAIT_TIME_EXTEND_4096,
    .post_wait_time = POST_WAIT_TIME_EXTEND_4096,
    .threshold_over = 3200,
    .threshold_under = 1280,
};

np0_adc_config_channels_s np0_adc_internal_config = {
    .wakeup_enable = 0,
    .over_threshold = 0x2B,
    .under_threshold = 0x28,
};

np0_adc_config_channels_s np0_adc_external_config = {
    .wakeup_enable = 0,
    .over_threshold = 0x2D,
    .under_threshold = 0x26,
};

np0_device_config_s np0_configuration = {
    .host_power_mode = HOST_POWER_MODE_LOGIC_OUTPUT,
    .power_switch_normal_mode_per1 = 0,
    .power_switch_normal_mode_per2 = 0,
    .power_switch_normal_mode_per3 = 1,
    .power_switch_normal_mode_per4 = 1,
    .power_switch_gate_boost = 0,
    .system_clock_divider = SCLK_DIV_DISABLE,
    .system_clock_source = SYS_CLOCK_10HZ,
    .io_strength = IO_STR_NORMAL,
    .i2c_pull_mode = I2C_PULL_DISABLE,
    .spi_auto = SPI_PINS_ALWAYS_ON,
    .xo_clock_out_sel = XO_CLK_OFF,
    .wake_up_per1 = 0,
    .wake_up_per2 = 0,
    .wake_up_per3 = 1,
    .wake_up_per4 = 1,
    .wake_up_any_or_all = WAKEUP_ANY,
    .global_timeout = 0x0BB8,
    .interrupt_pin_pull_up_pin1 = INT_PIN_PULL_DISABLED,
    .interrupt_pin_pull_up_pin2 = INT_PIN_PULL_DISABLED,
    .interrupt_pin_pull_up_pin3 = INT_PIN_PULL_DISABLED,
    .interrupt_pin_pull_up_pin4 = INT_PIN_PULL_DISABLED,
    .adc_ext_sampling_enable = 0,
    .adc_clock_sel = ADC_CLK_256,
    .adc_channels = {0, 0},
    .peripherals = {0, 0, &peripheral_3, &peripheral_4},
};

static void read_peripheral_temp(int peripheral_value)
{
    // Calculate the temperature in degrees Celsius
    float temperature = peripheral_value * 0.0078125; //     // AS6212 temperature sensor resolution is 0.0078125°C.
    printf("Calculated temperature: %d.%03d °C\r\n", (int) temperature, (int) ((temperature - (int) temperature) * 1000));
}

static void np0_read_status_registers(np0_status_s *status)
{
    // Read the first status register
    if (np0_read_STA1(&status->status1) != OK)
    {
        return;
    }

    // Handle status1
    if (status->status1.reset_source == RESETSOURCE_NONE)
    {
        printf("Reset source is None\r\n");
    }
    else if (status->status1.reset_source == RESETSOURCE_PWR_RESET)
    {
        printf("Power-on reset triggered\r\n");
    }
    else if (status->status1.reset_source == RESETSOURCE_SOFT_RESET)
    {
        printf("Soft reset triggered (via I2C command)\r\n");
    }
    else if (status->status1.reset_source == RESETSOURCE_EXT_RESET)
    {
        printf("External reset triggered (via RST pin)\r\n");
    }

    if (status->status1.ext_adc_triggered == 1)
    {
        if (!np0_device_handle_adc_external())
        {
            return;
        }
    }

    if (status->status1.int_adc_triggered == 1)
    {
        if (!np0_device_handle_adc_internal())
        {
            return;
        }
    }

    if (status->status1.global_timeout_triggered == 1)
    {
        printf("Global Timeout triggered before any wake up source triggered\r\n");
    }

    // Read the second status register
    if (np0_read_STA2(&status->status2) != OK)
    {
        return;
    }

    // Handle status2
    // Arrays to map peripherals and switches
    np0_psw_e switches[4] = {PSW_LP1, PSW_LP2, PSW_LP3, PSW_LP4};
    uint8_t triggered[4] = {status->status2.per1_triggered, status->status2.per2_triggered,
                            status->status2.per3_triggered, status->status2.per4_triggered};
    uint8_t timeouts[4] = {status->status2.per1_global_timeout, status->status2.per2_global_timeout,
                           status->status2.per3_global_timeout, status->status2.per4_global_timeout};

    // Iterate over each peripheral to check for triggers and timeouts
    for (int i = 0; i < 4; i++)
    {
        if (triggered[i]) // Check if peripheral is triggered
        {
            int peripheral_value = 0;

            np0_device_read_peripheral_value(switches[i], i,
                                             &peripheral_value); // Read the peripheral value based on the switch

            if (np0_configuration.peripherals[i]->communication_protocol == COM_I2C &&
                (np0_configuration.peripherals[i]->polling_mode == POLLING_MODE_PERIODIC_READ_COMPARE_THRESHOLD ||
                 np0_configuration.peripherals[i]->polling_mode ==
                     POLLING_MODE_PERIODIC_WAIT_INTERRUPT_COMPARE_THRESHOLD))
            {
                read_peripheral_temp(peripheral_value);
            }
        }

        if (timeouts[i]) // Check if global timeout is triggered for this peripheral
        {
            printf("Peripheral %d global timeout was triggered\r\n", i + 1); // Log the timeout event
        }
    }
}

/**@brief Function for find nPZero Gen1.
 */
uint8_t nP0_search(void)
{
    uint8_t sample_data;

    np0_hal_read(NP0_I2C_ADDRESS, REG_ID, &sample_data, 1, 5);

    if ((sample_data) == 0x60)
    {
    	printf("[--- nP0 GEN1 INIT OK ---]\r\n");
    	return 1;
    }
    else
    {
    	printf("[--- nP0 GEN1 INIT NOK 0x%x---]\r\n", sample_data);
    	return 0;
    }
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

    /* USER CODE BEGIN 1 */

    /* USER CODE END 1 */

    /* MCU Configuration--------------------------------------------------------*/

    /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
    HAL_Init();

    /* USER CODE BEGIN Init */

    /* USER CODE END Init */

    /* Configure the system clock */
    SystemClock_Config();

    /* USER CODE BEGIN SysInit */

    /* USER CODE END SysInit */

    /* Initialize all configured peripherals */
    MX_GPIO_Init();
    MX_I2C1_Init();
    MX_USART2_UART_Init();
    /* USER CODE BEGIN 2 */
    // Print welcome message
    printf("nPZero Host is active.........\r\n");

    // Initialize the nP0 interface
    np0_hal_init();

    HAL_Delay(1000);

    // Read the status registers of the nP0 device after every reset
    np0_status_s np0_status = { 0 };
    np0_read_status_registers(&np0_status);

    nP0_search();

    // Send the configuration to the device
    np0_device_configure(&np0_configuration);

    // Logs and reads all configuration registers for debugging purposes
    np0_log_configurations(&np0_configuration);

    // Add a delay in main, to give the user time to flash the MCU before it enters sleep
    // This delay should be removed in production code
    HAL_Delay(500);

    // At the end of your operations, put the device into sleep mode
    np0_device_go_to_sleep();

    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    while (1)
    {
        /* USER CODE END WHILE */

        /* USER CODE BEGIN 3 */
	    // Do nothing.
	    HAL_Delay(1000);
        printf("Main thread is alive...\r\n");
    }
    /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2|RCC_PERIPHCLK_I2C1;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_HSI;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x00503D58;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */
  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(Host_Led_GPIO_Port, Host_Led_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : Host_Led_Pin */
  GPIO_InitStruct.Pin = Host_Led_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(Host_Led_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void Check_Reset_Cause() {
	if (__HAL_RCC_GET_FLAG(RCC_FLAG_PINRST)) {
		printf("Reset due to NRST pin reset.\r\n");
	}
	if (__HAL_RCC_GET_FLAG(RCC_FLAG_PORRST)) {
		printf("Reset due to Power-on reset.\r\n");

	}
	if (__HAL_RCC_GET_FLAG(RCC_FLAG_SFTRST)) {
		printf("Reset due to Software reset.\r\n");
	}
	if (__HAL_RCC_GET_FLAG(RCC_FLAG_IWDGRST)) {
		printf("Reset due to Independent Watchdog reset.\r\n");
	}
	if (__HAL_RCC_GET_FLAG(RCC_FLAG_WWDGRST)) {
		printf("Reset due to Window Watchdog reset.\r\n");
	}
	if (__HAL_RCC_GET_FLAG(RCC_FLAG_LPWRRST)) {
		printf("Reset due to Low Power reset.\r\n");
	}
	if (__HAL_PWR_GET_FLAG(PWR_FLAG_SB)) {
		printf("STM32 Host Woke Up from Standby Mode......\r\n");
	}

	__HAL_RCC_CLEAR_RESET_FLAGS();
}

void Configure_BOR(void) {
	FLASH_OBProgramInitTypeDef obConfig = { 0 };

	// Unlock the FLASH control register access
	HAL_FLASH_Unlock();

	// Unlock the Options Bytes
	HAL_FLASH_OB_Unlock();

	// Get the current option bytes configuration
	HAL_FLASHEx_OBGetConfig(&obConfig);
	//printf("Current BOR level: %d\r\n", obConfig.BORLevel);

	// Check if BOR is set to the desired level
	if (obConfig.BORLevel != OB_BOR_LEVEL1) {
		obConfig.OptionType = OPTIONBYTE_BOR;
		obConfig.BORLevel = OB_BOR_LEVEL1;
		if (HAL_FLASHEx_OBProgram(&obConfig) != HAL_OK) {
			// Error occurred while setting BOR level
			printf("Error occurred while setting BOR level.\r\n");
			Error_Handler();
		}

		// Launch the option byte loading to apply the new BOR level
		printf("Launching Option Bytes programming to disable BOR.\r\n");
		HAL_FLASH_OB_Launch();
	} else {
		//printf("BOR level is already set to OB_BOR_LEVEL1.\r\n");
	}

	// Lock the Options Bytes and FLASH control register access
	HAL_FLASH_OB_Lock();
	HAL_FLASH_Lock();
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
	}
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
    /* User can add his own implementation to report the file name and line number,
       ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

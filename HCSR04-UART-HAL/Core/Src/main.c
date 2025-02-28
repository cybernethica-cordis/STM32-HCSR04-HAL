/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program file for measuring distance using the HC-SR04
  *                   ultrasonic sensor with STM32F446RET6 and UART output.
  *
  * This program:
  *   - Uses TIM2 configured at 1 MHz (1 tick = 1 µs) to generate
  *     micro-delays and measure the pulse duration from the sensor;
  *   - Calculates the speed of sound taking into account the air temperature;
  *   - Generates a 10 µs pulse on the TRIG output to initiate the measurement;
  *   - Waits for the signal on the ECHO pin using a timer and timeout
  *     to prevent hanging if no signal is received;
  *   - Measures the pulse duration (accounting for possible timer overflow) and
  *     filters the results based on a range of values;
  *   - Calculates the distance considering that the measured time corresponds
  *     to the signal traversing the path twice;
  *   - Applies averaging of three measurements to improve result stability.
  *
  * Connections:
  *   - TRIG: GPIOB PIN9 (MCU output);
  *   - ECHO: GPIOB PIN8 (MCU input).
  *
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// Definitions for ultrasonic sensor pins
#define TRIG_PIN GPIO_PIN_9   // Pin for the trigger signal.
#define ECHO_PIN GPIO_PIN_8   // Pin for receiving the echo signal.
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
// Global variables
TIM_HandleTypeDef htim2;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
float calibration_factor = 1.0;  // Calibration factor for adjusting measurements.
float temperature = 20.0;        // Air temperature in °C for calculating the speed of sound.
char uart_buffer[100];           // Buffer for forming the string to be sent via UART.
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
// Initialization function prototypes
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART2_UART_Init(void);

/* USER CODE BEGIN PFP */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// Function to create a delay in microseconds using TIM2
void micro_delay(uint32_t delay_us) {
  __HAL_TIM_SET_COUNTER(&htim2, 0); // Reset the timer counter to start the count.
  while (__HAL_TIM_GET_COUNTER(&htim2) < delay_us); // Wait in a loop until the counter reaches the specified value (delay_us microseconds).
}
/* USER CODE END 0 */

/**
  * @brief  Main application function (entry point).
  * @retval int
  */

int main(void) {
  /* USER CODE BEGIN 1 */
  // Declaration of local variables
  uint32_t pulse_start = 0;  // Timer value at the start of the echo pulse (in microseconds).
  uint32_t pulse_end = 0;    // Timer value at the end of the echo pulse.
  uint32_t pulse_width = 0;  // The difference between pulse_end and pulse_start, representing the echo pulse duration.
  uint32_t timeout = 30000;  // Maximum waiting time for the echo signal (30 ms).
  float distance = 0.0;      // Calculated distance to the object (in centimeters).
  float sound_speed = 0.0;   // Speed of sound (in cm/µs) calculated taking the air temperature into account.
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/
  /* Reset of all peripherals, Initializes the Flash interface and the Systick */
  HAL_Init();

  /* USER CODE BEGIN Init */
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  /* USER CODE END SysInit */

  // Initialize GPIO, timer, and UART according to the STM32CubeMX configuration.
  MX_GPIO_Init();
  MX_TIM2_Init();
  MX_USART2_UART_Init();

  /* USER CODE BEGIN 2 */
  // Start TIM2 (used for micro-delays and measuring pulse duration)
  HAL_TIM_Base_Start(&htim2);

  // Calculate the speed of sound considering the air temperature.
  // Formula: v = 331.3 + 0.606 * temperature, where v is the speed of sound in m/s.
  // Convert to cm/µs: 1 m/s = 100 cm / 1,000,000 µs = v / 10000 cm/µs.
  // At 20°C, we get approximately: (331.3 + 0.606 * 20) / 10000 ≈ 0.03434 cm/µs.
  sound_speed = (331.3 + 0.606 * temperature) / 10000.0;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) {
    // **** Generate trigger pulse for HC-SR04 ****
    // Reset the TRIG pin to LOW and wait 4 µs to stabilize the sensor.
    HAL_GPIO_WritePin(GPIOB, TRIG_PIN, GPIO_PIN_RESET);
    micro_delay(4);

    // Generate a HIGH pulse lasting 10 µs.
    HAL_GPIO_WritePin(GPIOB, TRIG_PIN, GPIO_PIN_SET);
    micro_delay(10);
    HAL_GPIO_WritePin(GPIOB, TRIG_PIN, GPIO_PIN_RESET);

    // **** Wait for the signal to appear on the ECHO pin ****
    // Use the timer to limit the waiting time (timeout) to avoid an infinite loop if no signal is received.
    uint32_t timeout_timer = __HAL_TIM_GET_COUNTER(&htim2); // Record the start time of the waiting period.
    while (HAL_GPIO_ReadPin(GPIOB, ECHO_PIN) == GPIO_PIN_RESET) {
      // If the difference between the current time and the start time exceeds the timeout, break the waiting loop.
      if ((__HAL_TIM_GET_COUNTER(&htim2) - timeout_timer) > timeout) {
        break;
      }
    }

    // After the waiting loop, check if the ECHO pin has been set to HIGH:
    // If the pin is still LOW, the signal was not received and the measurement is considered invalid.
    if (HAL_GPIO_ReadPin(GPIOB, ECHO_PIN) != GPIO_PIN_SET) {
      // Send an error message via UART and skip the current measurement.
      HAL_UART_Transmit(&huart2, (uint8_t*)"Error: No echo\r\n", strlen("Error: No echo\r\n"), 100);
      HAL_Delay(2000);
      continue;
    }

    // **** Measure the pulse duration on the ECHO pin ****
    // Record the time at the start of the pulse.
    pulse_start = __HAL_TIM_GET_COUNTER(&htim2);
    // Set an additional timeout for the pulse measurement.
    uint32_t pulse_timeout = pulse_start + timeout;

    // Wait while the ECHO pin remains HIGH:
    // If the measurement time exceeds the timeout, break out of the loop.
    while (HAL_GPIO_ReadPin(GPIOB, ECHO_PIN) == GPIO_PIN_SET) {
      if (__HAL_TIM_GET_COUNTER(&htim2) > pulse_timeout) {
        break;
      }
    }

    // Record the time at the end of the pulse.
    pulse_end = __HAL_TIM_GET_COUNTER(&htim2);

    // **** Calculate the pulse duration accounting for timer overflow ****
    // If the timer did not overflow, the difference between pulse_end and pulse_start gives the duration.
    // If an overflow occurred (pulse_end < pulse_start), adjust the value accordingly.
    if (pulse_end >= pulse_start) {
      pulse_width = pulse_end - pulse_start;
    } else {
      pulse_width = (0xFFFFFFFF - pulse_start) + pulse_end;
    }

    // **** Filter the measurements ****
    // Discard measurements that are too short (< 100 µs, which corresponds to ~1.7 cm)
    // or too long (> 25000 µs, which corresponds to ~425 cm), as they are likely erroneous.
    if (pulse_width > 25000 || pulse_width < 100) {
      HAL_UART_Transmit(&huart2, (uint8_t*)"Invalid pulse\r\n", strlen("Invalid pulse\r\n"), 100);
      continue;
    }

    // **** Calculate the distance ****
    // Use the formula: distance = (pulse_width * sound_speed * calibration_factor) / 2.
    // Division by 2 is required because the measured time corresponds to the signal traversing the path twice.
    distance = (pulse_width * sound_speed * calibration_factor) / 2.0;

    // **** Averaging multiple measurements ****
    // To improve result stability, store the last three measurements in a static buffer,
    // then calculate their average.
    static float avg_buf[3] = {0};
    static uint8_t idx = 0;
    avg_buf[idx++] = distance;
    if (idx >= 3) idx = 0;
    float avg = (avg_buf[0] + avg_buf[1] + avg_buf[2]) / 3;

    // **** Send the result via UART ****
    // Form a string with the averaged distance value and send it via UART.
    sprintf(uart_buffer, "Distance: %.1f cm\r\n", avg);
    HAL_UART_Transmit(&huart2, (uint8_t*)uart_buffer, strlen(uart_buffer), 100);

    // A short delay (200 ms) between measurements.
    HAL_Delay(200);
    /* USER CODE END WHILE */
    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */

void SystemClock_Config(void) {
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  in the RCC_OscInitTypeDef structure */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK) {
    Error_Handler();
  }
}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */

static void MX_TIM2_Init(void) {
  /* USER CODE BEGIN TIM2_Init 0 */
  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */
  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 15;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK) {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK) {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK) {
    Error_Handler();
  }

  /* USER CODE BEGIN TIM2_Init 2 */
  /* USER CODE END TIM2_Init 2 */
}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */

static void MX_USART2_UART_Init(void) {
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
  if (HAL_UART_Init(&huart2) != HAL_OK) {
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
static void MX_GPIO_Init(void) {
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */
  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, TRIG_PIN, GPIO_PIN_RESET);

  /*Configure GPIO pin: PB8 */
  GPIO_InitStruct.Pin = ECHO_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin: PB9 */
  GPIO_InitStruct.Pin = TRIG_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void) {
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
void assert_failed(uint8_t *file, uint32_t line) {
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}

#endif /* USE_FULL_ASSERT */

/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
UART_HandleTypeDef huart2;

volatile uint32_t step_count = 0;
volatile uint32_t total_steps = 0;
volatile bool isMoving = false;
volatile bool isSoftStopRequested = false;

volatile bool command_ready = false;
uint32_t step_period = 1000; // default period

// State Machine Decleration
MotionPhase motion_phase = MOTION_IDLE;

// Acceleration and Deceleration Setup
uint32_t accel_steps = 0;
uint32_t decel_steps = 0;
uint32_t const_steps = 0;
uint32_t target_rpm = 0;
uint32_t max_step_period = 10000;  // slowest
uint32_t min_step_period = 1000;   // fastest

// Motion Time Timestamps
volatile uint32_t step_start_time = 0;
volatile uint32_t step_end_time = 0;

char rx_byte;
char rx_buffer[64];
volatile uint8_t rx_index = 0;

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
void Parse_Command(char *cmd);
void Start_Stepper_With_Profile(uint32_t steps, uint32_t rpm, uint32_t direction);
uint32_t RPM_To_Period(uint32_t rpm);
uint32_t Calculate_Step_Period(uint32_t step_num);

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();

  // Start TIM2 in PWM mode
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);

  // Start TIM3 in Interrupt Mode
  HAL_TIM_Base_Start_IT(&htim3);

  char msg[] = "System ready\r\n";
  HAL_UART_Transmit(&huart2, (uint8_t *)msg, strlen(msg), HAL_MAX_DELAY);

  HAL_UART_Receive_IT(&huart2, (uint8_t *)&rx_byte, 1);

  while (1)
  {
    if (command_ready)
    {
      command_ready = false;
      Parse_Command(rx_buffer);
    }
  }
}

void Parse_Command(char *cmd)
{
  char dbg[64];
  uint32_t steps, rpm, dir;

  snprintf(dbg, sizeof(dbg), "Received: %s\r\n", cmd);
  HAL_UART_Transmit(&huart2, (uint8_t *)dbg, strlen(dbg), HAL_MAX_DELAY);

  // Check for Stop Soft (SS) or Stop Hard (SH)
  if (strcmp(cmd, "SS") == 0)
  {
    if (isMoving)
    {
      isSoftStopRequested = true;
      char msg[] = "SS Acknowledged\r\n";
      HAL_UART_Transmit(&huart2, (uint8_t *)msg, strlen(msg), HAL_MAX_DELAY);
    }
    else
    {
      char msg[] = "SS Ignored: Not moving\r\n";
      HAL_UART_Transmit(&huart2, (uint8_t *)msg, strlen(msg), HAL_MAX_DELAY);
    }
    return;
  }

  if (strcmp(cmd, "SH") == 0)
  {
    if (isMoving)
    {
      isMoving = false;
      HAL_TIM_Base_Stop_IT(&htim3);   // Stop profile timer
      HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1); // Stop motor pulses
      char msg[] = "SH Acknowledged\r\n";
      HAL_UART_Transmit(&huart2, (uint8_t *)msg, strlen(msg), HAL_MAX_DELAY);
    }
    else
    {
      char msg[] = "SH Ignored: Not moving\r\n";
      HAL_UART_Transmit(&huart2, (uint8_t *)msg, strlen(msg), HAL_MAX_DELAY);
    }
    return;
  }

  // Normal move command
  if (sscanf(cmd, "M:%lu,%lu,%lu", &steps, &rpm, &dir) == 3)
  {
    snprintf(dbg, sizeof(dbg), "Parsed steps: %lu, rpm: %lu, dir: %lu\r\n", steps, rpm, dir);
    HAL_UART_Transmit(&huart2, (uint8_t *)dbg, strlen(dbg), HAL_MAX_DELAY);

    if (!isMoving)
    {
      Start_Stepper_With_Profile(steps, rpm, dir);
    }
    else
    {
      char msg[] = "MB\r\n";
      HAL_UART_Transmit(&huart2, (uint8_t *)msg, strlen(msg), HAL_MAX_DELAY);
    }
  }
  else
  {
    char msg[] = "CE\r\n";
    HAL_UART_Transmit(&huart2, (uint8_t *)msg, strlen(msg), HAL_MAX_DELAY);
  }
}
void Start_Stepper_With_Profile(uint32_t steps, uint32_t rpm, uint32_t direction)
{
  target_rpm = rpm;
  min_step_period = RPM_To_Period(rpm);
  max_step_period = min_step_period * 4; // start slow, arbitrary factor

  accel_steps = steps / 32;
  decel_steps = steps / 32;
  const_steps = steps - (accel_steps + decel_steps);

  total_steps = steps * 2;
  step_count = 0;
  motion_phase = MOTION_ACCELERATING;

  // Set direction
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, direction ? GPIO_PIN_SET : GPIO_PIN_RESET);

  step_start_time = HAL_GetTick();

  // Start PWM
  HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
  __HAL_TIM_SET_AUTORELOAD(&htim2, max_step_period - 1);
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, (max_step_period - 1) / 2);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);

  // Sync TIM3 to same period
  __HAL_TIM_SET_AUTORELOAD(&htim3, max_step_period - 1);
  __HAL_TIM_SET_COUNTER(&htim3, 0);
  HAL_TIM_Base_Start_IT(&htim3);

  isMoving = true;
}
uint32_t RPM_To_Period(uint32_t rpm)
{
  // Use 1600 steps per revolution for 1/8 microstepping
  uint32_t steps_per_sec = (rpm * 1600) / 60;
  uint32_t timer_clk = 72000000 / 72;
  return timer_clk / (steps_per_sec * 2); // *2 for HIGH and LOW
}

uint32_t Calculate_Step_Period(uint32_t step_num)
{
  if (step_num < accel_steps) {
    float progress = (float)step_num / accel_steps;
    return max_step_period - (progress * (max_step_period - min_step_period));
  } else if (step_num < (accel_steps + const_steps)) {
    return min_step_period;
  } else {
    float progress = (float)(step_num - accel_steps - const_steps) / decel_steps;
    return min_step_period + (progress * (max_step_period - min_step_period));
  }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM3 && isMoving)
  {
    step_count++;

    // Check for soft stop request
    if (isSoftStopRequested && motion_phase != MOTION_DECELERATING)
    {
      motion_phase = MOTION_DECELERATING;

      // Immediately switch to deceleration from current step
      decel_steps = total_steps - step_count;

      // Clear accel and const phase counters to skip them
      accel_steps = 0;
      const_steps = 0;
    }

    if (step_count >= total_steps)
    {
      // Stop everything
      HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
      HAL_TIM_Base_Stop_IT(&htim3);
      isMoving = false;
      isSoftStopRequested = false;
      motion_phase = MOTION_IDLE;

      step_end_time = HAL_GetTick();
      char done[64];
      snprintf(done, sizeof(done), "Motion completed in %lu ms\r\n", step_end_time - step_start_time);
      HAL_UART_Transmit(&huart2, (uint8_t *)done, strlen(done), HAL_MAX_DELAY);
      return;
    }

    // Compute the period for next step
    uint32_t new_period = Calculate_Step_Period(step_count / 2);  // divide by 2 because each full step has 2 interrupts

    // Update PWM and TIM3 period
    __HAL_TIM_SET_AUTORELOAD(&htim2, new_period - 1);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, (new_period - 1) / 2);
    __HAL_TIM_SET_AUTORELOAD(&htim3, new_period - 1);

    // Optional status LED
    HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
  }
}

void TIM3_IRQHandler(void)
{
  HAL_TIM_IRQHandler(&htim3);  // ✅ Correct: call handler for TIM3
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART2)
  {
    if (rx_byte == '\n' || rx_byte == '\r')
    {
      rx_buffer[rx_index] = '\0';
      command_ready = true;  // Flag it for main loop
      rx_index = 0;
    }
    else
    {
      if (rx_index < sizeof(rx_buffer) - 1)
        rx_buffer[rx_index++] = rx_byte;
    }

    HAL_UART_Receive_IT(&huart2, (uint8_t *)&rx_byte, 1); // Always restart reception
  }
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 72;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 72-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1000-1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 500;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }

  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 72 - 1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 1000 - 1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */
  HAL_NVIC_SetPriority(TIM3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(TIM3_IRQn);
  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /* Configure GPIO pin for DIR (PA6) */
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
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

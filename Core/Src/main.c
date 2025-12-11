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
#include <stdint.h>

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */
void PCA9685_set_bit(uint8_t reg, uint8_t bit, uint8_t val);
void PCA9685_set_pwm_freq(uint16_t freq);
void PCA9685_init(uint16_t freq);
void PCA9685_set_pwm(uint8_t servo, uint16_t on_time, uint16_t off_time);
void PCA9685_set_servo_angle(uint8_t servo, float angle);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// PCA9865 Core Constants
#define PCA9685_OSC_CLOCK 25000000 // 25MHz internal clock
#define PCA9685_PWM_RESOLUTION 4096 // 12 bit PWM internal resolution
#define PCA9685_MIN_PWM 102.4 // 0.5ms
#define PCA9685_MAX_PWM 512 // 2.5ms

// PCA9685 Addresses
#define PCA9685_ADDRESS   (0x40 << 1)  // 0x40 (100 0000) in datasheet, but lshifted because HAL uses 8 bits internally for r/w bit
#define PCA9685_MODE1     0x00 // Using MODE1, sounds like it works the way I want it to
#define PCA9685_PRESCALE  0xFE // Register address for the prescaler that sets PWM frequency
#define PCA9685_PWM0_ON_L 0x06 // Location of first 8 bits of PWM0 register (out of 12 - leaving those blank)

// MODE1 (8 bits) Register Control Bits
#define PCA9685_MODE1_SLEEP_BIT   4 // Bit 4 of MODE1
#define PCA9685_MODE1_AI_BIT      5 // Auto Increment Bit
#define PCA9685_MODE1_RESTART_BIT 7 // Restart Bit

/* @brief for I2C Mem Read and Write
 * hi2c:       Pointer to a I2C_HandleTypeDef structure that contains the configuration information for the specified I2C.
 * DevAddress: Target device address - The device 7 bits address value in datasheet must be shifted to the left before calling the interface
 * MemAddress: Internal memory address
 * MemAddSize: Size of internal memory address
 * pData:      Pointer to data buffer
 * Size:       Amount of data to be sent
 * Timeout:    Timeout duration in ms */
void PCA9685_set_bit(uint8_t reg, uint8_t bit, uint8_t val){
	uint8_t register_value;
	HAL_I2C_Mem_Read(&hi2c1, PCA9685_ADDRESS, reg, 1, &register_value, 1, 10);  // Read current value of specified reg into register_value
	register_value = ((register_value & ~(1 << bit)) | (val << bit));             // Clear specified bit, then or with val (=val)
	HAL_I2C_Mem_Write(&hi2c1, PCA9685_ADDRESS, reg, 1, &register_value, 1, 10); // Write modified register to PCA9685
	HAL_Delay(1); // Time to process the write
}

/* @brief for setting PWM frequency
 * This function calculates and writes a prescale value to the PCA9685 given
 * A desired frequency value
 * Internal oscillator clock speed
 * Internal pwm resolution */
void PCA9685_set_pwm_freq(uint16_t freq){ // 12-bit PWM, so freq represented in 16 bits
	uint8_t prescale;
	if(freq >= 1526) prescale = 0x03;
	else if (freq <= 24) prescale = 0xFF;
	prescale = (PCA9685_OSC_CLOCK/(PCA9685_PWM_RESOLUTION * freq));
	PCA9685_set_bit(PCA9685_MODE1, PCA9685_MODE1_SLEEP_BIT, 1);
	HAL_I2C_Mem_Write(&hi2c1, PCA9685_ADDRESS, PCA9685_PRESCALE, 1, &prescale, 1, 10);
	PCA9685_set_bit(PCA9685_MODE1, PCA9685_MODE1_SLEEP_BIT, 0);
	PCA9685_set_bit(PCA9685_MODE1, PCA9685_MODE1_RESTART_BIT, 1);
}

/* @brief for initializing PCA9685
 * This function sets the PWM frequency for the PCA9685 and sets the Auto-Increment Bit in MODE1
 * AI = 1 allows for writing to multiple bytes stored contiguously in PCA9685's internal memory, see set_pwm function */
void PCA9685_init(uint16_t freq){
	PCA9685_set_pwm_freq(freq);
	PCA9685_set_bit(PCA9685_MODE1, PCA9685_MODE1_AI_BIT, 1);
}

/* @brief for selecting servo and setting PWM
 * Each LED (servo) has 4 bytes for PWM
 * ON_L, ON_H, OFF_L, OFF_H
 * Masking and shifting on_time and off_time into respective locations in array
 * on_time will always be 0, so period always begins with low pulse
 * MODE1 bit 5 (AI) allows for contiguous memory writing
 * */
void PCA9685_set_pwm(uint8_t servo, uint16_t on_time, uint16_t off_time){
	uint8_t pwm[4];
	uint8_t reg_address = PCA9685_PWM0_ON_L + (4 * servo);
	pwm[0] = on_time & 0xFF;
	pwm[1] = on_time >> 8;
	pwm[2] = off_time & 0xFF;
	pwm[3] = off_time >> 8;
	HAL_I2C_Mem_Write(&hi2c1, PCA9685_ADDRESS, reg_address, 1, pwm, 4, 10);
}

/* @brief for setting servo angle
 * Linear mapping from 0-180 degrees to to PWM.
 * Converts angle to pulse width between PCA9685_MIN_PWM and MAX_PWM
 * and writes it to the specified PCA9685 channel. */
void PCA9685_set_servo_angle(uint8_t servo, float angle){
	float val = PCA9685_MIN_PWM + (angle / 180.0f) * (PCA9685_MAX_PWM - PCA9685_MIN_PWM);
	PCA9685_set_pwm(servo, 0, (uint16_t)val);
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
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  PCA9685_init(50); // Servos require 50hz frequency

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
	  PCA9685_set_servo_angle(1, 0);
	  HAL_Delay(1000);
	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
	  PCA9685_set_servo_angle(1, 180);
	  HAL_Delay(1000);
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

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

/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define CE_PORT GPIOB // PB6 chip enable (aka slave select)
#define CE_PIN GPIO_PIN_6
#define DC_PORT GPIOA // PA0 data/control
#define DC_PIN GPIO_PIN_0
#define RESET_PORT GPIOA // PA1 reset
#define RESET_PIN GPIO_PIN_1
#define GLCD_WIDTH 84
#define GLCD_HEIGHT 48
#define NUM_BANKS 6
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;

SPI_HandleTypeDef hspi1;

/* USER CODE BEGIN PV */
const char font_table[][6] = {
{0x00, 0x00, 0x00, 0x00, 0x00, 0x00}, // space
{0x7E, 0x11, 0x11, 0x11, 0x7E, 0x00}, // 'A' 1
{0x7F, 0x49, 0x49, 0x49, 0x36, 0x00}, // 'B' 2
{0x3E, 0x41, 0x41, 0x41, 0x22, 0x00}, // 'C' 3
{0xFF, 0x81, 0x81, 0x7E, 0x00, 0x00}, // 'D' 4
{0x00, 0xFF, 0x89, 0x89, 0x89, 0x00}, // 'E' 5
{0x00, 0xFF, 0x09, 0x09, 0x09, 0x00}, // 'F' 6
{0x00, 0xFF, 0x81, 0x91, 0xF1, 0x00}, // 'G' 7
{0x00, 0xFF, 0x08, 0x08, 0xFF, 0x00}, // 'H' 8
{0x81, 0x81, 0xFF, 0x81, 0x81, 0x00}, // 'I' 9
{0x00, 0x40, 0x80, 0x81, 0x7E, 0x00}, // 'J' 10
{0x00, 0xFF, 0x2C, 0x42, 0x81, 0x00}, // 'K' 11
{0x00, 0xFF, 0x80, 0x80, 0x80, 0x00}, // 'L' 12
{0xFF, 0x03, 0x0C, 0x03, 0xFF, 0x00}, // 'M' 13
{0xFF, 0x06, 0x18, 0x60, 0xFF, 0x00}, // 'N' 14
{0x7E, 0x81, 0x81, 0x81, 0x7E, 0x00}, // 'O' 15
{0x00, 0xFF, 0x11, 0x11, 0x0E, 0x00}, // 'P' 16
{0x3E, 0x41, 0x41, 0x41, 0xBE, 0x00}, // 'Q' 17
{0x00, 0xFF, 0x31, 0x51, 0x8E, 0x00}, // 'R' 18
{0x86, 0x89, 0x89, 0x89, 0x71, 0x00}, // 'S' 19
{0x01, 0x01, 0xFF, 0x01, 0x01, 0x00}, // 'T' 20
{0x7F, 0x80, 0x80, 0x80, 0x7F, 0x00}, // 'U' 21
{0x3F, 0x40, 0x80, 0x40, 0x3F, 0x00}, // 'V' 22
{0x7F, 0x80, 0x60, 0x80, 0x7F, 0x00}, // 'W' 23
{0xC3, 0x24, 0x18, 0x24, 0xC3, 0x00}, // 'X' 24
{0x07, 0x08, 0xF0, 0x08, 0x07, 0x00}, // 'Y' 25
{0xE1, 0x91, 0x89, 0x85, 0x83, 0x00}, // 'Z' 26
{0x00, 0x82, 0xFF, 0x80, 0x00, 0x00}, // '1' 27
{0x00, 0xE2, 0x91, 0x8E, 0x00, 0x00}, // '2' 28
{0x00, 0x81, 0x99, 0x66, 0x00, 0x00}, // '3' 29
{0x00, 0x0F, 0x08, 0xFF, 0x00, 0x00}, // '4' 30
{0x00, 0x8F, 0x89, 0xF9, 0x00, 0x00}, // '5' 31
{0x00, 0xFF, 0x89, 0xF9, 0x00, 0x00}, // '6' 32
{0x00, 0x01, 0x01, 0xFF, 0x00, 0x00}, // '7' 33
{0x00, 0xFF, 0x89, 0xFF, 0x00, 0x00}, // '8' 34
{0x00, 0x0F, 0x09, 0xFF, 0x00, 0x00}, // '9' 35
{0x00, 0xFF, 0xB1, 0x8D, 0xFF, 0x00}, // '0' 36
{0x10, 0x10, 0x00, 0x5C, 0x54, 0x74}, // small -5 37
{0x00, 0x00, 0x00, 0x5C, 0x54, 0x74}, // small 5 38
{0x00, 0x10, 0xF8, 0x00, 0xB0, 0xD0}, // 1 ss 39
{0x00, 0x00, 0x00, 0x75, 0x55, 0x5D}, // /2 40
{0x70, 0x28, 0x70, 0x00, 0x78, 0x48}, // AC 41
{0x78, 0x48, 0x30, 0x00, 0x78, 0x48}, // DC 42
{0x00, 0x00, 0x00, 0x7C, 0x44, 0x7C}, // 0 43

};

uint8_t mode = 0; // 0 = dc, 1 = ac
uint16_t analog_measurement;
unsigned char display_height;
unsigned char bank_value;
unsigned char x_value;
unsigned char time_value = 0x7;
unsigned char bank_offset;
unsigned char x_offset;
int bank_array[76] = {0};
int x_array[76] = {0};
int i = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC2_Init(void);
/* USER CODE BEGIN PFP */
void SPI_write(unsigned char data);
void GLCD_data_write(unsigned char data);
void GLCD_command_write(unsigned char data);
void GLCD_init(void);
void GLCD_setCursor(unsigned char x, unsigned char y);
void GLCD_clear(void);
void GLCD_putchar(int font_table_row);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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

/* Configure the peripherals common clocks */
  PeriphCommonClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SPI1_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  /* USER CODE BEGIN 2 */
  GLCD_init(); // initialize the screen
  GLCD_clear(); // clear the screen

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  	  // random number is for less visible jitter
	  	  HAL_Delay(4.6148);
	  	  GLCD_clear();

	  // *** UI elements ***

	  	  // display 5 behind vertical line
	  	  GLCD_setCursor(0x0, 0x2);
	  	  GLCD_putchar(39);

	  	  // if mode set to AC, display AC UI elements
	  	  if (mode == 1)
	  	  {
	  		  bank_offset = 0x3;
	  		  x_offset = 0;
			  GLCD_setCursor(0x0, 0x0);
			  GLCD_putchar(38);
			  GLCD_setCursor(0x0, 0x4);
			  GLCD_putchar(37);
			  GLCD_setCursor(0x0, 0x1);
			  GLCD_putchar(40);
			  GLCD_setCursor(0x0, 0x5);
			  GLCD_putchar(40);
			  GLCD_setCursor(0x0, 0x3);
			  GLCD_putchar(41);

	  	  }
	  	  // if mode set to DC, display DC UI elements
	  	  else
	  	  {
	  		  bank_offset = 0x5;
	  		  x_offset = 7;
			  GLCD_setCursor(0x0, 0x0);
			  GLCD_putchar(38);
			  GLCD_setCursor(0x0, 0x5);
			  GLCD_putchar(43);
			  GLCD_setCursor(0x0, 0x3);
			  GLCD_putchar(42);
	  	  }

	  	  // Horizontal line
  		  i = 0x7;
		  while (i < 0x54)
		  {
			  GLCD_setCursor(i, bank_offset);
			  GLCD_data_write(0x1 << x_offset);
			  i++;
		  }

		  // vertical line
	  	  i = 0x0;
	  	  while (i < 0x6)
	  	  {
	  	  	  GLCD_setCursor(0x7, i);
	  	  	  GLCD_data_write(0xff);
	  	  	  i++;
	  	  }

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  // *** Interactable elements ***

	  	  // time is x axis, there is 54 pixels available and the first 7 pixels are reserved for the UI elements.
	  	  if (time_value == 0x54)
	  	  {
	  		  time_value = 0x7;
	  	  }
	  	  else
	  	  {
	  		  time_value += 0x1;
	  	  }

	  	  // get voltage input, if mode is AC use differential ADC, if DC use single ended ADC
	  	  if (mode == 1)
	  	  {
			  HAL_ADC_Start(&hadc1);
			  HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
			  analog_measurement = HAL_ADC_GetValue(&hadc1);
	  	  }
	  	  else
	  	  {
			  HAL_ADC_Start(&hadc2);
			  HAL_ADC_PollForConversion(&hadc2, HAL_MAX_DELAY);
			  analog_measurement = HAL_ADC_GetValue(&hadc2);
	  	  }


	  	  // Calculate bank position and x position in said bank
	  	  display_height = analog_measurement / 85.33;
	  	  x_value = 0;
	  	  bank_value = 0;
	  	  i = 0;
	  	  while (i <= display_height)
	  	  {
	  		  x_value++;
	  		  if (x_value == 8)
	  		  {
	  			  bank_value++;
	  			  x_value = 0;
	  		  }
	  		  i++;
	  	  }

	  	  // put voltage value into an array that is refreshed every frame
	  	  bank_array[time_value - 0x7] = bank_value;
	  	  x_array[time_value - 0x7] = x_value;

	  	  // display array of measurements
	  	  i=0;
	  	  while (i != 76)
	  	  {
	  		  bank_value = bank_array[i];
	  		  x_value = x_array[i];
	  		  i++;
			  GLCD_setCursor(i+7, bank_value);

			  // To insert the offset position of the UI line onto the display if the voltage value is near it
			  if (mode == 1)
			  {
				  bank_offset = 3;
				  x_offset = 0;
			  }
			  else
			  {
				  bank_offset = 5;
				  x_offset = 7;
			  }

			  // Code to correct the UI horizontal line if value would otherwise override it, if not near simply display value
			  if (bank_value == bank_offset)
			  {
				  if (x_value == x_offset)
				  {
					  GLCD_data_write(0x1 << x_offset);
				  }
				  else
				  {
					  GLCD_data_write((0x1 << x_value) + (0x1 << x_offset));
				  }
			  }
			  else
			  {
				  GLCD_data_write(0x1 << x_value);
			  }
	  	  }


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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief Peripherals Common Clock Configuration
  * @retval None
  */
void PeriphCommonClock_Config(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the peripherals clock
  */
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCCLKSOURCE_PLLSAI1;
  PeriphClkInit.PLLSAI1.PLLSAI1Source = RCC_PLLSOURCE_HSI;
  PeriphClkInit.PLLSAI1.PLLSAI1M = 1;
  PeriphClkInit.PLLSAI1.PLLSAI1N = 8;
  PeriphClkInit.PLLSAI1.PLLSAI1P = RCC_PLLP_DIV7;
  PeriphClkInit.PLLSAI1.PLLSAI1Q = RCC_PLLQ_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1R = RCC_PLLR_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1ClockOut = RCC_PLLSAI1_ADC1CLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
  sConfig.SingleDiff = ADC_DIFFERENTIAL_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */

  /** Common config
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc2.Init.LowPowerAutoWait = DISABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc2.Init.DMAContinuousRequests = DISABLE;
  hadc2.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc2.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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
  HAL_GPIO_WritePin(GPIOA, D_C_Pin|RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : D_C_Pin RST_Pin */
  GPIO_InitStruct.Pin = D_C_Pin|RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : CS_Pin */
  GPIO_InitStruct.Pin = CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(CS_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	// toggle AC or DC mode when the blue PC13 button is pressed
	if (mode != 0)
	{
		mode = 0;
	}
	else
	{
		mode = 1;
	}
}

void SPI_write(unsigned char data)
{
	// Chip enable, low is assert
	HAL_GPIO_WritePin(CE_PORT, CE_PIN, GPIO_PIN_RESET);

	// Send data over SPI1
	HAL_SPI_Transmit(&hspi1, (uint8_t*) &data, 1, HAL_MAX_DELAY);

	// Chip disable
	HAL_GPIO_WritePin(CE_PORT, CE_PIN, GPIO_PIN_SET);
}

void GLCD_data_write(unsigned char data)
{
	// Switch mode to data mode
	HAL_GPIO_WritePin(DC_PORT, DC_PIN, GPIO_PIN_SET);

	// Send data over SPI
	SPI_write(data);
}

void GLCD_command_write(unsigned char data)
{
	// Switch to command mode
	HAL_GPIO_WritePin(DC_PORT, DC_PIN, GPIO_PIN_RESET);

	// Send data over SPI
	SPI_write(data);
}

void GLCD_init(void)
{
	// Keep CE high when not transmitting
	HAL_GPIO_WritePin(CE_PORT, CE_PIN, GPIO_PIN_SET);

	// Reset screen
	HAL_GPIO_WritePin(RESET_PORT, RESET_PIN, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(RESET_PORT, RESET_PIN, GPIO_PIN_SET);

	// Configure screen
	GLCD_command_write(0x21); // enter extended command mode
	GLCD_command_write(0xB6); // set LCD Vop for contrast
	GLCD_command_write(0x04); // set temp coefficient
	GLCD_command_write(0x15); // set LCD bias mode
	GLCD_command_write(0x20); // return to normal command mode
	GLCD_command_write(0x0C); // set display mode to normal
}

void GLCD_setCursor(unsigned char x, unsigned char y)
{
	GLCD_command_write(0x80 | x); // column
	GLCD_command_write(0x40 | y); // bank
}

void GLCD_clear(void)
{
	int i;
	for (i = 0; i < (GLCD_WIDTH * NUM_BANKS); i++)
	{
		GLCD_data_write(0x00); // write zeros to all banks
	}
	GLCD_setCursor(0, 0); // return cursor to top left
}

void GLCD_putchar(int font_table_row)
{
	int i;
	for (i=0; i<6; i++)
	{
		GLCD_data_write(font_table[font_table_row][i]);
	}
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

/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "fatfs.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include <memory.h>
#include <string.h>
#include <stdarg.h> //for va_list var arg functions

#include "ws2812b.h"
//#include "walking.h"
#include "sd_card.h"

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
SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim5;
DMA_HandleTypeDef hdma_tim1_ch1;
DMA_HandleTypeDef hdma_tim5_ch1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM5_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

uint32_t hsl_to_rgb(uint8_t h, uint8_t s, uint8_t l);

Layers Layer1;
Layers Layer2;
Layers Layer3;
Layers Layer4;



/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
	/* USER CODE BEGIN 1 */
	Layer1.timer = &htim1;
	Layer1.dma = &hdma_tim1_ch1;
	Layer1.channel = TIM_CHANNEL_1;

	Layer2.timer = &htim5;
	Layer2.dma = &hdma_tim5_ch1;
	Layer2.channel = TIM_CHANNEL_4;

//	Layer3.timer = &htim3;
//	Layer3.dma = &hdma_tim3_ch2;
//	Layer3.channel = TIM_CHANNEL_2;
//
//	Layer4.timer = &htim4;
//	Layer4.dma = &hdma_tim4_ch1;
//	Layer4.channel = TIM_CHANNEL_1;
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
	MX_DMA_Init();
	MX_TIM1_Init();
	MX_TIM5_Init();
	MX_FATFS_Init();
	MX_SPI1_Init();
	MX_USART2_UART_Init();
	/* USER CODE BEGIN 2 */
	ret_status result = STATUS_NULL;
	sd_card_init();
	char buff[256];
	strcpy(buff, "0:/");
//	sd_card_scan_file(buff);

	uint8_t walk_array[3][256][3] = {0};
	result = sd_card_read_data((uint8_t *)walk_array);

	sd_card_close();

	if(STATUS_OK != result)
		while(1);

	for (size_t i = 0; i < 256; i++)
	{
		myprintf("{%d, %d, %d},", walk_array[0][i][0], walk_array[0][i][1], walk_array[0][i][2]);
	}


	uint8_t odd = 1;
	uint8_t index = 0;
	uint8_t counter = 0;
	uint8_t number_of_animation = 0;

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */

		for (uint16_t i = 0; i < 256; i++) {
			if (i % 16 == 0) {
				odd ^= 1;
				counter++;
			}
			if (odd) {
				index = (counter * 16 - 1) - (i % 16);
			} else {
				index = i;
			}

			led_set_RGB(&Layer1, i, walk_array[number_of_animation][index][0],
					walk_array[number_of_animation][index][1],
					walk_array[number_of_animation][index][2]);
			led_set_RGB(&Layer2, i, walk_array[number_of_animation][index][0],
					walk_array[number_of_animation][index][1],
					walk_array[number_of_animation][index][2]);
//			led_set_RGB(&Layer3, i, walk_array[number_of_animation][index][0],
//					walk_array[number_of_animation][index][1],
//					walk_array[number_of_animation][index][2]);
//			led_set_RGB(&Layer4, i, walk_array[number_of_animation][index][0],
//					walk_array[number_of_animation][index][1],
//					walk_array[number_of_animation][index][2]);

		}
		if (++number_of_animation == sizeof(walk_array) / sizeof(walk_array[0]))
			number_of_animation = 0;

		led_render(&Layer1);
		HAL_Delay(10);
		led_render(&Layer2);
		HAL_Delay(10);
//		led_render(&Layer3);
//		HAL_Delay(10);
//		led_render(&Layer4);
		HAL_Delay(500);
	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

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
	RCC_OscInitStruct.PLL.PLLM = 8;
	RCC_OscInitStruct.PLL.PLLN = 84;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = 3;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief SPI1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_SPI1_Init(void) {

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
	hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
	hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
	hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
	hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
	hspi1.Init.CRCPolynomial = 10;
	if (HAL_SPI_Init(&hspi1) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN SPI1_Init 2 */

	/* USER CODE END SPI1_Init 2 */

}

/**
 * @brief TIM1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM1_Init(void) {

	/* USER CODE BEGIN TIM1_Init 0 */

	/* USER CODE END TIM1_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };
	TIM_OC_InitTypeDef sConfigOC = { 0 };
	TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = { 0 };

	/* USER CODE BEGIN TIM1_Init 1 */

	/* USER CODE END TIM1_Init 1 */
	htim1.Instance = TIM1;
	htim1.Init.Prescaler = 0;
	htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim1.Init.Period = 104;
	htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim1.Init.RepetitionCounter = 0;
	htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim1) != HAL_OK) {
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_Init(&htim1) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
	sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
	if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1)
			!= HAL_OK) {
		Error_Handler();
	}
	sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
	sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
	sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
	sBreakDeadTimeConfig.DeadTime = 0;
	sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
	sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
	sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
	if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM1_Init 2 */

	/* USER CODE END TIM1_Init 2 */
	HAL_TIM_MspPostInit(&htim1);

}

/**
 * @brief TIM5 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM5_Init(void) {

	/* USER CODE BEGIN TIM5_Init 0 */

	/* USER CODE END TIM5_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };
	TIM_OC_InitTypeDef sConfigOC = { 0 };

	/* USER CODE BEGIN TIM5_Init 1 */

	/* USER CODE END TIM5_Init 1 */
	htim5.Instance = TIM5;
	htim5.Init.Prescaler = 0;
	htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim5.Init.Period = 104;
	htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim5) != HAL_OK) {
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_Init(&htim5) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if (HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_1)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM5_Init 2 */

	/* USER CODE END TIM5_Init 2 */
	HAL_TIM_MspPostInit(&htim5);

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
 * Enable DMA controller clock
 */
static void MX_DMA_Init(void) {

	/* DMA controller clock enable */
	__HAL_RCC_DMA2_CLK_ENABLE();
	__HAL_RCC_DMA1_CLK_ENABLE();

	/* DMA interrupt init */
	/* DMA1_Stream2_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA1_Stream2_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA1_Stream2_IRQn);
	/* DMA2_Stream1_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA2_Stream1_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA2_Stream1_IRQn);

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOA_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(SD_CS_GPIO_Port, SD_CS_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin : SD_CS_Pin */
	GPIO_InitStruct.Pin = SD_CS_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(SD_CS_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

uint32_t hsl_to_rgb(uint8_t h, uint8_t s, uint8_t l) {
	if (l == 0)
		return 0;

	volatile uint8_t r, g, b, lo, c, x, m;
	volatile uint16_t h1, l1, H;
	l1 = l + 1;
	if (l < 128)
		c = ((l1 << 1) * s) >> 8;
	else
		c = (512 - (l1 << 1)) * s >> 8;

	H = h * 6;              // 0 to 1535 (actually 1530)
	lo = H & 255;           // Low byte  = primary/secondary color mix
	h1 = lo + 1;

	if ((H & 256) == 0)
		x = h1 * c >> 8;          // even sextant, like red to yellow
	else
		x = (256 - h1) * c >> 8;  // odd sextant, like yellow to green

	m = l - (c >> 1);
	switch (H >> 8) {       // High byte = sextant of colorwheel
	case 0:
		r = c;
		g = x;
		b = 0;
		break; // R to Y
	case 1:
		r = x;
		g = c;
		b = 0;
		break; // Y to G
	case 2:
		r = 0;
		g = c;
		b = x;
		break; // G to C
	case 3:
		r = 0;
		g = x;
		b = c;
		break; // C to B
	case 4:
		r = x;
		g = 0;
		b = c;
		break; // B to M
	default:
		r = c;
		g = 0;
		b = x;
		break; // M to R
	}

	return (((uint32_t) r + m) << 16) | (((uint32_t) g + m) << 8)
			| ((uint32_t) b + m);
}

void HAL_TIM_PWM_PulseFinishedHalfCpltCallback(TIM_HandleTypeDef *htim) {

	Layers *layer;

	if (htim == &htim1) {
		layer = &Layer1;
	} else if (htim == &htim5) {
		layer = &Layer2;
//	} else if (htim == &htim3) {
//		layer = &Layer3;
//	} else if (htim == &htim4) {
//		layer = &Layer4;
	} else
		return;

	// DMA buffer set from LED(wr_buf_p) to LED(wr_buf_p + 1)
	if (layer->wr_buf_p < NUM_PIXELS) {
		// We're in. Fill the even buffer
		for (uint_fast8_t i = 0; i < 8; ++i) {
			layer->wr_buf[i] =
			PWM_LO << (((layer->rgb_arr[3 * layer->wr_buf_p] << i) & 0x80) > 0);
			layer->wr_buf[i + 8] = PWM_LO
					<< (((layer->rgb_arr[3 * layer->wr_buf_p + 1] << i) & 0x80)
							> 0);
			layer->wr_buf[i + 16] = PWM_LO
					<< (((layer->rgb_arr[3 * layer->wr_buf_p + 2] << i) & 0x80)
							> 0);
		}
		layer->wr_buf_p++;
	} else if (layer->wr_buf_p < NUM_PIXELS + 2) {
		// Last two transfers are resets.
		//                               WS2812B: 48 * 1.25 us = 60 us == good enough reset
		// First half reset zero fill
		for (uint8_t i = 0; i < WR_BUF_LEN / 2; ++i)
			layer->wr_buf[i] = 0;
		layer->wr_buf_p++;
	}
}

void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim) {

	Layers *layer;

	if (htim == &htim1) {
		layer = &Layer1;
	} else if (htim == &htim5) {
		layer = &Layer2;
//	} else if (htim == &htim3) {
//		layer = &Layer3;
//	} else if (htim == &htim4) {
//		layer = &Layer4;
	} else
		return;

	// DMA buffer set from LED(wr_buf_p) to LED(wr_buf_p + 1)
	if (layer->wr_buf_p < NUM_PIXELS) {
		// We're in. Fill the odd buffer
		for (uint_fast8_t i = 0; i < 8; ++i) {
			layer->wr_buf[i + 24] =
			PWM_LO << (((layer->rgb_arr[3 * layer->wr_buf_p] << i) & 0x80) > 0);
			layer->wr_buf[i + 32] = PWM_LO
					<< (((layer->rgb_arr[3 * layer->wr_buf_p + 1] << i) & 0x80)
							> 0);
			layer->wr_buf[i + 40] = PWM_LO
					<< (((layer->rgb_arr[3 * layer->wr_buf_p + 2] << i) & 0x80)
							> 0);
		}

		layer->wr_buf_p++;

	} else if (layer->wr_buf_p < NUM_PIXELS + 2) {
		// Second half reset zero fill
		for (uint8_t i = WR_BUF_LEN / 2; i < WR_BUF_LEN; ++i)
			layer->wr_buf[i] = 0;
		++layer->wr_buf_p;
	} else {
		// We're done. Lean back and until next time!
		layer->wr_buf_p = 0;
		HAL_TIM_PWM_Stop_DMA(layer->timer, layer->channel);
	}
}

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
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

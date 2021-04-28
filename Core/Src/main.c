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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "SevenSeg.h"
#include <stdlib.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

//counters
int timeCounter = 0;
int red_WaitCounter = 0;
int bounce_WaitCounter = 0;
int randomCounter = 0;

//other global integers
int red_WaitTime = 0;
int highScore = 9999;

//states
int LED_state = 0;
int nextState = 0;

//enums
enum {FALSE, TRUE};//boolean
enum {Ylw, Red1, Red2, Red3, Blu, Grn}; //Led-states

//booleans
int recordTime = FALSE; //true = system records the time (blue light on)
int redState = FALSE; //true = system is in ready-steady-go-state (red lights)
int waitOk = TRUE; //true = button is activated. De-bounce purpose
int newHS = FALSE;

//functions

int randomInt(int min, int max){
	return (rand() % (max-min)) + min;
}

void displayFourDigitNr(int nr){
	int digits[4];
	for(int i = 0; i < 4; i++){
		digits[i] = nr % 10;
		nr /= 10;
	}
	DisplayDigits(digits[3], digits[2], digits[1], digits[0], 0);
}


void gameState(int state){


	switch(state){
	case Ylw: //Warning state (Yellow light only)
		HAL_GPIO_WritePin(LED_Ylw_GPIO_Port, LED_Ylw_Pin, SET);

		HAL_GPIO_WritePin(LED_Red0_GPIO_Port, LED_Red0_Pin, RESET);
		HAL_GPIO_WritePin(LED_Red1_GPIO_Port, LED_Red1_Pin, RESET);
		HAL_GPIO_WritePin(LED_Red2_GPIO_Port, LED_Red2_Pin, RESET);
		HAL_GPIO_WritePin(LED_Blu_GPIO_Port, LED_Blu_Pin, RESET);
		HAL_GPIO_WritePin(LED_Grn_GPIO_Port, LED_Grn_Pin, RESET);

		displayFourDigitNr(highScore);
		nextState = Red1;
		break;
	case Red1: //Steady state1 (One Red light)
		HAL_GPIO_WritePin(LED_Red0_GPIO_Port, LED_Red0_Pin, SET);
		HAL_GPIO_WritePin(LED_Red1_GPIO_Port, LED_Red1_Pin, RESET);
		HAL_GPIO_WritePin(LED_Red2_GPIO_Port, LED_Red2_Pin, RESET);

		HAL_GPIO_WritePin(LED_Ylw_GPIO_Port, LED_Ylw_Pin, RESET);
		HAL_GPIO_WritePin(LED_Blu_GPIO_Port, LED_Blu_Pin, RESET);
		HAL_GPIO_WritePin(LED_Grn_GPIO_Port, LED_Grn_Pin, RESET);

		timeCounter = 0;
		red_WaitCounter = RESET;

		red_WaitTime = 1000;

		displayFourDigitNr(timeCounter);

		redState = TRUE;
		nextState = Red2;
		break;
	case Red2: //Steady state2 (Two Red Lights)
		HAL_GPIO_WritePin(LED_Red0_GPIO_Port, LED_Red0_Pin, SET);
		HAL_GPIO_WritePin(LED_Red1_GPIO_Port, LED_Red1_Pin, SET);
		HAL_GPIO_WritePin(LED_Red2_GPIO_Port, LED_Red2_Pin, RESET);

		HAL_GPIO_WritePin(LED_Ylw_GPIO_Port, LED_Ylw_Pin, RESET);
		HAL_GPIO_WritePin(LED_Blu_GPIO_Port, LED_Blu_Pin, RESET);
		HAL_GPIO_WritePin(LED_Grn_GPIO_Port, LED_Grn_Pin, RESET);

		red_WaitCounter = RESET;

		red_WaitTime = 1000;

		nextState = Red3;
		break;
	case Red3: //Steady state3 (All red lights)
		HAL_GPIO_WritePin(LED_Red0_GPIO_Port, LED_Red0_Pin, SET);
		HAL_GPIO_WritePin(LED_Red1_GPIO_Port, LED_Red1_Pin, SET);
		HAL_GPIO_WritePin(LED_Red2_GPIO_Port, LED_Red2_Pin, SET);

		HAL_GPIO_WritePin(LED_Ylw_GPIO_Port, LED_Ylw_Pin, RESET);
		HAL_GPIO_WritePin(LED_Blu_GPIO_Port, LED_Blu_Pin, RESET);
		HAL_GPIO_WritePin(LED_Grn_GPIO_Port, LED_Grn_Pin, RESET);

		red_WaitCounter = RESET;

		int delayMin = 1000;
		int delayMax = 5000;
		red_WaitTime = randomInt(delayMin, delayMax);

		nextState = Blu;
		break;
	case Blu: //Game state (Only Blue)
		HAL_GPIO_WritePin(LED_Blu_GPIO_Port, LED_Blu_Pin, SET);

		HAL_GPIO_WritePin(LED_Red0_GPIO_Port, LED_Red0_Pin, RESET);
		HAL_GPIO_WritePin(LED_Red1_GPIO_Port, LED_Red1_Pin, RESET);
		HAL_GPIO_WritePin(LED_Red2_GPIO_Port, LED_Red2_Pin, RESET);
		HAL_GPIO_WritePin(LED_Ylw_GPIO_Port, LED_Ylw_Pin, RESET);
		HAL_GPIO_WritePin(LED_Grn_GPIO_Port, LED_Grn_Pin, RESET);

		redState = FALSE;
		recordTime = TRUE;
		nextState = Grn;
		break;

	case Grn: //Goal state (Only Green
		HAL_GPIO_WritePin(LED_Grn_GPIO_Port, LED_Grn_Pin, SET);

		HAL_GPIO_WritePin(LED_Red0_GPIO_Port, LED_Red0_Pin, RESET);
		HAL_GPIO_WritePin(LED_Red1_GPIO_Port, LED_Red1_Pin, RESET);
		HAL_GPIO_WritePin(LED_Red2_GPIO_Port, LED_Red2_Pin, RESET);
		HAL_GPIO_WritePin(LED_Blu_GPIO_Port, LED_Blu_Pin, RESET);
		HAL_GPIO_WritePin(LED_Ylw_GPIO_Port, LED_Ylw_Pin, RESET);

		displayFourDigitNr(timeCounter);
		if(timeCounter < highScore){
			highScore = timeCounter;
			newHS = TRUE;
		}
		recordTime = FALSE;
		nextState = Ylw;
		break;

	default: //No LED
		HAL_GPIO_WritePin(LED_Ylw_GPIO_Port, LED_Ylw_Pin, RESET);
		HAL_GPIO_WritePin(LED_Red0_GPIO_Port, LED_Red0_Pin, RESET);
		HAL_GPIO_WritePin(LED_Red1_GPIO_Port, LED_Red1_Pin, RESET);
		HAL_GPIO_WritePin(LED_Red2_GPIO_Port, LED_Red2_Pin, RESET);
		HAL_GPIO_WritePin(LED_Blu_GPIO_Port, LED_Blu_Pin, RESET);
		HAL_GPIO_WritePin(LED_Grn_GPIO_Port, LED_Grn_Pin, RESET);
		break;

	}
}

void toNextState(){
		LED_state = nextState;
		gameState(LED_state);
}

//External interrupts

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	if(HAL_GPIO_ReadPin(BTN_EXTI0_GPIO_Port, BTN_EXTI0_Pin) && waitOk){

		waitOk = FALSE;
		bounce_WaitCounter = 0;

		if(redState){
			redState = FALSE;
			red_WaitCounter = RESET;
			gameState(Ylw);
		}
		else
			toNextState();
	}
}


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	if(htim->Instance == TIM1){

		if(recordTime){
			if(++timeCounter == 9999){
				recordTime = FALSE;
				toNextState();
			}
		}

		if(redState){
			red_WaitCounter++;
			if(red_WaitCounter == red_WaitTime){
				toNextState();
			}

		}
		if (!waitOk){
			int bounceWait = 250;
			if(++bounce_WaitCounter == bounceWait){
				waitOk = TRUE;
			}
		}
	}
}


/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart1;

PCD_HandleTypeDef hpcd_USB_FS;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USB_PCD_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */

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

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  MX_USB_PCD_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */


  HAL_TIM_Base_Start_IT(&htim1);
  displayFourDigitNr(timeCounter);
  gameState(LED_state);
  while (1)
  {

  }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Macro to configure the PLL multiplication factor
  */
  __HAL_RCC_PLL_PLLM_CONFIG(RCC_PLLM_DIV1);
  /** Macro to configure the PLL clock source
  */
  __HAL_RCC_PLL_PLLSOURCE_CONFIG(RCC_PLLSOURCE_MSI);
  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);
  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSE
                              |RCC_OSCILLATORTYPE_LSE|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.MSICalibrationValue = RCC_MSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure the SYSCLKSource, HCLK, PCLK1 and PCLK2 clocks dividers
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK4|RCC_CLOCKTYPE_HCLK2
                              |RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSE;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.AHBCLK2Divider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLK4Divider = RCC_SYSCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the peripherals clocks
  */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_SMPS|RCC_PERIPHCLK_USART1
                              |RCC_PERIPHCLK_USB;
  PeriphClkInitStruct.PLLSAI1.PLLN = 24;
  PeriphClkInitStruct.PLLSAI1.PLLP = RCC_PLLP_DIV2;
  PeriphClkInitStruct.PLLSAI1.PLLQ = RCC_PLLQ_DIV2;
  PeriphClkInitStruct.PLLSAI1.PLLR = RCC_PLLR_DIV2;
  PeriphClkInitStruct.PLLSAI1.PLLSAI1ClockOut = RCC_PLLSAI1_USBCLK;
  PeriphClkInitStruct.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInitStruct.UsbClockSelection = RCC_USBCLKSOURCE_PLLSAI1;
  PeriphClkInitStruct.SmpsClockSelection = RCC_SMPSCLKSOURCE_HSI;
  PeriphClkInitStruct.SmpsDivSelection = RCC_SMPSCLKDIV_RANGE0;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN Smps */

  /* USER CODE END Smps */
  /** Enable MSI Auto calibration
  */
  HAL_RCCEx_EnableMSIPLLMode();
}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 31;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 999;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_7B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USB Initialization Function
  * @param None
  * @retval None
  */
static void MX_USB_PCD_Init(void)
{

  /* USER CODE BEGIN USB_Init 0 */

  /* USER CODE END USB_Init 0 */

  /* USER CODE BEGIN USB_Init 1 */

  /* USER CODE END USB_Init 1 */
  hpcd_USB_FS.Instance = USB;
  hpcd_USB_FS.Init.dev_endpoints = 8;
  hpcd_USB_FS.Init.speed = PCD_SPEED_FULL;
  hpcd_USB_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
  hpcd_USB_FS.Init.Sof_enable = DISABLE;
  hpcd_USB_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_FS.Init.lpm_enable = DISABLE;
  hpcd_USB_FS.Init.battery_charging_enable = DISABLE;
  if (HAL_PCD_Init(&hpcd_USB_FS) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USB_Init 2 */

  /* USER CODE END USB_Init 2 */

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, SEG_CLK_Pin|SEG_DAT_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LED_Red0_Pin|LED_Red1_Pin|LED_Red2_Pin|LED_Blu_Pin
                          |LED_Ylw_Pin|LED_Grn_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LD2_Pin|LD3_Pin|LD1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : BTN_EXTI0_Pin */
  GPIO_InitStruct.Pin = BTN_EXTI0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(BTN_EXTI0_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SEG_CLK_Pin SEG_DAT_Pin */
  GPIO_InitStruct.Pin = SEG_CLK_Pin|SEG_DAT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_Red0_Pin LED_Red1_Pin LED_Red2_Pin LED_Blu_Pin
                           LED_Ylw_Pin LED_Grn_Pin */
  GPIO_InitStruct.Pin = LED_Red0_Pin|LED_Red1_Pin|LED_Red2_Pin|LED_Blu_Pin
                          |LED_Ylw_Pin|LED_Grn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD2_Pin LD3_Pin LD1_Pin */
  GPIO_InitStruct.Pin = LD2_Pin|LD3_Pin|LD1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : B2_Pin B3_Pin */
  GPIO_InitStruct.Pin = B2_Pin|B3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

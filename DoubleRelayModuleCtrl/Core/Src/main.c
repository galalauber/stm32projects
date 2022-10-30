/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
/* Define Relay Pin and Port connection here */
#define RELAY_1			1
#define RELAY_2			2
#define RELAY_1_PIN 	GPIO_PIN_5
#define RELAY_1_PORT	GPIOB
#define RELAY_2_PIN		GPIO_PIN_4
#define RELAY_2_PORT	GPIOB

#define RELAY_ON		GPIO_PIN_SET
#define RELAY_OFF		GPIO_PIN_RESET

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
//static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
int RelayModuleCtrl(int relay, int output);
void RelayModuleConfig(void);
void Inicializa_GPIO_C13_btnB1_HAL(void);

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
  //MX_GPIO_Init();
  RelayModuleConfig();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  Inicializa_GPIO_C13_btnB1_HAL();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	/* uncomment to change relay state to lv1*/
	if (!HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13)){
		RelayModuleCtrl(RELAY_1, RELAY_ON);
		RelayModuleCtrl(RELAY_2, RELAY_OFF);
	} else {
		RelayModuleCtrl(RELAY_1, RELAY_OFF);
		RelayModuleCtrl(RELAY_2, RELAY_ON);
	}
	HAL_Delay(200);
	  /* RelayModuleCtrl(RELAY_1, RELAY_ON); */
	  /* RelayModuleCtrl(RELAY_2, RELAY_ON); */
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
//
//static void MX_GPIO_Init(void)
//{
//  GPIO_InitTypeDef GPIO_InitStruct = {0};
//
//  /* GPIO Ports Clock Enable */
//  __HAL_RCC_GPIOC_CLK_ENABLE();
//  __HAL_RCC_GPIOH_CLK_ENABLE();
//  __HAL_RCC_GPIOA_CLK_ENABLE();
//  __HAL_RCC_GPIOB_CLK_ENABLE();
//
//  /*Configure GPIO pin Output Level */
//  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
//
//  /*Configure GPIO pin : B1_Pin */
//  GPIO_InitStruct.Pin = B1_Pin;
//  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
//  GPIO_InitStruct.Pull = GPIO_NOPULL;
//  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);
//
//  /*Configure GPIO pin : LD2_Pin */
//  GPIO_InitStruct.Pin = LD2_Pin;
//  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
//  GPIO_InitStruct.Pull = GPIO_NOPULL;
//  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
//  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);
//
//}

/* USER CODE BEGIN 4 */
int RelayModuleCtrl(int relay, int digitalOutput) {
	if (relay == RELAY_1) {
		if (digitalOutput == GPIO_PIN_RESET || digitalOutput == GPIO_PIN_SET) {
			HAL_GPIO_WritePin(RELAY_1_PORT, RELAY_1_PIN, digitalOutput);
			return 0;		// return OK
		}
	}
	if (relay == RELAY_2) {
		if (digitalOutput == GPIO_PIN_RESET || digitalOutput == GPIO_PIN_SET) {
			HAL_GPIO_WritePin(RELAY_2_PORT, RELAY_2_PIN, digitalOutput);
			return 0;		// return OK
		}
	}
	return -1;			// return ERROR
}

void RelayModuleConfig(void ) {
	// Using D4 (PB5) e D5 (PB4)

	/* GPIO Port Clock Enable */
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/* GPIO Output Value Pin Initial State */
	HAL_GPIO_WritePin(RELAY_1_PORT, RELAY_1_PIN, GPIO_PIN_RESET);		//D4

	/* GPIO PB5 Output Configuration*/
	GPIO_InitTypeDef Relay1_Struct = {0};
	Relay1_Struct.Pin = RELAY_1_PIN;
	Relay1_Struct.Mode = GPIO_MODE_OUTPUT_PP;
	Relay1_Struct.Pull = GPIO_NOPULL;
	Relay1_Struct.Speed = GPIO_SPEED_LOW;
	HAL_GPIO_Init(RELAY_1_PORT, &Relay1_Struct);


	/* GPIO Output Value Pin Initial State */
	HAL_GPIO_WritePin(RELAY_2_PORT, RELAY_2_PIN, GPIO_PIN_RESET);		//D5

	/* GPIO PB5 Output Configuration*/
	GPIO_InitTypeDef Relay2_Struct = {0};
	Relay2_Struct.Pin = RELAY_2_PIN;
	Relay2_Struct.Mode = GPIO_MODE_OUTPUT_PP;
	Relay2_Struct.Pull = GPIO_NOPULL;
	Relay2_Struct.Speed = GPIO_SPEED_LOW;
	HAL_GPIO_Init(RELAY_2_PORT, &Relay2_Struct);

}

void Inicializa_GPIO_C13_btnB1_HAL(void)
{
	GPIO_InitTypeDef GPIO_C13_Init_Struct = {0};

	__HAL_RCC_GPIOC_CLK_ENABLE();

	GPIO_C13_Init_Struct.Pin = GPIO_PIN_13;
	GPIO_C13_Init_Struct.Mode = GPIO_MODE_INPUT;
	GPIO_C13_Init_Struct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOC, &GPIO_C13_Init_Struct);
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

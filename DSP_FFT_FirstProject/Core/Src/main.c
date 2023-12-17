/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file : main.c
 * @brief : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 * opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "fonts.h"
#include "tft.h"
#include "user_setting.h"
#include "functions.h"
#include "arm_math.h"
#include "arm_const_structs.h"
#include "MPU6050_light_stm32.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define TEST_LENGTH_SAMPLES 2048
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
uint16_t ID=0;
/* -------------------------------------------------------------------
 * External Input and Output buffer Declarations for FFT Bin Example
 * ------------------------------------------------------------------- */
extern float32_t testInput_f32_10khz[TEST_LENGTH_SAMPLES];
static float32_t testOutput[TEST_LENGTH_SAMPLES/2];
float32_t eixoX[TEST_LENGTH_SAMPLES];
float32_t eixoY[TEST_LENGTH_SAMPLES];
float32_t eixoZ[TEST_LENGTH_SAMPLES];
/* ------------------------------------------------------------------
 * Global variables for FFT Bin Example
 * ------------------------------------------------------------------- */
uint32_t fftSize = 1024;
uint32_t ifftFlag = 0;
uint32_t doBitReverse = 1;
/* Reference index at which max energy of bin ocuurs */
uint32_t refIndex = 80, testIndex = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int x = 0;
/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{
	/* USER CODE BEGIN 1 */
	int32_t timer;
	int16_t index;
	float32_t max_amp_value = 0;
	int32_t size;
	char num[30];
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
	MX_TIM1_Init();
	MX_I2C1_Init();
	/* USER CODE BEGIN 2 */
	MPU6050_init(&hi2c1);
	//printf("Calculating gyro offset, do not move MPU6050\r\n");
	HAL_Delay(1000);
	MPU6050_calcGyroOffsets();
	//printf("Done!\r\n");

	tft_gpio_init();
	HAL_TIM_Base_Start(&htim1);
	ID = tft_readID();
	HAL_Delay(100);
	tft_init (ID);
	setRotation(1);
	fillScreen(BLUE);
	/* ----------------------------------------------------------------------
	 * Max magnitude FFT Bin test
	 * ------------------------------------------------------------------- */
	arm_status status;
	float32_t maxValue;
	status = ARM_MATH_SUCCESS;
	//------------Teste LCD--------------
	float32_t espelho_pre_processamento[320];
	index = 0;
	while (index < TEST_LENGTH_SAMPLES) {
		MPU6050_update();
		if(HAL_GetTick() - timer >= 1) {
			//MPU6050_getTemp();
			//eixoX[index] = MPU6050_getAccX();
			//eixoY[index] = MPU6050_getAccY();
			//eixoZ[index] = MPU6050_getAccZ();
			eixoX[index] = MPU6050_getGyroX();
			eixoY[index] = MPU6050_getGyroY();
			eixoZ[index] = MPU6050_getGyroZ();
			index++;
			eixoX[index] = 0;
			eixoY[index] = 0;
			eixoZ[index] = 0;
			index++;
			//printf("GYRO X : %.2f", MPU6050_getGyroX());
			//printf("\tY : %.2f", MPU6050_getGyroY());
			//printf("\tZ : %.2f\r\n", MPU6050_getGyroZ());
			//printf("ACC ANGLE X : %.2f", MPU6050_getAccAngleX());
			//printf("\tY : %.2f\r\n", MPU6050_getAccAngleY());
			//printf("ANGLE X : %.2f", MPU6050_getAngleX());
			//printf("\tY : %.2f", MPU6050_getAngleY());
			//printf("\tZ : %.2f\r\n", MPU6050_getAngleZ());
			//printf("=======================================================\r\n");
			timer = HAL_GetTick();
		}
	}


	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1)
	{

		for (int j=0; j<3; j++) {
			if (j == 0) {
				memcpy(testInput_f32_10khz, eixoX, TEST_LENGTH_SAMPLES * sizeof(float32_t));
				size = sprintf(num,"Eixo X: ");
			}

			if (j == 1) {
				memcpy(testInput_f32_10khz, eixoY, TEST_LENGTH_SAMPLES * sizeof(float32_t));
				size = sprintf(num,"Eixo Y: ");
			}

			if (j == 2) {
				memcpy(testInput_f32_10khz, eixoZ, TEST_LENGTH_SAMPLES * sizeof(float32_t));
				size = sprintf(num,"Eixo Z: ");
			}

			HAL_UART_Transmit(&huart2, num, size, 10);

			max_amp_value = 0;
			for(int k=0; k<320; k++) {
				espelho_pre_processamento[k] = testInput_f32_10khz[k*2]; //Pula os zeros da parte imaginária
				if (espelho_pre_processamento[k] > max_amp_value) {
					max_amp_value = espelho_pre_processamento[k];
				}
				size = sprintf(num,"%.2f, ", espelho_pre_processamento[k]);
				HAL_UART_Transmit(&huart2, num, size, 10);
			}
			/* Process the data through the CFFT/CIFFT module */
			arm_cfft_f32(&arm_cfft_sR_f32_len1024, testInput_f32_10khz, ifftFlag, doBitReverse);
			/* Process the data through the Complex Magnitude Module for calculating the magnitude at each bin */
			arm_cmplx_mag_f32(testInput_f32_10khz, testOutput, fftSize);
			/* Calculates maxValue and returns corresponding BIN value */
			arm_max_f32(testOutput, fftSize, &maxValue, &testIndex);
			// if (testIndex != refIndex)
			// {
			// status = ARM_MATH_TEST_FAILURE;
			// }
			/* ----------------------------------------------------------------------
			 ** Loop here if the signals fail the PASS check.
			 ** This denotes a test failure
			 ** ------------------------------------------------------------------- */
			if ( status != ARM_MATH_SUCCESS)
			{
				while (1);
			}
			//---------------Agora vai para o LCD----------------
			int32_t i, amp, escala, yini, yfinal, ycentro;

			//while
			fillScreen(BLACK);
			printnewtstr (15, WHITE, &mono12x7bold, 1, (uint8_t *)"FT-Dominio do Tempo");
			switch (j) {
			case 0:
				printnewtstr (35, WHITE, &mono12x7bold, 1, (uint8_t *)"Eixo X");
				size = sprintf(num,"Eixo X\r\n");
				HAL_UART_Transmit(&huart2, num, size, 10);
				break;
			case 1:
				printnewtstr (35, WHITE, &mono12x7bold, 1, (uint8_t *)"Eixo Y");
				size = sprintf(num,"Eixo Y\r\n");
				HAL_UART_Transmit(&huart2, num, size, 10);
				break;
			case 2:
				printnewtstr (35, WHITE, &mono12x7bold, 1, (uint8_t *)"Eixo Z");
				size = sprintf(num,"Eixo Z\r\n");
				HAL_UART_Transmit(&huart2, num, size, 10);
				break;
			default:
				break;
			}

			escala = 100; //Empírico (visa ocupar a tela na vertical)
			ycentro = 120;
			for(i=0; i<320; i++)
			{
				amp = (int32_t)(espelho_pre_processamento[i]*escala/max_amp_value);
				//size = sprintf(num,"%d\r\n",amp);
				//HAL_UART_Transmit(&huart2, num, size, 10);
				if(amp>0) { yini = ycentro-amp; yfinal = ycentro; }
				else { yini = ycentro; yfinal = ycentro-amp; }
				fillRect(i, yini, 1, yfinal-yini+1, YELLOW);
				//drawPixel(i, ycentro-amp, YELLOW);
			}
			HAL_Delay(3000);
			fillScreen(BLACK);
			printnewtstr (15, WHITE, &mono12x7bold, 1, (uint8_t *)"FT-Dominio da Freq.");
			escala = 200;
			ycentro = 230;
			for(i=0; i<320; i++)
			{
				amp = (int32_t)(testOutput[i]*escala/maxValue);
				//size = sprintf(num,"%d\r\n",amp);
				//HAL_UART_Transmit(&huart2, num, size, 10);
				if(amp>0) { yini = ycentro-amp; yfinal = ycentro; }
				else { yini = ycentro; yfinal = ycentro-amp; }
				fillRect(i, yini, 1, yfinal-yini+1, YELLOW);
			}
			size = sprintf(num,"freq pico: %d\r\n",testIndex);
			printnewtstr (32, WHITE, &mono12x7bold, 1, num);
			HAL_UART_Transmit(&huart2, num, size, 10);
			size = sprintf(num,"amp pico: %.2f\r\n",max_amp_value);
			printnewtstr (48, WHITE, &mono12x7bold, 1, num);
			HAL_UART_Transmit(&huart2, num, size, 10);

			HAL_Delay(3000);
			//endwhile
		}

		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
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
	hi2c1.Init.Timing = 0x00702991;
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
	htim1.Init.Prescaler = 79;
	htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim1.Init.Period = 0xFFFF-1;
	htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim1.Init.RepetitionCounter = 0;
	htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
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

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOH_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1|GPIO_PIN_7, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_4|LD2_Pin
			|GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_10|GPIO_PIN_3|GPIO_PIN_4
			|GPIO_PIN_5, GPIO_PIN_RESET);

	/*Configure GPIO pin : B1_Pin */
	GPIO_InitStruct.Pin = B1_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : PC1 PC7 */
	GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_7;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pins : PA0 PA1 PA4 LD2_Pin
                           PA8 PA9 PA10 */
	GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_4|LD2_Pin
			|GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pins : PB0 PB10 PB3 PB4
                           PB5 */
	GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_10|GPIO_PIN_3|GPIO_PIN_4
			|GPIO_PIN_5;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

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

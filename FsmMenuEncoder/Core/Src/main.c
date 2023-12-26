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
#include "fatfs.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
// Bibliotecas C Standard
#include <stdio.h>
#include <string.h>
#include <math.h>

// Bibliotecas do Display TFT
#include "fonts.h"
#include "tft.h"
#include "user_setting.h"
#include "functions.h"

// Bibliotecas do SD Card
#include "fatfs_sd.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

struct State {
	unsigned char out;		//saida do estado
	unsigned short wait;	//tempo de espera no estado atual
	unsigned char next[4];
};

typedef const struct State tipoS;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define A 			0
#define A1 			1
#define A2 			2
#define A3 			3
#define A1S 		4
#define A2S 		5
#define A3S 		6
#define B 			7
#define B1			8
#define B2 			9
#define B3 			10
#define B1S 		11
#define B2S 		12
#define B3S 		13
#define C 			14
#define C1			15
#define C2 			16
#define C3 			17
#define C1S 		18
#define C2S 		19
#define C3X 		20
#define C3Y 		21
#define C3Z 		22
#define C3XS 		23
#define C3YS 		24
#define C3ZS 		25

#define HORARIO		0
#define ANTIHORARIO 1
#define SELECIONA	2
#define VOLTA		3

#define pinCLK		GPIO_PIN_15	//PB15 - CLK Enc
#define portCLK		GPIOB
#define pinDT		GPIO_PIN_14	//PB14 - DT Enc
#define portDT		GPIOB
#define pinSW		GPIO_PIN_13	//PB13 - SW Enc
#define portSW		GPIOB
#define minEncVal	0
#define maxEncVal	9

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
uint16_t ID = 0;
FATFS fs0, fs1; /* Work area (filesystem object) for logical drives */
FIL fsrc, fdst; /* File objects */
BYTE buffer[513]; /* File copy buffer */
FRESULT fr; /* FatFs function common result code */
UINT br, bw; /* File read/write count */
int32_t size;
char text[60];
char filename[40];
volatile uint16_t uart_rx_flag;
uint8_t uart_rx;

tipoS Fsm[26] = {
		{ A, 100, { B, C, A1, A } },			// HOR, ANT, SEL, VOL
		{ A1, 100, { A2, A3, A1S, A } },
		{ A2, 100, { A3, A1, A2S, A } },
		{ A3, 100, { A1, A2, A3S, A } },
		{ A1S, 100, { A1S, A1S, A1S, A1 } },
		{ A2S, 100, { A2S, A2S, A2S, A2 } },
		{ A3S, 100, { A3S, A3S, A3S, A3 } },
		{ B, 100, { C, A, B1, B } },
		{ B1, 100, { B2, B3, B1S, B } },
		{ B2, 100, { B3, B1, B2S, B } },
		{ B3, 100, { B1, B2, B3S, B } },
		{ B1S, 100, { B1S, B1S, B1S, B1 } },
		{ B2S, 100, { B2S, B2S, B2S, B2 } },
		{ B3S, 100, { B3S, B3S, B3S, B3 } },
		{ C, 100, { A, B, C1, C } },
		{ C1, 100, { C2, C3, C1S, C } },
		{ C2, 100, { C3, C1, C2S, C } },
		{ C3, 100, { C1, C2, C3X, C } },
		{ C1S, 100, { C1S, C1S, C1S, C1 } },
		{ C2S, 100, { C2S, C2S, C2S, C2 } },
		{ C3X, 100, { C3Y, C3Z, C3XS,C3 } },
		{ C3Y, 100, { C3Z, C3X, C3YS, C3 } },
		{ C3Z, 100, { C3X, C3Y, C3ZS, C3 } },
		{ C3XS, 100, { C3XS, C3XS, C3XS, C3X } },
		{ C3YS, 100, { C3YS, C3YS, C3YS, C3Y } },
		{ C3ZS, 100, { C3ZS, C3ZS, C3ZS, C3Z } }
};

int encoderPosCount = 0;
GPIO_PinState pinCLKLast;
GPIO_PinState pinCLKVal;
GPIO_PinState pinSWLast;
GPIO_PinState pinSWVal;
int bCW;
uint8_t msgRotacao[] = "\r\nRotacao: ";
uint8_t msgHorario[] = "Horaria\r\n";
uint8_t msgAntiHor[] = "Anti-horaria\r\n";
uint8_t msgPosEnco[] = "Posicao Encoder: ";
char msgVariavel[64];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_SPI1_Init(void);
/* USER CODE BEGIN PFP */

void desenha_tela(unsigned char cMenu);
int config_encoder_pins(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
	/* USER CODE BEGIN 1 */

	unsigned char cState, pState, input = 0xFF;

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
	MX_SPI1_Init();
	MX_FATFS_Init();
	/* USER CODE BEGIN 2 */
	// Inicializacao do LCD
	tft_gpio_init();
	HAL_TIM_Base_Start(&htim1);
	ID = tft_readID();
	HAL_Delay(100);
	tft_init(ID);
	setRotation(1);
	fillScreen(WHITE);

	size = sprintf(text, "\r\n\r\nModelo de Menu com selecao por encoder\r\n");
	HAL_UART_Transmit(&huart2, (uint8_t*) text, size, 100);

	HAL_UART_Receive_IT(&huart2, &uart_rx, 1);// habilita interrupcao de recepcao pela UART2

	cState = A;
	pState = C;

	config_encoder_pins();
	pinCLKLast = HAL_GPIO_ReadPin(portCLK, pinCLK);
	pinSWLast = HAL_GPIO_ReadPin(portSW, pinSW);

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {

		// Controle de menu pelo Encoder KY-040
		pinCLKVal = HAL_GPIO_ReadPin(portCLK, pinCLK);
		if ((pinCLKVal != pinCLKLast) && (pinCLKVal != 0)) {
			if (HAL_GPIO_ReadPin(portDT, pinDT) != pinCLKVal) {
				if (encoderPosCount >= maxEncVal) {
					encoderPosCount = minEncVal;
				} else {
					encoderPosCount++;
				}
				input = HORARIO;
				bCW = 1;
			} else {
				if (encoderPosCount <= minEncVal) {
					encoderPosCount = maxEncVal;
				} else {
					encoderPosCount--;
				}
				input = ANTIHORARIO;
				bCW = 0;
			}
			HAL_UART_Transmit(&huart2, msgRotacao, sizeof(msgRotacao), 50);
			if (bCW)
				HAL_UART_Transmit(&huart2, msgHorario, sizeof(msgHorario), 50);
			else
				HAL_UART_Transmit(&huart2, msgAntiHor, sizeof(msgAntiHor), 50);
			HAL_UART_Transmit(&huart2, msgPosEnco, sizeof(msgPosEnco), 50);
			sprintf(msgVariavel, "%d\n", encoderPosCount);
			HAL_UART_Transmit(&huart2, (uint8_t*) msgVariavel,
					sizeof(msgVariavel), 50);
		}
		pinCLKLast = pinCLKVal;

		pinSWVal = HAL_GPIO_ReadPin(portSW, pinSW);
		if ( (pinSWVal != pinSWLast) && (pinSWVal == GPIO_PIN_SET) ){
			input = SELECIONA;
			HAL_Delay(100);
		}

		// Controle de menu pela UART2
		if (uart_rx_flag == 1) {
			input = 0xFF;
			if (uart_rx == 'd') {		// horario
				input = HORARIO;
			}

			if (uart_rx == 'a') {		// anti-horario
				input = ANTIHORARIO;
			}

			if (uart_rx == 'w') {		// seleciona
				input = SELECIONA;
			}

			if (uart_rx == 's') {		// volta
				input = VOLTA;
			}

			uart_rx_flag = 0;
			uart_rx = 0;
			HAL_UART_Receive_IT(&huart2, &uart_rx, 1);
		}

		if (input != 0xFF) {
			cState = Fsm[cState].next[input];
			input = 0xFF;
		}

		if (cState != pState) {
			size = sprintf(text, "\r\nNovo estado: %d", cState);
			HAL_UART_Transmit(&huart2, (uint8_t*) text, size, 100);

			desenha_tela(Fsm[cState].out);
			HAL_Delay(Fsm[cState].wait);
			pState = cState;

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
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

	/** Configure the main internal regulator output voltage
	 */
	if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1)
			!= HAL_OK) {
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
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK) {
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
	hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
	hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
	hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
	hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
	hspi1.Init.CRCPolynomial = 7;
	hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
	hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
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

	/* USER CODE BEGIN TIM1_Init 1 */

	/* USER CODE END TIM1_Init 1 */
	htim1.Instance = TIM1;
	htim1.Init.Prescaler = 79;
	htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim1.Init.Period = 0xFFFF - 1;
	htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim1.Init.RepetitionCounter = 0;
	htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
	if (HAL_TIM_Base_Init(&htim1) != HAL_OK) {
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig)
			!= HAL_OK) {
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
	huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
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
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOH_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1 | GPIO_PIN_7, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOA,
			GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_4 | GPIO_PIN_8 | GPIO_PIN_9
			| GPIO_PIN_10, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOB,
			GPIO_PIN_0 | GPIO_PIN_10 | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5,
			GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(SD_CS_GPIO_Port, SD_CS_Pin, GPIO_PIN_SET);

	/*Configure GPIO pin : B1_Pin */
	GPIO_InitStruct.Pin = B1_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : PC1 PC7 */
	GPIO_InitStruct.Pin = GPIO_PIN_1 | GPIO_PIN_7;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pins : PA0 PA1 PA4 PA8
	 PA9 PA10 */
	GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_4 | GPIO_PIN_8
			| GPIO_PIN_9 | GPIO_PIN_10;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pins : PB0 PB10 PB3 PB4
	 PB5 */
	GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_10 | GPIO_PIN_3 | GPIO_PIN_4
			| GPIO_PIN_5;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure GPIO pins : PB13 PB14 PB15 */
	//GPIO_InitStruct.Pin = GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15;
	//GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	//GPIO_InitStruct.Pull = GPIO_NOPULL;
	//HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure GPIO pin : SD_CS_Pin */
	GPIO_InitStruct.Pin = SD_CS_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
	HAL_GPIO_Init(SD_CS_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

void desenha_tela(unsigned char cMenu) {

	fillRect(80, 25, 160, 50, YELLOW);
	drawRect(80, 25, 160, 50, BLACK);
	fillRect(80, 95, 160, 50, YELLOW);
	drawRect(80, 95, 160, 50, BLACK);
	fillRect(80, 165, 160, 50, YELLOW);
	drawRect(80, 165, 160, 50, BLACK);

	if ((cMenu == A) || (cMenu == B) || (cMenu == C)) {
		printnewtstr(58, BLACK, &mono12x7bold, 1, (uint8_t*) "           A");
		printnewtstr(128, BLACK, &mono12x7bold, 1, (uint8_t*) "           B");
		printnewtstr(198, BLACK, &mono12x7bold, 1, (uint8_t*) "           C");
	}

	if (cMenu == A) {
		fillRect(80, 25, 160, 50, CYAN);
		drawRect(80, 25, 160, 50, BLACK);
		printnewtstr(58, BLACK, &mono12x7bold, 1, (uint8_t*) "           A");
	}

	if (cMenu == B) {
		fillRect(80, 95, 160, 50, CYAN);
		drawRect(80, 95, 160, 50, BLACK);
		printnewtstr(128, BLACK, &mono12x7bold, 1, (uint8_t*) "           B");
	}

	if (cMenu == C) {
		fillRect(80, 165, 160, 50, CYAN);
		drawRect(80, 165, 160, 50, BLACK);
		printnewtstr(198, BLACK, &mono12x7bold, 1, (uint8_t*) "           C");
	}

	if ((cMenu == A1) || (cMenu == A2) || (cMenu == A3) || (cMenu == A1S)
			|| (cMenu == A2S) || (cMenu == A3S)) {
		printnewtstr(58, BLACK, &mono12x7bold, 1, (uint8_t*) "           A1");
		printnewtstr(128, BLACK, &mono12x7bold, 1, (uint8_t*) "           A2");
		printnewtstr(198, BLACK, &mono12x7bold, 1, (uint8_t*) "           A3");
	}

	if (cMenu == A1) {
		fillRect(80, 25, 160, 50, CYAN);
		drawRect(80, 25, 160, 50, BLACK);
		printnewtstr(58, BLACK, &mono12x7bold, 1, (uint8_t*) "           A1");
	}

	if (cMenu == A2) {
		fillRect(80, 95, 160, 50, CYAN);
		drawRect(80, 95, 160, 50, BLACK);
		printnewtstr(128, BLACK, &mono12x7bold, 1, (uint8_t*) "           A2");
	}

	if (cMenu == A3) {
		fillRect(80, 165, 160, 50, CYAN);
		drawRect(80, 165, 160, 50, BLACK);
		printnewtstr(198, BLACK, &mono12x7bold, 1, (uint8_t*) "           A3");
	}

	if (cMenu == A1S) {
		fillRect(80, 25, 160, 50, BLUE);
		drawRect(80, 25, 160, 50, BLACK);
		printnewtstr(58, BLACK, &mono12x7bold, 1, (uint8_t*) "           A1");
	}

	if (cMenu == A2S) {
		fillRect(80, 95, 160, 50, BLUE);
		drawRect(80, 95, 160, 50, BLACK);
		printnewtstr(128, BLACK, &mono12x7bold, 1, (uint8_t*) "           A2");
	}

	if (cMenu == A3S) {
		fillRect(80, 165, 160, 50, BLUE);
		drawRect(80, 165, 160, 50, BLACK);
		printnewtstr(198, BLACK, &mono12x7bold, 1, (uint8_t*) "           A3");
	}

	if ((cMenu == B1) || (cMenu == B2) || (cMenu == B3) || (cMenu == B1S)
			|| (cMenu == B2S) || (cMenu == B3S)) {
		printnewtstr(58, BLACK, &mono12x7bold, 1, (uint8_t*) "           B1");
		printnewtstr(128, BLACK, &mono12x7bold, 1, (uint8_t*) "           B2");
		printnewtstr(198, BLACK, &mono12x7bold, 1, (uint8_t*) "           B3");
	}

	if (cMenu == B1) {
		fillRect(80, 25, 160, 50, CYAN);
		drawRect(80, 25, 160, 50, BLACK);
		printnewtstr(58, BLACK, &mono12x7bold, 1, (uint8_t*) "           B1");
	}

	if (cMenu == B2) {
		fillRect(80, 95, 160, 50, CYAN);
		drawRect(80, 95, 160, 50, BLACK);
		printnewtstr(128, BLACK, &mono12x7bold, 1, (uint8_t*) "           B2");
	}

	if (cMenu == B3) {
		fillRect(80, 165, 160, 50, CYAN);
		drawRect(80, 165, 160, 50, BLACK);
		printnewtstr(198, BLACK, &mono12x7bold, 1, (uint8_t*) "           B3");
	}

	if (cMenu == B1S) {
		fillRect(80, 25, 160, 50, BLUE);
		drawRect(80, 25, 160, 50, BLACK);
		printnewtstr(58, BLACK, &mono12x7bold, 1, (uint8_t*) "           B1");
	}

	if (cMenu == B2S) {
		fillRect(80, 95, 160, 50, BLUE);
		drawRect(80, 95, 160, 50, BLACK);
		printnewtstr(128, BLACK, &mono12x7bold, 1, (uint8_t*) "           B2");
	}

	if (cMenu == B3S) {
		fillRect(80, 165, 160, 50, BLUE);
		drawRect(80, 165, 160, 50, BLACK);
		printnewtstr(198, BLACK, &mono12x7bold, 1, (uint8_t*) "           B3");
	}

	if ((cMenu == C1) || (cMenu == C2) || (cMenu == C3) || (cMenu == C1S)
			|| (cMenu == C2S)) {
		printnewtstr(58, BLACK, &mono12x7bold, 1, (uint8_t*) "           C1");
		printnewtstr(128, BLACK, &mono12x7bold, 1, (uint8_t*) "           C2");
		printnewtstr(198, BLACK, &mono12x7bold, 1, (uint8_t*) "           C3");
	}

	if (cMenu == C1) {
		fillRect(80, 25, 160, 50, CYAN);
		drawRect(80, 25, 160, 50, BLACK);
		printnewtstr(58, BLACK, &mono12x7bold, 1, (uint8_t*) "           C1");
	}

	if (cMenu == C2) {
		fillRect(80, 95, 160, 50, CYAN);
		drawRect(80, 95, 160, 50, BLACK);
		printnewtstr(128, BLACK, &mono12x7bold, 1, (uint8_t*) "           C2");
	}

	if (cMenu == C3) {
		fillRect(80, 165, 160, 50, CYAN);
		drawRect(80, 165, 160, 50, BLACK);
		printnewtstr(198, BLACK, &mono12x7bold, 1, (uint8_t*) "           C3");
	}

	if (cMenu == C1S) {
		fillRect(80, 25, 160, 50, BLUE);
		drawRect(80, 25, 160, 50, BLACK);
		printnewtstr(58, BLACK, &mono12x7bold, 1, (uint8_t*) "           C1");
	}

	if (cMenu == C2S) {
		fillRect(80, 95, 160, 50, BLUE);
		drawRect(80, 95, 160, 50, BLACK);
		printnewtstr(128, BLACK, &mono12x7bold, 1, (uint8_t*) "           C2");
	}

	if ((cMenu == C3X) || (cMenu == C3Y) || (cMenu == C3Z) || (cMenu == C3XS)
			|| (cMenu == C3YS) || (cMenu == C3ZS)) {
		printnewtstr(58, BLACK, &mono12x7bold, 1, (uint8_t*) "          C3X");
		printnewtstr(128, BLACK, &mono12x7bold, 1, (uint8_t*) "          C3Y");
		printnewtstr(198, BLACK, &mono12x7bold, 1, (uint8_t*) "          C3Z");
	}

	if (cMenu == C3X) {
		fillRect(80, 25, 160, 50, CYAN);
		drawRect(80, 25, 160, 50, BLACK);
		printnewtstr(58, BLACK, &mono12x7bold, 1, (uint8_t*) "          C3X");
	}

	if (cMenu == C3Y) {
		fillRect(80, 95, 160, 50, CYAN);
		drawRect(80, 95, 160, 50, BLACK);
		printnewtstr(128, BLACK, &mono12x7bold, 1, (uint8_t*) "          C3Y");
	}

	if (cMenu == C3Z) {
		fillRect(80, 165, 160, 50, CYAN);
		drawRect(80, 165, 160, 50, BLACK);
		printnewtstr(198, BLACK, &mono12x7bold, 1, (uint8_t*) "          C3Z");
	}

	if (cMenu == C3XS) {
		fillRect(80, 25, 160, 50, BLUE);
		drawRect(80, 25, 160, 50, BLACK);
		printnewtstr(58, BLACK, &mono12x7bold, 1, (uint8_t*) "          C3X");
	}

	if (cMenu == C3YS) {
		fillRect(80, 95, 160, 50, BLUE);
		drawRect(80, 95, 160, 50, BLACK);
		printnewtstr(128, BLACK, &mono12x7bold, 1, (uint8_t*) "          C3Y");
	}

	if (cMenu == C3ZS) {
		fillRect(80, 165, 160, 50, BLUE);
		drawRect(80, 165, 160, 50, BLACK);
		printnewtstr(198, BLACK, &mono12x7bold, 1, (uint8_t*) "          C3Z");
	}

}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	uart_rx_flag = 1;
}

int config_encoder_pins(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };

	/* GPIO Port Clock Enable */
	if (portCLK == GPIOA) {
		__HAL_RCC_GPIOA_CLK_ENABLE();
	} else if (portCLK == GPIOB) {
		__HAL_RCC_GPIOB_CLK_ENABLE();
	} else if (portCLK == GPIOC) {
		__HAL_RCC_GPIOC_CLK_ENABLE();
	} else if (portCLK == GPIOH) {
		__HAL_RCC_GPIOH_CLK_ENABLE();
	} else {
		return -1;
	}

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(portCLK, pinCLK, GPIO_PIN_RESET);

	/*Configure GPIO pinA */
	GPIO_InitStruct.Pin = pinCLK;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(portCLK, &GPIO_InitStruct);

	// PORTB
	/* GPIO Port Clock Enable */
	if (portCLK != portDT) {
		if (portCLK == GPIOA) {
			__HAL_RCC_GPIOA_CLK_ENABLE();
		} else if (portCLK == GPIOB) {
			__HAL_RCC_GPIOB_CLK_ENABLE();
		} else if (portCLK == GPIOC) {
			__HAL_RCC_GPIOC_CLK_ENABLE();
		} else if (portCLK == GPIOH) {
			__HAL_RCC_GPIOH_CLK_ENABLE();
		} else {
			return -1;
		}
	}

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(portDT, pinDT, GPIO_PIN_RESET);

	/*Configure GPIO pin : B1_Pin */
	GPIO_InitStruct.Pin = pinDT;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(portDT, &GPIO_InitStruct);


	/* GPIO Port SW Switch Button */
	if ( (portSW != portDT) || (portSW != portCLK)) {
		if (portSW == GPIOA) {
			__HAL_RCC_GPIOA_CLK_ENABLE();
		} else if (portSW == GPIOB) {
			__HAL_RCC_GPIOB_CLK_ENABLE();
		} else if (portSW == GPIOC) {
			__HAL_RCC_GPIOC_CLK_ENABLE();
		} else if (portSW == GPIOH) {
			__HAL_RCC_GPIOH_CLK_ENABLE();
		} else {
			return -1;
		}
	}

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(portSW, pinSW, GPIO_PIN_RESET);

	/*Configure GPIO pin */
	GPIO_InitStruct.Pin = pinSW;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(portSW, &GPIO_InitStruct);


	return 0;
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

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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* Standard Includes */
#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>

/* LTC Library Includes */
#include "LT_HAL_SPI.h"
#include "LTC681x_HAL.h"
#include "LTC6813_HAL.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ENABLED 1
#define DISABLED 0
#define DATALOG_ENABLED 1
#define DATALOG_DISABLED 0
#define PWM 1
#define SCTL 2
#define TOTAL_IC 18 // Total no. of ICs in the accumulator
#define TOTAL_CELL 6 // No. of cells per stack
#define TOTAL_TEMP 4 // No. of temperatures per stack
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc;

CAN_HandleTypeDef hcan;

SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
/*******************************************************************
 Setup Variables
 The following variables can be modified to configure the software.
 ********************************************************************/
//const uint8_t TOTAL_IC = 18;//!< Number of ICs in the daisy chain; USING DEFINE INSTEAD
/********************************************************************
 ADC Command Configurations. See LTC681x.h for options
 *********************************************************************/
const uint8_t ADC_OPT = ADC_OPT_DISABLED; //!< ADC Mode option bit
const uint8_t ADC_CONVERSION_MODE = MD_7KHZ_3KHZ; //!< ADC Mode
const uint8_t ADC_DCP = DCP_DISABLED; //!< Discharge Permitted
const uint8_t CELL_CH_TO_CONVERT = CELL_CH_ALL; //!< Channel Selection for ADC conversion
const uint8_t AUX_CH_TO_CONVERT = AUX_CH_ALL; //!< Channel Selection for ADC conversion
const uint8_t STAT_CH_TO_CONVERT = STAT_CH_ALL; //!< Channel Selection for ADC conversion
const uint8_t SEL_ALL_REG = REG_ALL; //!< Register Selection
const uint8_t SEL_REG_A = REG_1; //!< Register Selection
const uint8_t SEL_REG_B = REG_2; //!< Register Selection

const uint16_t MEASUREMENT_LOOP_TIME = 500; //!< Loop Time in milliseconds(ms)

//Under Voltage and Over Voltage Thresholds
const uint16_t OV_THRESHOLD = 41000; //!< Over voltage threshold ADC Code. LSB = 0.0001 ---(4.1V)
const uint16_t UV_THRESHOLD = 30000; //!< Under voltage threshold ADC Code. LSB = 0.0001 ---(3V)

//Loop Measurement Setup. These Variables are ENABLED or DISABLED. Remember ALL CAPS
const uint8_t WRITE_CONFIG = DISABLED; //!< This is to ENABLED or DISABLED writing into to configuration registers in a continuous loop
const uint8_t READ_CONFIG = DISABLED; //!< This is to ENABLED or DISABLED reading the configuration registers in a continuous loop
const uint8_t MEASURE_CELL = ENABLED; //!< This is to ENABLED or DISABLED measuring the cell voltages in a continuous loop
const uint8_t MEASURE_AUX = DISABLED; //!< This is to ENABLED or DISABLED reading the auxiliary registers in a continuous loop
const uint8_t MEASURE_STAT = DISABLED; //!< This is to ENABLED or DISABLED reading the status registers in a continuous loop
const uint8_t PRINT_PEC = DISABLED; //!< This is to ENABLED or DISABLED printing the PEC Error Count in a continuous loop
/************************************
 END SETUP
 *************************************/

/*******************************************************
 Global Battery Variables received from 681x commands
 These variables store the results from the LTC6813
 register reads and the array lengths must be based
 on the number of ICs on the stack
 ******************************************************/
cell_asic BMS_IC[TOTAL_IC]; //!< Global Battery Variable

/*************************************************************************
 Set configuration register. Refer to the data sheet
 **************************************************************************/
bool REFON = true; //!< Reference Powered Up Bit
bool ADCOPT = false; //!< ADC Mode option bit
bool GPIOBITS_A[5] = { true, true, true, true, false }; //!< GPIO Pin Control // Gpio 1,2,3,4,5 - [1:4] pulled up for ADC
bool GPIOBITS_B[4] = { false, false, false, false }; //!< GPIO Pin Control // Gpio 6,7,8,9
uint16_t UV = UV_THRESHOLD; //!< Under voltage Comparison Voltage
uint16_t OV = OV_THRESHOLD; //!< Over voltage Comparison Voltage
bool DCCBITS_A[12] = { false, false, false, false, false, false, false, false,
false, false, false, false }; //!< Discharge cell switch //Dcc 1,2,3,4,5,6,7,8,9,10,11,12
bool DCCBITS_B[7] = { false, false, false, false, false, false, false }; //!< Discharge cell switch //Dcc 0,13,14,15
bool DCTOBITS[4] = { true, false, true, false }; //!< Discharge time value //Dcto 0,1,2,3  // Programed for 4 min
/*Ensure that Dcto bits are set according to the required discharge time. Refer to the data sheet */
bool FDRF = false; //!< Force Digital Redundancy Failure Bit
bool DTMEN = true; //!< Enable Discharge Timer Monitor
bool PSBITS[2] = { false, false }; //!< Digital Redundancy Path Selection//ps-0,1

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_CAN_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_ADC_Init(void);
/* USER CODE BEGIN PFP */
#ifdef __GNUC__
/* With GCC/RAISONANCE, small printf (option LD Linker->Libraries->Small printf
 set to 'Yes') calls __io_putchar() */
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */

void print_wrconfig(void);
void print_rxconfig(void);
void print_cells(uint8_t datalog_en);
void print_aux(uint8_t datalog_en);
void serial_print_hex(uint8_t data);
void check_error(int error);
uint16_t tempCalc(uint8_t ic, uint8_t temp);
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
  MX_SPI1_Init();
  MX_CAN_Init();
  MX_USART1_UART_Init();
  MX_ADC_Init();
  /* USER CODE BEGIN 2 */
	int8_t error = 0;

	LTC6813_init_cfg(TOTAL_IC, BMS_IC);
//  LTC6813_init_cfgb(TOTAL_IC,BMS_IC);
	for (uint8_t current_ic = 0; current_ic < TOTAL_IC; current_ic++) {
		LTC6813_set_cfgr(current_ic, BMS_IC, REFON, ADCOPT, GPIOBITS_A,
				DCCBITS_A, DCTOBITS, UV, OV);
//    LTC6813_set_cfgrb(current_ic,BMS_IC,FDRF,DTMEN,PSBITS,GPIOBITS_B,DCCBITS_B);
	}
	LTC6813_reset_crc_count(TOTAL_IC, BMS_IC);
	LTC6813_init_reg_limits(TOTAL_IC, BMS_IC);

	if (WRITE_CONFIG == ENABLED) {
		wakeup_idle(TOTAL_IC);
		LTC6813_wrcfg(TOTAL_IC, BMS_IC);
//    LTC6813_wrcfgb(TOTAL_IC,BMS_IC);
		print_wrconfig();
//    print_wrconfigb();
	}

	if (READ_CONFIG == ENABLED) {
		wakeup_idle(TOTAL_IC);
		error = LTC6813_rdcfg(TOTAL_IC, BMS_IC);
		check_error(error);
//          error = LTC6813_rdcfgb(TOTAL_IC,BMS_IC);
//          check_error(error);
		print_rxconfig();
//          print_rxconfigb();
	}
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {
		if (MEASURE_CELL == ENABLED) {
			wakeup_idle(TOTAL_IC);
			LTC6813_adcv(ADC_CONVERSION_MODE, ADC_DCP, CELL_CH_TO_CONVERT);
			LTC6813_pollAdc();
			wakeup_idle(TOTAL_IC);
			error = LTC6813_rdcv(0, TOTAL_IC, BMS_IC);
			check_error(error);
			print_cells(DATALOG_ENABLED);
		}

		if (MEASURE_AUX == ENABLED) {
			wakeup_idle(TOTAL_IC);
			LTC6813_adax(ADC_CONVERSION_MODE, AUX_CH_TO_CONVERT);
			LTC6813_pollAdc();
			wakeup_idle(TOTAL_IC);
			error = LTC6813_rdaux(0, TOTAL_IC, BMS_IC); // Set to read back all aux registers
			check_error(error);
			print_aux(DATALOG_ENABLED);
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI14|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSI14State = RCC_HSI14_ON;
  RCC_OscInitStruct.HSI14CalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC_Init(void)
{

  /* USER CODE BEGIN ADC_Init 0 */

  /* USER CODE END ADC_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC_Init 1 */

  /* USER CODE END ADC_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc.Instance = ADC1;
  hadc.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc.Init.Resolution = ADC_RESOLUTION_12B;
  hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc.Init.ScanConvMode = ADC_SCAN_DIRECTION_FORWARD;
  hadc.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc.Init.LowPowerAutoWait = DISABLE;
  hadc.Init.LowPowerAutoPowerOff = DISABLE;
  hadc.Init.ContinuousConvMode = DISABLE;
  hadc.Init.DiscontinuousConvMode = DISABLE;
  hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc.Init.DMAContinuousRequests = DISABLE;
  hadc.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  if (HAL_ADC_Init(&hadc) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_1;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_2;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_3;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_4;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_5;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_6;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_10;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_11;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_12;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_13;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC_Init 2 */

  /* USER CODE END ADC_Init 2 */

}

/**
  * @brief CAN Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN_Init(void)
{

  /* USER CODE BEGIN CAN_Init 0 */

  /* USER CODE END CAN_Init 0 */

  /* USER CODE BEGIN CAN_Init 1 */

  /* USER CODE END CAN_Init 1 */
  hcan.Instance = CAN;
  hcan.Init.Prescaler = 16;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_1TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_1TQ;
  hcan.Init.TimeTriggeredMode = DISABLE;
  hcan.Init.AutoBusOff = DISABLE;
  hcan.Init.AutoWakeUp = DISABLE;
  hcan.Init.AutoRetransmission = DISABLE;
  hcan.Init.ReceiveFifoLocked = DISABLE;
  hcan.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN_Init 2 */

  /* USER CODE END CAN_Init 2 */

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
  hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SPI1_CS1_GPIO_Port, SPI1_CS1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA7 PA8 PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PC4 PC5 PC6 PC7
                           PC8 PC9 PC10 PC11
                           PC12 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7
                          |GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11
                          |GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 PB2 PB10
                           PB11 PB12 PB13 PB14
                           PB15 SPI1_CS2_Pin PB8 PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_10
                          |GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14
                          |GPIO_PIN_15|SPI1_CS2_Pin|GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : SPI1_CS1_Pin */
  GPIO_InitStruct.Pin = SPI1_CS1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SPI1_CS1_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
/**
 * @brief  Retargets the C library printf function to the USART.
 * @param  None
 * @retval None
 */
PUTCHAR_PROTOTYPE {
	/* Place your implementation of fputc here */
	/* e.g. write a character to the USART1 and Loop until the end of transmission */
	HAL_UART_Transmit(&huart1, (uint8_t*) &ch, 1, 0xFFFF);

	return ch;
}

/*!******************************************************************************
 \brief Prints the configuration data that is going to be written to the LTC6813
 to the serial port.
 @return void
 ********************************************************************************/
void print_wrconfig(void) {
	int cfg_pec;
//    Serial.println(F("Written Configuration A Register: "))
	printf("Written Configuration A Register: \n");
	for (int current_ic = 0; current_ic < TOTAL_IC; current_ic++) {
//      Serial.print(F("CFGA IC "));
		printf("CFGA IC ");
//      Serial.print(current_ic+1,DEC);
		printf("%d", current_ic + 1);
		for (int i = 0; i < 6; i++) {
//        Serial.print(F(", 0x"));
			printf(", 0x");
			serial_print_hex(BMS_IC[current_ic].config.tx_data[i]);
		}
//      Serial.print(F(", Calculated PEC: 0x"));
		printf(", Calculated PEC: 0x");
		cfg_pec = pec15_calc(6, &BMS_IC[current_ic].config.tx_data[0]);
		serial_print_hex((uint8_t) (cfg_pec >> 8));
//      Serial.print(F(", 0x"));
		printf(", 0x");
		serial_print_hex((uint8_t) (cfg_pec));
//      Serial.println("\n");
		printf("/n");
	}
}

/*!*****************************************************************
 \brief Prints the configuration data that was read back from the
 LTC6813 to the serial port.
 @return void
 *******************************************************************/
void print_rxconfig(void) {
//  Serial.println(F("Received Configuration A Register: "));
	printf("Received Configuration A Register: \n");
	for (int current_ic = 0; current_ic < TOTAL_IC; current_ic++) {
//    Serial.print(F("CFGA IC "));
		printf("CFGA IC ");
//    Serial.print(current_ic+1,DEC);
		printf("%d", current_ic + 1);
		for (int i = 0; i < 6; i++) {
//      Serial.print(F(", 0x"));
			printf(", 0x");
			serial_print_hex(BMS_IC[current_ic].config.rx_data[i]);
		}
//    Serial.print(F(", Received PEC: 0x"));
		printf(", Received PEC: 0x");
		serial_print_hex(BMS_IC[current_ic].config.rx_data[6]);
//    Serial.print(F(", 0x"));
		printf(", 0x");
		serial_print_hex(BMS_IC[current_ic].config.rx_data[7]);
//    Serial.println("\n");
		printf("\n");
	}
}

/*!************************************************************
 \brief Prints cell voltage codes to the serial port
 @return void
 *************************************************************/
void print_cells(uint8_t datalog_en) {
	for (int current_ic = 0; current_ic < TOTAL_IC; current_ic++) {
		if (datalog_en == 0) {
//      Serial.print(" IC ");
			printf("IC ");
//      Serial.print(current_ic+1,DEC)
			printf("%d", current_ic + 1);
//      Serial.print(", ");
			printf(", ");
			for (int i = 0; i < TOTAL_CELL; i++) {
//        Serial.print(" C");
				printf(" C");
//        Serial.print(i+1,DEC);
				printf("%d", i + 1);
//        Serial.print(":");
				printf(":");
//        Serial.print(BMS_IC[current_ic].cells.c_codes[i]*0.0001,4);
				printf("%0.4f", BMS_IC[current_ic].cells.c_codes[i] * 0.0001);
//        Serial.print(",");
				printf(",");
			}
//      Serial.println();
			printf("\n");
		} else {
//      Serial.print(" Cells, ");
			printf(" Cells, ");
			for (int i = 0; i < TOTAL_CELL; i++) {
//        Serial.print(BMS_IC[current_ic].cells.c_codes[i]*0.0001,4);
				printf("%0.4f", BMS_IC[current_ic].cells.c_codes[i] * 0.0001);
//        Serial.print(",");
				printf(",");
			}
		}
	}
//  Serial.println("\n");
	printf("\n");
}

/*!****************************************************************************
 \brief Prints GPIO voltage codes and Vref2 voltage code onto the serial port
 @return void
 *****************************************************************************/
void print_aux(uint8_t datalog_en) {
	for (int current_ic = 0; current_ic < TOTAL_IC; current_ic++) {
		if (datalog_en == 0) {
//      Serial.print(" IC ");
			printf(" IC ");
//      Serial.print(current_ic+1,DEC);
			printf("%d", current_ic + 1);
			for (int i = 0; i < TOTAL_TEMP; i++) {
//        Serial.print(F(" GPIO-"));
				printf(" GPIO-");
//        Serial.print(i+1,DEC);
				printf("%d", i + 1);
//        Serial.print(":");
				printf(":");
//        Serial.print(BMS_IC[current_ic].aux.a_codes[i]*0.0001,4);
				printf("%0.2f", tempCalc(current_ic, i) * 0.01);
//        Serial.print(",");
				printf(",");
			}

//      for (int i=6; i < 10; i++)
//      {
//        Serial.print(F(" GPIO-"));
//        Serial.print(i,DEC);
//        Serial.print(":");
//        Serial.print(BMS_IC[current_ic].aux.a_codes[i]*0.0001,4);
//      }

//      Serial.print(F(" Vref2"));
			printf(" Vref2");
//      Serial.print(":");
			printf(":");
//      Serial.print(BMS_IC[current_ic].aux.a_codes[5]*0.0001,4);
			printf("%0.4f", BMS_IC[current_ic].aux.a_codes[5] * 0.0001);
//      Serial.println();
			printf("\n");

//      Serial.print(" OV/UV Flags : 0x");
			printf(" OV/UV Flags : 0x");
//      Serial.print((uint8_t)BMS_IC[current_ic].aux.a_codes[11],HEX);
			printf("%02X", (uint8_t) BMS_IC[current_ic].aux.a_codes[11]);
//      Serial.println();
			printf("\n");
		} else {
//      Serial.print(" AUX, ");
			printf(" AUX, ");

			for (int i = 0; i < 12; i++) {
//        Serial.print((uint8_t)BMS_IC[current_ic].aux.a_codes[i]*0.0001,4);
				printf("%.4f",
						(uint8_t) BMS_IC[current_ic].aux.a_codes[i] * 0.0001);
//        Serial.print(",");
				printf(",");
			}
		}
	}
// Serial.println("\n");
	printf("\n");
}

/*!****************************************************************************
 \brief Function to print in HEX form
 @return void
 *****************************************************************************/
void serial_print_hex(uint8_t data) {
	if (data < 16) {
//    Serial.print("0");
		printf("0");
//    Serial.print((byte)data,HEX);
		printf("%02X", (unsigned char) data);
	} else
//    Serial.print((byte)data,HEX);
		printf("%02X", (unsigned char) data);
}

// To calculate temperature from raw aux ADC value
uint16_t tempCalc(uint8_t ic, uint8_t temp) {
  vRef2 = BMS_IC[ic].aux.a_codes[5] * 0.0001;
  R = BMS_IC[ic].aux.a_codes[temp] / (vRef2 - (BMS_IC[ic].aux.a_codes[temp] * 0.0001));
  return ((3435 / log(R / r_inf)) - 273.15) * 100;
}

/*!****************************************************************************
 \brief Function to check error flag and print PEC error message
 @return void
 *****************************************************************************/
void check_error(int error) {
	if (error == -1) {
//    Serial.println(F("A PEC error was detected in the received data"));
		printf("A PEC error was detected in the received data\n");
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
	while (1) {
	}
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
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

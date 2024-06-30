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
#include 	"BNO08X.h"
#include 	"DHT22.h"
#include 	"Voltage_Current.h"
#include 	"HX711.h"
#include 	"MAX6675.h"
#include 	<stdint.h>
#include 	<string.h>
#include 	<stdio.h>

#include 	"communication_full.h"
#include	"fonts.h"
#include 	"tft.h"
#include	"functions.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define		USE_BNO08X
//#define		USE_DHT22
#define		USE_VOLT_CURRENT
#define		USE_LOADCELL
#define		USE_COM_CONTROL
#define		USE_COM_PC
//#define		USE_LCD

/////////////////////////////// INTERVAL TIME ///////////////////////////////////
#define		TEMP_INTERVAL		1000
#define		CURRENT_INTERVAL	1000
#define		VOLTAGE_INTERVAL	1000
#define		SENSOR_INTERVAL		1000
#define		BNO08X_INTERVAL		50
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// Typedef BNO08x
BNO08X_Typedef BNO08x_Data;

// Typedef DHT22
DHT_Typedef DHT22_Data;

// Typedef Voltage
Voltage_Current_Typedef Volt_Current_Data;

// Typedef Loadcell
hx711_t Loadcell_Data;

// Typedef All Sensor Data
sensor_package_t Sensor_Data;

// Typedef Communication PC
com_pc_get_t message_from_pc;

// Typedef Communication Control
com_ctrl_get_t message_from_ctrl;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

extern SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart6;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart6_rx;

/* USER CODE BEGIN PV */
// ID LCD
uint16_t ID = 37697;

// Buffer UART
char Buffer[50];
char BufferLast[50];

// Channel Loadcell
float chA= 0;
float chB = 0;

// Buffer Sending Data
uint32_t CurrentTick = 0,
		 TempTick = 0,
		 VoltageTick = 0,
		 BNO08XTick = 0,
		 LoadcellTick = 0,
		 SensorTick = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART6_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_SPI1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int16_t diff_data_yaw[5];
uint8_t sample_yaw = 0;
bool is_calibrated = false;
uint16_t id_astar= 0;
////////////////////////////////////// COMMUNICATION CALLBACK ////////////////////////////////////

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart == &huart2) {

		// Callback for BNO08X Data
		#ifdef USE_BNO08X
	    BNO08X_GetData(&BNO08x_Data);
	    // Handling Calibration
	    if(!is_calibrated && BNO08x_Data.yaw != 0){
	    	if(sample_yaw >= 4){
	    		// Find Average value
//	    		double sum_yaw = 0;
//	    		for(uint8_t i = 0; i < 5; i++){
//	    			sum_yaw += diff_data_yaw[i];
//	    		}
//	    		sum_yaw = sum_yaw/5;

//	    		sum_yaw = diff_data_yaw[4] - diff_data_yaw[0];

	    		// Decision making
	    		if(diff_data_yaw[0] <= 100 && diff_data_yaw[0] >= 100){
	    			is_calibrated = true;
	    		}
	    		else{
	    			// RESET STM
	    			HAL_NVIC_SystemReset();
	    		}
	    	}
	    	diff_data_yaw[sample_yaw] = BNO08x_Data.yaw;
	    	sample_yaw++;
	    }
    	#endif

	} else if (huart == &huart1) {
	    // Callback for Communicate to PC
		#ifdef USE_COM_PC
		rx_pc_get(&message_from_pc);
		#endif

	} else if(huart == &huart6){

		// Callback Receive data from STM32 Control
		#ifdef USE_COM_CONTROL
		rx_ctrl_get(&message_from_ctrl);

		#endif
	}
}

float data_loadA = 0;
float data_loadB = 0;
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
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART6_UART_Init();
  MX_TIM1_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */
  ////////////////////////////////////// SENSOR INITIALIZATION ////////////////////////////////////

    // BNO08X initialization
    #ifdef USE_BNO08X
    BNO08X_Init(&huart2);
    #endif

    // DHT22 Initialization
    #ifdef USE_DHT22
    DHT_Start();
    #endif

    // Volt & Current Initialization
    #ifdef USE_VOLT_CURRENT
    VoltCurrent_Init(&hadc1);
    #endif

    // Load cell Initialization
    #ifdef USE_LOADCELL
//    hx711_calibration(&Loadcell_Data, GPIOB, GPIO_PIN_0, GPIOB, GPIO_PIN_1);
    hx711_init(&Loadcell_Data, GPIOB, GPIO_PIN_0, GPIOB, GPIO_PIN_1);
    set_scale(&Loadcell_Data, 115.598, 72.818);
    #endif

    // Initialize Communication to Control
    #ifdef USE_COM_CONTROL
    komunikasi_ctrl_init(&huart6);
    rx_ctrl_start_get();
    #endif

    // Initialize Communication to PC
    #ifdef USE_COM_PC
    komunikasi_pc_init(&huart1);
    rx_pc_start_get();
    #endif

    // LCD Initialization
    #ifdef USE_LCD
    HAL_TIM_Base_Start(&htim1);
    readID();
    tft_init(ID);
    HAL_Delay(1000);
    fillScreen(BLACK);
    setRotation(135);
    #endif

    HAL_Delay(1000);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
//    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
  while (1)
  {
//	  if(id_astar < 20){
//		  tx_ctrl_send_Astar(message_from_pc,id_astar);
//		  id_astar++;
//	  }
//	  else{
//		  id_astar = 0;
//	  }
//	  data_loadA = get_weight(&Loadcell_Data, 10, CHANNEL_A);
//	  data_loadB = get_weight(&Loadcell_Data, 10, CHANNEL_B);
	  	  ////////////////////////////////////// ASYNCHRONOUS READING & SENDING ///////////////////////////////////////////

	  CurrentTick = HAL_GetTick();

	  if(CurrentTick-SensorTick > SENSOR_INTERVAL){

		  // Reading Data in MX7655 Sensor
		  Sensor_Data.temperature = (Max6675_Read_Temp()*100);

		  // Reading Data in Voltage Sensor
		  Get_Voltage_Measurement(&Volt_Current_Data);
		  Sensor_Data.voltage = (Volt_Current_Data.voltage*100);

		  // Reading Data in Current Sensor
		  Get_Current_Measurement(&Volt_Current_Data);
		  Sensor_Data.current = (Volt_Current_Data.current*100);

		  // Sending Sensor Data
		  tx_pc_send_Sensor(Sensor_Data);

		  // Sending Odometry Data
		  tx_pc_send_Odometry(message_from_ctrl.x_pos,message_from_ctrl.y_pos,message_from_ctrl.t_pos,message_from_ctrl.x_vel,message_from_ctrl.y_vel,message_from_ctrl.t_vel);


		  // Sending BNO08X Data
		  if(is_calibrated) tx_pc_send_BNO08X(BNO08x_Data);

		  SensorTick = CurrentTick;
	  }

//	  CurrentTick = HAL_GetTick();
//	  if(CurrentTick-BNO08XTick > BNO08X_INTERVAL){
//		  HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
//
		  // Sending BNO08X Data
//		  tx_ctrl_send_BNO08X(BNO08x_Data);//
//		  BNO08XTick = CurrentTick;
//	  }

	  ////////////////////////////////////// SENSOR READING ////////////////////////////////////

	  // Reading Data in DHT22 Sensor
//	  DHT_GetData(&DHT22_Data);
//	  Sensor_Data.temperature = DHT22_Data.Temperature;
//	  Sensor_Data.humidity = DHT22_Data.Humidity;

	  // Reading Data in MX7655 Sensor
//	  Sensor_Data.temperature = (Max6675_Read_Temp()*100);

	  // Reading Data in Voltage Sensor
//	  Get_Voltage_Measurement(&Volt_Current_Data);
//	  Sensor_Data.voltage = (Volt_Current_Data.voltage*100);

	  // Reading Data in Current Sensor
//	  Get_Current_Measurement(&Volt_Current_Data);
//	  Sensor_Data.current = (Volt_Current_Data.current*100);

	  // Reading Data in Load cell Sensor
//	  Sensor_Data.loadcell = hx711_measure_weight(Loadcell_Data);

	  ////////////////////////////////////// SENDING DATA TO PC ////////////////////////////////

//	  tx_pc_ping();
//	   Sending BNO08X Data
//	  if(is_calibrated) tx_pc_send_BNO08X(BNO08x_Data);

////	   Sending Sensor Data
//	  tx_pc_send_Sensor(Sensor_Data);
//
//	   Sending Sensor Data
//	  tx_pc_send_Odometry(message_from_ctrl.x_pos,message_from_ctrl.y_pos,message_from_ctrl.t_pos,message_from_ctrl.x_vel,message_from_ctrl.y_vel,message_from_ctrl.t_vel);


	  ////////////////////////////////////// SENDING DATA TO CONTROL ///////////////////////////

	  // Sending BNO08X Data
	  tx_ctrl_send_Astar();
//	  tx_ctrl_ping();
	  if(is_calibrated) tx_ctrl_send_BNO08X(BNO08x_Data);
//	  HAL_Delay(10);

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
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

//  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
//  sConfig.Channel = ADC_CHANNEL_1;
//  sConfig.Rank = 1;
//  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
//  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
//  {
//    Error_Handler();
//  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
//  sConfig.Rank = 2;
//  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
//  {
//    Error_Handler();
//  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  hspi1.Init.Direction = SPI_DIRECTION_2LINES_RXONLY;
  hspi1.Init.DataSize = SPI_DATASIZE_16BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
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
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 84-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
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
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  * @brief USART6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART6_UART_Init(void)
{

  /* USER CODE BEGIN USART6_Init 0 */

  /* USER CODE END USART6_Init 0 */

  /* USER CODE BEGIN USART6_Init 1 */

  /* USER CODE END USART6_Init 1 */
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 115200;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART6_Init 2 */

  /* USER CODE END USART6_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
  /* DMA2_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream1_IRQn);
  /* DMA2_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);

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
  HAL_GPIO_WritePin(LED_BUILTIN_GPIO_Port, LED_BUILTIN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, MX7665_Pin|LED_BLUE_Pin|LED_GREEN_Pin|LED_RED_Pin
                          |MUL_SCK_Pin|MUL_Latch_Pin|MUL_MOSI_Pin|LCD_CS_Pin
                          |LCD_RS_Pin|LCD_WR_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LCD_RST_GPIO_Port, LCD_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED_BUILTIN_Pin */
  GPIO_InitStruct.Pin = LED_BUILTIN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_BUILTIN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : DHT22_Pin */
  GPIO_InitStruct.Pin = DHT22_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(DHT22_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : MX7665_Pin LED_BLUE_Pin LED_GREEN_Pin LED_RED_Pin
                           MUL_SCK_Pin MUL_Latch_Pin MUL_MOSI_Pin LCD_CS_Pin
                           LCD_RS_Pin LCD_WR_Pin */
  GPIO_InitStruct.Pin = MX7665_Pin|LED_BLUE_Pin|LED_GREEN_Pin|LED_RED_Pin
                          |MUL_SCK_Pin|MUL_Latch_Pin|MUL_MOSI_Pin|LCD_CS_Pin
                          |LCD_RS_Pin|LCD_WR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : LCD_RST_Pin */
  GPIO_InitStruct.Pin = LCD_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LCD_RST_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LCD_RD_Pin */
  GPIO_InitStruct.Pin = LCD_RD_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(LCD_RD_GPIO_Port, &GPIO_InitStruct);

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

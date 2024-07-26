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
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "ADBMS.h"
#include "stdio.h"
#include "String.h"
#include "usbd_cdc_if.h"
#include "adBms6830Data.h"

//SoC
#include "BatterySOCEstimationV2.h"
#include "rtwtypes.h"

//FREERTOS
#include "FreeRTOS.h"
//#include "SEGGER_SYSVIEW.h"
#include "task.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define total_ic 6

extern I2C_HandleTypeDef hi2c1;

#define EEPROM_I2C &hi2c1

#define EEPROM_ADDR 0xA0

#define PAGE_SIZE 128     // in Bytes
#define PAGE_NUM  512    // number of pages
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

CAN_HandleTypeDef hcan1;

I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
cell_asic ic[total_ic];

uint16_t adc = 0;
uint16_t convCount = 0;
uint32_t adc_data[3];
uint16_t adc_buffer[3];
uint16_t analogAveragingBuffer[500][3] = {0};

uint16_t MAINBUFFER[40] = { 0 };

CAN_TxHeaderTypeDef TxHeader;
CAN_RxHeaderTypeDef RxHeader;

uint8_t RxData[8];
uint8_t TxData[8];
uint32_t TxMailbox;

uint8_t* usbbuf;
uint8_t usblen;

uint8_t VCUflag = 0;
uint8_t Chargerflag = 0;

uint8_t VehicleMode = 0;


//FREERTOS
TaskHandle_t VoltageRead;
TaskHandle_t Temp_OWire;
TaskHandle_t Measure_Precharge;
TaskHandle_t ChargeControl;
TaskHandle_t VCUcom;
TaskHandle_t USBTASK;
TaskHandle_t SoCTASK;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM2_Init(void);
static void MX_SPI1_Init(void);
static void MX_SPI2_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_CAN1_Init(void);
/* USER CODE BEGIN PFP */
static void vVoltageRead(void *parameter);
static void vTemp_OWire(void *parameter);
static void vMeasure_Precharge(void *parameter);
static void vChargeControl(void *parameter);
static void vVCUcom(void *parameter);
static void vUSBTASK(void *parameter);
static void vSoCTASK(void *parameter);
void faultLatch();
uint8_t IsSCActive();

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

#define DEV_ADDR 0xa0
uint8_t dataw1[] = "vay anam babam be burasi resmen bir harika\n";
uint8_t dataw2[] = "agalarlabms";
float dataw3 = 123.31;

// 5. sayfanın ilk iki byte'ı bozuk.

uint8_t datar1[100];
uint8_t datar2[100];
float datar3;


float volt_bc = 0;
float volt_ac = 0;

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
  MX_TIM2_Init();
  MX_SPI1_Init();
  MX_SPI2_Init();
  MX_I2C1_Init();
  MX_USART3_UART_Init();
  MX_TIM1_Init();
  MX_USB_DEVICE_Init();
  MX_CAN1_Init();
  /* USER CODE BEGIN 2 */

  HAL_Delay(1000);

//  MAINBUFFER[CHARGER_SET_VOLTAGE] = 4030;
//  MAINBUFFER[CHARGER_SET_CURRENT] = 60;
//  MAINBUFFER[ALLOWED_DISBALANCE] = 100;
//  MAINBUFFER[CHARGE_CURRENT] =(uint16_t) -60;
//  MAINBUFFER[PRECHARGE_PERCENTAGE] = 90;
//  MAINBUFFER[PRECHARGE_TIMEOUT] = 4000;
//  MAINBUFFER[CHARGE_OVER_CURRENT_TRESHOLD] = -500;
//  MAINBUFFER[DISCHARGE_OVER_CURRENT_TRESHOLD] = 3500;
//  MAINBUFFER[OVER_VOLTAGE_ERROR_DELAY] = 100;
//  MAINBUFFER[UNDER_VOLTAGE_ERROR_DELAY] = 100;
//  MAINBUFFER[OVER_CURRENT_ERROR_DELAY] = 100;
//  MAINBUFFER[OPEN_WIRE_ERROR_DELAY] = 100;
//  MAINBUFFER[HEAT_ERROR_DELAY] = 100;
//
//
//   EEPROM_SetConfigs((uint8_t *) MAINBUFFER, sizeof(MAINBUFFER));
//

    EEPROM_GetConfigs((uint8_t *) MAINBUFFER, sizeof(MAINBUFFER));
    MAINBUFFER[34]=-31.31;
    MAINBUFFER[33]= 4000;
    MAINBUFFER[CHARGER_SET_VOLTAGE] = 40310;


	HAL_ADC_Start_DMA(&hadc1,  adc_buffer, 3);
	HAL_CAN_Start(&hcan1);
	HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
//	HAL_GPIO_WritePin(GPIO, GPIO_Pin, PinState)
	TxHeader.IDE = CAN_ID_STD;
	TxHeader.StdId = 0x00; //değişecek
	TxHeader.RTR = CAN_RTR_DATA;
	TxHeader.DLC = 8; //değişebilir



//	if(HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox) != HAL_OK){
//		Error_Handler();
//	}


//	HAL_TIM_Base_Start(&htim1);

	adBms6830_init_config(total_ic, &ic[0]);


//	CDC_Transmit_FS(txdata, sizeof("Hello World from USB CDC\n"));
//while(1){
//
//		adBms6830_start_raux_voltage_measurment(total_ic,&ic[0]);
//				adBms6830_read_raux_voltages(total_ic,&ic[0]);
//
////				adBms6830_start_adc_cell_voltage_measurment(total_ic);
////				adBms6830_read_cell_voltages(total_ic,&ic[0]);
////		adBms6830tempMesurementand_open_wire_detection(total_ic,&ic[0]);
////				HAL_Delay(500);
////		for(uint8_t icCounter = 0; icCounter < 5; icCounter++){
////			adBmsCsLow();
////			HAL_Delay(4);
////			adBmsCsHigh();
//			HAL_Delay(150);
//}

//	union  CAN_Union {
//		uint8_t sendBuffer8bit[8];
//	    uint16_t sendBuffer16bit[4];
//	}canSend;
//
//	HAL_GPIO_WritePin(ERROR_OUT_GPIO_Port, ERROR_OUT_Pin, 1);
//	while(1){
////		TxHeader.IDE = CAN_ID_STD;
////		TxHeader.StdId = BMS_CAN_ID_VOLTAGES;
////		canSend.sendBuffer16bit[0] = 0X31;
////		canSend.sendBuffer16bit[1] = 0X31;
////		canSend.sendBuffer16bit[2] = 0X31;
////		canSend.sendBuffer16bit[3] = 0X31;
////		HAL_CAN_AddTxMessage(&hcan1, &TxHeader, canSend.sendBuffer8bit, &TxMailbox);
//
//		HAL_Delay(3000);
//
//		HAL_GPIO_TogglePin(PRE_OUT_GPIO_Port, PRE_OUT_Pin);
//
//		HAL_GPIO_TogglePin(AIR__OUT_GPIO_Port, AIR__OUT_Pin);
//
//
//		}
//
//		HAL_GPIO_WritePin(ERROR_OUT_GPIO_Port, ERROR_OUT_Pin, 0);
//		HAL_Delay(500);
//		HAL_GPIO_WritePin(ERROR_OUT_GPIO_Port, ERROR_OUT_Pin, 1);
//		HAL_Delay(500);
//
//
//	}

	//FREERTOS TASK CREATION PART

	//SEGGER_SYSVIEW_Conf();
	//SEGGER_SYSVIEW_Start();

	xTaskCreate(vVoltageRead, "VoltageRead", 300, NULL, 2, &VoltageRead);
	xTaskCreate(vTemp_OWire, "Temp_OWire", 400, NULL, 3, &Temp_OWire);
	xTaskCreate(vMeasure_Precharge, "Measure_Precharge", 200, NULL, 1, &Measure_Precharge);
	xTaskCreate(vChargeControl, "ChargeControl", 200, NULL, 5, &ChargeControl);
	xTaskCreate(vVCUcom, "VCUcom", 200, NULL, 6, &VCUcom);
	xTaskCreate(vUSBTASK, "USBTASK", 200, NULL, 15, &USBTASK);
	xTaskCreate(vSoCTASK, "SoCTASK", 200, NULL, 4, &SoCTASK);
	vTaskStartScheduler();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {

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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 15;
  RCC_OscInitStruct.PLL.PLLN = 144;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 5;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV4;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
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

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 3;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_112CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = 3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 12;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_3TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_1TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */
  CAN_FilterTypeDef canfilterconfig;

  canfilterconfig.FilterActivation = CAN_FILTER_ENABLE;
  canfilterconfig.FilterBank = 18;  // which filter bank to use from the assigned ones
  canfilterconfig.FilterFIFOAssignment = CAN_FILTER_FIFO0;
  canfilterconfig.FilterIdHigh = 0x00;
  canfilterconfig.FilterIdLow = 0x00;
  canfilterconfig.FilterMaskIdHigh = 0x00;
  canfilterconfig.FilterMaskIdLow = 0x00;
  canfilterconfig.FilterMode = CAN_FILTERMODE_IDMASK;
  canfilterconfig.FilterScale = CAN_FILTERSCALE_32BIT;
  canfilterconfig.SlaveStartFilterBank = 20;  // how many filters to assign to the CAN1 (master can)

  HAL_CAN_ConfigFilter(&hcan1, &canfilterconfig);
  /* USER CODE END CAN1_Init 2 */

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
  hi2c1.Init.ClockSpeed = 400000;
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
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_128;
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
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

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
  htim1.Init.Prescaler = 60000-1;
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
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 56;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1000-1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, SPI2_CS_Pin|SPI1_CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, ERROR_OUT_Pin|AIR__OUT_Pin|PRE_OUT_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : SPI2_CS_Pin SPI1_CS_Pin */
  GPIO_InitStruct.Pin = SPI2_CS_Pin|SPI1_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : SC_END_Pin */
  GPIO_InitStruct.Pin = SC_END_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(SC_END_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : ERROR_OUT_Pin AIR__OUT_Pin PRE_OUT_Pin */
  GPIO_InitStruct.Pin = ERROR_OUT_Pin|AIR__OUT_Pin|PRE_OUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan){

	BaseType_t pxHigherPriorityTaskWoken = pdFALSE;


	if(HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData)!= HAL_OK)
	{
		Error_Handler();
	}

	if(RxHeader.IDE == CAN_ID_STD)
	{
		if(RxHeader.StdId == VCU_ID)
		{
			vTaskNotifyGiveFromISR(VCUcom,&pxHigherPriorityTaskWoken);
			portYIELD_FROM_ISR(pxHigherPriorityTaskWoken);
			VCUflag = 1;
		}
	}
	else
	{
		if(RxHeader.ExtId == CHARGER_ID)
		{
			vTaskNotifyGiveFromISR(ChargeControl,&pxHigherPriorityTaskWoken);
			portYIELD_FROM_ISR(pxHigherPriorityTaskWoken);
			Chargerflag = 1;
		}

	}




}

static void vVoltageRead(void *parameter) {

	uint8_t pecflag = 0;
	uint8_t underVoltageflag = 0;
	uint8_t overVoltageflag = 0;
	uint8_t dummyflag = 0;

	uint32_t SlaveOpenWiretick = 0;
	uint32_t OverVoltagetick = 0;
	uint32_t UnderVoltagetick = 0;

	float TotalVoltage = 0;
	float MaxCellVoltage = 0;
	float MinCellVoltage = 0;

	for (;;) {

		//SECTION OF VOLTAGE READING AND DETECTION OF OVER AND UNDER VOLTAGE
		//
		//

		taskENTER_CRITICAL();
		adBmsWakeupIc(total_ic);
		adBmsWriteData(total_ic, &ic[0], WRCFGA, Config, A);
		adBmsWriteData(total_ic, &ic[0], WRCFGB, Config, B);
		adBmsWakeupIc(total_ic);
		adBms6830_Adcv_ALL(REDUNDANT_MEASUREMENT, CONTINUOUS, DISCHARGE_PERMITTED,
				RESET_FILTER, CELL_OPEN_WIRE_DETECTION,total_ic);
		vTaskDelay(1); // ADCs are updated at their conversion rate is 1ms
		adBms6830_Adcv_ALL(RD_ON, CONTINUOUS, DISCHARGE_PERMITTED, RESET_FILTER,
				CELL_OPEN_WIRE_DETECTION,total_ic);
		vTaskDelay(1); // ADCs are updated at their conversion rate is 1ms
		adBmsReadData(total_ic, &ic[0], RDCVA, Cell, A);
		adBmsReadData(total_ic, &ic[0], RDCVB, Cell, B);
		adBmsReadData(total_ic, &ic[0], RDCVC, Cell, C);
		adBmsReadData(total_ic, &ic[0], RDCVD, Cell, D);
		adBmsReadData(total_ic, &ic[0], RDCVE, Cell, E);
		adBmsReadData(total_ic, &ic[0], RDCVF, Cell, F);
		taskEXIT_CRITICAL();

		for (uint8_t icCounter = 0; icCounter < total_ic; icCounter++) {
			for (uint8_t cellCounter = 0; cellCounter < 16; cellCounter++) {
				//openwire check
				if (ic[icCounter].cccrc.cell_pec == 1){
					pecflag = 1;
				}
				else {
					if(ic[icCounter].cell.c_codes[cellCounter]>0){
						ic[icCounter].mesured_cell_voltages[cellCounter] =
								getVoltage(ic[icCounter].cell.c_codes[cellCounter]);
					}

					//printf("cell%d: %f\n", cellCounter + 1,
					//		ic[icCounter].mesured_cell_voltages[cellCounter]);

					if (ic[icCounter].mesured_cell_voltages[cellCounter] < CELL_UNDER_VOLTAGE_TRESHOLD) {
						underVoltageflag = 1;
					}

					if (ic[icCounter].mesured_cell_voltages[cellCounter] > CELL_OVER_VOLTAGE_TRESHOLD) {
						overVoltageflag = 1;
					}

					//MAINBUFFER CALCULATIONS
					if(dummyflag == 0)
					{
						MaxCellVoltage = ic[icCounter].mesured_cell_voltages[cellCounter];
						MinCellVoltage = ic[icCounter].mesured_cell_voltages[cellCounter];
						dummyflag = 1;
					}

					TotalVoltage += ic[icCounter].mesured_cell_voltages[cellCounter];

					if(ic[icCounter].mesured_cell_voltages[cellCounter] > MaxCellVoltage)
					{
						MaxCellVoltage = ic[icCounter].mesured_cell_voltages[cellCounter];
					}

					if(ic[icCounter].mesured_cell_voltages[cellCounter] < MinCellVoltage)
					{
						MinCellVoltage = ic[icCounter].mesured_cell_voltages[cellCounter];
					}
				}



			}
		}

		//MAINBUFFER LOADING

		MAINBUFFER[PACK_TOTAL_CELL_VOLTAGE] = (uint16_t) (TotalVoltage*100);
		MAINBUFFER[PACK_AVG_CELL_VOLTAGE] = (uint16_t) ((TotalVoltage/(16.0*total_ic))*100);
		MAINBUFFER[PACK_MAX_CELL_VOLTAGE] = (uint16_t) (MaxCellVoltage*100);
		MAINBUFFER[PACK_MIN_CELL_VOLTAGE] = (uint16_t) (MinCellVoltage*100);

		dummyflag = 0;
		TotalVoltage = 0;

		//FAULT PROCESS

		if (pecflag == 1) {

			if (SlaveOpenWiretick == 0) SlaveOpenWiretick = xTaskGetTickCount();
			else if(xTaskGetTickCount()-SlaveOpenWiretick >= MAINBUFFER[OPEN_WIRE_ERROR_DELAY]){
				MAINBUFFER[FAULTS] |= 1;
				faultLatch();
			}
			pecflag = 0;

		} else{

			MAINBUFFER[FAULTS] &= ~(1);
			SlaveOpenWiretick = 0;
		}


		if (underVoltageflag == 1) {
			if (UnderVoltagetick == 0) UnderVoltagetick = xTaskGetTickCount();
			else if(xTaskGetTickCount()-UnderVoltagetick >= MAINBUFFER[UNDER_VOLTAGE_ERROR_DELAY]){
				MAINBUFFER[FAULTS] |= 1 << 1;
				faultLatch();
			}
			underVoltageflag = 0;

		} else{
			MAINBUFFER[FAULTS] &= ~(1 << 1);
			UnderVoltagetick = 0;
		}

		if (overVoltageflag == 1) {
			if (OverVoltagetick == 0) OverVoltagetick = xTaskGetTickCount();
			else if(xTaskGetTickCount()-OverVoltagetick >= MAINBUFFER[OVER_VOLTAGE_ERROR_DELAY]){
				MAINBUFFER[FAULTS] |= 1 << 2;
				faultLatch();
			}
			overVoltageflag = 0;

		} else{
			MAINBUFFER[FAULTS] &= ~(1 << 2);
			OverVoltagetick = 0;
		}


		vTaskDelay(30);
		faultLatch();
	}

}
static void vTemp_OWire(void *parameter) {

	float voltage[total_ic][2][16];
	float MaxCellHeat =0;
	float MinCellHeat =0;
	float TotalCellHeat =0;
	float MaxSlaveHeat =0;

	uint8_t dummyflag = 0;
	uint8_t dummyflag2 = 0;
	uint8_t pecflag = 0;
	uint8_t cellOWflag = 0;
	uint8_t tempOWflag = 0;
	uint8_t slaveOHeatflag = 0;
	uint8_t overHeatflag = 0;
	uint8_t underHeatflag = 0;

	uint32_t SlaveOpenWiretick =0;
	uint32_t CellOpenWiretick =0;
	uint32_t TempOpenWiretick =0;
	uint32_t SlaveOverHeattick =0;
	uint32_t CellOverHeattick =0;
	uint32_t CellUnderHeattick =0;

	uint8_t ntcSellectionList[16] = { 15, 1, 14, 2, 13, 3, 12, 4, 11, 5, 10, 6,
			9, 7, 8, 0 };


	for (;;) {

		//SECTION OF CELL OPEN WIRE DETECTION
		//
		//
		taskENTER_CRITICAL();
		adBmsReadData(total_ic, &ic[0], RDSVA, S_volt, A);
		adBmsWakeupIc(total_ic);
		adBms6830_Adsv_ALL(CONTINUOUS, DISCHARGE_PERMITTED, OW_ON_EVEN_CH, total_ic);
		vTaskDelay(2);
		adBmsReadData(total_ic, &ic[0], RDSVA, S_volt, A);
		adBmsReadData(total_ic, &ic[0], RDSVB, S_volt, B);
		adBmsReadData(total_ic, &ic[0], RDSVC, S_volt, C);
		adBmsReadData(total_ic, &ic[0], RDSVD, S_volt, D);
		adBmsReadData(total_ic, &ic[0], RDSVE, S_volt, E);
		adBmsReadData(total_ic, &ic[0], RDSVF, S_volt, F);
		taskEXIT_CRITICAL();

		for (uint8_t icCounter = 0; icCounter < total_ic; icCounter++) {
			for (uint8_t cell = 0; cell < 16; cell++) {
				if (ic[icCounter].cccrc.scell_pec == 1)
					pecflag = 1;

				else {
					voltage[icCounter][0][cell] = getVoltage(
							ic[icCounter].scell.sc_codes[cell]);
					//				printf("ciftpack:%d,", (icCounter));
					//				printf("C%d=%fV,", (cell + 1), voltage[icCounter][0][cell]);
					//				printf("\n");

				}

			}
		}
		vTaskDelay(8);

		taskENTER_CRITICAL();
		adBmsWakeupIc(total_ic);
		adBms6830_Adsv_ALL(CONTINUOUS, DISCHARGE_PERMITTED, OW_ON_ODD_CH, total_ic);
		vTaskDelay(2);
		adBmsReadData(total_ic, &ic[0], RDSVB, S_volt, B);
		adBmsReadData(total_ic, &ic[0], RDSVA, S_volt, A);
		adBmsReadData(total_ic, &ic[0], RDSVC, S_volt, C);
		adBmsReadData(total_ic, &ic[0], RDSVD, S_volt, D);
		adBmsReadData(total_ic, &ic[0], RDSVE, S_volt, E);
		adBmsReadData(total_ic, &ic[0], RDSVF, S_volt, F);
		taskEXIT_CRITICAL();

		for (uint8_t icCounter = 0; icCounter < total_ic; icCounter++) {
			for (uint8_t cell = 0; cell < 16; cell++) {
				if (ic[icCounter].cccrc.scell_pec == 1)
					pecflag = 1;
				else {
					voltage[icCounter][1][cell] = getVoltage(
							ic[icCounter].scell.sc_codes[cell]);
					//				printf("tekpack:%d,", (icCounter));
					//				printf("C%d=%fV,", (cell + 1), voltage[icCounter][1][cell]);
					//				printf("\n");

				}
			}
		}
		for (uint8_t icCounter = 0; icCounter < total_ic; icCounter++) {
			for (uint8_t cell = 0; cell < 16; cell++) {
				for (uint8_t cifttek = 0; cifttek < 2; cifttek++) {
									//printf("ic:%d cell %d %d: %f\n", icCounter, cell + 1, cifttek,
										//	voltage[icCounter][cifttek][cell]);
					if (voltage[icCounter][cifttek][cell] < 2
							|| voltage[icCounter][cifttek][cell] > 4.5) {
						//printf("HATA:%d\n",cell+1);
						cellOWflag = 1;
					}

				}

			}

		}

		//SECTION OF TEMPERATURE OPEN WIRE DETECTION AND TEMPERATURE READING
		//
		//
        //SLAVE TEMPERATURE
//		adBmsCsLow();
//		vTaskDelay(4);
//		adBmsCsHigh();
//		vTaskDelay(4);
		taskENTER_CRITICAL();
		adBmsWakeupIc(total_ic);
		adBmsWriteData(total_ic, &ic[0], WRCFGA, Config, A);
		adBms6830_Adax_ALL(AUX_OW_OFF, PUP_DOWN, AUX_ALL,total_ic);
		adBmsReadData(total_ic, &ic[0], RDAUXA, Aux, A);
		taskEXIT_CRITICAL();

		for (uint8_t icCounter = 0; icCounter < total_ic; icCounter++) {
			ic[icCounter].TempOpenWire = 0;

			ic[icCounter].board_temp = ((getVoltage(ic[icCounter].aux.a_codes[0]))-0.4)*50;

			if(ic[icCounter].board_temp > SLAVE_OVER_HEAT_TRESHOLD) slaveOHeatflag = 1;

			if(dummyflag == 0)
			{
				MaxSlaveHeat = ic[icCounter].board_temp;
				dummyflag = 1;
		    }

			if(ic[icCounter].board_temp > MaxSlaveHeat)
			{
				MaxSlaveHeat = ic[icCounter].board_temp;
			}
	        //CELL TEMPERATURE
			//adBmsWakeupIc(total_ic);
			for (uint8_t ntcSellection = 0; ntcSellection < 16;
					ntcSellection++) {

				ic[icCounter].tx_cfga.gpo = ntcSellectionList[ntcSellection]
						<< 6 | 0x3F;


				taskENTER_CRITICAL();
				adBmsWakeupIc(total_ic);
				adBmsWriteData(total_ic, &ic[0], WRCFGA, Config, A);
				adBms6830_Adax_ALL(AUX_OW_OFF, PUP_DOWN, AUX_ALL,total_ic);
				adBmsWakeupIc(total_ic);
				adBmsReadData(total_ic, &ic[0], RDAUXB, Aux, B);
				adBmsReadData(total_ic, &ic[0], RDSTATA, Status, A);
				taskEXIT_CRITICAL();

				if (ic[icCounter].cccrc.aux_pec != 1)  {
					//FAULTS

					if (abs(getVoltage(ic[icCounter].aux.a_codes[5])- getVoltage(ic[icCounter].stata.vref2))< 0.001) {
						ic[icCounter].TempOpenWire |= 1 << ntcSellection;
						tempOWflag=1;
					}
					else ic[icCounter].mesured_cell_temps[ntcSellection] =
							Thermistor(getVoltage(ic[icCounter].aux.a_codes[5]),
									getVoltage(ic[icCounter].stata.vref2));
					//printf("ntc%d:%f %f \n",ntcSellection+1,ic[icCounter].mesured_cell_temps[ntcSellection],getVoltage(ic[icCounter].aux.a_codes[5]));
				    }

					if (ic[icCounter].mesured_cell_temps[ntcSellection]>CELL_OVER_HEAT_TRESHOLD) {
						overHeatflag=1;
					}

					if (ic[icCounter].mesured_cell_temps[ntcSellection]<CELL_UNDER_HEAT_TRESHOLD) {
						underHeatflag=1;
					}

					//MAINBUFFER CALCULATIONS
					if(dummyflag2 == 0)
					{
						MaxCellHeat = ic[icCounter].mesured_cell_temps[ntcSellection];
						MinCellHeat = ic[icCounter].mesured_cell_temps[ntcSellection];
						dummyflag2 = 1;
					}

					TotalCellHeat += ic[icCounter].mesured_cell_temps[ntcSellection];

					if(ic[icCounter].mesured_cell_temps[ntcSellection] > MaxCellHeat)
					{
						MaxCellHeat = ic[icCounter].mesured_cell_temps[ntcSellection];
					}

					if(ic[icCounter].mesured_cell_temps[ntcSellection] < MinCellHeat)
					{
						MinCellHeat = ic[icCounter].mesured_cell_temps[ntcSellection];
					}

					vTaskDelay(3);



			}

		}
		//MAINBUFFER LOADING
		MAINBUFFER[PACK_MAX_CELL_TEMP] = (uint16_t) (MaxCellHeat*100);
		MAINBUFFER[PACK_MIN_CELL_TEMP] = (uint16_t) (MinCellHeat*100);
		MAINBUFFER[PACK_AVG_CELL_TEMP] = (uint16_t) ((TotalCellHeat/(16.0*total_ic))*100.0);
		MAINBUFFER[PACK_MAX_SLAVE_TEMP] = (uint16_t) (MaxSlaveHeat*100);

        dummyflag = 0;
        dummyflag2 = 0;
        TotalCellHeat = 0;

        //FAULT

		if (pecflag == 1) {

			if (SlaveOpenWiretick == 0) SlaveOpenWiretick = xTaskGetTickCount();
			else if(xTaskGetTickCount()-SlaveOpenWiretick >= MAINBUFFER[OPEN_WIRE_ERROR_DELAY]){
				MAINBUFFER[FAULTS] |= 1;
				faultLatch();
			}
			pecflag = 0;

		} else{

			MAINBUFFER[FAULTS] &= ~(1);
			SlaveOpenWiretick = 0;
		}


		if (cellOWflag == 1) {
			if (CellOpenWiretick == 0) CellOpenWiretick = xTaskGetTickCount();
			else if(xTaskGetTickCount()-CellOpenWiretick >= MAINBUFFER[OPEN_WIRE_ERROR_DELAY]){
				MAINBUFFER[FAULTS] |= 1 << 7;
				faultLatch();
			}
			cellOWflag = 0;

		} else{
			MAINBUFFER[FAULTS] &= ~(1 << 7);
			CellOpenWiretick = 0;
		}

		if (tempOWflag == 1) {
			if (TempOpenWiretick == 0) TempOpenWiretick = xTaskGetTickCount();
			else if(xTaskGetTickCount()-TempOpenWiretick >= MAINBUFFER[HEAT_ERROR_DELAY]){
				MAINBUFFER[FAULTS] |= 1 << 12;
				faultLatch();
			}
			tempOWflag = 0;

		} else{
			MAINBUFFER[FAULTS] &= ~(1 << 12);
			TempOpenWiretick = 0;
		}

		if (slaveOHeatflag == 1) {
			if (SlaveOverHeattick == 0) SlaveOverHeattick = xTaskGetTickCount();
			else if(xTaskGetTickCount()-SlaveOverHeattick >= MAINBUFFER[HEAT_ERROR_DELAY]){
				MAINBUFFER[FAULTS] |= 1 << 9;
				faultLatch();
			}
			slaveOHeatflag = 0;

		} else{
			MAINBUFFER[FAULTS] &= ~(1 << 9);
			SlaveOverHeattick = 0;
		}

		if (overHeatflag == 1) {
			if (CellOverHeattick == 0) CellOverHeattick = xTaskGetTickCount();
			else if(xTaskGetTickCount()-CellOverHeattick >= MAINBUFFER[HEAT_ERROR_DELAY]){
				MAINBUFFER[FAULTS] |= 1 << 6;
				faultLatch();
			}
			overHeatflag = 0;

		} else{
			MAINBUFFER[FAULTS] &= ~(1 << 6);
			CellOverHeattick = 0;
		}

		if (underHeatflag == 1) {
			if (CellUnderHeattick == 0) CellUnderHeattick = xTaskGetTickCount();
			else if(xTaskGetTickCount()-CellUnderHeattick >= MAINBUFFER[HEAT_ERROR_DELAY]){
				MAINBUFFER[FAULTS] |= 1 << 5;
				faultLatch();
			}
			underHeatflag = 0;

		} else{
			MAINBUFFER[FAULTS] &= ~(1 << 5);
			CellUnderHeattick = 0;
		}

		faultLatch();
		vTaskDelay(20);

	}

}
static void vMeasure_Precharge(void *parameter) {

	uint8_t SCflag = 0;

	uint32_t prechargetick = 0;
	uint32_t PackOverVoltagetick = 0;
	uint32_t PackUnderVoltagetick = 0;
	uint32_t DischargeOverCurrenttick = 0;
	uint32_t ChargeOverCurrenttick = 0;
	uint32_t NoCurrentSensortick = 0;


	float current = 0;


	for (;;)
	{

		Get_ADC_Data();

		volt_bc = ((float) adc_data[0] - 56.15) / 5.083;
		volt_ac = ((float) adc_data[1] - 56.15) / 5.083;



		if(!IsSCActive()) //if SC is not active close the contactors
		{
			HAL_GPIO_WritePin(AIR__OUT_GPIO_Port, AIR__OUT_Pin, 0);
			MAINBUFFER[OUTPUTS] &= ~(1);
			SCflag = 0;
		}

	    if(IsSCActive() && SCflag == 0) //if sc is active for the first time
	    {
	    	if (volt_ac<=(volt_bc* (float)MAINBUFFER[PRECHARGE_PERCENTAGE]/100.0)){
	    		if(prechargetick == 0)//if timer is active for the first time
	    		{
	    			prechargetick=xTaskGetTickCount();
	    		}

	    		else if(xTaskGetTickCount()-prechargetick >= MAINBUFFER[PRECHARGE_TIMEOUT])
	    		{
	    			MAINBUFFER[FAULTS] |= 1 << 13;
	    			faultLatch();
	    			HAL_GPIO_WritePin(PRE_OUT_GPIO_Port, PRE_OUT_Pin, 0);
					MAINBUFFER[OUTPUTS] &= ~(1 << 1);
	    		}

				if ((((MAINBUFFER[FAULTS] >> 13) & 1) != 1)) {
		    		HAL_GPIO_WritePin(PRE_OUT_GPIO_Port, PRE_OUT_Pin, 1);
		    		MAINBUFFER[OUTPUTS] |= 1 << 1;
				}


	    	}else {
	    		prechargetick = 0;

	    		HAL_GPIO_WritePin(PRE_OUT_GPIO_Port, PRE_OUT_Pin, 0);
	    		MAINBUFFER[OUTPUTS] &= ~(1 << 1);

	    		HAL_GPIO_WritePin(AIR__OUT_GPIO_Port, AIR__OUT_Pin, 1);
	    		MAINBUFFER[OUTPUTS] |= 1 << 2;

	    		MAINBUFFER[FAULTS] &= ~(1 << 13);
	    		SCflag = 1;
	    	}
	    }

	    //MAINBUFFER LOADING
	    MAINBUFFER[PACK_VOLTAGE] = (uint16_t)(volt_bc*100);
	    if (current < 0.5 && current > -0.5) MAINBUFFER[PACK_CURRENT] = (uint16_t)(0*10);
	    else MAINBUFFER[PACK_CURRENT] = (uint16_t)(current*10);

	    //FAULT PROCESS
		if (((adc_data[2]*0.805)<301.0) || ((adc_data[2]*0.805)>3031.0) ){

			if(NoCurrentSensortick==0) NoCurrentSensortick=xTaskGetTickCount();
			else if(xTaskGetTickCount()-NoCurrentSensortick >= MAINBUFFER[OPEN_WIRE_ERROR_DELAY]){
				MAINBUFFER[FAULTS] |= 1 << 8;
				faultLatch();
			}

		}else{
			current = (((float)adc_data[2]*0.80586)-1667.32139)/3.10382376;
			NoCurrentSensortick=0;
			MAINBUFFER[FAULTS] &= ~(1 << 8);
		}

	    if (volt_bc > BATTERY_OVER_VOLTAGE_TRESHOLD)
	    {
	    	if(PackOverVoltagetick==0) PackOverVoltagetick=xTaskGetTickCount();
			else if(xTaskGetTickCount()-PackOverVoltagetick >= MAINBUFFER[OVER_VOLTAGE_ERROR_DELAY]){
		    	MAINBUFFER[FAULTS] |= 1 << 11;
				faultLatch();
			}

	    }else
	    {
	    	PackOverVoltagetick = 0;
	    	MAINBUFFER[FAULTS] &= ~(1 << 11);
	    }

	    if (volt_bc < BATTERY_UNDER_VOLTAGE_TRESHOLD)
	    {
	    	if(PackUnderVoltagetick==0) PackUnderVoltagetick=xTaskGetTickCount();
			else if(xTaskGetTickCount()-PackUnderVoltagetick >= MAINBUFFER[UNDER_VOLTAGE_ERROR_DELAY]){
		    	MAINBUFFER[FAULTS] |= 1 << 10;
				faultLatch();
			}

	    }else
	    {
	    	PackUnderVoltagetick = 0;
	    	MAINBUFFER[FAULTS] &= ~(1 << 10);
	    }
	    if (current > (double)MAINBUFFER[DISCHARGE_OVER_CURRENT_TRESHOLD]/10.0)
	    {
	    	if(DischargeOverCurrenttick==0) DischargeOverCurrenttick=xTaskGetTickCount();
			else if(xTaskGetTickCount()-DischargeOverCurrenttick >= MAINBUFFER[OVER_CURRENT_ERROR_DELAY]){
		    	MAINBUFFER[FAULTS] |= 1 << 3;
				faultLatch();
			}

	    }else
	    {
	    	DischargeOverCurrenttick = 0;
	    	MAINBUFFER[FAULTS] &= ~(1 << 3);
	    }

		if (current < ((double)(signed short int)MAINBUFFER[CHARGE_OVER_CURRENT_TRESHOLD])/10.0)
		{
	    	if(ChargeOverCurrenttick==0) ChargeOverCurrenttick=xTaskGetTickCount();
			else if(xTaskGetTickCount()-ChargeOverCurrenttick >= MAINBUFFER[OVER_CURRENT_ERROR_DELAY]){
				MAINBUFFER[FAULTS] |= 1 << 4;
				faultLatch();
			}

		}else
		{
			ChargeOverCurrenttick = 0;
			MAINBUFFER[FAULTS] &= ~(1 << 4);
		}


		faultLatch();
		vTaskDelay(5);

	}

}

static void vSoCTASK(void *parameter) {

	float SoC = 0;
	signed short int current;
	signed short int temperature;

	BatterySOCEstimationV2_initialize();

	Get_ADC_Data();

	rtU.OCV = (((float) adc_data[0] - 56.15) / 5.083)/total_ic*16.0; //OCV için araç açıldığı anda alınan voltaj değeri gerekli


	for(;;)
	{
		current = (signed short int) MAINBUFFER[PACK_CURRENT];
		temperature = (signed short int) MAINBUFFER[PACK_AVG_CELL_TEMP];

		//SoC Section
	    rtU.current = -1*((double)current)/10;
	    rtU.TerminalVoltage = ((double)MAINBUFFER[PACK_AVG_CELL_VOLTAGE])/100;
	    rtU.Temperature = ((double)temperature)/100;
	    BatterySOCEstimationV2_step();
	    SoC = rtY.SoC;

	    MAINBUFFER[OPEN_CIRCUIT_VOLTAGE] = (uint16_t)(rtU.OCV*100.0);
	    MAINBUFFER[ESTIMATED_SoC] = (uint16_t) (SoC*10000.0);


	    vTaskDelay(50);
	}
}

static void vChargeControl(void *parameter) {

	CHARG_STATE CHARGER = NO_CHARGER;

	uint8_t TXCharger[8];
    uint8_t BalanceEndflag = 1;

	for (;;)
	{
		//MAINBUFFER LOADING
		MAINBUFFER[CHARGING_STATE] = CHARGER;

		vTaskDelay(500);

		//IF WE COMMUNICATE WITH VCU, THERE IS NO NEED FOR THIS TASK
		if(VCUflag == 1)
		{
			vTaskDelete(NULL);
			vTaskDelay(portMAX_DELAY);
		}

		//CHECKING FOR ARE WE COMMUNICATING WITH CHARGER
		else if (VCUflag == 0 && Chargerflag == 0)
		{
			//baud rate değiştir
			HAL_CAN_Stop(&hcan1);
			HAL_CAN_DeInit(&hcan1);
		    hcan1.Init.Prescaler = 24;
		    hcan1.Init.TimeSeg1 = CAN_BS1_3TQ;
		    hcan1.Init.TimeSeg2 = CAN_BS2_1TQ;
			HAL_CAN_Init(&hcan1);
			HAL_CAN_Start(&hcan1);
			HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);

			ulTaskNotifyTake(pdFALSE,1500);

			if(Chargerflag == 0)
			{
				HAL_CAN_Stop(&hcan1);
				HAL_CAN_DeInit(&hcan1);
			    hcan1.Init.Prescaler = 12; //6;
			    hcan1.Init.TimeSeg1 = CAN_BS1_3TQ;
			    hcan1.Init.TimeSeg2 = CAN_BS2_1TQ;
				HAL_CAN_Init(&hcan1);
				HAL_CAN_Start(&hcan1);
				HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
			}


		}
		//IF WE COMMUNICATE WITH CHARGER GET IN
		else if(Chargerflag == 1)
		{
			if(IsSCActive())
			{
				if(CHARGER == NO_CHARGER) CHARGER = CHARGING;
			}

			switch(CHARGER)
			{

				case CHARGING:
				{

					TxHeader.IDE = CAN_ID_EXT;
					TxHeader.ExtId = 0x1806E5F4;
					TxHeader.DLC = 8;

					TXCharger[0]= (uint8_t)((MAINBUFFER[CHARGER_SET_VOLTAGE]/10) >> 8);
					TXCharger[1]= (uint8_t)(MAINBUFFER[CHARGER_SET_VOLTAGE]/10) ;
					TXCharger[2]= (uint8_t) (MAINBUFFER[CHARGER_SET_CURRENT] >> 8);
					TXCharger[3]= (uint8_t) MAINBUFFER[CHARGER_SET_CURRENT] ;
					TXCharger[4]= 0;
					TXCharger[5]= 0;
					TXCharger[6]= 0;
					TXCharger[7]= 0;
					HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TXCharger, &TxMailbox);

					MAINBUFFER[CHARGER_ACTUAL_VOLTAGE] = MAINBUFFER[PACK_VOLTAGE];
					MAINBUFFER[CHARGER_ACTUAL_CURRENT] = MAINBUFFER[PACK_CURRENT];

					if(RxData[4] != 0)
					{
						HAL_GPIO_WritePin(ERROR_OUT_GPIO_Port, ERROR_OUT_Pin, 0);
					}
					else HAL_GPIO_WritePin(ERROR_OUT_GPIO_Port, ERROR_OUT_Pin, 1);


					if((MAINBUFFER[PACK_VOLTAGE] >= MAINBUFFER[CHARGER_SET_VOLTAGE]-100)
						&& ((signed short int)MAINBUFFER[PACK_CURRENT] >= -1))
					{
						TxHeader.IDE = CAN_ID_EXT;
						TxHeader.ExtId = 0x1806E5F4;
						TxHeader.DLC = 8;
						TXCharger[0]= (uint8_t)((MAINBUFFER[CHARGER_SET_VOLTAGE]/10) >> 8);
						TXCharger[1]= (uint8_t)(MAINBUFFER[CHARGER_SET_VOLTAGE]/10) ;
						TXCharger[2]= (uint8_t) (MAINBUFFER[CHARGER_SET_CURRENT] >> 8);
						TXCharger[3]= (uint8_t) MAINBUFFER[CHARGER_SET_CURRENT] ;
						TXCharger[4]= 1;
						TXCharger[5]= 0;
						TXCharger[6]= 0;
						TXCharger[7]= 0;
						HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TXCharger, &TxMailbox);

						HAL_GPIO_WritePin(AIR__OUT_GPIO_Port, AIR__OUT_Pin, 0);

						CHARGER = BALANCING;
					}
					break;
				}
				case BALANCING:
				{

					for (uint8_t icCounter = 0; icCounter < total_ic; icCounter++) {
						for (uint8_t cellCounter = 0; cellCounter < 16; cellCounter++) {

							if((float)(MAINBUFFER[PACK_MAX_CELL_VOLTAGE]/100.0)-ic[icCounter].mesured_cell_voltages[cellCounter]
								> (float)MAINBUFFER[ALLOWED_DISBALANCE]/1000.0){

								ic[icCounter].tx_cfgb.dcc |= 1 << cellCounter ;
								BalanceEndflag = 0;

							}else ic[icCounter].tx_cfgb.dcc &= ~(1 << cellCounter);


						}
					}

					adBmsWakeupIc(total_ic);
					adBmsWriteData(total_ic, &ic[0], WRCFGA, Config, A);
					adBmsWriteData(total_ic, &ic[0], WRCFGB, Config, B);


					if(BalanceEndflag == 1)
					{
						CHARGER = CHARGING_COMPLETED;
					}

					BalanceEndflag = 1;

					break;
				}
				default:
				    break;

			}


		}





	}
}

static void vVCUcom(void *parameter) {

	union  CAN_Union {
		uint8_t sendBuffer8bit[8];
	    uint16_t sendBuffer16bit[4];
	}canSend;

	ulTaskNotifyTake(pdFALSE,portMAX_DELAY);


	for (;;)
	{
		if(RxHeader.StdId == VCU_ID)
		{
			if(RxData[0] == RaceMode || RxData[0]== NormalMode){
				VehicleMode = RxData[0];
			}
		}

		TxHeader.IDE = CAN_ID_STD;
		TxHeader.StdId = BMS_CAN_ID_VOLTAGES;
		TxHeader.DLC = 8;
		canSend.sendBuffer16bit[0] = MAINBUFFER[PACK_MAX_CELL_VOLTAGE];
		canSend.sendBuffer16bit[1] = MAINBUFFER[PACK_MIN_CELL_VOLTAGE];
		canSend.sendBuffer16bit[2] = MAINBUFFER[PACK_AVG_CELL_VOLTAGE];
		canSend.sendBuffer16bit[3] = MAINBUFFER[PACK_TOTAL_CELL_VOLTAGE];
		HAL_CAN_AddTxMessage(&hcan1, &TxHeader, canSend.sendBuffer8bit, &TxMailbox);

		vTaskDelay(1);

		TxHeader.StdId = BMS_CAN_ID_FAULTSandMODE;
		TxHeader.DLC = 3;
		canSend.sendBuffer16bit[0] = MAINBUFFER[FAULTS];
		canSend.sendBuffer16bit[1] = VehicleMode;
		HAL_CAN_AddTxMessage(&hcan1, &TxHeader, canSend.sendBuffer8bit, &TxMailbox);

		vTaskDelay(1);

		TxHeader.StdId = BMS_CAN_ID_TEMPS;
		TxHeader.DLC = 8;
		canSend.sendBuffer16bit[0] = MAINBUFFER[PACK_MAX_CELL_TEMP];
		canSend.sendBuffer16bit[1] = MAINBUFFER[PACK_MIN_CELL_TEMP];
		canSend.sendBuffer16bit[2] = MAINBUFFER[PACK_AVG_CELL_TEMP];
		canSend.sendBuffer16bit[3] = MAINBUFFER[PACK_MAX_SLAVE_TEMP];
		HAL_CAN_AddTxMessage(&hcan1, &TxHeader, canSend.sendBuffer8bit, &TxMailbox);

		vTaskDelay(1);


		TxHeader.StdId = BMS_CAN_ID_PACKandSoC;
		TxHeader.DLC = 8;
		canSend.sendBuffer16bit[0] = MAINBUFFER[PACK_CURRENT];
		canSend.sendBuffer16bit[1] = MAINBUFFER[PACK_VOLTAGE];
		canSend.sendBuffer16bit[2] = MAINBUFFER[ESTIMATED_SoC];
		canSend.sendBuffer16bit[3] = MAINBUFFER[OPEN_CIRCUIT_VOLTAGE];
		HAL_CAN_AddTxMessage(&hcan1, &TxHeader, canSend.sendBuffer8bit, &TxMailbox);


		vTaskDelay(5);

	}
}

static void vUSBTASK(void *parameter) {

	for (;;)
	{
		ulTaskNotifyTake(pdFALSE,portMAX_DELAY);

		switch (usblen){
		case 1:

			if (*usbbuf == 41){ //ALL VOLTAGES WILL BE SENT
				union data{
				        uint16_t twobytes[97];
				        uint8_t onebytes[194];
				} data;
				for (uint8_t icCounter = 0; icCounter < total_ic; icCounter++) {
					for (uint8_t cell = 0; cell < 16; cell++) {
						data.twobytes[((icCounter*16)+(cell))] = (uint16_t)(ic[icCounter].mesured_cell_voltages[cell]*100.0) ;
					}
				}
				data.onebytes[192] = 41;
				data.onebytes[193] = calculateCRC8(data.twobytes, 193);
				CDC_Transmit_FS(data.onebytes,194);
			}

			else if (*usbbuf == 42){ //ALL TEMPS WILL BE SENT
				union data{
				        uint16_t twobytes[97];
				        uint8_t onebytes[194];
				} data;
				for (uint8_t icCounter = 0; icCounter < total_ic; icCounter++) {
					for (uint8_t cell = 0; cell < 16; cell++) {
						data.twobytes[((icCounter*16)+(cell))] = (uint16_t)(ic[icCounter].mesured_cell_temps[cell]*100.0) ;
					}
				}
				data.onebytes[192] = 42;
				data.onebytes[193] = calculateCRC8(data.twobytes, 193);
				CDC_Transmit_FS(data.onebytes,194);
			}

			else if (*usbbuf == 43){ // BALANCE STATUS OF ALL CELL WILL BE SENT
				union data{
				        uint16_t twobytes[7];
				        uint8_t onebytes[14];
				} data;
				for (uint8_t icCounter = 0; icCounter < total_ic; icCounter++) {
					data.twobytes[icCounter] = ic[icCounter].tx_cfgb.dcc ;
				}
				data.onebytes[12] = 43;
				data.onebytes[13] = calculateCRC8(data.twobytes, 13);
				CDC_Transmit_FS(data.onebytes, 14);
			}

			else {
				USBTransmit2bytes(MAINBUFFER[*usbbuf] , *usbbuf);
			}

		    break;
		case 2:
			if (*usbbuf == 0x17 && *(usbbuf+1)==0x71)
			{
			  CDC_Transmit_FS((uint8_t*) usbbuf, 2);
			}
			break;
		case 3:
			MAINBUFFER[*usbbuf] = (*(usbbuf+2)<<8 | *(usbbuf+1));
			EEPROM_SetConfigs((uint8_t *)MAINBUFFER, sizeof(MAINBUFFER));
			EEPROM_GetConfigs((uint8_t *)MAINBUFFER, sizeof(MAINBUFFER));

			USBTransmit2bytes( MAINBUFFER[*usbbuf] , *usbbuf);
			//CDC_Transmit_FS((uint8_t*) &MAINBUFFER[*usbbuf], 2);
			break;

		default:
			break;

		}

	}

}

void HAL_ADC_ConvCpltCallback (ADC_HandleTypeDef *hadc)
{
	analogAveragingBuffer[convCount][0] = adc_buffer[0];
	analogAveragingBuffer[convCount][1] = adc_buffer[1];
	analogAveragingBuffer[convCount][2] = adc_buffer[2];

	convCount++;
	if(convCount >= 500) convCount = 0;
//	HAL_ADC_Start_DMA(&hadc1,  adc_buffer, 3);
}

void Get_ADC_Data(){

	for(uint8_t ucekadarsay =0; ucekadarsay<3; ucekadarsay++){
		adc_data[ucekadarsay]=0;
		for(uint16_t besyuzekadarsay =0; besyuzekadarsay<500; besyuzekadarsay++){
			adc_data[ucekadarsay] += analogAveragingBuffer[besyuzekadarsay][ucekadarsay];
		}
		adc_data[ucekadarsay]/=500;
	}
}

void USBTransmit2bytes( uint16_t TransmittBuffer , uint8_t transmittID){
	 union transmitt{
	        uint16_t twobytes[2];
	        uint8_t onebytes[4];
	    } transmitt;

	 transmitt.twobytes[0] =  TransmittBuffer ;
	 transmitt.onebytes[2] =  transmittID;
	 transmitt.onebytes[3] =  calculateCRC8(transmitt.onebytes, 3);


	CDC_Transmit_FS(transmitt.onebytes, 4);

}

uint8_t calculateCRC8(uint16_t *calculateData, uint8_t length) {
	 union data{
	        uint16_t twobytes[length/2];
	        uint8_t onebytes[length];
	    } data;

	memcpy(data.twobytes , calculateData ,length );
    uint8_t crc = 0x00;  // CRC başlangıç değeri
    for (size_t i = 0; i < length; i++) {
        crc ^= data.onebytes[i];  // Veri baytını CRC ile XOR yap
        for (uint8_t j = 0; j < 8; j++) {  // Her bir bit için döngü
            if (crc & 0x80) {  // Eğer en yüksek bit 1 ise
                crc = (crc << 1) ^ 0x07;  // CRC'yi sola kaydır ve polinom ile XOR yap
            } else {
                crc <<= 1;  // Sadece sola kaydır
            }
        }
    }
    return crc;  // Hesaplanan CRC8 değerini döndür
}


void faultLatch() {

	if(VehicleMode != RaceMode)
	{
		//OPEN WIRE FAULT
			if (MAINBUFFER[FAULTS]!=0 ) {
				HAL_GPIO_WritePin(ERROR_OUT_GPIO_Port, ERROR_OUT_Pin, 0);
				MAINBUFFER[OUTPUTS] |= 1 << 2;

			}

		    //NO FAULT HAPPENED
			else {
				HAL_GPIO_WritePin(ERROR_OUT_GPIO_Port, ERROR_OUT_Pin, 1);
				MAINBUFFER[OUTPUTS] &= ~(1 << 2);
			}

	}
	else {
		HAL_GPIO_WritePin(ERROR_OUT_GPIO_Port, ERROR_OUT_Pin, 1);
		MAINBUFFER[OUTPUTS] &= ~(1 << 2);
	}

}


uint8_t IsSCActive()
{
	return HAL_GPIO_ReadPin(SC_END_GPIO_Port, SC_END_Pin);
}

void USB_Handler(uint8_t *Buf, uint8_t Len) {

	BaseType_t pxHigherPriorityTaskWoken = pdFALSE;

	usbbuf = Buf;
	usblen = Len;
	vTaskNotifyGiveFromISR(USBTASK,&pxHigherPriorityTaskWoken);
	portYIELD_FROM_ISR(pxHigherPriorityTaskWoken);

}


int _write(int fd, char *ptr, int len) {
	HAL_UART_Transmit(&huart3, (uint8_t*) ptr, len, HAL_MAX_DELAY);
	return len;
}
/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
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

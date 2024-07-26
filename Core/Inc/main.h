/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.h
 * @brief          : Header for main.c file.
 *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
//configs

#define	FAULTS	0
#define	OUTPUTS	1
#define	CHARGING_STATE	2
#define	CHARGER_SET_VOLTAGE	3
#define	CHARGER_SET_CURRENT	4
#define	CHARGER_ACTUAL_VOLTAGE	5
#define	CHARGER_ACTUAL_CURRENT	6
#define	PACK_VOLTAGE	7
#define	PACK_CURRENT	8
#define	PACK_MAX_CELL_VOLTAGE	9
#define	PACK_MIN_CELL_VOLTAGE	10
#define	PACK_TOTAL_CELL_VOLTAGE	11
#define	PACK_MAX_CELL_TEMP	12
#define	PACK_MIN_CELL_TEMP	13
#define	PACK_AVG_CELL_VOLTAGE	14
#define	PACK_AVG_CELL_TEMP	15
#define	PACK_MAX_SLAVE_TEMP	16
#define ESTIMATED_SoC 17
#define OPEN_CIRCUIT_VOLTAGE  18
#define	ALLOWED_DISBALANCE	30
#define	CHARGE_CURRENT	31
#define	PRECHARGE_PERCENTAGE	32
#define	PRECHARGE_TIMEOUT	33
#define	CHARGE_OVER_CURRENT_TRESHOLD	34
#define	DISCHARGE_OVER_CURRENT_TRESHOLD	35
#define	OVER_VOLTAGE_ERROR_DELAY	36
#define	UNDER_VOLTAGE_ERROR_DELAY	37
#define	OVER_CURRENT_ERROR_DELAY	38
#define	OPEN_WIRE_ERROR_DELAY	39
#define	HEAT_ERROR_DELAY	40

//ADC BUFFER DEFINES
#define BEFORE_CONTACTOR_VOLTAGE 0
#define AFTER_CONTACTOR_VOLTAGE 1
#define CURRENT_HALL 2





//HARD CODED DEFINES
#define BATTERY_OVER_VOLTAGE_TRESHOLD 413
#define BATTERY_UNDER_VOLTAGE_TRESHOLD 40                 // test için değiştirildi normal değeri : 230
#define	CELL_OVER_VOLTAGE_TRESHOLD 4.3
#define	CELL_UNDER_VOLTAGE_TRESHOLD 2.4
#define	CELL_OVER_HEAT_TRESHOLD 60
#define	CELL_UNDER_HEAT_TRESHOLD -40
#define SLAVE_OVER_HEAT_TRESHOLD 70

#define BMS_CAN_ID_VOLTAGES 0x61
#define BMS_CAN_ID_FAULTSandMODE 0x62
#define BMS_CAN_ID_TEMPS 0x63
#define BMS_CAN_ID_PACKandSoC 0x64
#define VCU_ID  0x60
#define CHARGER_ID 0x18FF50E5
#define RaceMode 1
#define NormalMode 0
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
typedef enum
{
	NO_CHARGER = 1,
	CHARGING = 2,
	BALANCING = 3,
	CHARGING_COMPLETED = 4,
	CHARGING_ERROR = 5,

}CHARG_STATE;
/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */
#define GET_2S_COMPLEMENT(x) ((x) >= 0 ? (x) : (~((uint16_t)(-(x))) + 1))
/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
int _write(int file, char *ptr, int len);
void USB_Handler(uint8_t *Buf, uint8_t Len);
void USBTransmit( uint16_t TransmittBuffer);
void faultLatch();
uint8_t calculateCRC8(uint16_t *calculateData, uint8_t length);
void USBTransmit2bytes( uint16_t TransmittBuffer , uint8_t transmittID);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define SPI2_CS_Pin GPIO_PIN_0
#define SPI2_CS_GPIO_Port GPIOC
#define MCU_OCV_BC_Pin GPIO_PIN_1
#define MCU_OCV_BC_GPIO_Port GPIOA
#define MCU_OCV_AC_Pin GPIO_PIN_2
#define MCU_OCV_AC_GPIO_Port GPIOA
#define CURRENT_SHUNT_Pin GPIO_PIN_3
#define CURRENT_SHUNT_GPIO_Port GPIOA
#define CURRENT_HALL_Pin GPIO_PIN_4
#define CURRENT_HALL_GPIO_Port GPIOA
#define SC_END_Pin GPIO_PIN_4
#define SC_END_GPIO_Port GPIOC
#define ERROR_OUT_Pin GPIO_PIN_0
#define ERROR_OUT_GPIO_Port GPIOB
#define AIR__OUT_Pin GPIO_PIN_1
#define AIR__OUT_GPIO_Port GPIOB
#define PRE_OUT_Pin GPIO_PIN_2
#define PRE_OUT_GPIO_Port GPIOB
#define SPI1_CS_Pin GPIO_PIN_8
#define SPI1_CS_GPIO_Port GPIOC
#define LED_Pin GPIO_PIN_8
#define LED_GPIO_Port GPIOA

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

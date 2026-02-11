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
#include "stm32h7xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
	#include "cmsis_os.h"
	#include "FreeRTOS.h"
	#include "arm_math.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */
extern uint32_t time3;
extern uint32_t time4;
extern uint32_t time4print;
extern uint32_t time6;
	
extern float Ta_Force[2];
extern float Cu_Force[2];
extern float averageRMS[2];
extern int q1[2];
extern int q2[2];
extern int triggerStatus[2];
extern float EMG_Value[2], EMG_Max[2], EMG_Average[2], EMGinput[2], EMG_Normal[2];

typedef union {
    float32_t value;
    uint8_t byte[4];
} CANFloatData;

typedef struct {
		float AX_Float;        
		int16_t AX_int16;    

		float AY_Float;        
		int16_t AY_int16;    

		float AZ_Float;        
		int16_t AZ_int16;    

		float GX_Float;        
		int16_t GX_int16;    

		float GY_Float;        
		int16_t GY_int16;    

		float GZ_Float;        
		int16_t GZ_int16;    

		float HX_Float;        
		int16_t HX_int16;    

		float HY_Float;        
		int16_t HY_int16;    

		float HZ_Float;        
		int16_t HZ_int16;    

		float Roll_Float;        
		int16_t Roll_int16;    

		float Pitch_Float;        
		int16_t Pitch_int16;    

		float Yaw_Float;        
		int16_t Yaw_int16; 

		float Atmospheric_pressure_Float;
		int16_t Atmospheric_pressure_int16;
		
		float Height_Float;
		int16_t Height_int16;
		
} IMUDataStruct;

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/

/* USER CODE BEGIN Private defines */

#define FrameEndPointerOffset 123

#define M1CpointerOffset 5
#define M1VpointerOffset 21
#define M1PpointerOffset 37

#define M2CpointerOffset 5+48
#define M2VpointerOffset 21+48
#define M2PpointerOffset 37+48

#define LoadCell1pointerOffset 101
#define LoadCell2pointerOffset 103

#define EMG1pointerOffset 105
#define EMG2pointerOffset 107
#define JoystickStatePointerOffset 109

#define YearpointerOffset 110
#define MonthpointerOffset 112
#define DatepointerOffset 113
#define HourpointerOffset 114
#define MinutespointerOffset 115
#define SecondspointerOffset 116
#define MMSecondspointerOffset 117

#define FrameIDXpointerOffset 119

#define DCACHE_LINE_SIZE 32U
#define DCACHE_ALIGN(size)  (((size) + (DCACHE_LINE_SIZE - 1U)) & ~(DCACHE_LINE_SIZE - 1U))

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

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
#include "cmsis_os.h"
#include "adc.h"
#include "crc.h"
#include "dma.h"
#include "fdcan.h"
#include "i2c.h"
#include "rtc.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "MotorControl.h"
#include "atgm336h.h" 	
#include "arm_math.h"
#include "IIR.h"

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

/* USER CODE BEGIN PV */

extern UART_HandleTypeDef huart4;
extern UART_HandleTypeDef huart5;
extern UART_HandleTypeDef huart7;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;

extern DMA_HandleTypeDef hdma_uart4_rx;
extern DMA_HandleTypeDef hdma_uart4_tx;
extern DMA_HandleTypeDef hdma_uart5_rx;
extern DMA_HandleTypeDef hdma_uart5_tx;
extern DMA_HandleTypeDef hdma_uart7_rx;
extern DMA_HandleTypeDef hdma_uart7_tx;
extern DMA_HandleTypeDef hdma_usart2_rx;
extern DMA_HandleTypeDef hdma_usart2_tx;
extern DMA_HandleTypeDef hdma_uart8_rx;
extern DMA_HandleTypeDef hdma_usart3_tx;

extern uint8_t Motor1_CTRLBuffer[Motor_Buffer_Len];
uint8_t tempMotor1_CTRLBuffer[Motor_Buffer_Len];
extern uint8_t Motor1_Return_Data[32];

extern uint8_t Motor2_CTRLBuffer[Motor_Buffer_Len];
uint8_t tempMotor2_CTRLBuffer[Motor_Buffer_Len];
extern uint8_t Motor2_Return_Data[32];

extern uint32_t ADC_DATA[32];

extern uint8_t GPS_RXBuffer[GPS_RXBufferLen];

uint8_t ControllerIMUBuffer[100];
uint8_t ControllerIMURX_Flag=0;

extern osSemaphoreId ESPRetStateHandle;
extern osSemaphoreId GPSRetStateHandle;

extern osSemaphoreId Motor1RetStateHandle;
extern osSemaphoreId Motor2RetStateHandle;

extern uint8_t IMURX_Buffer[30];
uint8_t tempIMURX_Buffer[30];

uint8_t tempR_INSOLE_RX_Buffer[100];
uint8_t tempL_INSOLE_RX_Buffer[100];

uint8_t R_INSOLE_RX_Buffer[100];
uint8_t L_INSOLE_RX_Buffer[100];

uint32_t EMGSensor1ID=0x105;
uint32_t EMGSensor2ID=0x106;

uint8_t EMG1RMSValue[8]={0};
uint8_t EMG2RMSValue[8]={0};

extern uint8_t Motor1ReceiveFlag;
extern uint8_t Motor2ReceiveFlag;

uint32_t time3 = 0;
uint32_t time4 = 0;
uint32_t time4print = 0;
uint32_t time6 = 0;


uint32_t g_osRuntimeCounter;
volatile uint32_t CPU_RunTime;

extern RTC_TimeTypeDef rtcTime;
extern RTC_DateTypeDef rtcDate;

extern arm_biquad_casd_df1_inst_f32 LoadCell_LP_S1; 
extern arm_biquad_casd_df1_inst_f32 LoadCell_LP_S2; 

extern const float32_t IIRCoeffs32LP[5*LPnumStages];                   
extern float32_t ScaleValue_LP; 

extern float32_t IIRStateF32LP_LoadcellS1[4*LPnumStages];
extern float32_t IIRStateF32LP_LoadcellS2[4*LPnumStages];


uint32_t USART_RX_TIMESTAMP=0;

uint32_t USART_RX_TIMESTAMP_old=0;

uint32_t USART_RX_Period=0;

extern uint8_t CurRX_IMU;

extern uint8_t RightInsoleReceiveFlag;
extern uint8_t LeftInsoleReceiveFlag;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
void MX_FREERTOS_Init(void);
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

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

  /* Enable the CPU Cache */

  /* Enable I-Cache---------------------------------------------------------*/
  SCB_EnableICache();

  /* Enable D-Cache---------------------------------------------------------*/
  SCB_EnableDCache();

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
  MX_FDCAN1_Init();
  MX_RTC_Init();
  MX_ADC1_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM6_Init();
  MX_TIM17_Init();
  MX_UART4_Init();
  MX_UART5_Init();
  MX_UART7_Init();
  MX_CRC_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_UART8_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
	arm_biquad_cascade_df1_init_f32(&LoadCell_LP_S1, LPnumStages, (float32_t *)&IIRCoeffs32LP[0],(float32_t *)&IIRStateF32LP_LoadcellS1[0]);
	arm_biquad_cascade_df1_init_f32(&LoadCell_LP_S2, LPnumStages, (float32_t *)&IIRCoeffs32LP[0],(float32_t *)&IIRStateF32LP_LoadcellS2[0]);
	
	
	HAL_TIM_Base_Start_IT(&htim17);
	HAL_TIM_Base_Start_IT(&htim4);
	
	
	HAL_ADCEx_Calibration_Start(&hadc1,ADC_CALIB_OFFSET,ADC_SINGLE_ENDED);
	HAL_ADC_Start_DMA(&hadc1, (uint32_t *)ADC_DATA, 32);
	
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);
	
	HAL_UARTEx_ReceiveToIdle_IT(&huart5,tempMotor1_CTRLBuffer,Motor_Buffer_Len); 
	__HAL_UART_ENABLE_IT(&huart5, UART_IT_ERR);
//	//Receive the return state of motor 1
	
	HAL_UARTEx_ReceiveToIdle_IT(&huart7,tempMotor2_CTRLBuffer,Motor_Buffer_Len); //Receive the return state of motor 2
	__HAL_UART_ENABLE_IT(&huart7, UART_IT_ERR);

	
	HAL_UARTEx_ReceiveToIdle_IT(&huart8,tempR_INSOLE_RX_Buffer,100);  //Receive the GPS information
	__HAL_UART_ENABLE_IT(&huart8, UART_IT_ERR);
	
		HAL_UARTEx_ReceiveToIdle_IT(&huart3,tempIMURX_Buffer,29); //Receive the IMU information
	__HAL_UART_ENABLE_IT(&huart3, UART_IT_ERR);
	
	HAL_UARTEx_ReceiveToIdle_IT(&huart4,tempL_INSOLE_RX_Buffer,100); //Receive the IMU information
	__HAL_UART_ENABLE_IT(&huart4, UART_IT_ERR);

	FDCAN1_Config();
	HAL_RTC_GetTime(&hrtc,&rtcTime,RTC_FORMAT_BIN);
	HAL_RTC_GetDate(&hrtc,&rtcDate,RTC_FORMAT_BIN);
	


	arm_iir_init_LP();

	
  /* USER CODE END 2 */

  /* Call init function for freertos objects (in cmsis_os2.c) */
  MX_FREERTOS_Init();

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  __HAL_RCC_SYSCFG_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE|RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 5;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 12;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_2;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
	uint32_t RX_PackNum=0;

	uint32_t tmp;
	osStatus xReturn;
	if(huart->Instance==UART5)
	{
		RX_PackNum++;
		memcpy(Motor1_CTRLBuffer, tempMotor1_CTRLBuffer, Motor_Buffer_Len);
		HAL_UARTEx_ReceiveToIdle_IT(&huart5,tempMotor1_CTRLBuffer,Motor_Buffer_Len); //Receive the return state of motor 1
		__HAL_UNLOCK(&huart5);
		Motor1ReceiveFlag=1;

			
	}
		if(huart->Instance==UART7)
	{
		memcpy(Motor2_CTRLBuffer, tempMotor2_CTRLBuffer, Motor_Buffer_Len);
		HAL_UARTEx_ReceiveToIdle_IT(&huart7,tempMotor2_CTRLBuffer,Motor_Buffer_Len); //Receive the return state of motor 2
		__HAL_UNLOCK(&huart7);
		Motor2ReceiveFlag=1;
	}

	
	if(huart->Instance==UART8)
	{
		memcpy(R_INSOLE_RX_Buffer, tempR_INSOLE_RX_Buffer, 100);
		HAL_UARTEx_ReceiveToIdle_IT(&huart8,tempR_INSOLE_RX_Buffer,100); 
		__HAL_UNLOCK(&huart8);
		RightInsoleReceiveFlag=1;

		
	}
	
	
		if(huart->Instance==USART3)
	{


		memcpy(IMURX_Buffer, tempIMURX_Buffer, 29);
		CurRX_IMU=IMURX_Buffer[0];
		HAL_UARTEx_ReceiveToIdle_IT(&huart3,&tempIMURX_Buffer[0],29); 
		
	}
	

			if(huart->Instance==UART4)
	{
		memcpy(L_INSOLE_RX_Buffer, tempL_INSOLE_RX_Buffer, 100);
		HAL_UARTEx_ReceiveToIdle_IT(&huart4,&tempL_INSOLE_RX_Buffer[0],100); 
		__HAL_UNLOCK(&huart4);
		LeftInsoleReceiveFlag=1;
		
	}
	
}


extern uint32_t g_osRuntimeCounter;
void configureTimerForRunTimeStats(void)
{
g_osRuntimeCounter = 0;
}
 
unsigned long getRunTimeCounterValue(void)
{
//return 0;
	return g_osRuntimeCounter;
}


/* USER CODE END 4 */

 /* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x0;
  MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
  MPU_InitStruct.SubRegionDisable = 0x87;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */
	
	  if (htim->Instance == TIM17) {
    g_osRuntimeCounter++;
		}
		
		

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

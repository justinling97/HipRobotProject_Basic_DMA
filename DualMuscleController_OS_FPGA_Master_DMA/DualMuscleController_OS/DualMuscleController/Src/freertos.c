/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include "arm_math.h"
#include "MotorControl.h"
#include "atgm336h.h"
#include "pid.h"
#include "stdarg.h"	
#include "MB85RC1M.h"
#include "IIR.h"
#include "wit_IMU.h"
#include "wit_c_sdk.h"
#include "ControllerIMU.h"
#include "FPGA_motor_driver.h"
#include "BandwidthTest.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

#define ACC_UPDATE		0x01
#define GYRO_UPDATE		0x02
#define ANGLE_UPDATE	0x04
#define MAG_UPDATE		0x08
#define READ_UPDATE		0x80

static volatile char s_cDataUpdate = 0, s_cCmd = 0xff;
const uint32_t c_uiBaud[10] = {0, 4800, 9600, 19200, 38400, 57600, 115200, 230400, 460800, 921600};
uint8_t IMURX_Buffer[29];
extern uint8_t IMU_TX_Buf[8];
extern uint8_t tempIMURX_Buffer[30];
uint8_t CurRX_IMU=0x00;

IMUDataStruct IMU_R;
IMUDataStruct IMU_L;
IMUDataStruct IMU_Controller;

#define MaxCurrentLim 1000

#define HipRobot
//#define HipRobot2


/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

//#define SBS_Left_Mode_ON
//#define SBS_Left_Mode_OFF

#define SBS_Right_Mode_ON
//#define SBS_Right_Mode_OFF

//#define SOS_Mode_ON
//#define SOS_Mode_OFF


static float ONforceBias=5;

static float OFFforceBias=60;


float forceBias=5;


////#define forcetest
//#define EMG


//#define demo
#define force_track

#define LoadCellBufferLen 50

//For Loadcell
extern ADC_HandleTypeDef hadc1;
extern uint32_t ADC_DATA[32];

float32_t LoadCell1Buffer[LoadCellBufferLen]={0.0};
float32_t LoadCell2Buffer[LoadCellBufferLen]={0.0};

float32_t LoadCell1Voltage_filtered[2]={0.0};
float32_t LoadCell2Voltage_filtered[2]={0.0};


uint8_t curLoadCellBufferPointer=0;
float32_t LoadCell1Voltage[2]={0};
float32_t LoadCell2Voltage[2]={0};


float32_t LoadCellForce[2]={0};
uint16_t LoadCellForceU16[2]={0};
float Cu_Force[2] = {0};
float Ta_Force[2] = {0};
int q1_Motor[2] = {0};
int q2_Motor[2] = {0};
int triggerStatus[2]  = {0};
float EMG_Value[2] = {0}, EMG_Max[2] = {0}, EMG_Average[2] = {0}, EMGinput[2] = {0}, EMG_Normal[2] = {0};
//For MotorControl

__attribute__((at(0x38000580)))	  uint8_t _dbg_Buff[150];

uint8_t RemotePinState[3]={0};


uint8_t ControlState[2] = {0};
uint8_t ControlState_old[2] = {0};

extern int Motor1_PresentCurrent[4];
extern int Motor1_PresentPosition[4];
extern int Motor1_PresentVelocity[4];

extern int Motor2_PresentCurrent[4];
extern int Motor2_PresentPosition[4];
extern int Motor2_PresentVelocity[4];

uint8_t TempStateRetStr[100]={0};

extern uint8_t Motor1_CTRLBuffer[Motor_Buffer_Len];
extern uint8_t Motor1_Return_Data[32];

extern uint8_t Motor2_CTRLBuffer[Motor_Buffer_Len];
extern uint8_t Motor2_Return_Data[32];
uint8_t Motor1ReceiveFlag=0;
uint8_t Motor2ReceiveFlag=0;

uint8_t RightInsoleReceiveFlag=0;
uint8_t LeftInsoleReceiveFlag=0;

extern uint8_t EMG1RMSValue[8];
extern uint8_t EMG2RMSValue[8];
//For State return
uint8_t HelloServer[]={"Hello Server \r\n"};

extern uint8_t GPS_RXBuffer[GPS_RXBufferLen];
uint8_t UTC_CANSync[8]={0};
nmea_msg nmea_msg_STRUCT;

extern RTC_HandleTypeDef hrtc;

uint8_t CAN_Current[8]={0};
uint8_t ControllerIP[4]={0,0,0,0};
uint8_t ESP32InitFinishedFlag=0;

uint8_t ESPSendBuffer[5][300]={0};

uint8_t CPU_RunInfo[400];

extern RTC_TimeTypeDef rtcTime;
extern RTC_DateTypeDef rtcDate;

uint16_t curMSEC=0;

uint32_t SendFrameIDX=0;

extern arm_biquad_casd_df1_inst_f32 LoadCell_LP_S1; 
extern arm_biquad_casd_df1_inst_f32 LoadCell_LP_S2; 

extern const float32_t IIRCoeffs32LP[5*LPnumStages];                   

extern float32_t ScaleValue_LP; 

extern float32_t IIRStateF32LP_LoadcellS1[4*LPnumStages]; 
extern float32_t IIRStateF32LP_LoadcellS2[4*LPnumStages]; 

extern uint8_t CANTXmessage[8];

extern CANFloatData TKEO1;
extern CANFloatData TKEO2;

extern CANFloatData RMS1;
extern CANFloatData RMS2;

CANFloatData tempCANFloat;

uint8_t WData[4]={0x1,0x2,0x3,0x4};        
uint32_t WDataLen=4;

uint8_t RData[4]={0};       
uint32_t RDataLen=4;

struct CurrentStruct M1,M2,M3,M4,M5,M6,M7,M8;

const uint16_t MB85RC1M_TKEO_Coef_Addr = 0x000;   
const uint16_t MB85RC1M_TKEO_Bias_Addr = 0x004;   

const uint16_t MB85RC1M_RMS_Coef_Addr = 0x008;   
const uint16_t MB85RC1M_RMS_Bias_Addr = 0x0012;   

const uint16_t MB85RC1M_M1Position_Addr = 0x0016+4*0;   
const uint16_t MB85RC1M_M2Position_Addr = 0x0016+4*1;   
const uint16_t MB85RC1M_M3Position_Addr = 0x0016+4*2;   
const uint16_t MB85RC1M_M4Position_Addr = 0x0016+4*3;   
const uint16_t MB85RC1M_M5Position_Addr = 0x0016+4*4;   
const uint16_t MB85RC1M_M6Position_Addr = 0x0016+4*5;   
const uint16_t MB85RC1M_M7Position_Addr = 0x0016+4*6;   
const uint16_t MB85RC1M_M8Position_Addr = 0x0016+4*7;  

int Motor1_InitialPosition[4] = {0};
int Motor2_InitialPosition[4] = {0};


extern uint32_t USART_RX_TIMESTAMP;

uint8_t GaitTriggerEvent[2]={0};

extern uint16_t GRF_INSOLE1[3];
extern uint16_t GRF_INSOLE2[3];

extern uint8_t ControllerIMURX_Flag;
extern uint8_t ControllerIMUBuffer[100];

uint8_t 			CRC_H=0;
uint8_t 			CRC_L=0;


uint32_t IMU_R_Enq_Num=0;
uint32_t IMU_L_Enq_Num=0;
uint32_t IMU_CTL_Enq_Num=0;

uint32_t IMU_R_RSP_Num=0;
uint32_t IMU_L_RSP_Num=0;
uint32_t IMU_CTL_RSP_Num=0;

extern arm_fir_instance_f32 FIR_S1;
extern arm_fir_instance_f32 FIR_S2;

float PC0_VolVal=0;
float PA5_VolVal=0;

float32_t LoadCell1Buffer_Filtered[1]={0};
float32_t LoadCell2Buffer_Filtered[1]={0};
float LoadCellForce1_f=0;
float controlOutput_TSA1=0;

float LoadCellForce2_f=0;
float controlOutput_TSA2=0;

int16_t DesireForce=0;


extern MotorFrame g_Set_motorStatus_Group1;

extern MotorFrame g_motorStatus_feedback_Group1;


extern float multisine_signal[6001];
extern float EMG_Ref_Signal[1000];


float curPositionTarget_TSA1=0;

uint32_t idx=0;
uint32_t idx_offset=200;
float curPositionTarget=0;
uint64_t globalTime=0;

PID_Controller TSA1_PID_Struct;
PID_Controller TSA2_PID_Struct;

extern uint8_t R_INSOLE_RX_Buffer[100];
extern uint8_t L_INSOLE_RX_Buffer[100];
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
osThreadId TaskManagerHandle;
osThreadId TSA1ContollHandle;
osThreadId TSA2ContollHandle;
osThreadId MotorStatDecodeHandle;
osThreadId IMU_TaskHandle;
osThreadId StateRETHandle;
osTimerId LoadCellCALHandle;
osTimerId GetRTCTimeHandle;
osTimerId TaForceCALHandle;
osTimerId ControllerPWRHandle;
osTimerId CurrentPeriodCTRLHandle;
osTimerId IMU_EnqHandle;
osSemaphoreId ESPRetStateHandle;
osSemaphoreId GPSRetStateHandle;
osSemaphoreId Motor1RetStateHandle;
osSemaphoreId Motor2RetStateHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */


static void CopeSensorData(uint32_t uiReg, uint32_t uiRegNum);
static void AutoScanSensor(void);
void CopeCmdData(unsigned char ucData);
static void SensorUartSend(uint8_t *p_data, uint32_t uiSize);
static void Delayms(uint16_t ucMs);
static void CmdProcess(void);
uint16_t IMU_CRC16(uint8_t *puchMsg, uint16_t usDataLen);
/* USER CODE END FunctionPrototypes */

void TaskManagerFUN(void const * argument);
void TSA1ContollFUN(void const * argument);
void TSA2ContollFUN(void const * argument);
void MotorStatDecodeFUN(void const * argument);
void IMU_TaskFUN(void const * argument);
void StateRETFUN(void const * argument);
void LoadCellCALFUN(void const * argument);
void GetRTCTimeFUN(void const * argument);
void TaForceCALCallback(void const * argument);
void CTRLLERPWR(void const * argument);
void CurrentPeriodCTRLFUN(void const * argument);
void IMU_EnqFUN(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* GetTimerTaskMemory prototype (linked to static allocation support) */
void vApplicationGetTimerTaskMemory( StaticTask_t **ppxTimerTaskTCBBuffer, StackType_t **ppxTimerTaskStackBuffer, uint32_t *pulTimerTaskStackSize );

/* Hook prototypes */
void configureTimerForRunTimeStats(void);
unsigned long getRunTimeCounterValue(void);

/* USER CODE BEGIN 1 */
/* Functions needed when configGENERATE_RUN_TIME_STATS is on */
__weak void configureTimerForRunTimeStats(void)
{

}

__weak unsigned long getRunTimeCounterValue(void)
{
return 0;
}
/* USER CODE END 1 */

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

/* USER CODE BEGIN GET_TIMER_TASK_MEMORY */
static StaticTask_t xTimerTaskTCBBuffer;
static StackType_t xTimerStack[configTIMER_TASK_STACK_DEPTH];

void vApplicationGetTimerTaskMemory( StaticTask_t **ppxTimerTaskTCBBuffer, StackType_t **ppxTimerTaskStackBuffer, uint32_t *pulTimerTaskStackSize )
{
  *ppxTimerTaskTCBBuffer = &xTimerTaskTCBBuffer;
  *ppxTimerTaskStackBuffer = &xTimerStack[0];
  *pulTimerTaskStackSize = configTIMER_TASK_STACK_DEPTH;
  /* place for user code */
}
/* USER CODE END GET_TIMER_TASK_MEMORY */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* definition and creation of ESPRetState */
  osSemaphoreDef(ESPRetState);
  ESPRetStateHandle = osSemaphoreCreate(osSemaphore(ESPRetState), 1);

  /* definition and creation of GPSRetState */
  osSemaphoreDef(GPSRetState);
  GPSRetStateHandle = osSemaphoreCreate(osSemaphore(GPSRetState), 1);

  /* definition and creation of Motor1RetState */
  osSemaphoreDef(Motor1RetState);
  Motor1RetStateHandle = osSemaphoreCreate(osSemaphore(Motor1RetState), 1);

  /* definition and creation of Motor2RetState */
  osSemaphoreDef(Motor2RetState);
  Motor2RetStateHandle = osSemaphoreCreate(osSemaphore(Motor2RetState), 1);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* Create the timer(s) */
  /* definition and creation of LoadCellCAL */
  osTimerDef(LoadCellCAL, LoadCellCALFUN);
  LoadCellCALHandle = osTimerCreate(osTimer(LoadCellCAL), osTimerPeriodic, NULL);

  /* definition and creation of GetRTCTime */
  osTimerDef(GetRTCTime, GetRTCTimeFUN);
  GetRTCTimeHandle = osTimerCreate(osTimer(GetRTCTime), osTimerPeriodic, NULL);

  /* definition and creation of TaForceCAL */
  osTimerDef(TaForceCAL, TaForceCALCallback);
  TaForceCALHandle = osTimerCreate(osTimer(TaForceCAL), osTimerPeriodic, NULL);

  /* definition and creation of ControllerPWR */
  osTimerDef(ControllerPWR, CTRLLERPWR);
  ControllerPWRHandle = osTimerCreate(osTimer(ControllerPWR), osTimerPeriodic, NULL);

  /* definition and creation of CurrentPeriodCTRL */
  osTimerDef(CurrentPeriodCTRL, CurrentPeriodCTRLFUN);
  CurrentPeriodCTRLHandle = osTimerCreate(osTimer(CurrentPeriodCTRL), osTimerPeriodic, NULL);

  /* definition and creation of IMU_Enq */
  osTimerDef(IMU_Enq, IMU_EnqFUN);
  IMU_EnqHandle = osTimerCreate(osTimer(IMU_Enq), osTimerPeriodic, NULL);

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
	
	//osTimerStart(GetRTCTimeHandle,10);
	osTimerStart(LoadCellCALHandle,1);
	osTimerStart(TaForceCALHandle,5);
	
	osTimerStart(ControllerPWRHandle,500);
	osTimerStart(CurrentPeriodCTRLHandle,10);
	
	
	
	
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of TaskManager */
  osThreadDef(TaskManager, TaskManagerFUN, osPriorityNormal, 0, 128);
  TaskManagerHandle = osThreadCreate(osThread(TaskManager), NULL);

//  /* definition and creation of TSA1Contoll */
//  osThreadDef(TSA1Contoll, TSA1ContollFUN, osPriorityIdle, 0, 128);
//  TSA1ContollHandle = osThreadCreate(osThread(TSA1Contoll), NULL);

  /* definition and creation of TSA2Contoll */
  osThreadDef(TSA2Contoll, TSA2ContollFUN, osPriorityIdle, 0, 128);
  TSA2ContollHandle = osThreadCreate(osThread(TSA2Contoll), NULL);

  /* definition and creation of MotorStatDecode */
  osThreadDef(MotorStatDecode, MotorStatDecodeFUN, osPriorityNormal, 0, 128);
  MotorStatDecodeHandle = osThreadCreate(osThread(MotorStatDecode), NULL);

  /* definition and creation of IMU_Task */
  osThreadDef(IMU_Task, IMU_TaskFUN, osPriorityIdle, 0, 128);
  IMU_TaskHandle = osThreadCreate(osThread(IMU_Task), NULL);

  /* definition and creation of StateRET */
  osThreadDef(StateRET, StateRETFUN, osPriorityIdle, 0, 128);
  StateRETHandle = osThreadCreate(osThread(StateRET), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_TaskManagerFUN */
/**
  * @brief  Function implementing the TaskManager thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_TaskManagerFUN */
void TaskManagerFUN(void const * argument)
{
  /* USER CODE BEGIN TaskManagerFUN */

	
	
	
	
	
	uint8_t Format0[]={"  B:Block  R:Ready  D:Del  S:Paused  X:Running \r\n"};
	uint8_t Format1[]={"---------------------------------------------\r\n"};
	uint8_t Format2[]={"Task      Task_Counter       CPU_Usage\r\n"};
	uint8_t Format3[]={"---------------------------------------------\r\n"};
	uint8_t Format4[]={"ESP32 State\r\n"};
	
  /* Infinite loop */
  for(;;)
  {

		
		
				vTaskGetRunTimeStats((char *)&CPU_RunInfo); 
				__HAL_UNLOCK(&huart4);	
				osDelay(10);
				HAL_UART_Transmit_IT(&huart4, (uint8_t *)Format1, sizeof(Format1));	
				osDelay(10);
				HAL_UART_Transmit_IT(&huart4, (uint8_t *)Format4, sizeof(Format4));	
				osDelay(10);
				HAL_UART_Transmit_IT(&huart4, (uint8_t *)Format1, sizeof(Format1));		
				osDelay(10);
				HAL_UART_Transmit_IT(&huart4, (uint8_t *)Format1, sizeof(Format1));	
				osDelay(10);
				HAL_UART_Transmit_IT(&huart4, (uint8_t *)Format2, sizeof(Format2));	
				osDelay(10);;
				HAL_UART_Transmit_IT(&huart4, (uint8_t *)Format3, sizeof(Format3));	
				osDelay(10);
				HAL_UART_Transmit_IT(&huart4, (uint8_t *)CPU_RunInfo, 400);				
				osDelay(1000);
				memset(CPU_RunInfo,0,400);
  }
  /* USER CODE END TaskManagerFUN */
}

/* USER CODE BEGIN Header_TSA1ContollFUN */
/**
* @brief Function implementing the TSA1Contoll thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_TSA1ContollFUN */
void TSA1ContollFUN(void const * argument)
{
  /* USER CODE BEGIN TSA1ContollFUN */
	
	
	/* USER CODE BEGIN TSA1ContollFUN */
	
		float kp=0.030, ki=0.0000, kd=0.000;
		float OutMin=-2000,OutMax=2000;
	//
	
		int32_t CurrentUpperLimit=MaxCurrentLim;
		int32_t PosCurrentUpLimit=CurrentUpperLimit;
		int32_t NegCurrentUpLimit=-CurrentUpperLimit;
		
		InitMotorControl_Group1();       // ��ʼ���������
		PID_Init(&TSA1_PID_Struct, kp, ki, kd,OutMin, OutMax);
		TSA1_PID_Struct.setpoint=20;
	
		uint8_t PositionFlag=0;
		
	uint16_t curPositionIDX=0;
	
	
  /* Infinite loop */
  for(;;)
  {
		if(GRF_INSOLE1[0]==0x01){
			GaitTriggerEvent[0]=1;
		}
		else{
			GaitTriggerEvent[0]=0;
		}
		
		RemotePinState[0]=HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_8);
		RemotePinState[1]=HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_9);
		RemotePinState[2]=HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_10);

		if(RemotePinState[0]==0x01 && RemotePinState[1]==0x01 && RemotePinState[2]==0x01){//D button pressed
			ControlState[0]=0;
			if(PositionFlag==0){
			SetMotorMode_Group1(0, MOTOR_MODE_POSITION);  	// ����ģʽ
			SetMotorPositionTurns_Group1(0, 0.0f);        	// ����λ��Ϊ10.0fȦ
			SetMotorSpeedRPM_Group1     (0, 4000.0f);		// �����ٶ�Ϊ800.0fRPM
			SetMotorCurrentAmpere_Group1(0, 3.0f);       	// ���õ�������Ϊ0.5fA
				
			SetMotorMode_Group1(1, MOTOR_MODE_POSITION);  	// ����ģʽ
			SetMotorPositionTurns_Group1(1, 0.0f);        	// ����λ��Ϊ10.0fȦ
			SetMotorSpeedRPM_Group1     (1, 4000.0f);		// �����ٶ�Ϊ800.0fRPM
			SetMotorCurrentAmpere_Group1(1, 3.0f);       	// ���õ�������Ϊ0.5fA
				
			SetMotorMode_Group1(2, MOTOR_MODE_POSITION);  	// ����ģʽ
			SetMotorPositionTurns_Group1(2, 0.0f);        	// ����λ��Ϊ10.0fȦ
			SetMotorSpeedRPM_Group1     (2, 4000.0f);		// �����ٶ�Ϊ800.0fRPM
			SetMotorCurrentAmpere_Group1(2, 3.0f);       	// ���õ�������Ϊ0.5fA
				
			SetMotorMode_Group1(3, MOTOR_MODE_POSITION);  	// ����ģʽ
			SetMotorPositionTurns_Group1(3, 0.0f);        	// ����λ��Ϊ10.0fȦ
			SetMotorSpeedRPM_Group1     (3, 4000.0f);		// �����ٶ�Ϊ800.0fRPM
			SetMotorCurrentAmpere_Group1(3, 3.0f);       	// ���õ�������Ϊ0.5fA
			SendMotorControlFrame_Group1();  
				PositionFlag=1;
			}
			curPositionIDX=0;

			
		}
		else if (RemotePinState[0]==0x01 && RemotePinState[1]==0x01 && RemotePinState[2]==0x00){//C button pressed
			ControlState[0]=1;

		StopAllMotors_Group1();
		SendMotorControlFrame_Group1(); 
		PositionFlag=0;
		}
		else if (RemotePinState[0]==0x00 && RemotePinState[1]==0x01 && RemotePinState[2]==0x01){//A button pressed
			ControlState[0]=2;

			
			SetMotorMode_Group1(0, MOTOR_MODE_CURRENT);  	// ����ģʽ
			SetMotorPositionTurns_Group1(0, 0);        	// ����λ��Ϊ10.0fȦ
			SetMotorSpeedRPM_Group1     (0, 5000);		// �����ٶ�Ϊ800.0fRPM
			SetMotorCurrentAmpere_Group1(0, controlOutput_TSA1);       	// ���õ�������Ϊ0.5fA
			
			SetMotorMode_Group1(1, MOTOR_MODE_CURRENT);  	// ����ģʽ
			SetMotorPositionTurns_Group1(1, 0);        	// ����λ��Ϊ10.0fȦ
			SetMotorSpeedRPM_Group1     (1, 5000);		// �����ٶ�Ϊ800.0fRPM
			SetMotorCurrentAmpere_Group1(1, controlOutput_TSA1);       	// ���õ�������Ϊ0.5fA
			
			SetMotorMode_Group1(2, MOTOR_MODE_CURRENT);  	// ����ģʽ
			SetMotorPositionTurns_Group1(2, 0);        	// ����λ��Ϊ10.0fȦ
			SetMotorSpeedRPM_Group1     (2, 5000);		// �����ٶ�Ϊ800.0fRPM
			SetMotorCurrentAmpere_Group1(2, controlOutput_TSA1);       	// ���õ�������Ϊ0.5fA
			
			SetMotorMode_Group1(3, MOTOR_MODE_CURRENT);  	// ����ģʽ
			SetMotorPositionTurns_Group1(3, 0);        	// ����λ��Ϊ10.0fȦ
			SetMotorSpeedRPM_Group1     (3, 5000);		// �����ٶ�Ϊ800.0fRPM
			SetMotorCurrentAmpere_Group1(3, controlOutput_TSA1);       	// ���õ�������Ϊ0.5fA
			SendMotorControlFrame_Group1(); 
			PositionFlag=0;
		}
		else if (RemotePinState[0]==0x01 && RemotePinState[1]==0x00 && RemotePinState[2]==0x01){//B button pressed
			ControlState[0]=3;
			
			SetMotorMode_Group1(0, MOTOR_MODE_SPEED);  	// ����ģʽ
			SetMotorPositionTurns_Group1(0, 0.0f);        	// ����λ��Ϊ10.0fȦ
			SetMotorSpeedRPM_Group1     (0, -5000.0f);		// �����ٶ�Ϊ800.0fRPM
			SetMotorCurrentAmpere_Group1(0, 5.0f);       	// ���õ�������Ϊ0.5fA
			
			SetMotorMode_Group1(1, MOTOR_MODE_SPEED);  	// ����ģʽ
			SetMotorPositionTurns_Group1(1, 0.0f);        	// ����λ��Ϊ10.0fȦ
			SetMotorSpeedRPM_Group1     (1, -5000.0f);		// �����ٶ�Ϊ800.0fRPM
			SetMotorCurrentAmpere_Group1(1, 5.0f);       	// ���õ�������Ϊ0.5fA
			
			SetMotorMode_Group1(2, MOTOR_MODE_SPEED);  	// ����ģʽ
			SetMotorPositionTurns_Group1(2, 0.0f);        	// ����λ��Ϊ10.0fȦ
			SetMotorSpeedRPM_Group1     (2, -5000.0f);		// �����ٶ�Ϊ800.0fRPM
			SetMotorCurrentAmpere_Group1(2, 5.0f);       	// ���õ�������Ϊ0.5fA
			
			SetMotorMode_Group1(3, MOTOR_MODE_SPEED);  	// ����ģʽ
			SetMotorPositionTurns_Group1(3, 0.0f);        	// ����λ��Ϊ10.0fȦ
			SetMotorSpeedRPM_Group1     (3, -5000.0f);		// �����ٶ�Ϊ800.0fRPM
			SetMotorCurrentAmpere_Group1(3,5.0f);       	// ���õ�������Ϊ0.5fA
			SendMotorControlFrame_Group1(); 
			PositionFlag=0;
		}
		else{
			ControlState[0]=4;
		}
		
    osDelay(5);
  }
  /* USER CODE END TSA1ContollFUN */
}

/* USER CODE BEGIN Header_TSA2ContollFUN */
/**
* @brief Function implementing the TSA2Contoll thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_TSA2ContollFUN */
uint16_t RefEMG_IDX=0;
uint16_t CycleNum=0;
void TSA2ContollFUN(void const * argument)
{
  /* USER CODE BEGIN TSA2ContollFUN */
	uint8_t PositionFlag=0;
	uint16_t curPositionIDX=0;
	
	float kp=0.080, ki=0.0000, kd=0.001;
	float OutMin=-2000,OutMax=2000;
	PID_Init(&TSA2_PID_Struct, kp, ki, kd,OutMin, OutMax);
	TSA2_PID_Struct.setpoint=20;
	float EMG_Old=0;
	float EMG_Diff=0;
	
	float Gain=60/0.5;
	float CycleTime=0.5;
	
	uint16_t PretwistTime=200*5;//Cycles
	float f1=0;
	
	uint16_t CycleEndNum=CycleTime*100/5;
	
	float FeedforwardGain=100;
	
  /* Infinite loop */
  for(;;)
  {


		RemotePinState[0]=HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_8);
		RemotePinState[1]=HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_9);
		RemotePinState[2]=HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_10);

		if(RemotePinState[0]==0x01 && RemotePinState[1]==0x01 && RemotePinState[2]==0x01){//D button pressed
			ControlState[1]=0;
			if(PositionFlag==0){
			SetMotorMode_Group2(0, MOTOR_MODE_POSITION);  	// ����ģʽ
			SetMotorPositionTurns_Group2(0, 0.0f);        	// ����λ��Ϊ10.0fȦ
			SetMotorSpeedRPM_Group2     (0, 4000.0f);		// �����ٶ�Ϊ800.0fRPM
			SetMotorCurrentAmpere_Group2(0, 3.0f);       	// ���õ�������Ϊ0.5fA
				
			SetMotorMode_Group2(1, MOTOR_MODE_POSITION);  	// ����ģʽ
			SetMotorPositionTurns_Group2(1, 0.0f);        	// ����λ��Ϊ10.0fȦ
			SetMotorSpeedRPM_Group2     (1, 4000.0f);		// �����ٶ�Ϊ800.0fRPM
			SetMotorCurrentAmpere_Group2(1, 3.0f);       	// ���õ�������Ϊ0.5fA
				
			SetMotorMode_Group2(2, MOTOR_MODE_POSITION);  	// ����ģʽ
			SetMotorPositionTurns_Group2(2, 0.0f);        	// ����λ��Ϊ10.0fȦ
			SetMotorSpeedRPM_Group2     (2, 4000.0f);		// �����ٶ�Ϊ800.0fRPM
			SetMotorCurrentAmpere_Group2(2, 3.0f);       	// ���õ�������Ϊ0.5fA
				
			SetMotorMode_Group2(3, MOTOR_MODE_POSITION);  	// ����ģʽ
			SetMotorPositionTurns_Group2(3, 0.0f);        	// ����λ��Ϊ10.0fȦ
			SetMotorSpeedRPM_Group2     (3, 4000.0f);		// �����ٶ�Ϊ800.0fRPM
			SetMotorCurrentAmpere_Group2(3, 3.0f);       	// ���õ�������Ϊ0.5fA
			SendMotorControlFrame_Group2();  
				PositionFlag=1;
			}

			
		}
		else if (RemotePinState[0]==0x01 && RemotePinState[1]==0x01 && RemotePinState[2]==0x00){//C button pressed
			ControlState[1]=1;

		StopAllMotors_Group2();
		SendMotorControlFrame_Group2(); 
		PositionFlag=0;
		}
		else if (RemotePinState[0]==0x00 && RemotePinState[1]==0x01 && RemotePinState[2]==0x01){//A button pressed
			ControlState[1]=2;
			
			
//			CycleNum++;
//			//multisine_signal[1000]
//			//EMG_Ref_Signal[0]
//			if(CycleNum<PretwistTime){
//				//TSA2_PID_Struct.setpoint=EMG_Ref_Signal[0]*Gain;
//				TSA2_PID_Struct.setpoint=(multisine_signal[0]+1)*30;
//			}
//			else if(CycleNum>PretwistTime && RefEMG_IDX<6000){
////				if(CycleNum%2==0){
////				RefEMG_IDX=RefEMG_IDX+1;
////				}
//				
//				RefEMG_IDX=RefEMG_IDX+1;
//				//TSA2_PID_Struct.setpoint=EMG_Ref_Signal[RefEMG_IDX]*Gain;
//				
//				TSA2_PID_Struct.setpoint=(multisine_signal[RefEMG_IDX]+1)*30;
//				
//			}
//			else{
//				TSA2_PID_Struct.setpoint=0;
//			}
			//	DesireForce=(uint16_t)TSA2_PID_Struct.setpoint;	

			TSA2_PID_Struct.setpoint=(float)(EMG2RMSValue[0]-100);	
			
			if(TSA2_PID_Struct.setpoint<10){
				TSA2_PID_Struct.setpoint=10;
			}
			EMG_Diff=EMG_Old-TSA2_PID_Struct.setpoint;
			EMG_Old=TSA2_PID_Struct.setpoint;
			
			
			SetMotorMode_Group2(0, MOTOR_MODE_CURRENT);  	// ����ģʽ
			SetMotorPositionTurns_Group2(0, 0);        	// ����λ��Ϊ10.0fȦ
			SetMotorSpeedRPM_Group2     (0, 5000);		// �����ٶ�Ϊ800.0fRPM
			SetMotorCurrentAmpere_Group2(0, controlOutput_TSA2+EMG_Diff*FeedforwardGain);       	// ���õ�������Ϊ0.5fA
			
			SetMotorMode_Group2(1, MOTOR_MODE_CURRENT);  	// ����ģʽ
			SetMotorPositionTurns_Group2(1, 0);        	// ����λ��Ϊ10.0fȦ
			SetMotorSpeedRPM_Group2     (1, 5000);		// �����ٶ�Ϊ800.0fRPM
			SetMotorCurrentAmpere_Group2(1, controlOutput_TSA2+EMG_Diff*FeedforwardGain);       	// ���õ�������Ϊ0.5fA
			
			SetMotorMode_Group2(2, MOTOR_MODE_CURRENT);  	// ����ģʽ
			SetMotorPositionTurns_Group2(2, 0);        	// ����λ��Ϊ10.0fȦ
			SetMotorSpeedRPM_Group2     (2, 5000);		// �����ٶ�Ϊ800.0fRPM
			SetMotorCurrentAmpere_Group2(2, controlOutput_TSA2+EMG_Diff*FeedforwardGain);       	// ���õ�������Ϊ0.5fA
			
			SetMotorMode_Group2(3, MOTOR_MODE_CURRENT);  	// ����ģʽ
			SetMotorPositionTurns_Group2(3, 0);        	// ����λ��Ϊ10.0fȦ
			SetMotorSpeedRPM_Group2     (3, 5000);		// �����ٶ�Ϊ800.0fRPM
			SetMotorCurrentAmpere_Group2(3, controlOutput_TSA2+EMG_Diff*FeedforwardGain);       	// ���õ�������Ϊ0.5fA
			SendMotorControlFrame_Group2(); 

			PositionFlag=0;
		}
		else if (RemotePinState[0]==0x01 && RemotePinState[1]==0x00 && RemotePinState[2]==0x01){//B button pressed
			ControlState[1]=3;
			
			SetMotorMode_Group2(0, MOTOR_MODE_SPEED);  	// ����ģʽ
			SetMotorPositionTurns_Group2(0, 0.0f);        	// ����λ��Ϊ10.0fȦ
			SetMotorSpeedRPM_Group2     (0, -2000.0f);		// �����ٶ�Ϊ800.0fRPM
			SetMotorCurrentAmpere_Group2(0, 5.0f);       	// ���õ�������Ϊ0.5fA
			
			SetMotorMode_Group2(1, MOTOR_MODE_SPEED);  	// ����ģʽ
			SetMotorPositionTurns_Group2(1, 0.0f);        	// ����λ��Ϊ10.0fȦ
			SetMotorSpeedRPM_Group2     (1, -2000.0f);		// �����ٶ�Ϊ800.0fRPM
			SetMotorCurrentAmpere_Group2(1, 5.0f);       	// ���õ�������Ϊ0.5fA
			
			SetMotorMode_Group2(2, MOTOR_MODE_SPEED);  	// ����ģʽ
			SetMotorPositionTurns_Group2(2, 0.0f);        	// ����λ��Ϊ10.0fȦ
			SetMotorSpeedRPM_Group2     (2, -2000.0f);		// �����ٶ�Ϊ800.0fRPM
			SetMotorCurrentAmpere_Group2(2, 5.0f);       	// ���õ�������Ϊ0.5fA
			
			SetMotorMode_Group2(3, MOTOR_MODE_SPEED);  	// ����ģʽ
			SetMotorPositionTurns_Group2(3, 0.0f);        	// ����λ��Ϊ10.0fȦ
			SetMotorSpeedRPM_Group2     (3, -2000.0f);		// �����ٶ�Ϊ800.0fRPM
			SetMotorCurrentAmpere_Group2(3,5.0f);       	// ���õ�������Ϊ0.5fA
			SendMotorControlFrame_Group2(); 
			PositionFlag=0;
		}
		else{
			ControlState[1]=4;
		}
		
    osDelay(5);

  }
  /* USER CODE END TSA2ContollFUN */
}

/* USER CODE BEGIN Header_MotorStatDecodeFUN */
/**
* @brief Function implementing the MotorStatDecode thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_MotorStatDecodeFUN */
uint16_t RightPressureSum=0;
uint16_t LeftPressureSum=0;
uint16_t RightPressureSum_old=0;
uint16_t LeftPressureSum_old=0;

uint16_t RightInsoleCRC=0;
uint16_t LeftInsoleCRC=0;
void MotorStatDecodeFUN(void const * argument)
{
  /* USER CODE BEGIN MotorStatDecodeFUN */
	uint32_t SendCNT1=0;
	uint32_t SendCNT2=0;
	
	RTC_TimeTypeDef currtcTime;
	RTC_DateTypeDef currtcDate;



//  /* Infinite loop */
  for(;;)
  {
		if(Motor1ReceiveFlag){
			SendCNT1++;
			ProcessFeedbackFrame_Group1(&Motor1_CTRLBuffer[0]);
			memset(TempStateRetStr,0,100);
			Motor1ReceiveFlag=0;
		}
		
		if(Motor2ReceiveFlag){
			SendCNT1++;
			ProcessFeedbackFrame_Group2(&Motor2_CTRLBuffer[0]);
			memset(TempStateRetStr,0,100);
			Motor2ReceiveFlag=0;
		}
		
		if(RightInsoleReceiveFlag){
			RightPressureSum=0;
			for(int InsoleIDX=0;InsoleIDX<98;InsoleIDX++){
				RightInsoleCRC=RightInsoleCRC+R_INSOLE_RX_Buffer[InsoleIDX];
				if(InsoleIDX>1 && InsoleIDX<98){
					RightPressureSum=RightPressureSum+R_INSOLE_RX_Buffer[InsoleIDX];
				}
			}
			RightInsoleCRC=RightInsoleCRC&0x00ff;
			if(RightInsoleCRC==(uint16_t)R_INSOLE_RX_Buffer[98]){
				RightPressureSum_old=RightPressureSum;			}
			else{
				RightPressureSum=RightPressureSum_old;			}
			
			memset(R_INSOLE_RX_Buffer,0,100);
			RightInsoleCRC=0;
			RightInsoleReceiveFlag=0;
		}
		
		if(LeftInsoleReceiveFlag){
			LeftPressureSum=0;
			for(int LInsoleIDX=0;LInsoleIDX<98;LInsoleIDX++){
				LeftInsoleCRC=LeftInsoleCRC+L_INSOLE_RX_Buffer[LInsoleIDX];
				if(LInsoleIDX>1 && LInsoleIDX<98){
					LeftPressureSum=LeftPressureSum+L_INSOLE_RX_Buffer[LInsoleIDX];
				}
			}
			LeftInsoleCRC=LeftInsoleCRC&0x00ff;
			if(LeftInsoleCRC==(uint16_t)L_INSOLE_RX_Buffer[98]){
				LeftPressureSum_old=LeftPressureSum;			}
			else{
				LeftPressureSum=LeftPressureSum_old;			}
			
			memset(L_INSOLE_RX_Buffer,0,100);
			LeftInsoleCRC=0;
			LeftInsoleReceiveFlag=0;
		}

    osDelay(1);
  }
  /* USER CODE END MotorStatDecodeFUN */
}

/* USER CODE BEGIN Header_IMU_TaskFUN */
/**
* @brief Function implementing the IMU_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_IMU_TaskFUN */
void IMU_TaskFUN(void const * argument)
{
  /* USER CODE BEGIN IMU_TaskFUN */
	uint32_t TikTok=0;


	uint16_t curCRC_Val=0;

	
	
	WitInit(WIT_PROTOCOL_MODBUS, 0x50);
	WitSerialWriteRegister(SensorUartSend);
	WitRegisterCallBack(CopeSensorData);
	WitDelayMsRegister(Delayms);

  /* Infinite loop */
  for(;;)
  {
		
		
		__HAL_UNLOCK(&huart3);
		__HAL_UART_ENABLE_IT(&huart3, UART_IT_ERR);
		
		
		TikTok++;
		if(TikTok==1){
			WitReadReg(0x50,AX, 12);
			HAL_UART_Transmit_IT(&huart3, IMU_TX_Buf, 8);
			IMU_R_Enq_Num++;
		}
		else if(TikTok==2){
			WitReadReg(0x51,AX, 12);
			HAL_UART_Transmit_IT(&huart3, IMU_TX_Buf, 8);
			IMU_L_Enq_Num++;
		}
		else if(TikTok==3){
			WitReadReg(0x53,AX, 12);
			HAL_UART_Transmit_IT(&huart3, IMU_TX_Buf, 8);
			IMU_CTL_Enq_Num++;
		}
		else if(TikTok==4){
			//Empty
		}
		else{
			TikTok=0;
		}
		
		if(CurRX_IMU!=0){
			
			curCRC_Val=IMU_CRC16(&IMURX_Buffer[0], 27);
			CRC_H=(uint8_t)((curCRC_Val&0xff00)>>8);
			CRC_L=(uint8_t)((curCRC_Val&0x00ff));
			if((CRC_H==IMURX_Buffer[27])&&(CRC_L==IMURX_Buffer[28])){
				if(IMURX_Buffer[0]==0x50){
					
					IMU_R.AX_int16=(IMURX_Buffer[3]<<8)+IMURX_Buffer[4];
					IMU_R.AX_Float=((float)IMU_R.AX_int16)*0.0048828125;
					IMU_R.AY_int16=(IMURX_Buffer[5]<<8)+IMURX_Buffer[6];
					IMU_R.AY_Float=((float)IMU_R.AY_int16)*0.0048828125;
					IMU_R.AZ_int16=(IMURX_Buffer[7]<<8)+IMURX_Buffer[8];
					IMU_R.AZ_Float=((float)IMU_R.AZ_int16)*0.00478515625;
					
					
					IMU_R.GX_int16=(IMURX_Buffer[9]<<8)+IMURX_Buffer[10];
					IMU_R.GX_Float=((float)IMU_R.GX_int16)*0.06103515625;
					IMU_R.GY_int16=(IMURX_Buffer[11]<<8)+IMURX_Buffer[12];
					IMU_R.GY_Float=((float)IMU_R.GY_int16)*0.06103515625;
					IMU_R.GZ_int16=(IMURX_Buffer[13]<<8)+IMURX_Buffer[14];
					IMU_R.GZ_Float=((float)IMU_R.GZ_int16)*0.06103515625;
					
					IMU_R.HX_int16=(IMURX_Buffer[15]<<8)+IMURX_Buffer[16];
					IMU_R.HX_Float=((float)IMU_R.HX_int16);
					IMU_R.HY_int16=(IMURX_Buffer[17]<<8)+IMURX_Buffer[18];
					IMU_R.HY_Float=((float)IMU_R.HY_int16);
					IMU_R.HZ_int16=(IMURX_Buffer[19]<<8)+IMURX_Buffer[20];
					IMU_R.HZ_Float=((float)IMU_R.HZ_int16);
					
					IMU_R.Roll_int16=(IMURX_Buffer[21]<<8)+IMURX_Buffer[22];
					IMU_R.Roll_Float=((float)IMU_R.Roll_int16)*0.0054931640625;
					IMU_R.Pitch_int16=(IMURX_Buffer[23]<<8)+IMURX_Buffer[24];
					IMU_R.Pitch_Float=((float)IMU_R.Pitch_int16)*0.0054931640625;
					IMU_R.Yaw_int16=(IMURX_Buffer[25]<<8)+IMURX_Buffer[26];
					IMU_R.Yaw_Float=((float)IMU_R.Yaw_int16)*0.0054931640625;
					IMU_R_RSP_Num++;
					memset(IMURX_Buffer,0,30);
					CurRX_IMU=0;
					}
				else if(IMURX_Buffer[0]==0x51){
					
					IMU_L.AX_int16=(IMURX_Buffer[3]<<8)+IMURX_Buffer[4];
					IMU_L.AX_Float=((float)IMU_L.AX_int16)*0.0048828125;
					IMU_L.AY_int16=(IMURX_Buffer[5]<<8)+IMURX_Buffer[6];
					IMU_L.AY_Float=((float)IMU_L.AY_int16)*0.0048828125;
					IMU_L.AZ_int16=(IMURX_Buffer[7]<<8)+IMURX_Buffer[8];
					IMU_L.AZ_Float=((float)IMU_L.AZ_int16)*0.00478515625;
					
					
					IMU_L.GX_int16=(IMURX_Buffer[9]<<8)+IMURX_Buffer[10];
					IMU_L.GX_Float=((float)IMU_L.GX_int16)*0.06103515625;
					IMU_L.GY_int16=(IMURX_Buffer[11]<<8)+IMURX_Buffer[12];
					IMU_L.GY_Float=((float)IMU_L.GY_int16)*0.06103515625;
					IMU_L.GZ_int16=(IMURX_Buffer[13]<<8)+IMURX_Buffer[14];
					IMU_L.GZ_Float=((float)IMU_L.GZ_int16)*0.06103515625;
					
					IMU_L.HX_int16=(IMURX_Buffer[15]<<8)+IMURX_Buffer[16];
					IMU_L.HX_Float=((float)IMU_L.HX_int16);
					IMU_L.HY_int16=(IMURX_Buffer[17]<<8)+IMURX_Buffer[18];
					IMU_L.HY_Float=((float)IMU_L.HY_int16);
					IMU_L.HZ_int16=(IMURX_Buffer[19]<<8)+IMURX_Buffer[20];
					IMU_L.HZ_Float=((float)IMU_L.HZ_int16);
					
					IMU_L.Roll_int16=(IMURX_Buffer[21]<<8)+IMURX_Buffer[22];
					IMU_L.Roll_Float=((float)IMU_L.Roll_int16)*0.0054931640625;
					IMU_L.Pitch_int16=(IMURX_Buffer[23]<<8)+IMURX_Buffer[24];
					IMU_L.Pitch_Float=((float)IMU_L.Pitch_int16)*0.0054931640625;
					IMU_L.Yaw_int16=(IMURX_Buffer[25]<<8)+IMURX_Buffer[26];
					IMU_L.Yaw_Float=((float)IMU_L.Yaw_int16)*0.0054931640625;
					IMU_L_RSP_Num++;
					memset(IMURX_Buffer,0,30);
					CurRX_IMU=0;
					}
				else if(IMURX_Buffer[0]==0x53){
					
					IMU_Controller.AX_int16=(IMURX_Buffer[3]<<8)+IMURX_Buffer[4];
					IMU_Controller.AX_Float=((float)IMU_Controller.AX_int16)*0.0048828125;
					IMU_Controller.AY_int16=(IMURX_Buffer[5]<<8)+IMURX_Buffer[6];
					IMU_Controller.AY_Float=((float)IMU_Controller.AY_int16)*0.0048828125;
					IMU_Controller.AZ_int16=(IMURX_Buffer[7]<<8)+IMURX_Buffer[8];
					IMU_Controller.AZ_Float=((float)IMU_Controller.AZ_int16)*0.00478515625;
					
					
					IMU_Controller.GX_int16=(IMURX_Buffer[9]<<8)+IMURX_Buffer[10];
					IMU_Controller.GX_Float=((float)IMU_Controller.GX_int16)*0.06103515625;
					IMU_Controller.GY_int16=(IMURX_Buffer[11]<<8)+IMURX_Buffer[12];
					IMU_Controller.GY_Float=((float)IMU_Controller.GY_int16)*0.06103515625;
					IMU_Controller.GZ_int16=(IMURX_Buffer[13]<<8)+IMURX_Buffer[14];
					IMU_Controller.GZ_Float=((float)IMU_Controller.GZ_int16)*0.06103515625;
					
					IMU_Controller.HX_int16=(IMURX_Buffer[15]<<8)+IMURX_Buffer[16];
					IMU_Controller.HX_Float=((float)IMU_Controller.HX_int16);
					IMU_Controller.HY_int16=(IMURX_Buffer[17]<<8)+IMURX_Buffer[18];
					IMU_Controller.HY_Float=((float)IMU_Controller.HY_int16);
					IMU_Controller.HZ_int16=(IMURX_Buffer[19]<<8)+IMURX_Buffer[20];
					IMU_Controller.HZ_Float=((float)IMU_Controller.HZ_int16);
					
					IMU_Controller.Roll_int16=(IMURX_Buffer[21]<<8)+IMURX_Buffer[22];
					IMU_Controller.Roll_Float=((float)IMU_Controller.Roll_int16)*0.0054931640625;
					IMU_Controller.Pitch_int16=(IMURX_Buffer[23]<<8)+IMURX_Buffer[24];
					IMU_Controller.Pitch_Float=((float)IMU_Controller.Pitch_int16)*0.0054931640625;
					IMU_Controller.Yaw_int16=(IMURX_Buffer[25]<<8)+IMURX_Buffer[26];
					IMU_Controller.Yaw_Float=((float)IMU_Controller.Yaw_int16)*0.0054931640625;
					IMU_CTL_RSP_Num++;
					memset(IMURX_Buffer,0,30);
					CurRX_IMU=0;
					}
			}

		}


		osDelay(10);

    
  }
  /* USER CODE END IMU_TaskFUN */
}

/* USER CODE BEGIN Header_StateRETFUN */
/**
* @brief Function implementing the StateRET thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StateRETFUN */
void StateRETFUN(void const * argument)
{
  /* USER CODE BEGIN StateRETFUN */
	uint32_t RET_TikTok=0;
	int32_t int32_t_tempData=0;
	
	CANFloatData tempFloatData;
  /* Infinite loop */
  for(;;)
  {
		
		if(ControlState[0]!=ControlState_old[0]){
			ControlState_old[0]=ControlState[0];
			CAN_senddata(0x001,&ControlState[0],FDCAN_DLC_BYTES_2);
		}
		
		if(ControlState[1]!=ControlState_old[1]){
			ControlState_old[1]=ControlState[1];
			CAN_senddata(0x001,&ControlState[0],FDCAN_DLC_BYTES_2);
		}
		
		RET_TikTok++;
		
		if(RET_TikTok%2==0){
		//==================M1Current===============================
			
		tempFloatData.value=g_motorStatus_feedback_Group1.actual_motors[0].current_ampere;
			
		CANTXmessage[0]=tempFloatData.byte[0];
		CANTXmessage[1]=tempFloatData.byte[1];
		CANTXmessage[2]=tempFloatData.byte[2];
		CANTXmessage[3]=tempFloatData.byte[3];

		CAN_senddata(0x318,&CANTXmessage[0],FDCAN_DLC_BYTES_4);

		tempFloatData.value=g_motorStatus_feedback_Group1.actual_motors[1].current_ampere;
			
		CANTXmessage[0]=tempFloatData.byte[0];
		CANTXmessage[1]=tempFloatData.byte[1];
		CANTXmessage[2]=tempFloatData.byte[2];
		CANTXmessage[3]=tempFloatData.byte[3];

		CAN_senddata(0x319,&CANTXmessage[0],FDCAN_DLC_BYTES_4);

		tempFloatData.value=g_motorStatus_feedback_Group1.actual_motors[2].current_ampere;
			
		CANTXmessage[0]=tempFloatData.byte[0];
		CANTXmessage[1]=tempFloatData.byte[1];
		CANTXmessage[2]=tempFloatData.byte[2];
		CANTXmessage[3]=tempFloatData.byte[3];

		CAN_senddata(0x31A,&CANTXmessage[0],FDCAN_DLC_BYTES_4);

		tempFloatData.value=g_motorStatus_feedback_Group1.actual_motors[3].current_ampere;
			
		CANTXmessage[0]=tempFloatData.byte[0];
		CANTXmessage[1]=tempFloatData.byte[1];
		CANTXmessage[2]=tempFloatData.byte[2];
		CANTXmessage[3]=tempFloatData.byte[3];

		CAN_senddata(0x31B,&CANTXmessage[0],FDCAN_DLC_BYTES_4);

		//==================M1Velosity===============================


		tempFloatData.value=g_motorStatus_feedback_Group1.actual_motors[0].speed_rpm;
			
		CANTXmessage[0]=tempFloatData.byte[0];
		CANTXmessage[1]=tempFloatData.byte[1];
		CANTXmessage[2]=tempFloatData.byte[2];
		CANTXmessage[3]=tempFloatData.byte[3];

		CAN_senddata(0x314,&CANTXmessage[0],FDCAN_DLC_BYTES_4);

		tempFloatData.value=g_motorStatus_feedback_Group1.actual_motors[1].speed_rpm;
			
		CANTXmessage[0]=tempFloatData.byte[0];
		CANTXmessage[1]=tempFloatData.byte[1];
		CANTXmessage[2]=tempFloatData.byte[2];
		CANTXmessage[3]=tempFloatData.byte[3];

		CAN_senddata(0x315,&CANTXmessage[0],FDCAN_DLC_BYTES_4);

		tempFloatData.value=g_motorStatus_feedback_Group1.actual_motors[2].speed_rpm;
			
		CANTXmessage[0]=tempFloatData.byte[0];
		CANTXmessage[1]=tempFloatData.byte[1];
		CANTXmessage[2]=tempFloatData.byte[2];
		CANTXmessage[3]=tempFloatData.byte[3];

		CAN_senddata(0x316,&CANTXmessage[0],FDCAN_DLC_BYTES_4);

		tempFloatData.value=g_motorStatus_feedback_Group1.actual_motors[3].speed_rpm;
			
		CANTXmessage[0]=tempFloatData.byte[0];
		CANTXmessage[1]=tempFloatData.byte[1];
		CANTXmessage[2]=tempFloatData.byte[2];
		CANTXmessage[3]=tempFloatData.byte[3];

		CAN_senddata(0x317,&CANTXmessage[0],FDCAN_DLC_BYTES_4);

		//==================M1Position===============================


		tempFloatData.value=g_motorStatus_feedback_Group1.actual_motors[0].position_turns;
			
		CANTXmessage[0]=tempFloatData.byte[0];
		CANTXmessage[1]=tempFloatData.byte[1];
		CANTXmessage[2]=tempFloatData.byte[2];
		CANTXmessage[3]=tempFloatData.byte[3];

		CAN_senddata(0x310,&CANTXmessage[0],FDCAN_DLC_BYTES_4);

		tempFloatData.value=g_motorStatus_feedback_Group1.actual_motors[1].position_turns;
			
		CANTXmessage[0]=tempFloatData.byte[0];
		CANTXmessage[1]=tempFloatData.byte[1];
		CANTXmessage[2]=tempFloatData.byte[2];
		CANTXmessage[3]=tempFloatData.byte[3];

		CAN_senddata(0x311,&CANTXmessage[0],FDCAN_DLC_BYTES_4);

		tempFloatData.value=g_motorStatus_feedback_Group1.actual_motors[2].position_turns;
			
		CANTXmessage[0]=tempFloatData.byte[0];
		CANTXmessage[1]=tempFloatData.byte[1];
		CANTXmessage[2]=tempFloatData.byte[2];
		CANTXmessage[3]=tempFloatData.byte[3];

		CAN_senddata(0x312,&CANTXmessage[0],FDCAN_DLC_BYTES_4);

		tempFloatData.value=g_motorStatus_feedback_Group1.actual_motors[3].position_turns;
			
		CANTXmessage[0]=tempFloatData.byte[0];
		CANTXmessage[1]=tempFloatData.byte[1];
		CANTXmessage[2]=tempFloatData.byte[2];
		CANTXmessage[3]=tempFloatData.byte[3];

		CAN_senddata(0x313,&CANTXmessage[0],FDCAN_DLC_BYTES_4);

/*

//		//==================M2Current===============================

//		CANTXmessage[0]=(Motor2_PresentCurrent[0]>>24)&0x000000ff;
//		CANTXmessage[1]=(Motor2_PresentCurrent[0]>>16)&0x000000ff;
//		CANTXmessage[2]=(Motor2_PresentCurrent[0]>>8)&0x000000ff;
//		CANTXmessage[3]=(Motor2_PresentCurrent[0])&0x000000ff;

//		CAN_senddata(0x328,&CANTXmessage[0],FDCAN_DLC_BYTES_4);

//		CANTXmessage[4]=(Motor2_PresentCurrent[1]>>24)&0x000000ff;
//		CANTXmessage[5]=(Motor2_PresentCurrent[1]>>16)&0x000000ff;
//		CANTXmessage[6]=(Motor2_PresentCurrent[1]>>8)&0x000000ff;
//		CANTXmessage[7]=(Motor2_PresentCurrent[1])&0x000000ff;

//		CAN_senddata(0x329,&CANTXmessage[0],FDCAN_DLC_BYTES_4);

//		CANTXmessage[0]=(Motor2_PresentCurrent[2]>>24)&0x000000ff;
//		CANTXmessage[1]=(Motor2_PresentCurrent[2]>>16)&0x000000ff;
//		CANTXmessage[2]=(Motor2_PresentCurrent[2]>>8)&0x000000ff;
//		CANTXmessage[3]=(Motor2_PresentCurrent[2])&0x000000ff;

//		CAN_senddata(0x32A,&CANTXmessage[0],FDCAN_DLC_BYTES_4);

//		CANTXmessage[4]=(Motor2_PresentCurrent[3]>>24)&0x000000ff;
//		CANTXmessage[5]=(Motor2_PresentCurrent[3]>>16)&0x000000ff;
//		CANTXmessage[6]=(Motor2_PresentCurrent[3]>>8)&0x000000ff;
//		CANTXmessage[7]=(Motor2_PresentCurrent[3])&0x000000ff;

//		CAN_senddata(0x32B,&CANTXmessage[0],FDCAN_DLC_BYTES_4);

//		//==================M2Velosity===============================


//		CANTXmessage[0]=(Motor2_PresentVelocity[0]>>24)&0x000000ff;
//		CANTXmessage[1]=(Motor2_PresentVelocity[0]>>16)&0x000000ff;
//		CANTXmessage[2]=(Motor2_PresentVelocity[0]>>8)&0x000000ff;
//		CANTXmessage[3]=(Motor2_PresentVelocity[0])&0x000000ff;

//		CAN_senddata(0x324,&CANTXmessage[0],FDCAN_DLC_BYTES_4);

//		CANTXmessage[4]=(Motor2_PresentVelocity[1]>>24)&0x000000ff;
//		CANTXmessage[5]=(Motor2_PresentVelocity[1]>>16)&0x000000ff;
//		CANTXmessage[6]=(Motor2_PresentVelocity[1]>>8)&0x000000ff;
//		CANTXmessage[7]=(Motor2_PresentVelocity[1])&0x000000ff;

//		CAN_senddata(0x325,&CANTXmessage[0],FDCAN_DLC_BYTES_4);

//		CANTXmessage[0]=(Motor2_PresentVelocity[2]>>24)&0x000000ff;
//		CANTXmessage[1]=(Motor2_PresentVelocity[2]>>16)&0x000000ff;
//		CANTXmessage[2]=(Motor2_PresentVelocity[2]>>8)&0x000000ff;
//		CANTXmessage[3]=(Motor2_PresentVelocity[2])&0x000000ff;

//		CAN_senddata(0x326,&CANTXmessage[0],FDCAN_DLC_BYTES_4);

//		CANTXmessage[4]=(Motor2_PresentVelocity[3]>>24)&0x000000ff;
//		CANTXmessage[5]=(Motor2_PresentVelocity[3]>>16)&0x000000ff;
//		CANTXmessage[6]=(Motor2_PresentVelocity[3]>>8)&0x000000ff;
//		CANTXmessage[7]=(Motor2_PresentVelocity[3])&0x000000ff;

//		CAN_senddata(0x327,&CANTXmessage[0],FDCAN_DLC_BYTES_4);

//		//==================M2Position===============================


//		CANTXmessage[0]=(Motor2_PresentPosition[0]>>24)&0x000000ff;
//		CANTXmessage[1]=(Motor2_PresentPosition[0]>>16)&0x000000ff;
//		CANTXmessage[2]=(Motor2_PresentPosition[0]>>8)&0x000000ff;
//		CANTXmessage[3]=(Motor2_PresentPosition[0])&0x000000ff;

//		CAN_senddata(0x320,&CANTXmessage[0],FDCAN_DLC_BYTES_4);

//		CANTXmessage[4]=(Motor2_PresentPosition[1]>>24)&0x000000ff;
//		CANTXmessage[5]=(Motor2_PresentPosition[1]>>16)&0x000000ff;
//		CANTXmessage[6]=(Motor2_PresentPosition[1]>>8)&0x000000ff;
//		CANTXmessage[7]=(Motor2_PresentPosition[1])&0x000000ff;

//		CAN_senddata(0x321,&CANTXmessage[0],FDCAN_DLC_BYTES_4);

//		CANTXmessage[0]=(Motor2_PresentPosition[2]>>24)&0x000000ff;
//		CANTXmessage[1]=(Motor2_PresentPosition[2]>>16)&0x000000ff;
//		CANTXmessage[2]=(Motor2_PresentPosition[2]>>8)&0x000000ff;
//		CANTXmessage[3]=(Motor2_PresentPosition[2])&0x000000ff;

//		CAN_senddata(0x322,&CANTXmessage[0],FDCAN_DLC_BYTES_4);

//		CANTXmessage[4]=(Motor2_PresentPosition[3]>>24)&0x000000ff;
//		CANTXmessage[5]=(Motor2_PresentPosition[3]>>16)&0x000000ff;
//		CANTXmessage[6]=(Motor2_PresentPosition[3]>>8)&0x000000ff;
//		CANTXmessage[7]=(Motor2_PresentPosition[3])&0x000000ff;

//		CAN_senddata(0x323,&CANTXmessage[0],FDCAN_DLC_BYTES_4);

*/


	

		CANTXmessage[0]=(LoadCellForceU16[0]>>8)&0x000000ff;
		CANTXmessage[1]=(LoadCellForceU16[0])&0x000000ff;

		CAN_senddata(0x210,&CANTXmessage[0],FDCAN_DLC_BYTES_2);

		CANTXmessage[0]=(LoadCellForceU16[1]>>8)&0x000000ff;
		CANTXmessage[1]=(LoadCellForceU16[1])&0x000000ff;

		CAN_senddata(0x220,&CANTXmessage[0],FDCAN_DLC_BYTES_2);
		
		tempFloatData.value=curPositionTarget_TSA1;

		CANTXmessage[0]=tempFloatData.byte[0];
		CANTXmessage[1]=tempFloatData.byte[1];
		CANTXmessage[2]=tempFloatData.byte[2];
		CANTXmessage[3]=tempFloatData.byte[3];

		CAN_senddata(0x222,&CANTXmessage[0],FDCAN_DLC_BYTES_4);
		
		CANTXmessage[0]=(DesireForce>>8)&0x000000ff;
		CANTXmessage[1]=(DesireForce)&0x000000ff;

		CAN_senddata(0x521,&CANTXmessage[0],FDCAN_DLC_BYTES_2);
		
		CANTXmessage[0]=(RightPressureSum>>8)&0x000000ff;
		CANTXmessage[1]=(RightPressureSum)&0x000000ff;

		CAN_senddata(0x522,&CANTXmessage[0],FDCAN_DLC_BYTES_2);
		
		CANTXmessage[0]=(LeftPressureSum>>8)&0x000000ff;
		CANTXmessage[1]=(LeftPressureSum)&0x000000ff;

		CAN_senddata(0x523,&CANTXmessage[0],FDCAN_DLC_BYTES_2);
		
	}
		else{
		
		//IMU Data R
		// IMU �߼��ٶ�
		CANTXmessage[0]=(IMU_R.AX_int16>>8)&0x000000ff;
		CANTXmessage[1]=(IMU_R.AX_int16)&0x000000ff;
		
		CANTXmessage[2]=(IMU_R.AY_int16>>8)&0x000000ff;
		CANTXmessage[3]=(IMU_R.AY_int16)&0x000000ff;
		
		CANTXmessage[4]=(IMU_R.AZ_int16>>8)&0x000000ff;
		CANTXmessage[5]=(IMU_R.AZ_int16)&0x000000ff;
		
		CAN_senddata(0x340,&CANTXmessage[0],FDCAN_DLC_BYTES_6);
		
		// IMU ���ٶ�
		
		CANTXmessage[0]=(IMU_R.GX_int16>>8)&0x000000ff;
		CANTXmessage[1]=(IMU_R.GX_int16)&0x000000ff;
		
		CANTXmessage[2]=(IMU_R.GY_int16>>8)&0x000000ff;
		CANTXmessage[3]=(IMU_R.GY_int16)&0x000000ff;
		
		CANTXmessage[4]=(IMU_R.GZ_int16>>8)&0x000000ff;
		CANTXmessage[5]=(IMU_R.GZ_int16)&0x000000ff;
		
		CAN_senddata(0x341,&CANTXmessage[0],FDCAN_DLC_BYTES_6);
		
		// IMU �Ƕ�
		
		CANTXmessage[0]=(IMU_R.Roll_int16>>8)&0x000000ff;
		CANTXmessage[1]=(IMU_R.Roll_int16)&0x000000ff;
		
		CANTXmessage[2]=(IMU_R.Pitch_int16>>8)&0x000000ff;
		CANTXmessage[3]=(IMU_R.Pitch_int16)&0x000000ff;
		
		CANTXmessage[4]=(IMU_R.Yaw_int16>>8)&0x000000ff;
		CANTXmessage[5]=(IMU_R.Yaw_int16)&0x000000ff;
		
		CAN_senddata(0x342,&CANTXmessage[0],FDCAN_DLC_BYTES_6);
		
		
			//IMU Data L
		// IMU �߼��ٶ�
		CANTXmessage[0]=(IMU_L.AX_int16>>8)&0x000000ff;
		CANTXmessage[1]=(IMU_L.AX_int16)&0x000000ff;
		
		CANTXmessage[2]=(IMU_L.AY_int16>>8)&0x000000ff;
		CANTXmessage[3]=(IMU_L.AY_int16)&0x000000ff;
		
		CANTXmessage[4]=(IMU_L.AZ_int16>>8)&0x000000ff;
		CANTXmessage[5]=(IMU_L.AZ_int16)&0x000000ff;
		
		CAN_senddata(0x343,&CANTXmessage[0],FDCAN_DLC_BYTES_6);
		
		// IMU ���ٶ�
		
		CANTXmessage[0]=(IMU_L.GX_int16>>8)&0x000000ff;
		CANTXmessage[1]=(IMU_L.GX_int16)&0x000000ff;
		
		CANTXmessage[2]=(IMU_L.GY_int16>>8)&0x000000ff;
		CANTXmessage[3]=(IMU_L.GY_int16)&0x000000ff;
		
		CANTXmessage[4]=(IMU_L.GZ_int16>>8)&0x000000ff;
		CANTXmessage[5]=(IMU_L.GZ_int16)&0x000000ff;
		
		CAN_senddata(0x344,&CANTXmessage[0],FDCAN_DLC_BYTES_6);
		
		// IMU �Ƕ�
		
		CANTXmessage[0]=(IMU_L.Roll_int16>>8)&0x000000ff;
		CANTXmessage[1]=(IMU_L.Roll_int16)&0x000000ff;
		
		CANTXmessage[2]=(IMU_L.Pitch_int16>>8)&0x000000ff;
		CANTXmessage[3]=(IMU_L.Pitch_int16)&0x000000ff;
		
		CANTXmessage[4]=(IMU_L.Yaw_int16>>8)&0x000000ff;
		CANTXmessage[5]=(IMU_L.Yaw_int16)&0x000000ff;
		
		CAN_senddata(0x345,&CANTXmessage[0],FDCAN_DLC_BYTES_6);
		
		
		//IMU Data Controller
		// IMU �߼��ٶ�
		
		tempCANFloat.value=IMU_Controller.AX_Float;
		CANTXmessage[0]=tempCANFloat.byte[0];
		CANTXmessage[1]=tempCANFloat.byte[1];
		CANTXmessage[2]=tempCANFloat.byte[2];
		CANTXmessage[3]=tempCANFloat.byte[3];
		
		tempCANFloat.value=IMU_Controller.AY_Float;
		CANTXmessage[4]=tempCANFloat.byte[0];
		CANTXmessage[5]=tempCANFloat.byte[1];
		CANTXmessage[6]=tempCANFloat.byte[2];
		CANTXmessage[7]=tempCANFloat.byte[3];
		
		CAN_senddata(0x346,&CANTXmessage[0],FDCAN_DLC_BYTES_8);
		
		tempCANFloat.value=IMU_Controller.AZ_Float;
		CANTXmessage[0]=tempCANFloat.byte[0];
		CANTXmessage[1]=tempCANFloat.byte[1];
		CANTXmessage[2]=tempCANFloat.byte[2];
		CANTXmessage[3]=tempCANFloat.byte[3];
		CAN_senddata(0x347,&CANTXmessage[0],FDCAN_DLC_BYTES_4);


		// IMU ���ٶ�
		
		tempCANFloat.value=IMU_Controller.GX_Float;
		CANTXmessage[0]=tempCANFloat.byte[0];
		CANTXmessage[1]=tempCANFloat.byte[1];
		CANTXmessage[2]=tempCANFloat.byte[2];
		CANTXmessage[3]=tempCANFloat.byte[3];
		
		tempCANFloat.value=IMU_Controller.GY_Float;
		CANTXmessage[4]=tempCANFloat.byte[0];
		CANTXmessage[5]=tempCANFloat.byte[1];
		CANTXmessage[6]=tempCANFloat.byte[2];
		CANTXmessage[7]=tempCANFloat.byte[3];
		
		CAN_senddata(0x348,&CANTXmessage[0],FDCAN_DLC_BYTES_8);
		
		tempCANFloat.value=IMU_Controller.GZ_Float;
		CANTXmessage[0]=tempCANFloat.byte[0];
		CANTXmessage[1]=tempCANFloat.byte[1];
		CANTXmessage[2]=tempCANFloat.byte[2];
		CANTXmessage[3]=tempCANFloat.byte[3];
		CAN_senddata(0x349,&CANTXmessage[0],FDCAN_DLC_BYTES_4);
		
		// IMU �Ƕ� ��  �߶�
		
		tempCANFloat.value=IMU_Controller.Roll_Float;
		CANTXmessage[0]=tempCANFloat.byte[0];
		CANTXmessage[1]=tempCANFloat.byte[1];
		CANTXmessage[2]=tempCANFloat.byte[2];
		CANTXmessage[3]=tempCANFloat.byte[3];
		
		tempCANFloat.value=IMU_Controller.Pitch_Float;
		CANTXmessage[4]=tempCANFloat.byte[0];
		CANTXmessage[5]=tempCANFloat.byte[1];
		CANTXmessage[6]=tempCANFloat.byte[2];
		CANTXmessage[7]=tempCANFloat.byte[3];
		
		CAN_senddata(0x34A,&CANTXmessage[0],FDCAN_DLC_BYTES_8);
		
		tempCANFloat.value=IMU_Controller.Yaw_Float;
		CANTXmessage[0]=tempCANFloat.byte[0];
		CANTXmessage[1]=tempCANFloat.byte[1];
		CANTXmessage[2]=tempCANFloat.byte[2];
		CANTXmessage[3]=tempCANFloat.byte[3];
		
		tempCANFloat.value=IMU_Controller.Height_Float;
		CANTXmessage[4]=tempCANFloat.byte[0];
		CANTXmessage[5]=tempCANFloat.byte[1];
		CANTXmessage[6]=tempCANFloat.byte[2];
		CANTXmessage[7]=tempCANFloat.byte[3];
		
		CAN_senddata(0x34B,&CANTXmessage[0],FDCAN_DLC_BYTES_8);
	}
	
    osDelay(5);
  }
  /* USER CODE END StateRETFUN */
}

/* LoadCellCALFUN function */
void LoadCellCALFUN(void const * argument)
{
  /* USER CODE BEGIN LoadCellCALFUN */
	globalTime++;
	USART_RX_TIMESTAMP++;

	HAL_ADC_Start_DMA(&hadc1, (uint32_t *)ADC_DATA, 32);
	
	


 
	/*   FIR FILTER */
	LoadCell1Buffer[0]=(float32_t)ADC_DATA[0]/16383.0*3.3;
	LoadCell2Buffer[0]=(float32_t)ADC_DATA[2]/16383.0*3.3;
	
	PC0_VolVal=(float32_t)ADC_DATA[4]/16383.0*3.3;
	PA5_VolVal=(float32_t)ADC_DATA[5]/16383.0*3.3;
	
	arm_fir_f32(&FIR_S1,&LoadCell1Buffer[0],&LoadCell1Buffer_Filtered[0],1);
	
	arm_fir_f32(&FIR_S2,&LoadCell2Buffer[0],&LoadCell2Buffer_Filtered[0],1);
	
		LoadCellForce[1]=210.91*LoadCell2Buffer_Filtered[0]-266.41;
		LoadCellForce[0]=-212.16*LoadCell1Buffer_Filtered[0]+269;
	

		LoadCellForceU16[0]=(uint16_t)LoadCellForce[0];
		LoadCellForceU16[1]=(uint16_t)LoadCellForce[1];
		
		Cu_Force[0] = LoadCellForce[0];
		Cu_Force[1] = LoadCellForce[1];
		
		q2_Motor[0] = fabs(Cu_Force[0] - Ta_Force[0]);
		q2_Motor[1] = fabs(Cu_Force[1] - Ta_Force[1]);





  /* USER CODE END LoadCellCALFUN */
}

/* GetRTCTimeFUN function */
void GetRTCTimeFUN(void const * argument)
{
  /* USER CODE BEGIN GetRTCTimeFUN */
			HAL_RTC_GetTime(&hrtc,&rtcTime,RTC_FORMAT_BIN);
			HAL_RTC_GetDate(&hrtc,&rtcDate,RTC_FORMAT_BIN);
	
			curMSEC=(uint16_t)(((float32_t)rtcTime.SubSeconds)/((float32_t)rtcTime.SecondFraction)*1000.0);

			ESPSendBuffer[0][YearpointerOffset+1]=rtcDate.Year;
			ESPSendBuffer[0][MonthpointerOffset]=rtcDate.Month;
			ESPSendBuffer[0][DatepointerOffset]=rtcDate.Date;

			ESPSendBuffer[0][HourpointerOffset]=rtcTime.Hours;
			ESPSendBuffer[0][MinutespointerOffset]=rtcTime.Minutes;
			ESPSendBuffer[0][SecondspointerOffset]=rtcTime.Seconds;
			ESPSendBuffer[0][MMSecondspointerOffset]=(uint8_t)((curMSEC&0xff00)>>8);
			ESPSendBuffer[0][MMSecondspointerOffset+1]=(uint8_t)((curMSEC&0x00ff));
	
  /* USER CODE END GetRTCTimeFUN */
}

/* TaForceCALCallback function */
void TaForceCALCallback(void const * argument)
{
  /* USER CODE BEGIN TaForceCALCallback */
	float dt = 0.005f;
	LoadCellForce1_f=(float)LoadCellForceU16[0];
	LoadCellForce2_f=(float)LoadCellForceU16[1];
	controlOutput_TSA1 = PID_Update(&TSA1_PID_Struct, LoadCellForce1_f, dt);
	controlOutput_TSA2 = PID_Update(&TSA2_PID_Struct, LoadCellForce2_f, dt);

	

  /* USER CODE END TaForceCALCallback */
}

/* CTRLLERPWR function */
void CTRLLERPWR(void const * argument)
{
  /* USER CODE BEGIN CTRLLERPWR */
	
	HAL_GPIO_TogglePin(GPIOE,GPIO_PIN_2);

  /* USER CODE END CTRLLERPWR */
}

/* CurrentPeriodCTRLFUN function */
void CurrentPeriodCTRLFUN(void const * argument)
{
  /* USER CODE BEGIN CurrentPeriodCTRLFUN */
	//arm_iir_f32_LP();
	
	
	uint16_t curFRAM_Addr=MB85RC1M_M1Position_Addr;
	for(uint8_t PositonIDX=0;PositonIDX<4;PositonIDX++){
		WData[0]=(uint8_t)((Motor1_PresentPosition[PositonIDX]>>24)&0x000000ff);
		WData[1]=(uint8_t)((Motor1_PresentPosition[PositonIDX]>>16)&0x000000ff);
		WData[2]=(uint8_t)((Motor1_PresentPosition[PositonIDX]>>8)&0x000000ff);
		WData[3]=(uint8_t)((Motor1_PresentPosition[PositonIDX])&0x000000ff);
		MB85RC1M_Write(curFRAM_Addr, WData, 4); //Write data
		curFRAM_Addr=curFRAM_Addr+4;
		memset(WData,0,4);
	}
	curFRAM_Addr=MB85RC1M_M5Position_Addr;

	for(uint8_t PositonIDX=0;PositonIDX<4;PositonIDX++){
		WData[0]=(uint8_t)((Motor2_PresentPosition[PositonIDX]>>24)&0x000000ff);
		WData[1]=(uint8_t)((Motor2_PresentPosition[PositonIDX]>>16)&0x000000ff);
		WData[2]=(uint8_t)((Motor2_PresentPosition[PositonIDX]>>8)&0x000000ff);
		WData[3]=(uint8_t)((Motor2_PresentPosition[PositonIDX])&0x000000ff);
		MB85RC1M_Write(curFRAM_Addr, WData, 4); //Write data
		curFRAM_Addr=curFRAM_Addr+4;
		memset(WData,0,4);
	}
	


	
  /* USER CODE END CurrentPeriodCTRLFUN */
}

/* IMU_EnqFUN function */
void IMU_EnqFUN(void const * argument)
{
  /* USER CODE BEGIN IMU_EnqFUN */
	
//		WitReadReg(AX, 12);
//		HAL_UART_Transmit_IT(&huart3, IMU_TX_Buf, 8);

  /* USER CODE END IMU_EnqFUN */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
void printf_DMA(const char *format,...)
{
    uint32_t length;
    va_list args;
    uint8_t  temp=0;

    va_start(args, format);
    length = vsnprintf((char*)_dbg_Buff, sizeof(_dbg_Buff)+1, (char*)format, args);
    va_end(args);

	  SCB_CleanInvalidateDCache_by_Addr((uint32_t *) _dbg_Buff, sizeof(_dbg_Buff));
    HAL_UART_Transmit_DMA(&huart4,_dbg_Buff,length);

}

static void CopeSensorData(uint32_t uiReg, uint32_t uiRegNum)
{
	int i;
    for(i = 0; i < uiRegNum; i++)
    {
        switch(uiReg)
        {
//            case AX:
//            case AY:
            case AZ:
				s_cDataUpdate |= ACC_UPDATE;
            break;
//            case GX:
//            case GY:
            case GZ:
				s_cDataUpdate |= GYRO_UPDATE;
            break;
//            case HX:
//            case HY:
            case HZ:
				s_cDataUpdate |= MAG_UPDATE;
            break;
//            case Roll:
//            case Pitch:
            case Yaw:
				s_cDataUpdate |= ANGLE_UPDATE;
            break;
            default:
				s_cDataUpdate |= READ_UPDATE;
			break;
        }
		uiReg++;
    }
}



void CopeCmdData(unsigned char ucData)
{
	static unsigned char s_ucData[50], s_ucRxCnt = 0;
	
	s_ucData[s_ucRxCnt++] = ucData;
	if(s_ucRxCnt<3)return;										//Less than three data returned
	if(s_ucRxCnt >= 50) s_ucRxCnt = 0;
	if(s_ucRxCnt >= 3)
	{
		if((s_ucData[1] == '\r') && (s_ucData[2] == '\n'))
		{
			s_cCmd = s_ucData[0];
			memset(s_ucData,0,50);//
			s_ucRxCnt = 0;
		}
		else 
		{
			s_ucData[0] = s_ucData[1];
			s_ucData[1] = s_ucData[2];
			s_ucRxCnt = 2;
			
		}
	}

}

static void SensorUartSend(uint8_t *p_data, uint32_t uiSize)
{
	HAL_UART_Transmit_DMA(&huart3, &IMU_TX_Buf[0], 8);
}

static void Delayms(uint16_t ucMs)
{
	osDelay(ucMs);
}

static void CmdProcess(void)
{
	switch(s_cCmd)
	{
		case 'a':	
			if(WitStartAccCali() != WIT_HAL_OK) 
				printf("\r\nSet AccCali Error\r\n");
			break;
		case 'm':	
			if(WitStartMagCali() != WIT_HAL_OK) 
				printf("\r\nSet MagCali Error\r\n");
			break;
		case 'e':	
			if(WitStopMagCali() != WIT_HAL_OK)
				printf("\r\nSet MagCali Error\r\n");
			break;
		case 'u':	
			if(WitSetBandwidth(BANDWIDTH_5HZ) != WIT_HAL_OK) 
				printf("\r\nSet Bandwidth Error\r\n");
			break;
		case 'U':	
			if(WitSetBandwidth(BANDWIDTH_256HZ) != WIT_HAL_OK) 
				printf("\r\nSet Bandwidth Error\r\n");
			break;
		case 'B':	
			if(WitSetUartBaud(WIT_BAUD_115200) != WIT_HAL_OK) 
				printf("\r\nSet Baud Error\r\n");
			else
				//Usart2Init(c_uiBaud[WIT_BAUD_115200]);
			break;
		case 'b':	
			if(WitSetUartBaud(WIT_BAUD_9600) != WIT_HAL_OK)
				printf("\r\nSet Baud Error\r\n"); 
			else 
				//Usart2Init(c_uiBaud[WIT_BAUD_9600]);
			break;
		case 'h':	
			//ShowHelp();
			break;
		default :return;
	}
	s_cCmd = 0xff;
}

static const uint8_t IMU_auchCRCHi[256] = {
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,
    0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
    0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01,
    0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81,
    0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01,
    0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,
    0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
    0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01,
    0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,
    0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01,
    0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,
    0x40
};
static const uint8_t IMU_auchCRCLo[256] = {
    0x00, 0xC0, 0xC1, 0x01, 0xC3, 0x03, 0x02, 0xC2, 0xC6, 0x06, 0x07, 0xC7, 0x05, 0xC5, 0xC4,
    0x04, 0xCC, 0x0C, 0x0D, 0xCD, 0x0F, 0xCF, 0xCE, 0x0E, 0x0A, 0xCA, 0xCB, 0x0B, 0xC9, 0x09,
    0x08, 0xC8, 0xD8, 0x18, 0x19, 0xD9, 0x1B, 0xDB, 0xDA, 0x1A, 0x1E, 0xDE, 0xDF, 0x1F, 0xDD,
    0x1D, 0x1C, 0xDC, 0x14, 0xD4, 0xD5, 0x15, 0xD7, 0x17, 0x16, 0xD6, 0xD2, 0x12, 0x13, 0xD3,
    0x11, 0xD1, 0xD0, 0x10, 0xF0, 0x30, 0x31, 0xF1, 0x33, 0xF3, 0xF2, 0x32, 0x36, 0xF6, 0xF7,
    0x37, 0xF5, 0x35, 0x34, 0xF4, 0x3C, 0xFC, 0xFD, 0x3D, 0xFF, 0x3F, 0x3E, 0xFE, 0xFA, 0x3A,
    0x3B, 0xFB, 0x39, 0xF9, 0xF8, 0x38, 0x28, 0xE8, 0xE9, 0x29, 0xEB, 0x2B, 0x2A, 0xEA, 0xEE,
    0x2E, 0x2F, 0xEF, 0x2D, 0xED, 0xEC, 0x2C, 0xE4, 0x24, 0x25, 0xE5, 0x27, 0xE7, 0xE6, 0x26,
    0x22, 0xE2, 0xE3, 0x23, 0xE1, 0x21, 0x20, 0xE0, 0xA0, 0x60, 0x61, 0xA1, 0x63, 0xA3, 0xA2,
    0x62, 0x66, 0xA6, 0xA7, 0x67, 0xA5, 0x65, 0x64, 0xA4, 0x6C, 0xAC, 0xAD, 0x6D, 0xAF, 0x6F,
    0x6E, 0xAE, 0xAA, 0x6A, 0x6B, 0xAB, 0x69, 0xA9, 0xA8, 0x68, 0x78, 0xB8, 0xB9, 0x79, 0xBB,
    0x7B, 0x7A, 0xBA, 0xBE, 0x7E, 0x7F, 0xBF, 0x7D, 0xBD, 0xBC, 0x7C, 0xB4, 0x74, 0x75, 0xB5,
    0x77, 0xB7, 0xB6, 0x76, 0x72, 0xB2, 0xB3, 0x73, 0xB1, 0x71, 0x70, 0xB0, 0x50, 0x90, 0x91,
    0x51, 0x93, 0x53, 0x52, 0x92, 0x96, 0x56, 0x57, 0x97, 0x55, 0x95, 0x94, 0x54, 0x9C, 0x5C,
    0x5D, 0x9D, 0x5F, 0x9F, 0x9E, 0x5E, 0x5A, 0x9A, 0x9B, 0x5B, 0x99, 0x59, 0x58, 0x98, 0x88,
    0x48, 0x49, 0x89, 0x4B, 0x8B, 0x8A, 0x4A, 0x4E, 0x8E, 0x8F, 0x4F, 0x8D, 0x4D, 0x4C, 0x8C,
    0x44, 0x84, 0x85, 0x45, 0x87, 0x47, 0x46, 0x86, 0x82, 0x42, 0x43, 0x83, 0x41, 0x81, 0x80,
    0x40
};

uint16_t IMU_CRC16(uint8_t *puchMsg, uint16_t usDataLen)
{
    uint8_t uchCRCHi = 0xFF;
    uint8_t uchCRCLo = 0xFF;
    uint8_t uIndex;
    int i = 0;
    uchCRCHi = 0xFF;
    uchCRCLo = 0xFF;
    for (; i<usDataLen; i++)
    {
        uIndex = uchCRCHi ^ puchMsg[i];
        uchCRCHi = uchCRCLo ^ IMU_auchCRCHi[uIndex];
        uchCRCLo = IMU_auchCRCLo[uIndex] ;
    }
    return (uint16_t)(((uint16_t)uchCRCHi << 8) | (uint16_t)uchCRCLo) ;
}

/* USER CODE END Application */

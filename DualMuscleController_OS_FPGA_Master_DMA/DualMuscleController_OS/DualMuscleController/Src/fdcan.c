/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    fdcan.c
  * @brief   This file provides code for the configuration
  *          of the FDCAN instances.
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
#include "fdcan.h"

/* USER CODE BEGIN 0 */
#include "arm_math.h"
#include "main.h"

FDCAN_TxHeaderTypeDef TXHeader;
FDCAN_RxHeaderTypeDef RXHeader;

uint8_t CANTXmessage[8] = {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
uint8_t CANRXmessage[8] = {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};

uint8_t EMGRXmessage1[8] = {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
uint8_t EMGRXmessage2[8] = {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};

uint16_t GRF_INSOLE1[3]={0};
uint16_t GRF_INSOLE2[3]={0};

uint16_t RXID=0;
uint8_t RXLen=0;

uint16_t averageRMS_uint[2] = {0};
float averageRMS[2] = {0};

extern uint32_t EMGSensor1ID;
extern uint32_t EMGSensor2ID;
extern uint8_t EMG1RMSValue[8];
extern uint8_t EMG2RMSValue[8];

extern uint8_t ESPSendBuffer[5][300];

CANFloatData TKEO1;
CANFloatData TKEO_mean1;
CANFloatData TKEO_std1;
CANFloatData RMS1;

CANFloatData TKEO2;
CANFloatData TKEO_mean2;
CANFloatData TKEO_std2;
CANFloatData RMS2;


uint8_t pointerIDX=0;

uint8_t CANERROR_FLAG=0;

/* USER CODE END 0 */

FDCAN_HandleTypeDef hfdcan1;

/* FDCAN1 init function */
void MX_FDCAN1_Init(void)
{

  /* USER CODE BEGIN FDCAN1_Init 0 */

  /* USER CODE END FDCAN1_Init 0 */

  /* USER CODE BEGIN FDCAN1_Init 1 */

  /* USER CODE END FDCAN1_Init 1 */
  hfdcan1.Instance = FDCAN1;
  hfdcan1.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
  hfdcan1.Init.Mode = FDCAN_MODE_NORMAL;
  hfdcan1.Init.AutoRetransmission = ENABLE;
  hfdcan1.Init.TransmitPause = DISABLE;
  hfdcan1.Init.ProtocolException = DISABLE;
  hfdcan1.Init.NominalPrescaler = 4;
  hfdcan1.Init.NominalSyncJumpWidth = 9;
  hfdcan1.Init.NominalTimeSeg1 = 10;
  hfdcan1.Init.NominalTimeSeg2 = 9;
  hfdcan1.Init.DataPrescaler = 4;
  hfdcan1.Init.DataSyncJumpWidth = 9;
  hfdcan1.Init.DataTimeSeg1 = 10;
  hfdcan1.Init.DataTimeSeg2 = 9;
  hfdcan1.Init.MessageRAMOffset = 0;
  hfdcan1.Init.StdFiltersNbr = 1;
  hfdcan1.Init.ExtFiltersNbr = 1;
  hfdcan1.Init.RxFifo0ElmtsNbr = 32;
  hfdcan1.Init.RxFifo0ElmtSize = FDCAN_DATA_BYTES_8;
  hfdcan1.Init.RxFifo1ElmtsNbr = 2;
  hfdcan1.Init.RxFifo1ElmtSize = FDCAN_DATA_BYTES_8;
  hfdcan1.Init.RxBuffersNbr = 1;
  hfdcan1.Init.RxBufferSize = FDCAN_DATA_BYTES_8;
  hfdcan1.Init.TxEventsNbr = 2;
  hfdcan1.Init.TxBuffersNbr = 1;
  hfdcan1.Init.TxFifoQueueElmtsNbr = 32;
  hfdcan1.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  hfdcan1.Init.TxElmtSize = FDCAN_DATA_BYTES_8;
  if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN FDCAN1_Init 2 */

  /* USER CODE END FDCAN1_Init 2 */

}

void HAL_FDCAN_MspInit(FDCAN_HandleTypeDef* fdcanHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};
  if(fdcanHandle->Instance==FDCAN1)
  {
  /* USER CODE BEGIN FDCAN1_MspInit 0 */

  /* USER CODE END FDCAN1_MspInit 0 */

  /** Initializes the peripherals clock
  */
    PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_FDCAN;
    PeriphClkInitStruct.FdcanClockSelection = RCC_FDCANCLKSOURCE_PLL;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
    {
      Error_Handler();
    }

    /* FDCAN1 clock enable */
    __HAL_RCC_FDCAN_CLK_ENABLE();

    __HAL_RCC_GPIOD_CLK_ENABLE();
    /**FDCAN1 GPIO Configuration
    PD0     ------> FDCAN1_RX
    PD1     ------> FDCAN1_TX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF9_FDCAN1;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

    /* FDCAN1 interrupt Init */
    HAL_NVIC_SetPriority(FDCAN1_IT0_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(FDCAN1_IT0_IRQn);
  /* USER CODE BEGIN FDCAN1_MspInit 1 */

  /* USER CODE END FDCAN1_MspInit 1 */
  }
}

void HAL_FDCAN_MspDeInit(FDCAN_HandleTypeDef* fdcanHandle)
{

  if(fdcanHandle->Instance==FDCAN1)
  {
  /* USER CODE BEGIN FDCAN1_MspDeInit 0 */

  /* USER CODE END FDCAN1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_FDCAN_CLK_DISABLE();

    /**FDCAN1 GPIO Configuration
    PD0     ------> FDCAN1_RX
    PD1     ------> FDCAN1_TX
    */
    HAL_GPIO_DeInit(GPIOD, GPIO_PIN_0|GPIO_PIN_1);

    /* FDCAN1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(FDCAN1_IT0_IRQn);
  /* USER CODE BEGIN FDCAN1_MspDeInit 1 */

  /* USER CODE END FDCAN1_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */


void FDCAN1_Config(void) 
	{
			FDCAN_FilterTypeDef can_filter_st;
			can_filter_st.IdType = FDCAN_STANDARD_ID;
			can_filter_st.FilterIndex = 0;
			can_filter_st.FilterType = FDCAN_FILTER_RANGE;
			can_filter_st.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
			can_filter_st.FilterID1 = 0x000;
			can_filter_st.FilterID2 = 0x7FF;
			HAL_FDCAN_ConfigFilter(&hfdcan1, &can_filter_st);

			can_filter_st.IdType = FDCAN_EXTENDED_ID;
			can_filter_st.FilterIndex = 0;
			can_filter_st.FilterType = FDCAN_FILTER_RANGE_NO_EIDM;
			can_filter_st.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
			can_filter_st.FilterID1 = 0;
			can_filter_st.FilterID2 = 0x01ffffff;
			HAL_FDCAN_ConfigFilter(&hfdcan1, &can_filter_st);  
			HAL_FDCAN_ConfigFifoWatermark(&hfdcan1,FDCAN_CFG_RX_FIFO0,2);
			HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, FDCAN_REJECT, FDCAN_REJECT, ENABLE, ENABLE);
			HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);

			HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_BUS_OFF, 0);

			HAL_FDCAN_ConfigTxDelayCompensation(&hfdcan1, hfdcan1.Init.DataPrescaler * hfdcan1.Init.DataTimeSeg1, 0);
			HAL_FDCAN_EnableTxDelayCompensation(&hfdcan1);
			
			HAL_FDCAN_Start(&hfdcan1);
}
	

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    if((RxFifo0ITs&FDCAN_IT_RX_FIFO0_NEW_MESSAGE)!=RESET)  
    {
			
				HAL_FDCAN_GetRxMessage(&hfdcan1,FDCAN_RX_FIFO0,&RXHeader,&CANRXmessage[0]);

			
			
				if(RXHeader.Identifier==0x110){
						
						for(pointerIDX=0;pointerIDX<4;pointerIDX++){
							TKEO1.byte[pointerIDX]=CANRXmessage[pointerIDX];
						}
						memset(CANRXmessage,0,8);
					}
				
				if(RXHeader.Identifier==0x111){
						
						for(pointerIDX=0;pointerIDX<4;pointerIDX++){
							RMS1.byte[pointerIDX]=CANRXmessage[pointerIDX];
						}
						memset(CANRXmessage,0,8);
					}

					
				if(RXHeader.Identifier==0x120){
						
						for(pointerIDX=0;pointerIDX<4;pointerIDX++){
							TKEO2.byte[pointerIDX]=CANRXmessage[pointerIDX];
						}
						memset(CANRXmessage,0,8);
					}
				if(RXHeader.Identifier==0x121){
						
						for(pointerIDX=0;pointerIDX<4;pointerIDX++){
							RMS2.byte[pointerIDX]=CANRXmessage[pointerIDX];
						}
						memset(CANRXmessage,0,8);
					}
				
				if(RXHeader.Identifier==0x101){
					
					GRF_INSOLE1[0]=((CANRXmessage[0]));
					memset(CANRXmessage,0,8);
					
					}
				
				if(RXHeader.Identifier==0x102){
					
					GRF_INSOLE2[0]=((CANRXmessage[0]));
					memset(CANRXmessage,0,8);
					}
				
				if(RXHeader.Identifier==0x705){
					EMG1RMSValue[0]=((CANRXmessage[0]));
					EMG1RMSValue[1]=((CANRXmessage[5]));
					memset(CANRXmessage,0,8);
					}
				if(RXHeader.Identifier==0x706){
					EMG2RMSValue[0]=((CANRXmessage[0]));
					EMG2RMSValue[1]=((CANRXmessage[5]));
					memset(CANRXmessage,0,8);
					}
				
				
    }
}

void CAN_senddata(uint32_t can_id,uint8_t* TX_data,uint32_t DataLength)
{
	
		TXHeader.Identifier = can_id;
		TXHeader.IdType = FDCAN_STANDARD_ID;
		TXHeader.TxFrameType = FDCAN_DATA_FRAME;
		TXHeader.DataLength = DataLength;
		TXHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
		TXHeader.BitRateSwitch = FDCAN_BRS_OFF;
		TXHeader.FDFormat = FDCAN_CLASSIC_CAN;
		TXHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
		TXHeader.MessageMarker = 0;  //Tx Event FIFO Use
		HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TXHeader, TX_data);
}


void HAL_FDCAN_ErrorCallback(FDCAN_HandleTypeDef *hfdcan)
{
	if (hfdcan->ErrorCode != HAL_FDCAN_ERROR_NONE) {
        // ������������Ӵ�������룬�����¼����״̬
        

        // ��ѡ��������������� FDCAN ��������������ز���
        // HAL_FDCAN_Stop(hfdcan);
         HAL_FDCAN_Start(hfdcan);
    }

}


/* USER CODE END 1 */

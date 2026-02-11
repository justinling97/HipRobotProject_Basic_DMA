#include "MotorControl.h"
#include "Main.h"
#include "dma.h"
#include "usart.h"
#include "tim.h"
#include "cmsis_os.h"
#include "arm_math.h"
#include "IIR.h"

__attribute__((at(0x38000080)))unsigned char str1[100]={0};

__attribute__((at(0x38000180)))unsigned char str2[100]={0};

unsigned char str_len = 0;
unsigned int aa1[6] ={0};
unsigned int aa2[6] ={0};
unsigned int Mode_Present = 4;
int mode_flag=0;

uint8_t DriverHead[2]={0xaa,0xbb};

int HeadPosition=0;


char data[100] = {0};
int data_len=90;

//unsigned char Return_Data[100];
int Motor1_PresentCurrent[4] = {0};
int Motor1_PresentPosition[4] = {0};
int Motor1_PresentVelocity[4] = {0};

int Motor2_PresentCurrent[4] = {0};
int Motor2_PresentPosition[4] = {0};
int Motor2_PresentVelocity[4] = {0};

uint8_t Motor1Mode[2]={0};
uint8_t Motor2Mode[2]={0};

uint8_t Motor1_CTRLBuffer[Motor_Buffer_Len];
uint8_t Motor1_Return_Data[32];

uint8_t Motor2_CTRLBuffer[Motor_Buffer_Len];
uint8_t Motor2_Return_Data[32];

extern uint8_t ESPSendBuffer[5][300];



arm_biquad_casd_df1_inst_f32 LoadCell_LP_S1; 
arm_biquad_casd_df1_inst_f32 LoadCell_LP_S2; 

const float32_t IIRCoeffs32LP[5*LPnumStages] = {
1.0,  2.0,  -1.0,  1.700964331943525920110005245078355073929,  -0.788499739815297973066776648920495063066,
1.0,  2.0,  -1.0,  1.479674216931193386770360120863188058138, -0.555821543282489005655122582538751885295                           
};                   

float32_t ScaleValue_LP = 0.021883851967943023647533706821377563756*0.019036831587823873496168047836363257375; 

float32_t IIRStateF32LP_LoadcellS1[4*LPnumStages]={0.0}; 
float32_t IIRStateF32LP_LoadcellS2[4*LPnumStages]={0.0}; 

extern struct CurrentStruct M1,M2,M3,M4,M5,M6,M7,M8;


void Motor_Set_Mode(unsigned int motor_mode)
{
	mode_flag= motor_mode;

//	str_len = Dynamixel_Master_Send(Dynamixel_Instruction_Write,Dynamixel_Address_Mode,aa,1,str);
//	if (time3 >= 50)
//	{
//	Mode_Present = motor_mode;
//	SCB_CleanInvalidateDCache_by_Addr((uint32_t *) str, sizeof(str));
//	HAL_UART_Transmit_DMA(&huart5,str,str_len);
//	time3 = 0;	
//	}
//	HAL_UART_Transmit_DMA(&huart4,str,str_len);
}

void Motor1_Set_Position(unsigned int Mot_Pos_1, unsigned int Mot_Pos_2, unsigned int Mot_Pos_3, unsigned int Mot_Pos_4)
{
	if (Mode_Present !=0)
	{
		Motor_Set_Mode(Mode_Position);

	}
	
	aa1[0] = Mot_Pos_1; 		// position of motor 1
	aa1[1] = Mot_Pos_2;    // position of motor 2
	aa1[2] = Mot_Pos_3;    // position of motor 3
	aa1[3] = Mot_Pos_4;    // position of motor 4
	aa1[4] = mode_flag;
	str_len = Dynamixel_Master_Send(Dynamixel_Instruction_Write,Dynamixel_Address_GoalPosition1,aa1,17,str1);

	SCB_CleanInvalidateDCache_by_Addr((uint32_t *) str1, sizeof(str1));
	HAL_UART_Transmit_DMA(&huart5,str1,str_len);
	memset(str1,0,100);

}


void Motor2_Set_Position(unsigned int Mot_Pos_1, unsigned int Mot_Pos_2, unsigned int Mot_Pos_3, unsigned int Mot_Pos_4)
{
	if (Mode_Present !=0)
	{
		Motor_Set_Mode(Mode_Position);

	}
	
	aa2[0] = Mot_Pos_1; 		// position of motor 1
	aa2[1] = Mot_Pos_2;    // position of motor 2
	aa2[2] = Mot_Pos_3;    // position of motor 3
	aa2[3] = Mot_Pos_4;    // position of motor 4
	aa2[4] = mode_flag;
	str_len = Dynamixel_Master_Send(Dynamixel_Instruction_Write,Dynamixel_Address_GoalPosition1,aa2,17,str2);

	SCB_CleanInvalidateDCache_by_Addr((uint32_t *) str2, sizeof(str2));

	HAL_UART_Transmit_DMA(&huart7,str2,str_len);
	memset(str2,0,100);

}

void Motor1_Set_Current(int Mot_Cur_1, int Mot_Cur_2, int Mot_Cur_3, int Mot_Cur_4)
{
	if (Mode_Present !=2)
	{
		Motor_Set_Mode(Mode_Current);

	}
	M1.SetMotorCurrent[0]=(float32_t)Mot_Cur_1;
	M2.SetMotorCurrent[0]=(float32_t)Mot_Cur_2;
	M3.SetMotorCurrent[0]=(float32_t)Mot_Cur_3;
	M4.SetMotorCurrent[0]=(float32_t)Mot_Cur_4;
	
	Motor_arm_iir_f32_LP_Group1();

//	aa1[0] =Mot_Cur_1; 		// current of motor 1
//	aa1[1]  =Mot_Cur_2;    // current of motor 2
//	aa1[2] =Mot_Cur_3;    // current of motor 3
//	aa1[3]  =Mot_Cur_4;    // current of motor 4
	
	aa1[0] =(int)M1.OutputMotorCurrent[0]; 		// current of motor 1
	aa1[1]  =(int)M2.OutputMotorCurrent[0];    // current of motor 2
	aa1[2] =(int)M3.OutputMotorCurrent[0];    // current of motor 3
	aa1[3]  =(int)M4.OutputMotorCurrent[0];    // current of motor 4
	
	
	aa1[4] = mode_flag;
	str_len = Dynamixel_Master_Send(Dynamixel_Instruction_Write,Dynamixel_Address_GoalCurrent1,aa1,9,str1);
		
		
	
	SCB_CleanInvalidateDCache_by_Addr((uint32_t *) str1, sizeof(str1));
	HAL_UART_Transmit_DMA(&huart5,str1,str_len);
		memset(str1,0,100);


}

int current_flag=0;
void Motor2_Set_Current(int Mot_Cur_1, int Mot_Cur_2, int Mot_Cur_3, int Mot_Cur_4)
{
	if (Mode_Present !=2)
	{
		Motor_Set_Mode(Mode_Current);

	}
	
	M5.SetMotorCurrent[0]=(float32_t)Mot_Cur_1;
	M6.SetMotorCurrent[0]=(float32_t)Mot_Cur_2;
	M7.SetMotorCurrent[0]=(float32_t)Mot_Cur_3;
	M8.SetMotorCurrent[0]=(float32_t)Mot_Cur_4;
	
	Motor_arm_iir_f32_LP_Group2();

//	aa2[0] =Mot_Cur_1; 		// current of motor 5
//	aa2[1]  =Mot_Cur_2;    // current of motor 6
//	aa2[2] =Mot_Cur_3;    // current of motor 7
//	aa2[3]  =Mot_Cur_4;    // current of motor 8

	aa2[0] =(int)M5.OutputMotorCurrent[0]; 		// current of motor 1
	aa2[1]  =(int)M6.OutputMotorCurrent[0];    // current of motor 2
	aa2[2] =(int)M7.OutputMotorCurrent[0];    // current of motor 3
	aa2[3]  =(int)M8.OutputMotorCurrent[0];    // current of motor 4
	
	aa2[4] = mode_flag;
	str_len = Dynamixel_Master_Send(Dynamixel_Instruction_Write,Dynamixel_Address_GoalCurrent1,aa2,9,str2);

	SCB_CleanInvalidateDCache_by_Addr((uint32_t *) str2, sizeof(str2));
	HAL_UART_Transmit_DMA(&huart7,str2,str_len);
		memset(str2,0,100);


}



void Motor1_Set_Velocity(int Mot_Velo_1,int Mot_Velo_2, int Mot_Velo_3, int Mot_Velo_4)
{
	if (Mode_Present !=1)
	{
		Motor_Set_Mode(Mode_Velocity);

	}
	
	aa1[0] = Mot_Velo_1; 	 // position of motor 1
	aa1[1] = Mot_Velo_2;    // position of motor 2
	aa1[2] = Mot_Velo_3;    // position of motor 3
	aa1[3] = Mot_Velo_4;    // position of motor 4
	aa1[4] = mode_flag;
	str_len = Dynamixel_Master_Send(Dynamixel_Instruction_Write,Dynamixel_Address_GoalVelocity1,aa1,9,str1);

	SCB_CleanInvalidateDCache_by_Addr((uint32_t *) str1, sizeof(str1));
	HAL_UART_Transmit_DMA(&huart5,str1,str_len);
	memset(str1,0,100);


}

void Motor2_Set_Velocity(int Mot_Velo_1,int Mot_Velo_2, int Mot_Velo_3, int Mot_Velo_4)
{
	if (Mode_Present !=1)
	{
		Motor_Set_Mode(Mode_Velocity);

	}
	
	aa2[0] = Mot_Velo_1; 	 // position of motor 1
	aa2[1] = Mot_Velo_2;    // position of motor 2
	aa2[2] = Mot_Velo_3;    // position of motor 3
	aa2[3] = Mot_Velo_4;    // position of motor 4
	aa2[4] = mode_flag;
	str_len = Dynamixel_Master_Send(Dynamixel_Instruction_Write,Dynamixel_Address_GoalVelocity1,aa2,9,str2);

	SCB_CleanInvalidateDCache_by_Addr((uint32_t *) str2, sizeof(str2));
	HAL_UART_Transmit_DMA(&huart7,str2,str_len);
		memset(str2,0,100);

}

int Driver1_Receive(void)
{
	
//    for(int i=0;i<data_len;i++)
//    {
//      data[i] = aRxBuffer[i];
//      if(i>90)
//      {
//          data_len=90;
//          break;
//      }
//    }
	//uart_start_receive();	
//		for(int i=0;i<data_len;i+=23)
//    {
//unsigned int Dynamixel_Master_Receive(unsigned char *Data, unsigned char Data_Length, unsigned char *Return_Data)
	
	HeadPosition = DriverfindStr(&Motor1_CTRLBuffer[0], &DriverHead[0], 26*3, 2);
	if(HeadPosition==-1){
		return 0;
	}
			
       if(Dynamixel_Master_Receive(&Motor1_CTRLBuffer[HeadPosition],25,Motor1_Return_Data)==Dynamixel_State_Success)
       {

           if(Motor1_Return_Data[0]==Dynamixel_Address_PresentCurrent1)
           {
//               qDebug()<<QString::number(i);
//               ui->PresentCurrentEdit_1->setText(QString::number(Dynamixel_Value_Forward(Return_Data,2,3)));
							Motor1_PresentCurrent[0] = Dynamixel_Value_Forward(Motor1_Return_Data,2,3);
//               ui->PresentCurrentEdit_2->setText(QString::number(Dynamixel_Value_Forward(Return_Data,4,5)));
						  Motor1_PresentCurrent[1] = Dynamixel_Value_Forward(Motor1_Return_Data,4,5);
//               ui->PresentPositionEdit_1->setText(QString::number(Dynamixel_Value_Forward(Return_Data,6,9)));
						  Motor1_PresentPosition[0] = Dynamixel_Value_Forward(Motor1_Return_Data,6,9);
//               ui->PresentPositionEdit_2->setText(QString::number(Dynamixel_Value_Forward(Return_Data,10,13)));
						  Motor1_PresentPosition[1] = Dynamixel_Value_Forward(Motor1_Return_Data,10,13);
//               ui->PresentVelocityEdit_1->setText(QString::number(Dynamixel_Value_Forward(Return_Data,14,15)*10));
						  Motor1_PresentVelocity[0] = Dynamixel_Value_Forward(Motor1_Return_Data,14,15)*10;
//               ui->PresentVelocityEdit_2->setText(QString::number(Dynamixel_Value_Forward(Return_Data,16,17)*10));
						  Motor1_PresentVelocity[1] = Dynamixel_Value_Forward(Motor1_Return_Data,16,17)*10;
						 
						 Motor1Mode[0]=Motor1_Return_Data[18];
						 
           }
           if(Motor1_Return_Data[0]==Dynamixel_Address_PresentCurrent2)
           {
//               qDebug()<<QString::number(i);
//               ui->PresentCurrentEdit_3->setText(QString::number(Dynamixel_Value_Forward(Return_Data,2,3)));
						  Motor1_PresentCurrent[2] = Dynamixel_Value_Forward(Motor1_Return_Data,2,3);
//               ui->PresentCurrentEdit_4->setText(QString::number(Dynamixel_Value_Forward(Return_Data,4,5)));
						  Motor1_PresentCurrent[3] = Dynamixel_Value_Forward(Motor1_Return_Data,4,5);
//               ui->PresentPositionEdit_3->setText(QString::number(Dynamixel_Value_Forward(Return_Data,6,9)));
							Motor1_PresentPosition[2] = Dynamixel_Value_Forward(Motor1_Return_Data,6,9);
//               ui->PresentPositionEdit_4->setText(QString::number(Dynamixel_Value_Forward(Return_Data,10,13)));
						  Motor1_PresentPosition[3] = Dynamixel_Value_Forward(Motor1_Return_Data,10,13);
//               ui->PresentVelocityEdit_3->setText(QString::number(Dynamixel_Value_Forward(Return_Data,14,15)*10));
						  Motor1_PresentVelocity[2] = Dynamixel_Value_Forward(Motor1_Return_Data,14,15)*10;
//               ui->PresentVelocityEdit_4->setText(QString::number(Dynamixel_Value_Forward(Return_Data,16,17)*10));
						  Motor1_PresentVelocity[3] = Dynamixel_Value_Forward(Motor1_Return_Data,16,17)*10;
						 
							Motor1Mode[1]=Motor1_Return_Data[18];
           }

       }
//       if(i>=66)
//           break;
    //}
		return 0;
}

int Driver2_Receive(void)
{
//    for(int i=0;i<data_len;i++)
//    {
//      data[i] = aRxBuffer[i];
//      if(i>90)
//      {
//          data_len=90;
//          break;
//      }
//    }
	//uart_start_receive();	
//		for(int i=0;i<data_len;i+=23)
//    {
//unsigned int Dynamixel_Master_Receive(unsigned char *Data, unsigned char Data_Length, unsigned char *Return_Data)
			
       if(Dynamixel_Master_Receive((unsigned char*)Motor2_CTRLBuffer,25,Motor2_Return_Data)==Dynamixel_State_Success)
       {

           if(Motor2_Return_Data[0]==Dynamixel_Address_PresentCurrent1)
           {
//               qDebug()<<QString::number(i);
//               ui->PresentCurrentEdit_1->setText(QString::number(Dynamixel_Value_Forward(Return_Data,2,3)));
							Motor2_PresentCurrent[0] = Dynamixel_Value_Forward(Motor2_Return_Data,2,3);
//               ui->PresentCurrentEdit_2->setText(QString::number(Dynamixel_Value_Forward(Return_Data,4,5)));
						  Motor2_PresentCurrent[1] = Dynamixel_Value_Forward(Motor2_Return_Data,4,5);
//               ui->PresentPositionEdit_1->setText(QString::number(Dynamixel_Value_Forward(Return_Data,6,9)));
						  Motor2_PresentPosition[0] = Dynamixel_Value_Forward(Motor2_Return_Data,6,9);
//               ui->PresentPositionEdit_2->setText(QString::number(Dynamixel_Value_Forward(Return_Data,10,13)));
						  Motor2_PresentPosition[1] = Dynamixel_Value_Forward(Motor2_Return_Data,10,13);
//               ui->PresentVelocityEdit_1->setText(QString::number(Dynamixel_Value_Forward(Return_Data,14,15)*10));
						  Motor2_PresentVelocity[0] = Dynamixel_Value_Forward(Motor2_Return_Data,14,15)*10;
//               ui->PresentVelocityEdit_2->setText(QString::number(Dynamixel_Value_Forward(Return_Data,16,17)*10));
						  Motor2_PresentVelocity[1] = Dynamixel_Value_Forward(Motor2_Return_Data,16,17)*10;
						 
						 Motor2Mode[0]=Motor2_Return_Data[18];
						 
           }
           if(Motor2_Return_Data[0]==Dynamixel_Address_PresentCurrent2)
           {
//               qDebug()<<QString::number(i);
//               ui->PresentCurrentEdit_3->setText(QString::number(Dynamixel_Value_Forward(Return_Data,2,3)));
						  Motor2_PresentCurrent[2] = Dynamixel_Value_Forward(Motor2_Return_Data,2,3);
//               ui->PresentCurrentEdit_4->setText(QString::number(Dynamixel_Value_Forward(Return_Data,4,5)));
						  Motor2_PresentCurrent[3] = Dynamixel_Value_Forward(Motor2_Return_Data,4,5);
//               ui->PresentPositionEdit_3->setText(QString::number(Dynamixel_Value_Forward(Return_Data,6,9)));
							Motor2_PresentPosition[2] = Dynamixel_Value_Forward(Motor2_Return_Data,6,9);
//               ui->PresentPositionEdit_4->setText(QString::number(Dynamixel_Value_Forward(Return_Data,10,13)));
						  Motor2_PresentPosition[3] = Dynamixel_Value_Forward(Motor2_Return_Data,10,13);
//               ui->PresentVelocityEdit_3->setText(QString::number(Dynamixel_Value_Forward(Return_Data,14,15)*10));
						  Motor2_PresentVelocity[2] = Dynamixel_Value_Forward(Motor2_Return_Data,14,15)*10;
//               ui->PresentVelocityEdit_4->setText(QString::number(Dynamixel_Value_Forward(Return_Data,16,17)*10));
						  Motor2_PresentVelocity[3] = Dynamixel_Value_Forward(Motor2_Return_Data,16,17)*10;
						 
							Motor2Mode[1]=Motor2_Return_Data[18];
           }

       }
//       if(i>=66)
//           break;
    //}
		return 0;
}

int DriverfindStr(uint8_t* ESPreport, uint8_t* Template, uint8_t ReportLen, uint8_t TemplateLen){
	
		
		uint8_t k = 0;
		uint8_t i=0;
    for(i = 0; i<ReportLen ;i++){
        if( k==0&&ESPreport[i] == Template[k]){
            k++;
            continue;
        }
        if(k>0 && ESPreport[i] == Template[k]){
            k++;
            if(k==TemplateLen){
                return i-k+1;//????????????-????+1
            }
            continue;
        }
        k = 0;
    }
    return -1;//?????1
}

void MotorRetCMDGen(void){
	

	
		//Motor1 Current  ==============================================================================
		ESPSendBuffer[0][M1CpointerOffset+0]=(uint8_t)((Motor1_PresentCurrent[0]&0xff000000)>>24);
		ESPSendBuffer[0][M1CpointerOffset+1]=(uint8_t)((Motor1_PresentCurrent[0]&0x00ff0000)>>16);
		ESPSendBuffer[0][M1CpointerOffset+2]=(uint8_t)((Motor1_PresentCurrent[0]&0x0000ff00)>>8);
		ESPSendBuffer[0][M1CpointerOffset+3]=(uint8_t)((Motor1_PresentCurrent[0]&0x000000ff));

		ESPSendBuffer[0][M1CpointerOffset+4]=(uint8_t)((Motor1_PresentCurrent[1]&0xff000000)>>24);
		ESPSendBuffer[0][M1CpointerOffset+5]=(uint8_t)((Motor1_PresentCurrent[1]&0x00ff0000)>>16);
		ESPSendBuffer[0][M1CpointerOffset+6]=(uint8_t)((Motor1_PresentCurrent[1]&0x0000ff00)>>8);
		ESPSendBuffer[0][M1CpointerOffset+7]=(uint8_t)((Motor1_PresentCurrent[1]&0x000000ff));

		ESPSendBuffer[0][M1CpointerOffset+8]=(uint8_t)((Motor1_PresentCurrent[2]&0xff000000)>>24);
		ESPSendBuffer[0][M1CpointerOffset+9]=(uint8_t)((Motor1_PresentCurrent[2]&0x00ff0000)>>16);
		ESPSendBuffer[0][M1CpointerOffset+10]=(uint8_t)((Motor1_PresentCurrent[2]&0x0000ff00)>>8);
		ESPSendBuffer[0][M1CpointerOffset+11]=(uint8_t)((Motor1_PresentCurrent[2]&0x000000ff));

		ESPSendBuffer[0][M1CpointerOffset+12]=(uint8_t)((Motor1_PresentCurrent[3]&0xff000000)>>24);
		ESPSendBuffer[0][M1CpointerOffset+13]=(uint8_t)((Motor1_PresentCurrent[3]&0x00ff0000)>>16);
		ESPSendBuffer[0][M1CpointerOffset+14]=(uint8_t)((Motor1_PresentCurrent[3]&0x0000ff00)>>8);
		ESPSendBuffer[0][M1CpointerOffset+15]=(uint8_t)((Motor1_PresentCurrent[3]&0x000000ff));
	
	
		//Motor1 Velosity====================================================================================
		ESPSendBuffer[0][M1VpointerOffset+0]=(uint8_t)((Motor1_PresentVelocity[0]&0xff000000)>>24);
		ESPSendBuffer[0][M1VpointerOffset+1]=(uint8_t)((Motor1_PresentVelocity[0]&0x00ff0000)>>16);
		ESPSendBuffer[0][M1VpointerOffset+2]=(uint8_t)((Motor1_PresentVelocity[0]&0x0000ff00)>>8);
		ESPSendBuffer[0][M1VpointerOffset+3]=(uint8_t)((Motor1_PresentVelocity[0]&0x000000ff));

		ESPSendBuffer[0][M1VpointerOffset+4]=(uint8_t)((Motor1_PresentVelocity[1]&0xff000000)>>24);
		ESPSendBuffer[0][M1VpointerOffset+5]=(uint8_t)((Motor1_PresentVelocity[1]&0x00ff0000)>>16);
		ESPSendBuffer[0][M1VpointerOffset+6]=(uint8_t)((Motor1_PresentVelocity[1]&0x0000ff00)>>8);
		ESPSendBuffer[0][M1VpointerOffset+7]=(uint8_t)((Motor1_PresentVelocity[1]&0x000000ff));

		ESPSendBuffer[0][M1VpointerOffset+8]=(uint8_t)((Motor1_PresentVelocity[2]&0xff000000)>>24);
		ESPSendBuffer[0][M1VpointerOffset+9]=(uint8_t)((Motor1_PresentVelocity[2]&0x00ff0000)>>16);
		ESPSendBuffer[0][M1VpointerOffset+10]=(uint8_t)((Motor1_PresentVelocity[2]&0x0000ff00)>>8);
		ESPSendBuffer[0][M1VpointerOffset+11]=(uint8_t)((Motor1_PresentVelocity[2]&0x000000ff));

		ESPSendBuffer[0][M1VpointerOffset+12]=(uint8_t)((Motor1_PresentVelocity[3]&0xff000000)>>24);
		ESPSendBuffer[0][M1VpointerOffset+13]=(uint8_t)((Motor1_PresentVelocity[3]&0x00ff0000)>>16);
		ESPSendBuffer[0][M1VpointerOffset+14]=(uint8_t)((Motor1_PresentVelocity[3]&0x0000ff00)>>8);
		ESPSendBuffer[0][M1VpointerOffset+15]=(uint8_t)((Motor1_PresentVelocity[3]&0x000000ff));
		
		
		//Motor1 Position=========================================================================================
		ESPSendBuffer[0][M1PpointerOffset+0]=(uint8_t)((Motor1_PresentPosition[0]&0xff000000)>>24);
		ESPSendBuffer[0][M1PpointerOffset+1]=(uint8_t)((Motor1_PresentPosition[0]&0x00ff0000)>>16);
		ESPSendBuffer[0][M1PpointerOffset+2]=(uint8_t)((Motor1_PresentPosition[0]&0x0000ff00)>>8);
		ESPSendBuffer[0][M1PpointerOffset+3]=(uint8_t)((Motor1_PresentPosition[0]&0x000000ff));

		ESPSendBuffer[0][M1PpointerOffset+4]=(uint8_t)((Motor1_PresentPosition[1]&0xff000000)>>24);
		ESPSendBuffer[0][M1PpointerOffset+5]=(uint8_t)((Motor1_PresentPosition[1]&0x00ff0000)>>16);
		ESPSendBuffer[0][M1PpointerOffset+6]=(uint8_t)((Motor1_PresentPosition[1]&0x0000ff00)>>8);
		ESPSendBuffer[0][M1PpointerOffset+7]=(uint8_t)((Motor1_PresentPosition[1]&0x000000ff));

		ESPSendBuffer[0][M1PpointerOffset+8]=(uint8_t)((Motor1_PresentPosition[2]&0xff000000)>>24);
		ESPSendBuffer[0][M1PpointerOffset+9]=(uint8_t)((Motor1_PresentPosition[2]&0x00ff0000)>>16);
		ESPSendBuffer[0][M1PpointerOffset+10]=(uint8_t)((Motor1_PresentPosition[2]&0x0000ff00)>>8);
		ESPSendBuffer[0][M1PpointerOffset+11]=(uint8_t)((Motor1_PresentPosition[2]&0x000000ff));

		ESPSendBuffer[0][M1PpointerOffset+12]=(uint8_t)((Motor1_PresentPosition[3]&0xff000000)>>24);
		ESPSendBuffer[0][M1PpointerOffset+13]=(uint8_t)((Motor1_PresentPosition[3]&0x00ff0000)>>16);
		ESPSendBuffer[0][M1PpointerOffset+14]=(uint8_t)((Motor1_PresentPosition[3]&0x0000ff00)>>8);
		ESPSendBuffer[0][M1PpointerOffset+15]=(uint8_t)((Motor1_PresentPosition[3]&0x000000ff));
		
		
		
		//Motor2 Current==============================================================================================
		ESPSendBuffer[0][M2CpointerOffset+0]=(uint8_t)((Motor2_PresentCurrent[0]&0xff000000)>>24);
		ESPSendBuffer[0][M2CpointerOffset+1]=(uint8_t)((Motor2_PresentCurrent[0]&0x00ff0000)>>16);
		ESPSendBuffer[0][M2CpointerOffset+2]=(uint8_t)((Motor2_PresentCurrent[0]&0x0000ff00)>>8);
		ESPSendBuffer[0][M2CpointerOffset+3]=(uint8_t)((Motor2_PresentCurrent[0]&0x000000ff));

		ESPSendBuffer[0][M2CpointerOffset+4]=(uint8_t)((Motor2_PresentCurrent[1]&0xff000000)>>24);
		ESPSendBuffer[0][M2CpointerOffset+5]=(uint8_t)((Motor2_PresentCurrent[1]&0x00ff0000)>>16);
		ESPSendBuffer[0][M2CpointerOffset+6]=(uint8_t)((Motor2_PresentCurrent[1]&0x0000ff00)>>8);
		ESPSendBuffer[0][M2CpointerOffset+7]=(uint8_t)((Motor2_PresentCurrent[1]&0x000000ff));

		ESPSendBuffer[0][M2CpointerOffset+8]=(uint8_t)((Motor2_PresentCurrent[2]&0xff000000)>>24);
		ESPSendBuffer[0][M2CpointerOffset+9]=(uint8_t)((Motor2_PresentCurrent[2]&0x00ff0000)>>16);
		ESPSendBuffer[0][M2CpointerOffset+10]=(uint8_t)((Motor2_PresentCurrent[2]&0x0000ff00)>>8);
		ESPSendBuffer[0][M2CpointerOffset+11]=(uint8_t)((Motor2_PresentCurrent[2]&0x000000ff));

		ESPSendBuffer[0][M2CpointerOffset+12]=(uint8_t)((Motor2_PresentCurrent[3]&0xff000000)>>24);
		ESPSendBuffer[0][M2CpointerOffset+13]=(uint8_t)((Motor2_PresentCurrent[3]&0x00ff0000)>>16);
		ESPSendBuffer[0][M2CpointerOffset+14]=(uint8_t)((Motor2_PresentCurrent[3]&0x0000ff00)>>8);
		ESPSendBuffer[0][M2CpointerOffset+15]=(uint8_t)((Motor2_PresentCurrent[3]&0x000000ff));
	
	
		//Motor2 Velosity================================================================================================
		ESPSendBuffer[0][M2VpointerOffset+0]=(uint8_t)((Motor2_PresentVelocity[0]&0xff000000)>>24);
		ESPSendBuffer[0][M2VpointerOffset+1]=(uint8_t)((Motor2_PresentVelocity[0]&0x00ff0000)>>16);
		ESPSendBuffer[0][M2VpointerOffset+2]=(uint8_t)((Motor2_PresentVelocity[0]&0x0000ff00)>>8);
		ESPSendBuffer[0][M2VpointerOffset+3]=(uint8_t)((Motor2_PresentVelocity[0]&0x000000ff));

		ESPSendBuffer[0][M2VpointerOffset+4]=(uint8_t)((Motor2_PresentVelocity[1]&0xff000000)>>24);
		ESPSendBuffer[0][M2VpointerOffset+5]=(uint8_t)((Motor2_PresentVelocity[1]&0x00ff0000)>>16);
		ESPSendBuffer[0][M2VpointerOffset+6]=(uint8_t)((Motor2_PresentVelocity[1]&0x0000ff00)>>8);
		ESPSendBuffer[0][M2VpointerOffset+7]=(uint8_t)((Motor2_PresentVelocity[1]&0x000000ff));

		ESPSendBuffer[0][M2VpointerOffset+8]=(uint8_t)((Motor2_PresentVelocity[2]&0xff000000)>>24);
		ESPSendBuffer[0][M2VpointerOffset+9]=(uint8_t)((Motor2_PresentVelocity[2]&0x00ff0000)>>16);
		ESPSendBuffer[0][M2VpointerOffset+10]=(uint8_t)((Motor2_PresentVelocity[2]&0x0000ff00)>>8);
		ESPSendBuffer[0][M2VpointerOffset+11]=(uint8_t)((Motor2_PresentVelocity[2]&0x000000ff));

		ESPSendBuffer[0][M2VpointerOffset+12]=(uint8_t)((Motor2_PresentVelocity[3]&0xff000000)>>24);
		ESPSendBuffer[0][M2VpointerOffset+13]=(uint8_t)((Motor2_PresentVelocity[3]&0x00ff0000)>>16);
		ESPSendBuffer[0][M2VpointerOffset+14]=(uint8_t)((Motor2_PresentVelocity[3]&0x0000ff00)>>8);
		ESPSendBuffer[0][M2VpointerOffset+15]=(uint8_t)((Motor2_PresentVelocity[3]&0x000000ff));
		
		
		//Motor2 Position==============================================================================================
		ESPSendBuffer[0][M2PpointerOffset+0]=(uint8_t)((Motor2_PresentPosition[0]&0xff000000)>>24);
		ESPSendBuffer[0][M2PpointerOffset+1]=(uint8_t)((Motor2_PresentPosition[0]&0x00ff0000)>>16);
		ESPSendBuffer[0][M2PpointerOffset+2]=(uint8_t)((Motor2_PresentPosition[0]&0x0000ff00)>>8);
		ESPSendBuffer[0][M2PpointerOffset+3]=(uint8_t)((Motor2_PresentPosition[0]&0x000000ff));

		ESPSendBuffer[0][M2PpointerOffset+4]=(uint8_t)((Motor2_PresentPosition[1]&0xff000000)>>24);
		ESPSendBuffer[0][M2PpointerOffset+5]=(uint8_t)((Motor2_PresentPosition[1]&0x00ff0000)>>16);
		ESPSendBuffer[0][M2PpointerOffset+6]=(uint8_t)((Motor2_PresentPosition[1]&0x0000ff00)>>8);
		ESPSendBuffer[0][M2PpointerOffset+7]=(uint8_t)((Motor2_PresentPosition[1]&0x000000ff));

		ESPSendBuffer[0][M2PpointerOffset+8]=(uint8_t)((Motor2_PresentPosition[2]&0xff000000)>>24);
		ESPSendBuffer[0][M2PpointerOffset+9]=(uint8_t)((Motor2_PresentPosition[2]&0x00ff0000)>>16);
		ESPSendBuffer[0][M2PpointerOffset+10]=(uint8_t)((Motor2_PresentPosition[2]&0x0000ff00)>>8);
		ESPSendBuffer[0][M2PpointerOffset+11]=(uint8_t)((Motor2_PresentPosition[2]&0x000000ff));

		ESPSendBuffer[0][M2PpointerOffset+12]=(uint8_t)((Motor2_PresentPosition[3]&0xff000000)>>24);
		ESPSendBuffer[0][M2PpointerOffset+13]=(uint8_t)((Motor2_PresentPosition[3]&0x00ff0000)>>16);
		ESPSendBuffer[0][M2PpointerOffset+14]=(uint8_t)((Motor2_PresentPosition[3]&0x0000ff00)>>8);
		ESPSendBuffer[0][M2PpointerOffset+15]=(uint8_t)((Motor2_PresentPosition[3]&0x000000ff));
	

}

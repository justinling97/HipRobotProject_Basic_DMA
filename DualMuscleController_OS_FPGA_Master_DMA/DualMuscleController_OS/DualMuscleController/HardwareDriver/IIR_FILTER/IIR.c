#include "IIR.h"
#include "arm_math.h"

float32_t M1IIRStateF32LP[4*BPnumStages]; 
float32_t M2IIRStateF32LP[4*BPnumStages]; 
float32_t M3IIRStateF32LP[4*BPnumStages]; 
float32_t M4IIRStateF32LP[4*BPnumStages]; 
float32_t M5IIRStateF32LP[4*BPnumStages]; 
float32_t M6IIRStateF32LP[4*BPnumStages]; 
float32_t M7IIRStateF32LP[4*BPnumStages]; 
float32_t M8IIRStateF32LP[4*BPnumStages]; 

arm_biquad_casd_df1_inst_f32 M1IIR_LP; 
arm_biquad_casd_df1_inst_f32 M2IIR_LP; 
arm_biquad_casd_df1_inst_f32 M3IIR_LP; 
arm_biquad_casd_df1_inst_f32 M4IIR_LP; 
arm_biquad_casd_df1_inst_f32 M5IIR_LP; 
arm_biquad_casd_df1_inst_f32 M6IIR_LP; 
arm_biquad_casd_df1_inst_f32 M7IIR_LP; 
arm_biquad_casd_df1_inst_f32 M8IIR_LP; 

float32_t SetMotor1Current[MotorCurrentBufferLen];
float32_t SetMotor2Current[MotorCurrentBufferLen];
float32_t SetMotor3Current[MotorCurrentBufferLen];
float32_t SetMotor4Current[MotorCurrentBufferLen];
float32_t SetMotor5Current[MotorCurrentBufferLen];
float32_t SetMotor6Current[MotorCurrentBufferLen];
float32_t SetMotor7Current[MotorCurrentBufferLen];
float32_t SetMotor8Current[MotorCurrentBufferLen];

float32_t OutputMotor1Current[MotorCurrentBufferLen];
float32_t OutputMotor2Current[MotorCurrentBufferLen];
float32_t OutputMotor3Current[MotorCurrentBufferLen];
float32_t OutputMotor4Current[MotorCurrentBufferLen];
float32_t OutputMotor5Current[MotorCurrentBufferLen];
float32_t OutputMotor6Current[MotorCurrentBufferLen];
float32_t OutputMotor7Current[MotorCurrentBufferLen];
float32_t OutputMotor8Current[MotorCurrentBufferLen];


extern struct CurrentStruct M1,M2,M3,M4,M5,M6,M7,M8;


const float32_t IIRCoeffs32LP_Motor[5*BPnumStages] = {
1.0,  2.0,  1.0,  0.000000000000000160589618511030003047089,  -0.4464626921716895457947771319595631212,
1.0,  0.0,  1.0, 0.000000000000000115415025303164958547199,   -0.039566129896580051750198947502212831751                           
};                   

float32_t ScaleValue_LP_Motor = 0.361615673042922358693118667360977269709*0.259891532474144981712527169293025508523; 


arm_fir_instance_f32 FIR_S1;
arm_fir_instance_f32 FIR_S2;

int BL1 = 21;
float32_t B1[21] = {
   0.007252480626024, 0.009322776346974,  0.01530767937901,   0.0246494988388,
    0.03645113050735,  0.04956446503779,  0.06270453813828,  0.07457793039764,
    0.08401242685588,  0.09007477850144,  0.09216459074162,  0.09007477850144,
    0.08401242685588,  0.07457793039764,  0.06270453813828,  0.04956446503779,
    0.03645113050735,   0.0246494988388,  0.01530767937901, 0.009322776346974,
   0.007252480626024
};

int BL2 = 21;
float32_t B2[21] = {
   0.007252480626024, 0.009322776346974,  0.01530767937901,   0.0246494988388,
    0.03645113050735,  0.04956446503779,  0.06270453813828,  0.07457793039764,
    0.08401242685588,  0.09007477850144,  0.09216459074162,  0.09007477850144,
    0.08401242685588,  0.07457793039764,  0.06270453813828,  0.04956446503779,
    0.03645113050735,   0.0246494988388,  0.01530767937901, 0.009322776346974,
   0.007252480626024
};

float32_t  FIR_pState1[21]={0.0f}; //FIR滤波器状态变量暂存：数组的大小=BL+blocksize-1

float32_t  FIR_pState2[21]={0.0f}; //FIR滤波器状态变量暂存：数组的大小=BL+blocksize-1

uint32_t   FIR_blockSize = 1;    //块处理大小，即ADC采样的数据个数


void arm_iir_init_LP(void){
	
	arm_biquad_cascade_df1_init_f32(&M1.IIR_LP, BPnumStages, (float32_t *)&IIRCoeffs32LP_Motor[0],(float32_t *)&M1.IIRStateF32LP[0]);
	arm_biquad_cascade_df1_init_f32(&M2.IIR_LP, BPnumStages, (float32_t *)&IIRCoeffs32LP_Motor[0],(float32_t *)&M2.IIRStateF32LP[0]);
	arm_biquad_cascade_df1_init_f32(&M3.IIR_LP, BPnumStages, (float32_t *)&IIRCoeffs32LP_Motor[0],(float32_t *)&M3.IIRStateF32LP[0]);
	arm_biquad_cascade_df1_init_f32(&M4.IIR_LP, BPnumStages, (float32_t *)&IIRCoeffs32LP_Motor[0],(float32_t *)&M4.IIRStateF32LP[0]);
	arm_biquad_cascade_df1_init_f32(&M5.IIR_LP, BPnumStages, (float32_t *)&IIRCoeffs32LP_Motor[0],(float32_t *)&M5.IIRStateF32LP[0]);
	arm_biquad_cascade_df1_init_f32(&M6.IIR_LP, BPnumStages, (float32_t *)&IIRCoeffs32LP_Motor[0],(float32_t *)&M6.IIRStateF32LP[0]);
	arm_biquad_cascade_df1_init_f32(&M7.IIR_LP, BPnumStages, (float32_t *)&IIRCoeffs32LP_Motor[0],(float32_t *)&M7.IIRStateF32LP[0]);
	arm_biquad_cascade_df1_init_f32(&M8.IIR_LP, BPnumStages, (float32_t *)&IIRCoeffs32LP_Motor[0],(float32_t *)&M8.IIRStateF32LP[0]);
	
	arm_fir_init_f32(&FIR_S1,BL1,B1,FIR_pState1,FIR_blockSize);
	arm_fir_init_f32(&FIR_S2,BL2,B2,FIR_pState2,FIR_blockSize);
}

void Motor_arm_iir_f32_LP_Group1(void)
{
		arm_biquad_cascade_df1_f32(&M1.IIR_LP, &M1.SetMotorCurrent[0], &M1.OutputMotorCurrent[0], MotorCurrentBufferLen); 
		arm_scale_f32(&M1.OutputMotorCurrent[0],ScaleValue_LP_Motor,&M1.OutputMotorCurrent[0],MotorCurrentBufferLen);
	
			arm_biquad_cascade_df1_f32(&M2.IIR_LP, &M2.SetMotorCurrent[0], &M2.OutputMotorCurrent[0], MotorCurrentBufferLen); 
		arm_scale_f32(&M2.OutputMotorCurrent[0],ScaleValue_LP_Motor,&M2.OutputMotorCurrent[0],MotorCurrentBufferLen);
	
			arm_biquad_cascade_df1_f32(&M3.IIR_LP, &M3.SetMotorCurrent[0], &M3.OutputMotorCurrent[0], MotorCurrentBufferLen); 
		arm_scale_f32(&M3.OutputMotorCurrent[0],ScaleValue_LP_Motor,&M3.OutputMotorCurrent[0],MotorCurrentBufferLen);
	
			arm_biquad_cascade_df1_f32(&M4.IIR_LP, &M4.SetMotorCurrent[0], &M4.OutputMotorCurrent[0], MotorCurrentBufferLen); 
		arm_scale_f32(&M4.OutputMotorCurrent[0],ScaleValue_LP_Motor,&M4.OutputMotorCurrent[0],MotorCurrentBufferLen);
	
}
void Motor_arm_iir_f32_LP_Group2(void)
{
	
			arm_biquad_cascade_df1_f32(&M5.IIR_LP, &M5.SetMotorCurrent[0], &M5.OutputMotorCurrent[0], MotorCurrentBufferLen); 
		arm_scale_f32(&M5.OutputMotorCurrent[0],ScaleValue_LP_Motor,&M5.OutputMotorCurrent[0],MotorCurrentBufferLen);
	
			arm_biquad_cascade_df1_f32(&M6.IIR_LP, &M6.SetMotorCurrent[0], &M6.OutputMotorCurrent[0], MotorCurrentBufferLen); 
		arm_scale_f32(&M6.OutputMotorCurrent[0],ScaleValue_LP_Motor,&M6.OutputMotorCurrent[0],MotorCurrentBufferLen);
	
			arm_biquad_cascade_df1_f32(&M7.IIR_LP, &M7.SetMotorCurrent[0], &M7.OutputMotorCurrent[0], MotorCurrentBufferLen); 
		arm_scale_f32(&M7.OutputMotorCurrent[0],ScaleValue_LP_Motor,&M7.OutputMotorCurrent[0],MotorCurrentBufferLen);
	
			arm_biquad_cascade_df1_f32(&M8.IIR_LP, &M8.SetMotorCurrent[0], &M8.OutputMotorCurrent[0], MotorCurrentBufferLen); 
		arm_scale_f32(&M8.OutputMotorCurrent[0],ScaleValue_LP_Motor,&M8.OutputMotorCurrent[0],MotorCurrentBufferLen);
}

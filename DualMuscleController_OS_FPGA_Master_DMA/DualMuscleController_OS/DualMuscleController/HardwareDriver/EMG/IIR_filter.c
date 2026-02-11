
#include "IIR_filter.h"



uint32_t blockSize = BLOCK_SIZE;

float32_t ADC_max_volume=3.3;


uint32_t numBlocks = ADC_result_length/BLOCK_SIZE;

float32_t IIRStateF32CH1[4*numStages]; 
float32_t IIRStateF32CH2[4*numStages]; 
float32_t IIRStateF32CH3[4*numStages]; 
float32_t IIRStateF32CH4[4*numStages]; 


float32_t temp_result;

float32_t ADC_result[4];

float32_t ADC_ch1_result[ADC_result_length];
float32_t ADC_ch2_result[ADC_result_length];
float32_t ADC_ch3_result[ADC_result_length];
float32_t ADC_ch4_result[ADC_result_length];




float32_t ADC_ch1_filtered_result[ADC_result_length];
float32_t ADC_ch2_filtered_result[ADC_result_length];
float32_t ADC_ch3_filtered_result[ADC_result_length];
float32_t ADC_ch4_filtered_result[ADC_result_length];

float32_t rms_array[8];


arm_biquad_casd_df1_inst_f32 S1; 
arm_biquad_casd_df1_inst_f32 S2; 
arm_biquad_casd_df1_inst_f32 S3; 
arm_biquad_casd_df1_inst_f32 S4; 


arm_biquad_casd_df1_inst_f32 LoadCell_S1; 

float32_t IIRStateF32_LoadCell[4*numStages]; 
float32_t LoadCellArray[ADC_result_length];
float32_t LoadCellArray_filtered[ADC_result_length];



static float32_t IIRStateF32[4*numStages];                      /* 状态缓存 */
      
/* 巴特沃斯高通滤波器系数 20-450Hz */                                                                                                                                         
const float32_t IIRCoeffs32HP[5*numStages] = {
1.0,  0.0,  -1.0,  1.822479602696468248268502065911889076233,  -0.837254258763775571772214334487216547132,
1.0,  0.0,  -1.0,  -1.55970814421609649258471108623780310154,   -0.641614681774543060832627361378399655223                           
};                   

float32_t ScaleValue = 0.855583120775896821541550707479473203421*0.855583120775896821541550707479473203421; 


const float32_t IIRCoeffs32LP[5*numStages] = {
1.0f,  2.0f,  1.0f,  1.970162486978045635055423190351575613022f,  -0.971563336669560295710823538684053346515f,
1.0f,  2.0f,  1.0f,  1.931327815655594415389373352809343487024f,  -0.932701052631050631092080038797575980425f                           
};                   

float32_t ScaleValue_LP = 0.000350212422878718777647516491313695042*0.000343309243864080542840006016191978233; 

/*
*********************************************************************************************************
*    函 数 名: arm_iir_f32_hp
*    功能说明: 调用函数arm_iir_f32_hp实现高通滤波器
*    形    参：无
*    返 回 值: 无
*********************************************************************************************************
*/


void arm_iir_init(void){


	arm_biquad_cascade_df1_init_f32(&S1, numStages, (float32_t *)&IIRCoeffs32HP[0],(float32_t *)&IIRStateF32CH1[0]);
	arm_biquad_cascade_df1_init_f32(&S2, numStages, (float32_t *)&IIRCoeffs32HP[0],(float32_t *)&IIRStateF32CH2[0]);
	arm_biquad_cascade_df1_init_f32(&S3, numStages, (float32_t *)&IIRCoeffs32HP[0],(float32_t *)&IIRStateF32CH3[0]);
	arm_biquad_cascade_df1_init_f32(&S4, numStages, (float32_t *)&IIRCoeffs32HP[0],(float32_t *)&IIRStateF32CH4[0]);

}


void LoadCellFilterInit(void){

		arm_biquad_cascade_df1_init_f32(&LoadCell_S1, numStages, (float32_t *)&IIRCoeffs32LP[0],(float32_t *)&IIRStateF32_LoadCell[0]);
	
}



void arm_iir_f32_bp_ch1(void)
{
    uint32_t i;
    
					for(i=0; i < numBlocks; i++)
		{
		arm_biquad_cascade_df1_f32(&S1, &ADC_ch1_result[0]+(i * blockSize), &ADC_ch1_filtered_result[0]+(i * blockSize), blockSize); 
		}

		for(i=0;i<ADC_result_length;i++){
			ADC_ch1_filtered_result[i]=ADC_ch1_filtered_result[i]*ScaleValue;
		}
}

void arm_iir_f32_bp_ch2(void)
{
    uint32_t i;
					for(i=0; i < numBlocks; i++)
		{
		arm_biquad_cascade_df1_f32(&S2, &ADC_ch2_result[0]+(i * blockSize), &ADC_ch2_filtered_result[0]+(i * blockSize), blockSize); 
		}
		
		for(i=0;i<ADC_result_length;i++){
			ADC_ch2_filtered_result[i]=ADC_ch2_filtered_result[i]*ScaleValue;
		}
}

void arm_iir_f32_bp_ch3(void)
{
    uint32_t i;
    
					for(i=0; i < numBlocks; i++)
		{ 
		arm_biquad_cascade_df1_f32(&S3, &ADC_ch3_result[0]+(i * blockSize), &ADC_ch3_filtered_result[0]+(i * blockSize), blockSize); 
		}



		
		for(i=0;i<ADC_result_length;i++){
			ADC_ch3_filtered_result[i]=ADC_ch3_filtered_result[i]*ScaleValue;
		
		}
}

void arm_iir_f32_bp_ch4(void)
{
    uint32_t i;
    
					for(i=0; i < numBlocks; i++)
		{ 
		arm_biquad_cascade_df1_f32(&S4, &ADC_ch4_result[0]+(i * blockSize), &ADC_ch4_filtered_result[0]+(i * blockSize), blockSize); 
		}



		
		for(i=0;i<ADC_result_length;i++){
			ADC_ch4_filtered_result[i]=ADC_ch4_filtered_result[i]*ScaleValue;
		
		}
}
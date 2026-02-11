#ifndef IIR_H
#define IIR_H

#endif

#ifdef __cplusplus
extern "C" {
#endif
	
#include "arm_math.h"
	
#define BPnumStages 2 
	
#define MotorCurrentBufferLen 1
	
	struct CurrentStruct
{
	arm_biquad_casd_df1_inst_f32 IIR_LP; 
	float32_t IIRStateF32LP[4*BPnumStages];
	float32_t SetMotorCurrent[MotorCurrentBufferLen];
	float32_t OutputMotorCurrent[MotorCurrentBufferLen];
};

	
void arm_iir_init_LP(void);
void Motor_arm_iir_f32_LP_Group1(void);
void Motor_arm_iir_f32_LP_Group2(void);
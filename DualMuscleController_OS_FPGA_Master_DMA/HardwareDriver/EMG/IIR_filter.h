#ifndef IIR_FILTER_H
#define IIR_FILTER_H

#endif
#include "arm_math.h"

#ifdef __cplusplus
extern "C" {
#endif

#include "fdacoefs_BP.h"
#include "fdacoefs_BS.h"


//==================IIR Filter=========================


#define ADC_result_length 100

#define numStages 2 

#define BLOCK_SIZE            1

#define EMGChannelNum 4


//===========================================================================



void arm_iir_init(void);
void arm_iir_f32_bp_ch1(void);
void arm_iir_f32_bp_ch2(void);
void arm_iir_f32_bp_ch3(void);
void arm_iir_f32_bp_ch4(void);

void LoadCellFilterInit(void);

#ifdef __cplusplus
}
#endif






#ifndef CONTROLLERIMU_H_
#define CONTROLLERIMU_H_

#include "main.h"

// 传输时转换比例--------------
#define scaleAccel       0.00478515625f // 加速度 [-16g~+16g]    9.8*16/32768
#define scaleQuat        0.000030517578125f // 四元数 [-1~+1]         1/32768
#define scaleAngle       0.0054931640625f // 角度   [-180~+180]     180/32768
#define scaleAngleSpeed  0.06103515625f // 角速度 [-2000~+2000]    2000/32768
#define scaleMag         0.15106201171875f // 磁场 [-4950~+4950]   4950/32768
#define scaleTemperature 0.01f // 温度
#define scaleAirPressure 0.0002384185791f // 气压 [-2000~+2000]    2000/8388608
#define scaleHeight      0.0010728836f    // 高度 [-9000~+9000]    9000/8388608


#define pow2(x) ((x)*(x)) // 求平方

// 基本数据类型用于简化书写
typedef signed char            S8;
typedef unsigned char          U8;
typedef signed short           S16;   
typedef unsigned short         U16;  
typedef signed long            S32;   
typedef unsigned long          U32;  
typedef float                  F32;   


void Cmd_RxUnpack(uint8_t *buf, U8 Dlen);

#endif
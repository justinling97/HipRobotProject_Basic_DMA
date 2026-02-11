#include "stm32h7xx_hal.h"
#define     MODEL_P          1
#define     MODEL_PD         2
#define     MODEL_PID     	 3
 
 
typedef struct
{
    uint8_t choose_model;        
    
    float Sv;     //set value
    float Cv;        //current value
 
    float Kp;        //kp
    float T;      //PID simpling cycle
    uint16_t   Tdata;    //if PID cycle is reached
    float Ti;        //Ti
    float Td;       //Td
    
    
    
    float En;          //error this time
    float En_1;        //error last time
		float En_2;        //error last last time
    float SEk;         //all error
    
    float Iout;        //Iout
    float Pout;        //Pout   
    float Dout;        //Dout
    
    float OUT0;        //maintain output prevent lost control
 
    float OUT;        //Output
    
    
    uint16_t upperlimit;
    
    
}PID;

// PID结构体定义
typedef struct {
    float kp;        // 比例系数
    float ki;        // 积分系数
    float kd;        // 微分系数

    float setpoint;  // 目标值
    float prevError; // 上次误差
    float integral;  // 积分累计值

    float outputMin; // 输出最小值
    float outputMax; // 输出最大值
} PID_Controller;

void PID_Init(PID_Controller *pid, float kp, float ki, float kd, float outputMin, float outputMax);
float PID_Update(PID_Controller *pid, float current, float dt);
 
extern PID pid1; //store PID data
extern PID pid2; //store PID data

int PID1_Calc(float,float,int); //pid calculation
void PID1_Init(void);        //PID init    

int PID2_Calc(float,float,int); //pid calculation
void PID2_Init(void);        //PID init    
 
 
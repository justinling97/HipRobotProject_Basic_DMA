#include "main.h"
#include "pid.h"
 
PID pid1; 
PID pid2; 

#define global_kp 4
#define global_ki 0.1
#define global_kd 20

#define global_upperLimit 1500

void PID1_Init()            
{
    pid1.choose_model = MODEL_PD;
    
    pid1.T = 20;                
    
		pid1.Sv=0;                
    pid1.Kp=global_kp;                
		pid1.Ti=global_ki;            
    pid1.Td=global_kd;                    
    pid1.OUT0=0;                
    
	
		pid1.Pout = pid1.Iout = pid1.Dout = pid1.OUT= 0;
    pid1.upperlimit = global_upperLimit;    
}
    
int PID1_Calc(float PID_Value,float PID_Set, int q1)  
{
    float DelEk;            //the difference between this time to last time
    float ti,ki;
    float td;
    float kd;
    float out;    
		float dk1;
		float dk2;
	
    if(pid1.Tdata < (pid1.T))  //do not reach the smallest cycle
     {	 
            return q1;
     }
    pid1.Tdata = 0;
		pid1.Sv = PID_Set;
		pid1.Cv = PID_Value;
    pid1.En = pid1.Sv-pid1.Cv;               //current error
//		dk1 = pid1.En-pid1.En_1;
//		dk2 = pid1.En-2*pid1.En_1+pid1.En_2; 
		 
    pid1.Pout=pid1.Kp*pid1.En;          //P
		 
    pid1.SEk+=pid1.En;
		DelEk=pid1.En-pid1.En_1;
		ti=pid1.T/pid1.Ti;
    ki=ti*pid1.Kp;
		 
    pid1.Iout=ki*pid1.SEk*pid1.Kp;  //I    
    td=pid1.Td/pid1.T;
    kd=pid1.Kp*td;

		 
		 pid1.Dout=kd*DelEk;  //D

 
    
     switch(pid1.choose_model)
     {
         case MODEL_P:   pid1.OUT= pid1.Pout;                                                
             break;
         
         case MODEL_PD:  pid1.OUT= pid1.Pout+ pid1.Dout;                            
             break;
                 
         case MODEL_PID: pid1.OUT= pid1.Pout+ pid1.Iout+ pid1.Dout;        
             break;
     }
    if (pid1.OUT > global_upperLimit)
		{
			pid1.OUT = global_upperLimit;
		}else if (pid1.OUT < -global_upperLimit)
		{
			pid1.OUT = -global_upperLimit;
		}
		
        /*Make sure the output meets the requirement*/
//		 if(out>pid1.upperlimit)        //limit
//		 {
//				pid1.OUT=pid1.upperlimit;
//		 }
//		 else if(out<0)             //not N
//		 {
//				pid1.OUT=pid1.OUT0; 
//		 }
//		 else 
//		 {
//        pid1.OUT=out;
//     }
     pid1.En_2 = pid1.En_1;
			pid1.En_1 = pid1.En;

     return pid1.OUT;   
}



void PID2_Init()            
{
    pid2.choose_model = MODEL_PD;
    
    pid2.T = 20;                
    
		pid2.Sv=0;                
    pid2.Kp=global_kp;                
		pid2.Ti=global_ki;            
    pid2.Td=global_kd;                    
    pid2.OUT0=0;                
    
	
		pid2.Pout = pid2.Iout = pid2.Dout = pid2.OUT= 0;
    pid2.upperlimit = global_upperLimit;    
}
    
int PID2_Calc(float PID_Value,float PID_Set, int q1)  
{
    float DelEk;            //the difference between this time to last time
    float ti,ki;
    float td;
    float kd;
    float out;    
		float dk1;
		float dk2;
	
    if(pid2.Tdata < (pid2.T))  //do not reach the smallest cycle
     {	 
            return q1;
     }
    pid2.Tdata = 0;
		pid2.Sv = PID_Set;
		pid2.Cv = PID_Value;
    pid2.En = pid2.Sv-pid2.Cv;               //current error
//		dk1 = pid2.En-pid2.En_1;
//		dk2 = pid2.En-2*pid2.En_1+pid2.En_2; 
		 
    pid2.Pout=pid2.Kp*pid2.En;          //P
		 
    pid2.SEk+=pid2.En;
		DelEk=pid2.En-pid2.En_1;
		ti=pid2.T/pid2.Ti;
    ki=ti*pid2.Kp;
		 
    pid2.Iout=ki*pid2.SEk*pid2.Kp;  //I    
    td=pid2.Td/pid2.T;
    kd=pid2.Kp*td;

		 
		 pid2.Dout=kd*DelEk;  //D

 
    
     switch(pid2.choose_model)
     {
         case MODEL_P:   pid2.OUT= pid2.Pout;                                                
             break;
         
         case MODEL_PD:  pid2.OUT= pid2.Pout+ pid2.Dout;                            
             break;
                 
         case MODEL_PID: pid2.OUT= pid2.Pout+ pid2.Iout+ pid2.Dout;        
             break;
     }
    if (pid2.OUT > pid2.upperlimit)
		{
			pid2.OUT = pid2.upperlimit;
		}else if (pid2.OUT < -pid2.upperlimit)
		{
			pid2.OUT = -pid2.upperlimit;
		}
		
        /*Make sure the output meets the requirement*/
//		 if(out>pid2.upperlimit)        //limit
//		 {
//				pid2.OUT=pid2.upperlimit;
//		 }
//		 else if(out<0)             //not N
//		 {
//				pid2.OUT=pid2.OUT0; 
//		 }
//		 else 
//		 {
//        pid2.OUT=out;
//     }
     pid2.En_2 = pid2.En_1;
			pid2.En_1 = pid2.En;

     return pid2.OUT;   
}


// 初始化 PID 控制器
void PID_Init(PID_Controller *pid, float kp, float ki, float kd, float outputMin, float outputMax) {
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;

    pid->setpoint = 0.0f;
    pid->prevError = 0.0f;
    pid->integral = 0.0f;

    pid->outputMin = outputMin;
    pid->outputMax = outputMax;
}

// PID 更新计算
float PID_Update(PID_Controller *pid, float current, float dt) {
    // 计算误差
    float error = pid->setpoint - current;

    // 计算积分（防止积分累积过大导致积分饱和）
    pid->integral += error * dt;
//    if (pid->integral > IntergralMax) pid->integral = pid->outputMax;
//    if (pid->integral < -IntergralMax) pid->integral = pid->outputMin;

    // 计算微分
    float derivative = (error - pid->prevError) / dt;

    // 计算输出
    float output = pid->kp * error + pid->ki * pid->integral + pid->kd * derivative;

    // 限制输出范围
    if (output > pid->outputMax) output = pid->outputMax;
    if (output < pid->outputMin) output = pid->outputMin;

    // 保存当前误差以备下次使用
    pid->prevError = error;

    return output;
}
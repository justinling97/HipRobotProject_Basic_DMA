#include "dynamixel_rd.h"
#include "stm32h7xx_hal.h"
#include "usart.h"

#define Mode_Position 	0
#define Mode_Velocity 	1
#define Mode_Current	 	2

#define Motor_Buffer_Len	64
#define Motor_Buffer_quantity 5


#define LPnumStages 2 

extern unsigned int Mode_Present;
extern int mode_flag;

void Motor_Set_Mode(unsigned int motor_mode);
void Motor1_Set_Position(unsigned int Mot_Pos_1, unsigned int Mot_Pos_2, unsigned int Mot_Pos_3, unsigned int Mot_Pos_4);
void Motor1_Set_Current(int Mot_Cur_1, int Mot_Cur_2, int Mot_Cur_3, int Mot_Cur_4);
void Motor1_Set_Velocity(int Mot_Velo_1, int Mot_Velo_2, int Mot_Velo_3, int Mot_Velo_4);


void Motor2_Set_Position(unsigned int Mot_Pos_1, unsigned int Mot_Pos_2, unsigned int Mot_Pos_3, unsigned int Mot_Pos_4);
void Motor2_Set_Current(int Mot_Cur_1, int Mot_Cur_2, int Mot_Cur_3, int Mot_Cur_4);
void Motor2_Set_Velocity(int Mot_Velo_1, int Mot_Velo_2, int Mot_Velo_3, int Mot_Velo_4);

int Driver1_Receive(void);
int Driver2_Receive(void);

int DriverfindStr(uint8_t* ESPreport, uint8_t* Template, uint8_t ReportLen, uint8_t TemplateLen);

void MotorRetCMDGen(void);
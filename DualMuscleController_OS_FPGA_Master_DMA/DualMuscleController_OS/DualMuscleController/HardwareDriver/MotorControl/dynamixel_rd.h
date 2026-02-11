#ifndef __DYNAMIXEL_RD_H
#define	__DYNAMIXEL_RD_H

#define Dynamixel_Reserved 0x00// reserved

/*********Instruction************/
#define Dynamixel_Instruction_Read 0x02         //Instruction read
#define Dynamixel_Instruction_Write 0x03       //Instruction Write
#define Dynamixel_Instruction_Return 0x55       //Instruction return
#define Dynamixel_Instruction_Error 0x66        //Instruction error


#define Dynamixel_State_Success      0
#define Dynamixel_State_Incomplete   0x01     //error
#define Dynamixel_State_overrange    0x02     //error
#define Dynamixel_State_Error        0x03

#define Dynamixel_Address_PresentCurrent1       0
#define Dynamixel_Address_PresentCurrent2       2
#define Dynamixel_Address_PresentCurrent3       4
#define Dynamixel_Address_PresentCurrent4       6

#define Dynamixel_Address_PresentPosition1      8
#define Dynamixel_Address_PresentPosition2      12
#define Dynamixel_Address_PresentPosition3      16
#define Dynamixel_Address_PresentPosition4      20

#define Dynamixel_Address_PresentVelocity1      24
#define Dynamixel_Address_PresentVelocity2      26
#define Dynamixel_Address_PresentVelocity3      28
#define Dynamixel_Address_PresentVelocity4      30
/*****************************************************************/
#define Dynamixel_Address_GoalCurrent1          32
#define Dynamixel_Address_GoalCurrent2          34
#define Dynamixel_Address_GoalCurrent3          36
#define Dynamixel_Address_GoalCurrent4          38

#define Dynamixel_Address_GoalPosition1         40
#define Dynamixel_Address_GoalPosition2         44
#define Dynamixel_Address_GoalPosition3         48
#define Dynamixel_Address_GoalPosition4         52

#define Dynamixel_Address_GoalVelocity1         56
#define Dynamixel_Address_GoalVelocity2         58
#define Dynamixel_Address_GoalVelocity3         60
#define Dynamixel_Address_GoalVelocity4         62

#define Dynamixel_Address_Mode        			64

#define Dynamixel_Return_Position_Value_Max 150000
#define Dynamixel_Return_Success 150001
#define Dynamixel_Return_Error   150002

extern unsigned char Data_Buffer[90];

int Dynamixel_Value_Forward(unsigned char *data_blk_ptr, unsigned char data_blk_size_start, unsigned char data_blk_size_end);
void  Dynamixel_Value_Dackward(int Value, unsigned char data_blk_size, unsigned char *Return_data);
unsigned short Dynamixel_update_crc(unsigned short crc_accum, unsigned char *data_blk_ptr, unsigned short data_blk_size);
unsigned char  CheckSum(unsigned char uData[], unsigned char leng);
unsigned int Dynamixel_Master_Receive(unsigned char *Data, unsigned char Data_Length, unsigned char *Return_Data);
unsigned int Dynamixel_Slave_Receive(unsigned char *Data, unsigned char Data_Length, unsigned char *Return_Data);
unsigned char Dynamixel_Slave_Send( unsigned char *DataBuffer, unsigned char Address, unsigned char data_length, unsigned char *Return_Data);
unsigned char Dynamixel_Master_Send(unsigned char Instruction, unsigned short Address, unsigned int *Value,
                                    unsigned char Value_Length, unsigned char *Return_Data);
void Dynamixel_Buffer_Write(unsigned int Address, int Value, unsigned char Length);
#endif//

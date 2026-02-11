#ifndef FPGA_MOTOR_DRIVER_H
#define FPGA_MOTOR_DRIVER_H

#include "main.h"
#include "usart.h"

// Protocol Constants
#define FRAME_HEADER_1    0xAA
#define FRAME_HEADER_2    0xBB
#define FRAME_HEADER_3    0x00
#define FRAME_LENGTH      0x23
#define FRAME_TYPE_CONTROL 0x03
#define FRAME_TYPE_FEEDBACK 0x55
#define FRAME_ADDRESS     0x28

// Motor Mode Constants
#define MOTOR_MODE_STOP    0x00
#define MOTOR_MODE_POSITION 0x01
#define MOTOR_MODE_SPEED   0x02
#define MOTOR_MODE_CURRENT 0x03

// Frame size constants
#define FRAME_TOTAL_SIZE  0x27  // bytes //39
#define RX_CACHE_SIZE    (FRAME_TOTAL_SIZE * 3)
#define MOTOR_COUNT       4
#define RX_BUFFER_SIZE 256  // 足够大，防止溢出

// 参数转换常量
#define POSITION_TO_TURNS     46.0f//12.0f*3.75f    // 位置值到实际圈数的转换比例//这里原来是1024，已修改
#define SPEED_TO_RPM         (7.5f/60.0f)  // 速度值到实际转速(RPM)的转换比例
#define CURRENT_TO_AMPERE    ((3.3f/4096.0f) * 5.0f)  // 电流值到实际电流(A)的转换比例

// Structure for motor data (实际值)
typedef struct {
    uint8_t mode;
    float position_turns;    // 实际位置（圈数）
    float speed_rpm;        // 实际速度（转/分钟）
    float current_ampere;   // 实际电流（安培）
} MotorActualData;

// Structure for motor data (原始数据)
typedef struct {
    uint8_t mode;
    int32_t position;       // 24位位置值
    int16_t speed;         	// 16位速度值
    int16_t current;       	// 16位电流值
} MotorData;

// Structure for complete frame
typedef struct {
    MotorData motors[MOTOR_COUNT];
    MotorActualData actual_motors[MOTOR_COUNT];  // 添加实际值数据
} MotorFrame;
//Motor Group1
// Function declarations
void InitMotorControl_Group1(void);
void SendMotorControlFrame_Group1(void);
void ProcessFeedbackFrame_Group1(uint8_t *data);
void PrintMotorStatus(void);
uint8_t CalculateChecksum(uint8_t *data, uint16_t length);

// 电机控制函数（使用实际值）
void SetMotorMode_Group1(uint8_t motorIndex, uint8_t mode);
void SetMotorPositionTurns_Group1(uint8_t motorIndex, float turns);
void SetMotorSpeedRPM_Group1(uint8_t motorIndex, float rpm);
void SetMotorCurrentAmpere_Group1(uint8_t motorIndex, float ampere);
void StopMotor_Group1(uint8_t motorIndex);
void StopAllMotors_Group1(void);

//Motor Group2
// Function declarations
void InitMotorControl_Group2(void);
void SendMotorControlFrame_Group2(void);
void ProcessFeedbackFrame_Group2(uint8_t *data);
void PrintMotorStatus(void);
uint8_t CalculateChecksum(uint8_t *data, uint16_t length);

// 电机控制函数（使用实际值）
void SetMotorMode_Group2(uint8_t motorIndex, uint8_t mode);
void SetMotorPositionTurns_Group2(uint8_t motorIndex, float turns);
void SetMotorSpeedRPM_Group2(uint8_t motorIndex, float rpm);
void SetMotorCurrentAmpere_Group2(uint8_t motorIndex, float ampere);
void StopMotor_Group2(uint8_t motorIndex);
void StopAllMotors_Group2(void);



// 数据转换函数
int32_t TurnsToPosition(float turns);
int16_t RPMToSpeed(float rpm);
int16_t AmpereToCurrentValue(float ampere);
float PositionToTurns(int32_t position);
float SpeedToRPM(int16_t speed);
float CurrentToAmpere(int16_t current);

extern UART_HandleTypeDef huart5;  // Motor communication
extern UART_HandleTypeDef huart7;  // Debug output
extern MotorFrame g_motorStatus;
//extern uint8_t g_rxBuffer[FRAME_TOTAL_SIZE];



// 全局变量声明
extern uint8_t g_txBuffer[];
extern char g_printBuffer[];
extern uint8_t g_rxCache[];
extern volatile uint16_t g_rxCacheIndex;
extern volatile uint8_t g_dataReady;
extern uint8_t g_rxByte;

#endif // MOTOR_PROTOCOL_H




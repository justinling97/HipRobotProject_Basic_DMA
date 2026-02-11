#include "FPGA_motor_driver.h"
#include <string.h>
#include <stdio.h>

// 全局变量定义
MotorFrame g_motorStatus;           // 电机状态数据结构

MotorFrame g_Set_motorStatus_Group1;           // 电机状态数据结构
MotorFrame g_Set_motorStatus_Group2;           // 电机状态数据结构


uint8_t g_txBuffer_Group1[FRAME_TOTAL_SIZE];  // 发送缓冲区
uint8_t g_txBuffer_Group2[FRAME_TOTAL_SIZE];  // 发送缓冲区
char g_printBuffer[128];           // 打印信息缓冲区

MotorFrame g_motorStatus_feedback_Group1;           // 电机状态反馈数据结构
MotorFrame g_motorStatus_feedback_Group2;           // 电机状态反馈数据结构


uint8_t g_rxCache[RX_CACHE_SIZE]; // 大缓存区
volatile uint16_t g_rxCacheIndex = 0; // 当前写入位置
volatile uint8_t g_dataReady = 0; // 标志：缓存满，主循环可处理
uint8_t g_rxByte; // 单字节接收缓冲

/**
 * @brief 将实际圈数转换为位置值
 * @param turns: 实际圈数
 * @return 位置值（24位）
 * @note 1圈 = 1024个位置单位
 */
int32_t TurnsToPosition(float turns)
{
    return (int32_t)(turns * POSITION_TO_TURNS);
}
/**
 * @brief 将位置值转换为实际圈数
 * @param position: 位置值（24位）
 * @return 实际圈数
 */
float PositionToTurns(int32_t position)
{
	float position_f=0;
	position_f=(float)position;
    return position_f / POSITION_TO_TURNS;
}


/**
 * @brief 将实际转速(RPM)转换为速度值
 * @param rpm: 实际转速（转/分钟）RPM
 * @return 速度值（16位）
 * @note 速度值 = 实际RPM * (1/60 * 7.5)
 */
int16_t RPMToSpeed(float rpm)
{
    return (int16_t)(rpm * SPEED_TO_RPM);
}
/**
 * @brief 将速度值转换为实际转速(RPM)
 * @param speed: 速度值（16位）
 * @return 实际转速（转/分钟）
 */
float SpeedToRPM(int16_t speed)
{
    return (float)(speed / SPEED_TO_RPM);
}







/**
 * @brief 将实际电流值(A)转换为电流值
 * @param ampere: 实际电流（安培）
 * @return 电流值（16位）
 * @note 实际电流 = 电流值 * ((3.3/4096) * 5)
 */
int16_t AmpereToCurrentValue(float ampere)
{
    return (int16_t)(ampere / CURRENT_TO_AMPERE);
}
/**
 * @brief 将电流值转换为实际电流(A)
 * @param current: 电流值（16位）
 * @return 实际电流（安培）
 */
float CurrentToAmpere(int16_t current)
{
    return (float)current * CURRENT_TO_AMPERE;
}


/**
 * @brief 初始化电机控制
 */
void InitMotorControl_Group1(void)
{
	
	g_rxCacheIndex = 0;
	g_dataReady = 0;	
	HAL_UART_Receive_IT(&huart5, &g_rxByte, 1);
	
	// 清零所有电机状态
	memset(&g_Set_motorStatus_Group1, 0, sizeof(g_Set_motorStatus_Group1));	

}

void InitMotorControl_Group2(void)
{
	
	g_rxCacheIndex = 0;
	g_dataReady = 0;	
	HAL_UART_Receive_IT(&huart7, &g_rxByte, 1);
	
	// 清零所有电机状态
	memset(&g_Set_motorStatus_Group2, 0, sizeof(g_Set_motorStatus_Group2));	

}


/**
 * @brief 设置指定电机的运行模式
 * @param motorIndex: 电机索引（0-3）
 * @param mode: 运行模式（停止/位置/速度/电流）
 */
void SetMotorMode_Group1(uint8_t motorIndex, uint8_t mode)
{
    if(motorIndex < MOTOR_COUNT) {
        g_Set_motorStatus_Group1.motors[motorIndex].mode = mode;
        g_Set_motorStatus_Group1.actual_motors[motorIndex].mode = mode;
    }
}

void SetMotorMode_Group2(uint8_t motorIndex, uint8_t mode)
{
    if(motorIndex < MOTOR_COUNT) {
        g_Set_motorStatus_Group2.motors[motorIndex].mode = mode;
        g_Set_motorStatus_Group2.actual_motors[motorIndex].mode = mode;
    }
}

/**
 * @brief 设置指定电机的目标位置（圈数）
 * @param motorIndex: 电机索引（0-3）
 * @param turns: 目标圈数
 */
void SetMotorPositionTurns_Group1(uint8_t motorIndex, float turns)
{
    if(motorIndex < MOTOR_COUNT) {
        g_Set_motorStatus_Group1.motors[motorIndex].position = TurnsToPosition(turns);
        g_Set_motorStatus_Group1.actual_motors[motorIndex].position_turns = turns;
    }
}

void SetMotorPositionTurns_Group2(uint8_t motorIndex, float turns)
{
    if(motorIndex < MOTOR_COUNT) {
        g_Set_motorStatus_Group2.motors[motorIndex].position = TurnsToPosition(turns);
        g_Set_motorStatus_Group2.actual_motors[motorIndex].position_turns = turns;
    }
}


/**
 * @brief 设置指定电机的目标速度（RPM）
 * @param motorIndex: 电机索引（0-3）
 * @param rpm: 目标转速（转/分钟）
 */

int aflag=0;
void SetMotorSpeedRPM_Group1(uint8_t motorIndex, float rpm)
{
    if(motorIndex < MOTOR_COUNT) {
        g_Set_motorStatus_Group1.motors[motorIndex].speed = RPMToSpeed(rpm);
        g_Set_motorStatus_Group1.actual_motors[motorIndex].speed_rpm = rpm;
    }
		
}

void SetMotorSpeedRPM_Group2(uint8_t motorIndex, float rpm)
{
    if(motorIndex < MOTOR_COUNT) {
        g_Set_motorStatus_Group2.motors[motorIndex].speed = RPMToSpeed(rpm);
        g_Set_motorStatus_Group2.actual_motors[motorIndex].speed_rpm = rpm;
    }
		
}

/**
 * @brief 设置指定电机的电流限制（A）
 * @param motorIndex: 电机索引（0-3）
 * @param ampere: 电流限制（安培）
 */
void SetMotorCurrentAmpere_Group1(uint8_t motorIndex, float ampere)
{
    if(motorIndex < MOTOR_COUNT) {
        g_Set_motorStatus_Group1.motors[motorIndex].current = AmpereToCurrentValue(ampere);
        g_Set_motorStatus_Group1.actual_motors[motorIndex].current_ampere = ampere;
    }
}

void SetMotorCurrentAmpere_Group2(uint8_t motorIndex, float ampere)
{
    if(motorIndex < MOTOR_COUNT) {
        g_Set_motorStatus_Group2.motors[motorIndex].current = AmpereToCurrentValue(ampere);
        g_Set_motorStatus_Group2.actual_motors[motorIndex].current_ampere = ampere;
    }
}

/**
 * @brief 停止指定电机
 * @param motorIndex: 电机索引（0-3）
 */
void StopMotor_Group1(uint8_t motorIndex)
{
    if(motorIndex < MOTOR_COUNT) {
        g_Set_motorStatus_Group1.motors[motorIndex].mode = MOTOR_MODE_STOP;
        g_Set_motorStatus_Group1.motors[motorIndex].speed = 0;
        g_Set_motorStatus_Group1.motors[motorIndex].current = 0;
        g_Set_motorStatus_Group1.actual_motors[motorIndex].mode = MOTOR_MODE_STOP;
        g_Set_motorStatus_Group1.actual_motors[motorIndex].speed_rpm = 0;
        g_Set_motorStatus_Group1.actual_motors[motorIndex].current_ampere = 0;
    }
}

void StopMotor_Group2(uint8_t motorIndex)
{
    if(motorIndex < MOTOR_COUNT) {
        g_Set_motorStatus_Group2.motors[motorIndex].mode = MOTOR_MODE_STOP;
        g_Set_motorStatus_Group2.motors[motorIndex].speed = 0;
        g_Set_motorStatus_Group2.motors[motorIndex].current = 0;
        g_Set_motorStatus_Group2.actual_motors[motorIndex].mode = MOTOR_MODE_STOP;
        g_Set_motorStatus_Group2.actual_motors[motorIndex].speed_rpm = 0;
        g_Set_motorStatus_Group2.actual_motors[motorIndex].current_ampere = 0;
    }
}

/**
 * @brief 停止所有电机
 */
void StopAllMotors_Group1(void)
{
    for(uint8_t i = 0; i < MOTOR_COUNT; i++) {
        StopMotor_Group1(i);
    }
}

void StopAllMotors_Group2(void)
{
    for(uint8_t i = 0; i < MOTOR_COUNT; i++) {
        StopMotor_Group2(i);
    }
}


/**
 * @brief 发送电机控制数据帧
 */
void SendMotorControlFrame_Group1(void)
{
    int index = 0;
    
    // 填充帧头
    g_txBuffer_Group1[index++] = FRAME_HEADER_1;
    g_txBuffer_Group1[index++] = FRAME_HEADER_2;
    g_txBuffer_Group1[index++] = FRAME_HEADER_3;
    g_txBuffer_Group1[index++] = FRAME_LENGTH;
    g_txBuffer_Group1[index++] = FRAME_TYPE_CONTROL;
    g_txBuffer_Group1[index++] = FRAME_ADDRESS;
    
    // 填充每个电机的数据
    for(int i = 0; i < MOTOR_COUNT; i++) {
        // 运动模式（1字节）
        g_txBuffer_Group1[index++] = g_Set_motorStatus_Group1.motors[i].mode;
        
        // 位置设定（3字节，24位）
        g_txBuffer_Group1[index++] = (g_Set_motorStatus_Group1.motors[i].position >> 0)  & 0xFF;
        g_txBuffer_Group1[index++] = (g_Set_motorStatus_Group1.motors[i].position >> 8)  & 0xFF;
        g_txBuffer_Group1[index++] = (g_Set_motorStatus_Group1.motors[i].position >> 16) & 0xFF;
        
        // 速度设定（2字节，16位）
        g_txBuffer_Group1[index++] = (g_Set_motorStatus_Group1.motors[i].speed >> 0) & 0xFF;
        g_txBuffer_Group1[index++] = (g_Set_motorStatus_Group1.motors[i].speed >> 8) & 0xFF;
        
        // 电流设定（2字节，16位）
        g_txBuffer_Group1[index++] = (g_Set_motorStatus_Group1.motors[i].current >> 0) & 0xFF;
        g_txBuffer_Group1[index++] = (g_Set_motorStatus_Group1.motors[i].current >> 8) & 0xFF;
    }
    
    // 计算并添加校验和
    g_txBuffer_Group1[index] = CalculateChecksum(g_txBuffer_Group1, index);
    
    // 通过UART5发送数据帧
		HAL_UART_Transmit_IT(&huart5, g_txBuffer_Group1, 39);
}

void SendMotorControlFrame_Group2(void)
{
    int index = 0;
    
    // 填充帧头
    g_txBuffer_Group2[index++] = FRAME_HEADER_1;
    g_txBuffer_Group2[index++] = FRAME_HEADER_2;
    g_txBuffer_Group2[index++] = FRAME_HEADER_3;
    g_txBuffer_Group2[index++] = FRAME_LENGTH;
    g_txBuffer_Group2[index++] = FRAME_TYPE_CONTROL;
    g_txBuffer_Group2[index++] = FRAME_ADDRESS;
    
    // 填充每个电机的数据
    for(int i = 0; i < MOTOR_COUNT; i++) {
        // 运动模式（1字节）
        g_txBuffer_Group2[index++] = g_Set_motorStatus_Group2.motors[i].mode;
        
        // 位置设定（3字节，24位）
        g_txBuffer_Group2[index++] = (g_Set_motorStatus_Group2.motors[i].position >> 0)  & 0xFF;
        g_txBuffer_Group2[index++] = (g_Set_motorStatus_Group2.motors[i].position >> 8)  & 0xFF;
        g_txBuffer_Group2[index++] = (g_Set_motorStatus_Group2.motors[i].position >> 16) & 0xFF;
        
        // 速度设定（2字节，16位）
        g_txBuffer_Group2[index++] = (g_Set_motorStatus_Group2.motors[i].speed >> 0) & 0xFF;
        g_txBuffer_Group2[index++] = (g_Set_motorStatus_Group2.motors[i].speed >> 8) & 0xFF;
        
        // 电流设定（2字节，16位）
        g_txBuffer_Group2[index++] = (g_Set_motorStatus_Group2.motors[i].current >> 0) & 0xFF;
        g_txBuffer_Group2[index++] = (g_Set_motorStatus_Group2.motors[i].current >> 8) & 0xFF;
    }
    
    // 计算并添加校验和
    g_txBuffer_Group2[index] = CalculateChecksum(g_txBuffer_Group2, index);
    
    // 通过UART5发送数据帧
		HAL_UART_Transmit_IT(&huart7, g_txBuffer_Group2, 39);
}

/**
 * @brief 处理接收到的反馈数据帧
 * @param data: 接收到的数据帧
 */
void ProcessFeedbackFrame_Group1(uint8_t *data)
{
    // 验证帧头和帧类型
    if(data[0] != FRAME_HEADER_1 || data[1] != FRAME_HEADER_2 || data[2] != FRAME_HEADER_3 ) 
	{
        return;
    }
    
    // 验证校验和
    uint8_t checksum = CalculateChecksum(data, FRAME_TOTAL_SIZE - 1);
    if(checksum != data[FRAME_TOTAL_SIZE - 1]) {
        return;
    }
    
    int index = 6; // 跳过帧头和控制字节
    // 解析每个电机的数据
    for(int i = 0; i < MOTOR_COUNT; i++) {
        // 运动模式（1字节）		
        g_motorStatus_feedback_Group1.motors[i].mode = data[index++];
        
        // 位置信息（3字节，24位）
		uint32_t rawPos = ((uint32_t)data[index] << 0) | ((uint32_t)data[index + 1] << 8) | ((uint32_t)data[index + 2] << 16);
		int32_t position;// 符号扩展到 32 位（关键：判断第 23 位）
		if (rawPos & (1 << 23)) {
			position = rawPos | 0xFF000000; // 高位补 1，还原负数
		} else {
			position = rawPos; // 高位补 0，保持正数
		}
		g_motorStatus_feedback_Group1.motors[i].position = position;
		index += 3;		
	
        
        // 速度信息（2字节，16位）
        g_motorStatus_feedback_Group1.motors[i].speed    = ((int16_t)data[index] << 0  ) | (data[index + 1] << 8 );
        index += 2;
        
        // 电流信息（2字节，16位）
        g_motorStatus_feedback_Group1.motors[i].current  = ((int16_t)data[index] << 0  ) | (data[index + 1] << 8 );
        index += 2;
		
        // 更新实际值
        g_motorStatus_feedback_Group1.actual_motors[i].mode 			= g_motorStatus_feedback_Group1.motors[i].mode;
        g_motorStatus_feedback_Group1.actual_motors[i].position_turns 	= PositionToTurns(g_motorStatus_feedback_Group1.motors[i].position	);
        g_motorStatus_feedback_Group1.actual_motors[i].speed_rpm 		= SpeedToRPM     (g_motorStatus_feedback_Group1.motors[i].speed		);
        g_motorStatus_feedback_Group1.actual_motors[i].current_ampere 	= CurrentToAmpere(g_motorStatus_feedback_Group1.motors[i].current   );
    }
    
    // 打印电机状态
    //PrintMotorStatus();
}


void ProcessFeedbackFrame_Group2(uint8_t *data)
{
    // 验证帧头和帧类型
    if(data[0] != FRAME_HEADER_1 || data[1] != FRAME_HEADER_2 || data[2] != FRAME_HEADER_3 ) 
	{
        return;
    }
    
    // 验证校验和
    uint8_t checksum = CalculateChecksum(data, FRAME_TOTAL_SIZE - 1);
    if(checksum != data[FRAME_TOTAL_SIZE - 1]) {
        return;
    }
    
    int index = 6; // 跳过帧头和控制字节
    // 解析每个电机的数据
    for(int i = 0; i < MOTOR_COUNT; i++) {
        // 运动模式（1字节）		
        g_motorStatus_feedback_Group2.motors[i].mode = data[index++];
        
        // 位置信息（3字节，24位）
		uint32_t rawPos = ((uint32_t)data[index] << 0) | ((uint32_t)data[index + 1] << 8) | ((uint32_t)data[index + 2] << 16);
		int32_t position;// 符号扩展到 32 位（关键：判断第 23 位）
		if (rawPos & (1 << 23)) {
			position = rawPos | 0xFF000000; // 高位补 1，还原负数
		} else {
			position = rawPos; // 高位补 0，保持正数
		}
		g_motorStatus_feedback_Group2.motors[i].position = position;
		index += 3;		
	
        
        // 速度信息（2字节，16位）
        g_motorStatus_feedback_Group2.motors[i].speed    = ((int16_t)data[index] << 0  ) | (data[index + 1] << 8 );
        index += 2;
        
        // 电流信息（2字节，16位）
        g_motorStatus_feedback_Group2.motors[i].current  = ((int16_t)data[index] << 0  ) | (data[index + 1] << 8 );
        index += 2;
		
        // 更新实际值
        g_motorStatus_feedback_Group2.actual_motors[i].mode 			= g_motorStatus_feedback_Group2.motors[i].mode;
        g_motorStatus_feedback_Group2.actual_motors[i].position_turns 	= PositionToTurns(g_motorStatus_feedback_Group2.motors[i].position	);
        g_motorStatus_feedback_Group2.actual_motors[i].speed_rpm 		= SpeedToRPM     (g_motorStatus_feedback_Group2.motors[i].speed		);
        g_motorStatus_feedback_Group2.actual_motors[i].current_ampere 	= CurrentToAmpere(g_motorStatus_feedback_Group2.motors[i].current   );
    }
    
    // 打印电机状态
   // PrintMotorStatus();
}


/**
 * @brief 打印电机状态信息
 */
//void PrintMotorStatus(void)
//{
//    for(int i = 0; i < MOTOR_COUNT; i++) {
//        sprintf(g_printBuffer, 
//                "Motor%d: mode=%d, pos=%.3f turns, speed=%.2f rpm, current=%.3f A\r\n",
//                i + 1,
//                g_motorStatus.actual_motors[i].mode,
//                g_motorStatus.actual_motors[i].position_turns,
//                g_motorStatus.actual_motors[i].speed_rpm,
//                g_motorStatus.actual_motors[i].current_ampere);
//		
//        HAL_UART_Transmit(&huart7, (uint8_t*)g_printBuffer, strlen(g_printBuffer), 1000);
//    }
//    HAL_UART_Transmit(&huart7, (uint8_t*)"\r\n", 2, 100);
//}

/**
 * @brief 计算数据校验和
 * @param data: 数据数组
 * @param length: 数据长度
 * @return 校验和
 */

// Dynamixel CRC查找表（256项）
const uint16_t Dynamixel_crc_table[256] = {
    0x0000, 0x8005, 0x800F, 0x000A, 0x801B, 0x001E, 0x0014, 0x8011,
    0x8033, 0x0036, 0x003C, 0x8039, 0x0028, 0x802D, 0x8027, 0x0022,
    0x8063, 0x0066, 0x006C, 0x8069, 0x0078, 0x807D, 0x8077, 0x0072,
    0x0050, 0x8055, 0x805F, 0x005A, 0x804B, 0x004E, 0x0044, 0x8041,
    0x80C3, 0x00C6, 0x00CC, 0x80C9, 0x00D8, 0x80DD, 0x80D7, 0x00D2,
    0x00F0, 0x80F5, 0x80FF, 0x00FA, 0x80EB, 0x00EE, 0x00E4, 0x80E1,
    0x00A0, 0x80A5, 0x80AF, 0x00AA, 0x80BB, 0x00BE, 0x00B4, 0x80B1,
    0x8093, 0x0096, 0x009C, 0x8099, 0x0088, 0x808D, 0x8087, 0x0082,
    0x8183, 0x0186, 0x018C, 0x8189, 0x0198, 0x819D, 0x8197, 0x0192,
    0x01B0, 0x81B5, 0x81BF, 0x01BA, 0x81AB, 0x01AE, 0x01A4, 0x81A1,
    0x01E0, 0x81E5, 0x81EF, 0x01EA, 0x81FB, 0x01FE, 0x01F4, 0x81F1,
    0x81D3, 0x01D6, 0x01DC, 0x81D9, 0x01C8, 0x81CD, 0x81C7, 0x01C2,
    0x0140, 0x8145, 0x814F, 0x014A, 0x815B, 0x015E, 0x0154, 0x8151,
    0x8173, 0x0176, 0x017C, 0x8179, 0x0168, 0x816D, 0x8167, 0x0162,
    0x8123, 0x0126, 0x012C, 0x8129, 0x0138, 0x813D, 0x8137, 0x0132,
    0x0110, 0x8115, 0x811F, 0x011A, 0x810B, 0x010E, 0x0104, 0x8101,
    0x8303, 0x0306, 0x030C, 0x8309, 0x0318, 0x831D, 0x8317, 0x0312,
    0x0330, 0x8335, 0x833F, 0x033A, 0x832B, 0x032E, 0x0324, 0x8321,
    0x0360, 0x8365, 0x836F, 0x036A, 0x837B, 0x037E, 0x0374, 0x8371,
    0x8353, 0x0356, 0x035C, 0x8359, 0x0348, 0x834D, 0x8347, 0x0342,
    0x03C0, 0x83C5, 0x83CF, 0x03CA, 0x83DB, 0x03DE, 0x03D4, 0x83D1,
    0x83F3, 0x03F6, 0x03FC, 0x83F9, 0x03E8, 0x83ED, 0x83E7, 0x03E2,
    0x83A3, 0x03A6, 0x03AC, 0x83A9, 0x03B8, 0x83BD, 0x83B7, 0x03B2,
    0x0390, 0x8395, 0x839F, 0x039A, 0x838B, 0x038E, 0x0384, 0x8381,
    0x0280, 0x8285, 0x828F, 0x028A, 0x829B, 0x029E, 0x0294, 0x8291,
    0x82B3, 0x02B6, 0x02BC, 0x82B9, 0x02A8, 0x82AD, 0x82A7, 0x02A2,
    0x82E3, 0x02E6, 0x02EC, 0x82E9, 0x02F8, 0x82FD, 0x82F7, 0x02F2,
    0x02D0, 0x82D5, 0x82DF, 0x02DA, 0x82CB, 0x02CE, 0x02C4, 0x82C1,
    0x8243, 0x0246, 0x024C, 0x8249, 0x0258, 0x825D, 0x8257, 0x0252,
    0x0270, 0x8275, 0x827F, 0x027A, 0x826B, 0x026E, 0x0264, 0x8261,
    0x0220, 0x8225, 0x822F, 0x022A, 0x823B, 0x023E, 0x0234, 0x8231,
    0x8213, 0x0216, 0x021C, 0x8219, 0x0208, 0x820D, 0x8207, 0x0202
};

// CRC校验函数
uint8_t CalculateChecksum(uint8_t *dataFrameCmdSend, uint16_t length) {
	
	
    uint16_t crc_accum = 0;
    for (uint16_t j = 3; j < length; j++) {
        uint8_t i = ((crc_accum >> 8)^dataFrameCmdSend[j])&0xFF;
        crc_accum = ((crc_accum << 8)^Dynamixel_crc_table[i])%65536;
    }
    crc_accum = crc_accum % 256;
    return (uint8_t)crc_accum;
}























#ifndef __ESP32_H_
#define __ESP32_H_

#include "main.h"
#include <stdio.h>
 
#ifdef __CPLUSPLUS
extern "C" {
#endif
	
#define TEXT_LENTH 100		//TEXT_TO_SEND字符串长度(不包含结束符)

#define DATA_LENTH  300			//TEXT_TO_SEND字符串长度(不包含结束符)
	
#define ESP32_RXLen 100
#define ESP32_TXLen 100
	
void ESP32Init(void);
void ESP32ConnectAPFun(uint8_t* SensorIP);
void ESP32ServerConnect(void);
void ESP32SendIP(uint8_t* EMG_IP, uint8_t SensorNum);
void ESP32DeepSleep(void);
int findStr(uint8_t* ESPreport, uint8_t* Template, uint8_t ReportLen, uint8_t TemplateLen);
void CommandGenFUN(uint8_t* SendBuff, uint8_t* Command, uint8_t TextLen,unsigned char CommandLen);
void ClearUSARTTXBuffer(void);
void ClearUSARTRXBuffer(void);
int LookESPError(uint8_t* RXBuffer);
 
#endif  




	
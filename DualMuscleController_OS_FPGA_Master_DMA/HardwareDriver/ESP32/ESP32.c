#include "main.h"
#include <stdio.h>
#include "esp32.h"
#include "usart.h"
#include "string.h"
#include "rtc.h"

uint8_t ESP32ExitTransit[]={"+++"};
uint8_t ESP32Restart[]={"AT+RST"};

uint8_t ESP32StartCommand[]={"AT"};
uint8_t ESP32SetStation[]={"AT+CWMODE=1"};
uint8_t ESP32ConnectAP[]={"AT+CWJAP=\"iReachEMGAP2\",\"iReachLab\""};//The receiver AP should not be changed
uint8_t ESP32EnquireIP[]={"AT+CIPSTA?"};
uint8_t ESP32StateRET[]={"AT+SYSMSG=4"};

uint8_t ESP32ConnectServer[]={"AT+CIPSTART=\"TCP\",\"192.168.101.126\",9001"};
uint8_t ESP32TCPTransitRX[]={"AT+CIPMODE=1"};
uint8_t ESP32TCPTransitTX[]={"AT+CIPSEND"};

uint8_t ESP32TCPTransitRXExit[]={"AT+CIPMODE=0"};
uint8_t ESP32TCPExit[]={"AT+CIPCLOSE"};

uint8_t ESP32DeepSleepcmd[]={"AT+GSLP=600000"};//进入Deep-sleep状态10分钟

uint8_t ESPOKStr[]={"OK"};
uint8_t ESPCONNECTEDStr[]={"CONNECTED"};

uint8_t totalPointer=0;

uint8_t ESPSetSNTPServer[]={"AT+CIPSNTPCFG=1,8,\"cn.ntp.org.cn\",\"ntp.sjtu.edu.cn\""};
uint8_t ESPEnqTime[]={"AT+CIPSNTPTIME?"};



int ipPointer=0;


extern UART_HandleTypeDef huart2;
extern DMA_HandleTypeDef hdma_usart2_rx;
extern DMA_HandleTypeDef hdma_usart2_tx;


uint8_t ESP32_UART_RXBuffer[ESP32_RXLen];
uint8_t ESP32_UART_TXBuffer[ESP32_TXLen];
int findStrResult=0;

int OKPosition=0;

extern RTC_TimeTypeDef rtcTime;
extern RTC_DateTypeDef rtcDate;

extern RTC_HandleTypeDef hrtc;


void ESP32Init(void){
	
	
	int state=2;
	uint8_t i=0;
	
//	state=2;

//		
	for(i=0;i<sizeof(ESP32ExitTransit);i=i+1)//Ìî³äASCII×Ö·û¼¯Êý¾Ý
    {
			ESP32_UART_TXBuffer[i]=ESP32ExitTransit[i];
    }
	
	HAL_UART_Transmit_DMA(&huart2, (uint8_t *)ESP32ExitTransit, sizeof(ESP32ExitTransit)-1);
	HAL_Delay(2000);
	ClearUSARTTXBuffer();
	ClearUSARTRXBuffer();

	state=2;
	do{
	ClearUSARTTXBuffer();
	ClearUSARTRXBuffer();
	CommandGenFUN(ESP32_UART_TXBuffer, ESP32StartCommand, TEXT_LENTH,sizeof(ESP32StartCommand));
	HAL_UART_Transmit_DMA(&huart2, &ESP32_UART_TXBuffer[0], sizeof(ESP32StartCommand)+1);
	
	HAL_Delay(100);
	state=LookESPError(ESP32_UART_RXBuffer);
  }while(state==-1);
	
	state=2;
	do{
	CommandGenFUN(ESP32_UART_TXBuffer, ESP32StateRET, TEXT_LENTH,sizeof(ESP32StateRET));
		
	HAL_UART_Transmit_DMA(&huart2, (uint8_t *)ESP32_UART_TXBuffer, sizeof(ESP32StateRET)+2);
	
	
	HAL_Delay(200);
	state=LookESPError(ESP32_UART_RXBuffer);
  }while(state==-1);
	
	
	
	
	state=2;
	do{
	CommandGenFUN(ESP32_UART_TXBuffer, ESP32SetStation, TEXT_LENTH,sizeof(ESP32SetStation));
		
	HAL_UART_Transmit_DMA(&huart2, (uint8_t *)ESP32_UART_TXBuffer, sizeof(ESP32SetStation)+2);
	
	
	HAL_Delay(200);
	state=LookESPError(ESP32_UART_RXBuffer);
  }while(state==-1);
}


void ESP32ConnectAPFun(uint8_t* SensorIP){

	//Connect AP
	int state=2;
	uint8_t templateStr[]={"PSTA:ip:"};
	uint8_t i=0;
	int currentIP1=0;
	int currentIP2=0;
	uint8_t IPsegPointer=0;
	uint8_t curipPointer=0;
	
	uint8_t SNTPHour=0;
	uint8_t SNTPMinute=0;
	uint8_t SNTPSecond=0;
	
	do{
	CommandGenFUN(ESP32_UART_TXBuffer, ESP32ConnectAP, TEXT_LENTH,sizeof(ESP32ConnectAP));
	
	HAL_UART_Transmit_DMA(&huart2, (uint8_t *)ESP32_UART_TXBuffer, sizeof(ESP32ConnectAP)+2);

	HAL_Delay(500);
	state=LookESPError(ESP32_UART_RXBuffer);;
  }while(state==-1);
	
	do{
		
	CommandGenFUN(ESP32_UART_TXBuffer, ESPSetSNTPServer, TEXT_LENTH,sizeof(ESPSetSNTPServer));
	HAL_UART_Transmit_DMA(&huart2, (uint8_t *)ESP32_UART_TXBuffer, sizeof(ESPSetSNTPServer)+2);
	HAL_Delay(500);
		
		
	CommandGenFUN(ESP32_UART_TXBuffer, ESPEnqTime, TEXT_LENTH,sizeof(ESPEnqTime));
	HAL_UART_Transmit_DMA(&huart2, (uint8_t *)ESP32_UART_TXBuffer, sizeof(ESPEnqTime)+2);
	HAL_Delay(500);
		
	
	findStrResult=findStr(&ESP32_UART_RXBuffer[0], &ESPOKStr[0], ESP32_RXLen, sizeof(ESPOKStr)-1);

		
	OKPosition=LookESPError(ESP32_UART_RXBuffer);;
		
	if(OKPosition!=-1){
		SNTPHour=(ESP32_UART_RXBuffer[OKPosition-15]-0x30)*10+(ESP32_UART_RXBuffer[OKPosition-14]-0x30);
		SNTPMinute=(ESP32_UART_RXBuffer[OKPosition-12]-0x30)*10+(ESP32_UART_RXBuffer[OKPosition-11]-0x30);
		SNTPSecond=(ESP32_UART_RXBuffer[OKPosition-9]-0x30)*10+(ESP32_UART_RXBuffer[OKPosition-8]-0x30);
		
		
		rtcTime.Hours = SNTPHour;
		rtcTime.Minutes = SNTPMinute;
		rtcTime.Seconds = SNTPSecond;
		
		rtcDate.WeekDay = RTC_WEEKDAY_MONDAY;
		rtcDate.Month = 1;
		rtcDate.Date = 1;
		rtcDate.Year = 1;
		
		HAL_RTC_SetDate(&hrtc, &rtcDate, RTC_FORMAT_BIN);
		HAL_RTC_SetTime(&hrtc, &rtcTime, RTC_FORMAT_BIN);
		break;
	
	}
		
  }while(OKPosition==-1);
	
	
	
	//HAL_Delay(200);
//	//Enquire IP
//	state=2;
//	do{
//	CommandGenFUN(ESP32_UART_TXBuffer, ESP32EnquireIP, TEXT_LENTH,sizeof(ESP32EnquireIP));
//	
//	HAL_UART_Transmit_DMA(&huart2, (uint8_t *)ESP32_UART_TXBuffer, sizeof(ESP32EnquireIP)+2);
//	HAL_Delay(50);
//	ipPointer=(int)totalPointer;
//	
//	//Find Sensor IP
//		//There have a BUG in this place, the stm32 will ignore the following code that detects the SENSOR IP
//	
//		curipPointer=findStr(ESP32_UART_RXBuffer, templateStr, ipPointer, sizeof(templateStr)-1);
//	if(curipPointer==28){
//		for(i=ipPointer+9;i<ipPointer+9+15;i++){
//			
//			if (ESP32_UART_RXBuffer[i+ipPointer]>=0x30 && ESP32_UART_RXBuffer[i+ipPointer]<=0x39){
//				currentIP1=(ESP32_UART_RXBuffer[i+ipPointer]-0x30)+currentIP2*10;
//				currentIP2=currentIP1;
//			}
//			if (ESP32_UART_RXBuffer[i+ipPointer]==0x2E || ESP32_UART_RXBuffer[i+ipPointer]==0x0D){
//				SensorIP[IPsegPointer]=currentIP1;
//				IPsegPointer++;
//				currentIP1=0;
//				currentIP2=0;
//			}
//		}
//	}
//	
//	state=LookESPError(ESP32_UART_RXBuffer);
//  }while(state==-1);
	

}


void ESP32ServerConnect(void){
	int state=2;
	do{
	CommandGenFUN(ESP32_UART_TXBuffer, ESP32ConnectServer, TEXT_LENTH,sizeof(ESP32ConnectServer));
	
	HAL_UART_Transmit_DMA(&huart2, (uint8_t *)ESP32_UART_TXBuffer, sizeof(ESP32ConnectServer)+2);
		
	HAL_Delay(1000);
	state=LookESPError(ESP32_UART_RXBuffer);
  }while(state==-1);
	
	state=2;
	do{
	CommandGenFUN(ESP32_UART_TXBuffer, ESP32TCPTransitRX, TEXT_LENTH,sizeof(ESP32TCPTransitRX));
	
	HAL_UART_Transmit_DMA(&huart2, (uint8_t *)ESP32_UART_TXBuffer, sizeof(ESP32TCPTransitRX)+2);

		
		
	HAL_Delay(1000);
	state=LookESPError(ESP32_UART_RXBuffer);
	ClearUSARTRXBuffer();
  }while(state==-1);
	
	state=2;
	do{
	CommandGenFUN(ESP32_UART_TXBuffer, ESP32TCPTransitTX, TEXT_LENTH,sizeof(ESP32TCPTransitTX));
	
	HAL_UART_Transmit_DMA(&huart2, (uint8_t *)ESP32_UART_TXBuffer, sizeof(ESP32TCPTransitTX)+2);

		
	HAL_Delay(1000);
	state=LookESPError(ESP32_UART_RXBuffer);
	ClearUSARTRXBuffer();
  }while(state==-1);
	
}


void ESP32SendIP(uint8_t* EMG_IP, uint8_t SensorNum){
	uint32_t TCPSendDataNum=0;
	ClearUSARTTXBuffer();
	ESP32_UART_TXBuffer[0]=SensorNum;
	
	ESP32_UART_TXBuffer[1]=(uint8_t)(TCPSendDataNum>>24)&0x000000ff;
	ESP32_UART_TXBuffer[2]=(uint8_t)(TCPSendDataNum>>16)&0x000000ff;
	ESP32_UART_TXBuffer[3]=(uint8_t)(TCPSendDataNum>>8)&0x000000ff;
	ESP32_UART_TXBuffer[4]=(uint8_t)(TCPSendDataNum)&0x000000ff;
	
	ESP32_UART_TXBuffer[5]=0x4;
	
	ESP32_UART_TXBuffer[6]=EMG_IP[0];
	ESP32_UART_TXBuffer[7]=EMG_IP[1];
	ESP32_UART_TXBuffer[8]=EMG_IP[2];
	ESP32_UART_TXBuffer[9]=EMG_IP[3];
	
	ESP32_UART_TXBuffer[10]=0x0d;
	ESP32_UART_TXBuffer[11]=0x0a;
	
	HAL_UART_Transmit_DMA(&huart2, (uint8_t *)ESP32_UART_TXBuffer, 11);
	
}


void ESP32DeepSleep(void){
	
	
	int state=2;

	state=2;
	do{
	CommandGenFUN(ESP32_UART_TXBuffer, ESP32DeepSleepcmd, TEXT_LENTH,sizeof(ESP32DeepSleepcmd));
	
	HAL_UART_Transmit_DMA(&huart2, (uint8_t *)ESP32_UART_TXBuffer, sizeof(ESP32DeepSleepcmd));
	HAL_Delay(50);
	state=LookESPError(ESP32_UART_RXBuffer);
  }while(state==-1);
}


int findStr(uint8_t* ESPreport, uint8_t* Template, uint8_t ReportLen, uint8_t TemplateLen){
	
		
		uint8_t k = 0;
		uint8_t i=0;
    for(i = 0; i<ReportLen ;i++){
        if( k==0&&ESPreport[i] == Template[k]){
            k++;
            continue;
        }
        if(k>0 && ESPreport[i] == Template[k]){
            k++;
            if(k==TemplateLen){
                return i-k+1;//返回目标字符串的子串末尾-子串长度+1
            }
            continue;
        }
        k = 0;
    }
    return -1;//没找到返回1
}

void CommandGenFUN(uint8_t* SendBuff, uint8_t* Command, uint8_t TextLen,unsigned char CommandLen){
	
uint8_t i=0;
for(i=0;i<CommandLen;i=i+1)
    {
			SendBuff[i]=Command[i];
    }
SendBuff[i-1]=0x0d;
SendBuff[i]=0x0a;
		
}

void ClearUSARTTXBuffer(void){
	memset(ESP32_UART_TXBuffer,0,ESP32_TXLen);
}

void ClearUSARTRXBuffer(void){
	memset(ESP32_UART_RXBuffer,0,ESP32_TXLen);
}

int LookESPError(uint8_t* RXBuffer){
	
	findStrResult=findStr(&RXBuffer[0], &ESPOKStr[0], ESP32_RXLen, sizeof(ESPOKStr)-1);
	return findStrResult;
	
//	if(totalPointer==0) return 0;
//	
//	
//	if(RXBuffer[totalPointer-1]==0x0A && RXBuffer[totalPointer-2]==0x0D && RXBuffer[totalPointer-3]==0x4B && RXBuffer[totalPointer-4]==0x4F ){
//		totalPointer=0;
//		ClearUSARTRXBuffer();
//		return 1;        }
//	if(RXBuffer[totalPointer-1]==0x0A && RXBuffer[totalPointer-2]==0x0D && RXBuffer[totalPointer-3]==0x52 && RXBuffer[totalPointer-4]==0x4F && RXBuffer[totalPointer-5]==0x52){
//		totalPointer=0; 
//		ClearUSARTRXBuffer();
//	return 2;        }
//	if(RXBuffer[totalPointer-1]==0x3E){
//		totalPointer=0;
//		ClearUSARTRXBuffer();
//		return 3;}
//	if(RXBuffer[totalPointer-1]==0x0A && RXBuffer[totalPointer-2]==0x0D && RXBuffer[totalPointer-3]==0x52 && RXBuffer[totalPointer-4]==0x4F && RXBuffer[totalPointer-5]==0x52 && RXBuffer[0]==0x2B&& RXBuffer[1]==0x2B&& RXBuffer[2]==0x2B){
//		totalPointer=0;
//		ClearUSARTRXBuffer();
//		return 5;}
//	if(RXBuffer[0]==0x2B&& RXBuffer[1]==0x00){
//	totalPointer=0;
//	ClearUSARTRXBuffer();
//	return 6;}
//	
//	if(RXBuffer[0]==0x00&& RXBuffer[1]==0x00&& RXBuffer[2]==0x00&& RXBuffer[3]==0x00){
//	totalPointer=0;
//	ClearUSARTRXBuffer();
//	return 7;}
//	
//	totalPointer=0;
//	ClearUSARTRXBuffer();
//	return 4;

}

int LookESPConnect(uint8_t* RXBuffer){
	findStrResult=findStr(&RXBuffer[0], &ESPCONNECTEDStr[0], ESP32_TXLen, sizeof(ESPCONNECTEDStr)-1);
	return findStrResult;
}

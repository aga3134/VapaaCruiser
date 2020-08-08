#include "status.h"
#include "ultrasound.h"
#include "FIFOBuffer.h"
#include "gps.h"
#include <stdio.h>
#include <string.h>

extern UART_HandleTypeDef huart1;

FIFOBufferInstance g_StatusTxBuffer;
static unsigned char g_StatusTxLock = 0;
static unsigned char g_StatusOutData;
#define STATUS_MSG_SIZE 128
char g_StatusMsg[STATUS_MSG_SIZE] = {0};


void SendSensorStatus(){
	float lat,lng;
	int len = 0;
	GetGPSPos(&lat,&lng);
	
	float distArr[6];
	GetUltrasoundDist(distArr);
	
	memset(g_StatusMsg,0,STATUS_MSG_SIZE);
	sprintf(g_StatusMsg,"%f,%f,%f,%f,%f,%f,%f,%f\r\n",lat,lng,distArr[US_LF],distArr[US_F],distArr[US_RF],distArr[US_LB],distArr[US_B],distArr[US_RB]);
	//HAL_UART_Transmit(&huart1, (uint8_t*)g_Msg,MSG_SIZE,1000);
	
	//put data into g_TxBuffer
	len = strlen(g_StatusMsg);
	unsigned short num = FIFOBufferPutData(&g_StatusTxBuffer,(unsigned char*)g_StatusMsg,len);
	if(num == 0) return;	//滿了
	
	//若沒在傳輸就開始傳
	if(!g_StatusTxLock){
		if(FIFOBufferGetData(&g_StatusTxBuffer,&g_StatusOutData,1)){
			g_StatusTxLock = 1;
			HAL_UART_Transmit_IT(&huart1, &g_StatusOutData,1);
		}
	}
}

void ContinueStatusSend(UART_HandleTypeDef *UartHandle){
	if(UartHandle->Instance != USART1) return;
	
	//如果g_TxBuffer有資料就繼續送下一筆資料
  if(FIFOBufferGetData(&g_StatusTxBuffer,&g_StatusOutData,1)){
		HAL_UART_Transmit_IT(&huart1, &g_StatusOutData,1);
	}
	else g_StatusTxLock = 0;
}

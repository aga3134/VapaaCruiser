#include "status.h"
#include "ultrasound.h"
#include "FIFOBuffer.h"
#include "gps.h"
#include <stdio.h>
#include <string.h>

extern UART_HandleTypeDef huart1;

FIFOBufferInstance g_TxBuffer;
static unsigned char g_TxLock = 0;
static unsigned char g_OutData;
#define MSG_SIZE 128
char g_Msg[MSG_SIZE] = {0};


void SendSensorStatus(){
	float lat,lng;
	GetGPSPos(&lat,&lng);
	
	float distArr[6];
	GetUltrasoundDist(distArr);
	
	memset(g_Msg,0,MSG_SIZE);
	sprintf(g_Msg,"%f,%f,%f,%f,%f,%f,%f,%f\r\n",lat,lng,distArr[US_LF],distArr[US_F],distArr[US_RF],distArr[US_LB],distArr[US_B],distArr[US_RB]);
	//HAL_UART_Transmit(&huart1, (uint8_t*)g_Msg,MSG_SIZE,1000);
	
	//put data into g_TxBuffer
	unsigned short num = FIFOBufferPutData(&g_TxBuffer,(unsigned char*)g_Msg,MSG_SIZE);
	if(num == 0) return;	//滿了
	
	//若沒在傳輸就開始傳
	if(!g_TxLock){
		if(FIFOBufferGetData(&g_TxBuffer,&g_OutData,1)){
			g_TxLock = 1;
			HAL_UART_Transmit_IT(&huart1, &g_OutData,1);
		}
	}
}

void ContinueStatusSend(UART_HandleTypeDef *UartHandle){
	//如果g_TxBuffer有資料就繼續送下一筆資料
  if(FIFOBufferGetData(&g_TxBuffer,&g_OutData,1)){
		HAL_UART_Transmit_IT(&huart1, &g_OutData,1);
	}
	else g_TxLock = 0;
}

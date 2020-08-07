#include "status.h"
#include "ultrasound.h"
#include <stdio.h>
#include <string.h>

extern UART_HandleTypeDef huart1;

#define MSG_SIZE 128
char g_Msg[MSG_SIZE] = {0};

void SendSensorStatus(){
	float distArr[6];
	GetUltrasoundDist(distArr);
	memset(g_Msg,0,MSG_SIZE);
	sprintf(g_Msg,"%f,%f,%f,%f,%f,%f\r\n",distArr[US_LF],distArr[US_F],distArr[US_RF],distArr[US_LB],distArr[US_B],distArr[US_RB]);
	HAL_UART_Transmit(&huart1, (uint8_t*)g_Msg,MSG_SIZE,1000);
}


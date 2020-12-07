#ifndef GPS_H
#define GPS_H

#include "main.h"

void StartGPSReceive(void);
void StopGPSReceive(void);

//由usart1的receive complete interrupt驅動
void ReceiveGPSInfo(UART_HandleTypeDef *UartHandle);
void ProcessGPS(void);
unsigned char GetGPSPos(float* lat, float* lng);

#endif

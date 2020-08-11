#ifndef GPS_H
#define GPS_H

#include "main.h"

void StartGPSReceive(void);
void StopGPSReceive(void);

//��usart1��receive complete interrupt�X��
void ReceiveGPSInfo(UART_HandleTypeDef *UartHandle);
void ProcessGPS(void);
unsigned char GetGPSPos(float* lat, float* lng);

#endif

#ifndef STATUS_H
#define STATUS_H

#include "main.h"

void SendSensorStatus(void);

//��usart1��transmit complete interrupt�X��
void ContinueStatusSend(UART_HandleTypeDef *UartHandle);

#endif

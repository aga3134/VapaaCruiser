#ifndef STATUS_H
#define STATUS_H

#include "main.h"

void SendSensorStatus(void);

//由usart1的transmit complete interrupt驅動
void ContinueStatusSend(UART_HandleTypeDef *UartHandle);

#endif

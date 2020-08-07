#ifndef STATUS_H
#define STATUS_H

#include "main.h"

void SendSensorStatus(void);

//¥Ñusart1ªºtransmit complete interruptÅX°Ê
void ContinueStatusSend(UART_HandleTypeDef *UartHandle);

#endif

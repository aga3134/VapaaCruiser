#ifndef COMMAND_H
#define COMMAND_H

#include "main.h"

void StartCommandReceive(void);
void StopCommandReceive(void);
void ReceiveCommand(UART_HandleTypeDef *UartHandle);
unsigned char ProcessCommand(void);

#endif

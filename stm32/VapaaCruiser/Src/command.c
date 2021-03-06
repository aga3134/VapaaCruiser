#include "command.h"
#include "motor.h"
#include "FIFOBuffer.h"

extern UART_HandleTypeDef huart1;
static FIFOBufferInstance g_CmdRxBuffer;
static unsigned char g_CmdInData;
static unsigned char g_CmdStart = 0;

enum ParseState{
	HEADER,
	CMD,
	ARG_NUM,
	ARGS,
	CHECKSUM
};

#define CMD_MAX_ARG 20
#define CMD_STOP 0x00
#define CMD_MOVE 0x01

typedef struct{
	uint8_t cmd;
	uint8_t argNum;
	uint8_t args[CMD_MAX_ARG];
}Command;


void StartCommandReceive(){
	HAL_UART_Receive_IT(&huart1,&g_CmdInData,1);
	g_CmdStart = 1;
}

void StopCommandReceive(){
	HAL_UART_AbortReceive_IT(&huart1);
	g_CmdStart = 0;
}

void ReceiveCommand(UART_HandleTypeDef *UartHandle){
	HAL_UART_Receive_IT(&huart1,&g_CmdInData,1);
	if(UartHandle->Instance != USART1) return;
	FIFOBufferPutData(&g_CmdRxBuffer,&g_CmdInData,1);
	//echo回去
	//HAL_UART_Transmit_IT(&huart1, &g_InData,1);
}

void ProcessCommand(Command* pCmd){
	float forward,turn;
	switch(pCmd->cmd){
		case CMD_MOVE:
			forward = (pCmd->args[0]/255.0-0.5)*2;
			turn = (pCmd->args[1]/255.0-0.5)*2;
			SetMotorSpeed(forward,turn);
			break;
		case CMD_STOP:
			StopMotor();
			break;
		default:
			break;
	}
}

unsigned char ParseCommand(){
	int i=0;
	int len = FIFOBufferGetDataSize(&g_CmdRxBuffer);
	enum ParseState state = HEADER;
	Command cmd = {0};
	unsigned char d, sum = 0, valid = 0, err = 0;
	unsigned short start = 0, end = 0, argIndex = 0;
	
	//判斷是否有完整command
	for(i=0;i<len && !valid;i++){
		if(valid || err) break;	//已讀到完整command或出現error
		
		FIFOBufferPeekData(&g_CmdRxBuffer,&d,i);
		switch(state){
			case HEADER:
				if(d == 0xFE){
					state = CMD;
					start = i;
					end = i;
					sum = d;
				}
				break;
			case CMD:
				state = ARG_NUM;
				end++;
				sum += d;
				cmd.cmd = d;
				break;
			case ARG_NUM:
				end++;
				sum += d;
				cmd.argNum = d;
				if(cmd.argNum == 0) state = CHECKSUM;
				else{
					if(cmd.argNum > CMD_MAX_ARG) err = 1;	//invalid argNum
					state = ARGS;
					argIndex = 0;
				}
				break;
			case ARGS:
				end++;
				sum += d;
				cmd.args[argIndex] = d;
				argIndex++;
				if(argIndex >= cmd.argNum) state = CHECKSUM;
				break;
			case CHECKSUM:
				end++;
				if(sum == d){
					valid = 1;
				}
				break;
		}
	}
	
	if(state == CHECKSUM){	//已讀到完整command
		//清掉command及之前的資料
		FIFOBufferClear(&g_CmdRxBuffer,end);
		if(valid){	//checksum檢驗ok
			ProcessCommand(&cmd);
			return 1;
		}
	}
	else{
		//有錯誤，清空到錯誤的地方
		if(err){
			FIFOBufferClear(&g_CmdRxBuffer,end);
		}
		//清掉之前的資料，保留未完整的command
		else if(start > 0){
			FIFOBufferClear(&g_CmdRxBuffer,start-1);
		}
	}
	
	//有時候接收會斷掉，這邊定時重啟接收
	if(g_CmdStart){
		HAL_UART_Receive_IT(&huart1,&g_CmdInData,1);
	}
	
	
	return 0;	//command未完整，下次再讀
}

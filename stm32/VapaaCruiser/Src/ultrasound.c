#include "ultrasound.h"

extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;

typedef struct{
	UltrasoundInstance* pLF;
	UltrasoundInstance* pF;
	UltrasoundInstance* pRF;
	UltrasoundInstance* pLB;
	UltrasoundInstance* pB;
	UltrasoundInstance* pRB;
}UltrasoundResource;

static UltrasoundResource g_UR;

void StartUltrasound(UltrasoundInstance* pInstance){
	switch(pInstance->dir){
		case US_LF:
			g_UR.pLF = pInstance;
			HAL_TIM_IC_Start_IT(&htim3,TIM_CHANNEL_1);
			break;
		case US_F:
			g_UR.pF = pInstance;
			HAL_TIM_IC_Start_IT(&htim3,TIM_CHANNEL_2);
			break;
		case US_RF:
			g_UR.pRF = pInstance;
			HAL_TIM_IC_Start_IT(&htim3,TIM_CHANNEL_3);
			break;
		case US_LB:
			g_UR.pLB = pInstance;
			HAL_TIM_IC_Start_IT(&htim2,TIM_CHANNEL_2);
			break;
		case US_B:
			g_UR.pB = pInstance;
			HAL_TIM_IC_Start_IT(&htim2,TIM_CHANNEL_3);
			break;
		case US_RB:
			g_UR.pRB = pInstance;
			HAL_TIM_IC_Start_IT(&htim2,TIM_CHANNEL_4);
			break;
		default:
			break;
	}
}

void StopUltrasound(UltrasoundInstance* pInstance){
	switch(pInstance->dir){
		case US_LF:
			g_UR.pLF = NULL;
			HAL_TIM_IC_Stop_IT(&htim3,TIM_CHANNEL_1);
			break;
		case US_F:
			g_UR.pF = NULL;
			HAL_TIM_IC_Stop_IT(&htim3,TIM_CHANNEL_2);
			break;
		case US_RF:
			g_UR.pRF = NULL;
			HAL_TIM_IC_Stop_IT(&htim3,TIM_CHANNEL_3);
			break;
		case US_LB:
			g_UR.pLB = NULL;
			HAL_TIM_IC_Stop_IT(&htim2,TIM_CHANNEL_2);
			break;
		case US_B:
			g_UR.pB = NULL;
			HAL_TIM_IC_Stop_IT(&htim2,TIM_CHANNEL_3);
			break;
		case US_RB:
			g_UR.pRB = NULL;
			HAL_TIM_IC_Stop_IT(&htim2,TIM_CHANNEL_4);
			break;
		default:
			break;
	}
}


void SendUltrasoundTrigger(){
	HAL_GPIO_WritePin(US_Trigger_GPIO_Port,US_Trigger_Pin,GPIO_PIN_SET);
	HAL_Delay(1);
	HAL_GPIO_WritePin(US_Trigger_GPIO_Port,US_Trigger_Pin,GPIO_PIN_RESET);
}

void ComputeUltrasoundDist(TIM_HandleTypeDef *htim){
	UltrasoundInstance* target = NULL;
	uint32_t channel = 0;
	int diff = 0;
	
	if(htim->Instance == TIM3){
		switch(htim->Channel){
			case HAL_TIM_ACTIVE_CHANNEL_1:
				target = g_UR.pLF;
				channel = TIM_CHANNEL_1;
				break;
			case HAL_TIM_ACTIVE_CHANNEL_2:
				target = g_UR.pF;
				channel = TIM_CHANNEL_2;
				break;
			case HAL_TIM_ACTIVE_CHANNEL_3:
				target = g_UR.pRF;
				channel = TIM_CHANNEL_3;
				break;
			default:
				break;
		}
	}
	else if(htim->Instance == TIM2){
		switch(htim->Channel){
			case HAL_TIM_ACTIVE_CHANNEL_2:
				target = g_UR.pLB;
				channel = TIM_CHANNEL_2;
				break;
			case HAL_TIM_ACTIVE_CHANNEL_3:
				target = g_UR.pB;
				channel = TIM_CHANNEL_3;
				break;
			case HAL_TIM_ACTIVE_CHANNEL_4:
				target = g_UR.pRB;
				channel = TIM_CHANNEL_4;
				break;
			default:
				break;
		}
	}
	if(!target) return;
	
	if(target->firstCapture == 0){
		target->value1 = HAL_TIM_ReadCapturedValue(htim, channel);
		target->firstCapture = 1;
		__HAL_TIM_SET_CAPTUREPOLARITY(htim, channel, TIM_INPUTCHANNELPOLARITY_FALLING);
	}
	else{
		target->value2 = HAL_TIM_ReadCapturedValue(htim, channel);
		if (target->value2 >= target->value1){
			diff = target->value2 - target->value1;
		}
		else{
			diff = (htim->Init.Period - target->value1) + target->value2;
		}

		//音波走的距離=速度x時間 時間=diff(us 假設timer設定成1m hz) 速度=音速=340(m/s)
		//反射音波走的距離=物體距離*2 =>物體距離=diff*10^-6*340*0.5(m) = diff*0.17*10^-3(m)=diff*0.17(mm)
		target->dist = diff*0.17;
		target->firstCapture = 0;
		__HAL_TIM_SET_CAPTUREPOLARITY(htim, channel, TIM_INPUTCHANNELPOLARITY_RISING);
	}
}

void GetUltrasoundDist(float* distArr){
	if(g_UR.pLF) distArr[US_LF] = g_UR.pLF->dist;
	else distArr[US_LF] = -1;
	
	if(g_UR.pF) distArr[US_F] = g_UR.pF->dist;
	else distArr[US_F] = -1;
	
	if(g_UR.pRF) distArr[US_RF] = g_UR.pRF->dist;
	else distArr[US_RF] = -1;
	
	if(g_UR.pLB) distArr[US_LB] = g_UR.pLB->dist;
	else distArr[US_LB] = -1;
	
	if(g_UR.pB) distArr[US_B] = g_UR.pB->dist;
	else distArr[US_B] = -1;
	
	if(g_UR.pRB) distArr[US_RB] = g_UR.pRB->dist;
	else distArr[US_RB] = -1;
}

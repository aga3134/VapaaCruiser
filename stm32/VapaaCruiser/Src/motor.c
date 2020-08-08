#include "motor.h"
#include "main.h"

#define MAX_FORWARD 0.4
#define MIN_FORWARD 0.3
#define MAX_TURN 0.5
#define MIN_TURN 0.2

extern TIM_HandleTypeDef htim8;
float g_TargetForward, g_TargetTurn;
float g_CurForward, g_CurTurn;
float g_TargetRate = 0.5;
uint8_t g_IsStart = 0;

void StartMotor(){
	HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_1);
	htim8.Instance->CCR1 = 0;
	HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_2);
	htim8.Instance->CCR2 = 0;
	g_TargetForward = 0;
	g_TargetTurn = 0;
	g_CurForward = 0;
	g_CurTurn = 0;
	g_IsStart = 1;
}

void StopMotor(){
	HAL_TIM_PWM_Stop(&htim8,TIM_CHANNEL_1);
	htim8.Instance->CCR1 = 0;
	HAL_TIM_PWM_Stop(&htim8,TIM_CHANNEL_2);
	htim8.Instance->CCR2 = 0;
	g_TargetForward = 0;
	g_TargetTurn = 0;
	g_CurForward = 0;
	g_CurTurn = 0;
	g_IsStart = 0;
}

void SetMotorSpeed(float forward, float turn, float rate){
	if(g_IsStart == 0) StartMotor();
	g_TargetForward = forward;
	g_TargetTurn = turn;
	g_TargetRate = rate;
}

void UpdateMotorSpeed(){
	float rate = g_TargetRate;
	float invRate = 1-rate;
	uint32_t period = htim8.Init.Period+1;
	uint32_t pwmF, pwmT;
	//控制速度-1~1
	g_CurForward = g_CurForward*invRate+g_TargetForward*rate;
	g_CurTurn = g_CurTurn*invRate+g_TargetTurn*rate;
	
	//轉換成pulse width
	pwmF = MIN_FORWARD+(MAX_FORWARD-MIN_FORWARD)*(g_CurForward+1)*0.5*period;
	pwmT = MIN_TURN+(MAX_TURN-MIN_TURN)*(g_CurTurn+1)*0.5*period;
	htim8.Instance->CCR1 = pwmF;
	htim8.Instance->CCR2 = pwmT;
}

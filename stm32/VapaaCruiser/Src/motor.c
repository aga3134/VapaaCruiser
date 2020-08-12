#include "motor.h"
#include "main.h"

//馬達pwm週期為50hz(20000 micro second)=timer數一圈週期
//pulse width為544~2400 micro second，544對應0%，2400對應100%
//為避免爆衝，再用MAX_FORWARD,MIN_FORWARD,MAX_TURN,MIN_TURN限制輸出%
#define MIN_PULSE 544
#define MAX_PULSE 2400
#define MAX_FORWARD 0.55
#define MIN_FORWARD 0.43
#define MAX_TURN 0.8
#define MIN_TURN 0.3

extern TIM_HandleTypeDef htim8;
float g_TargetForward = 0, g_TargetTurn = 0;
float g_CurForward = 0, g_CurTurn = 0;
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
	uint32_t minPulseF,maxPulseF,minPulseT,maxPulseT;
	uint32_t pwmF, pwmT;
	//控制速度-1~1
	g_CurForward = g_CurForward*invRate+g_TargetForward*rate;
	g_CurTurn = g_CurTurn*invRate+g_TargetTurn*rate;
	
	//最大最小pulse限制
	minPulseF = MIN_PULSE+(MAX_PULSE-MIN_PULSE)*MIN_FORWARD;
	maxPulseF = MIN_PULSE+(MAX_PULSE-MIN_PULSE)*MAX_FORWARD;
	minPulseT = MIN_PULSE+(MAX_PULSE-MIN_PULSE)*MIN_TURN;
	maxPulseT = MIN_PULSE+(MAX_PULSE-MIN_PULSE)*MAX_TURN;
	//計算要輸出的pulse
	pwmF = minPulseF+(maxPulseF-minPulseF)*(g_CurForward+1)*0.5f;
	pwmT = minPulseT+(maxPulseT-minPulseT)*(g_CurTurn+1)*0.5f;
	htim8.Instance->CCR1 = pwmF;
	htim8.Instance->CCR2 = pwmT;
}

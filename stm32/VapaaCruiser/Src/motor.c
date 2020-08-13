#include "motor.h"
#include "main.h"

//���Fpwm�g����50hz(20000 micro second)=timer�Ƥ@��g��
//pulse width��544~2400 micro second�A544����0%�A2400����100%
//���קK�z�ġA�A��MAX_FORWARD,MIN_FORWARD,MAX_TURN,MIN_TURN�����X%
#define MIN_PULSE 544
#define MAX_PULSE 2400
#define MAX_FORWARD 0.555
#define MIN_FORWARD 0.442
#define MAX_TURN 0.8
#define MIN_TURN 0.4

extern TIM_HandleTypeDef htim8;
float g_TargetForward = 0, g_TargetTurn = 0;
uint8_t g_IsStart = 0;

void StartMotor(){
	HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_1);
	htim8.Instance->CCR1 = 0;
	HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_2);
	htim8.Instance->CCR2 = 0;
	g_TargetForward = 0;
	g_TargetTurn = 0;
	g_IsStart = 1;
}

void StopMotor(){
	HAL_TIM_PWM_Stop(&htim8,TIM_CHANNEL_1);
	htim8.Instance->CCR1 = 0;
	HAL_TIM_PWM_Stop(&htim8,TIM_CHANNEL_2);
	htim8.Instance->CCR2 = 0;
	g_TargetForward = 0;
	g_TargetTurn = 0;
	g_IsStart = 0;
}

void SetMotorSpeed(float forward, float turn){
	if(g_IsStart == 0) StartMotor();
	g_TargetForward = forward;
	g_TargetTurn = turn;
}

void UpdateMotorSpeed(){
	uint32_t minPulseF,maxPulseF,minPulseT,maxPulseT;
	uint32_t pwmF, pwmT;
	
	//�̤j�̤ppulse����
	minPulseF = MIN_PULSE+(MAX_PULSE-MIN_PULSE)*MIN_FORWARD;
	maxPulseF = MIN_PULSE+(MAX_PULSE-MIN_PULSE)*MAX_FORWARD;
	minPulseT = MIN_PULSE+(MAX_PULSE-MIN_PULSE)*MIN_TURN;
	maxPulseT = MIN_PULSE+(MAX_PULSE-MIN_PULSE)*MAX_TURN;
	//�p��n��X��pulse
	pwmF = minPulseF+(maxPulseF-minPulseF)*(g_TargetForward+1)*0.5f;
	pwmT = minPulseT+(maxPulseT-minPulseT)*(g_TargetTurn+1)*0.5f;
	
	//���F���q�ܲ��ͫ�h��double click
	if(g_TargetForward > -0.05f && g_TargetForward < 0.05f){
		htim8.Instance->CCR1 = MIN_PULSE;
	}
	else htim8.Instance->CCR1 = pwmF;
	htim8.Instance->CCR2 = pwmT;
}

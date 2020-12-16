#include "motor.h"
#include "main.h"
#include "ultrasound.h"

//馬達pwm週期為50hz(20000 micro second)=timer數一圈週期
//pulse width為544~2400 micro second，544對應0%，2400對應100%
//為避免爆衝，再用MAX_FORWARD,MIN_FORWARD,MAX_TURN,MIN_TURN限制輸出%
#define MIN_PULSE 544
#define MAX_PULSE 2400
#define MAX_FORWARD 0.58
#define MIN_FORWARD 0.42
#define MAX_TURN 0.85
#define MIN_TURN 0.35
#define LIMIT_SPEED_BY_US 1

extern TIM_HandleTypeDef htim8;
float g_TargetForward = 0, g_TargetTurn = 0;
uint8_t g_IsStart = 0;
uint32_t g_UpdateCountAfterCmd = 0;

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

	if(forward > 1) forward = 1;
	else if(forward < -1) forward = -1;
	if(turn > 1) turn = 1;
	else if(turn < -1) turn = -1;

	g_TargetForward = forward;
	g_TargetTurn = turn;
	g_UpdateCountAfterCmd = 0;
}

void UpdateMotorSpeed(){
	uint32_t minPulseF,maxPulseF,minPulseT,maxPulseT;
	uint32_t pwmF, pwmT;
	float distArr[6] = {0};
	float turnThresh = 0.3;
	//依障礙物距離*distScale當速限,distScale=0.001表示障礙物超過1000mm時可全速前進
	float distScale = 0.001;
	float speedLimit = 0;
	
	//最大最小pulse限制
	minPulseF = MIN_PULSE+(MAX_PULSE-MIN_PULSE)*MIN_FORWARD;
	maxPulseF = MIN_PULSE+(MAX_PULSE-MIN_PULSE)*MAX_FORWARD;
	minPulseT = MIN_PULSE+(MAX_PULSE-MIN_PULSE)*MIN_TURN;
	maxPulseT = MIN_PULSE+(MAX_PULSE-MIN_PULSE)*MAX_TURN;

	//用超音波測距限制速度
	if(LIMIT_SPEED_BY_US){
		GetUltrasoundDist(distArr);
		if(g_TargetTurn > turnThresh){	//右轉
			if(g_TargetForward > 0){
				speedLimit = distArr[US_RF]*distScale;
				if(g_TargetForward > speedLimit){
					g_TargetForward = speedLimit;
				}
			}
			else{
				speedLimit = -distArr[US_RB]*distScale;
				if(g_TargetForward < speedLimit){
					g_TargetForward = speedLimit;
				}
			}
		}
		else if(g_TargetTurn < -turnThresh){	//左轉
			if(g_TargetForward > 0){
				speedLimit = distArr[US_LF]*distScale;
				if(g_TargetForward > speedLimit){
					g_TargetForward = speedLimit;
				}
			}
			else{
				speedLimit = -distArr[US_LB]*distScale;
				if(g_TargetForward < speedLimit){
					g_TargetForward = speedLimit;
				}
			}
		}
		else{	//直行
			if(g_TargetForward > 0){
				speedLimit = distArr[US_F]*distScale;
				if(g_TargetForward > speedLimit){
					g_TargetForward = speedLimit;
				}
			}
			else{
				speedLimit = -distArr[US_B]*distScale;
				if(g_TargetForward < speedLimit){
					g_TargetForward = speedLimit;
				}
			}
		}
	}

	//計算要輸出的pulse
	pwmF = minPulseF+(maxPulseF-minPulseF)*(g_TargetForward+1)*0.5f;
	pwmT = minPulseT+(maxPulseT-minPulseT)*(g_TargetTurn+1)*0.5f;
	
	//若超過一定update次數都沒收到指令，就停止動作
	if(g_UpdateCountAfterCmd++ > 30){
		StopMotor();
	}
	
	//為了讓電變產生後退需double click
	if(g_TargetForward > -0.05f && g_TargetForward < 0.05f){
		htim8.Instance->CCR1 = MIN_PULSE;
	}
	else htim8.Instance->CCR1 = pwmF;
	htim8.Instance->CCR2 = pwmT;
}

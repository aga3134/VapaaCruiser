#ifndef ULTRASOUND_H
#define ULTRASOUND_H

#include "main.h"

#define US_LF 0
#define US_F 1
#define US_RF 2
#define US_LB 3
#define US_B 4
#define US_RB 5

typedef struct{
	unsigned char dir;
	float dist;
	unsigned char firstCapture;
	uint32_t value1;
	uint32_t value2;
}UltrasoundInstance;


void StartUltrasound(UltrasoundInstance* pInstance);
void StopUltrasound(UltrasoundInstance* pInstance);

void SendUltrasoundTrigger(void);
//由timer的capture input訊號驅動
void ComputeUltrasoundDist(TIM_HandleTypeDef *htim);
void GetUltrasoundDist(float* distArr);

#endif

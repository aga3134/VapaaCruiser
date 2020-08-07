#ifndef ULTRASOUND_H
#define ULTRASOUND_H

#include "main.h"

#define US_UNKNOWN 0
#define US_LF 1
#define US_F 2
#define US_RF 3
#define US_LB 4
#define US_B 5
#define US_RB 6

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
void ComputeUltrasoundDist(TIM_HandleTypeDef *htim);

#endif

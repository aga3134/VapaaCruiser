#ifndef MOTOR_H
#define MOTOR_H

void StartMotor(void);
void StopMotor(void);
//forward:-1~1, turn:-1~1, rate:0~1�A�V����1�t�ק��ܶV��
void SetMotorSpeed(float forward, float turn);
//��rate�]�w�v�����t�ױ���SetSpeed�]�w���ƭ�
void UpdateMotorSpeed(void);

#endif

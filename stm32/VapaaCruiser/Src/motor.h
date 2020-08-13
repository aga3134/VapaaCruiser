#ifndef MOTOR_H
#define MOTOR_H

void StartMotor(void);
void StopMotor(void);
//forward:-1~1, turn:-1~1, rate:0~1，越接近1速度改變越快
void SetMotorSpeed(float forward, float turn);
//依rate設定逐漸讓速度接近SetSpeed設定的數值
void UpdateMotorSpeed(void);

#endif

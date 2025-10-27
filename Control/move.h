#ifndef MOVE_H
#define MOVE_H


#ifdef __cplusplus
extern "C" {
#endif

void PID_Init(void);
void move_task(void);
void Motor_Measure( void);
void Motor_Turn(void);


// C �ɵ��ýӿ�
void Move_SetSpeed(uint8_t v);
uint8_t Move_GetSpeed(void);
float Move_GetSpeedL_Out(void);
float Move_GetSpeedR_Out(void);
uint8_t Move_GetSpeedL_Measure(void);
uint8_t Move_GetSpeedR_Measure(void);
uint8_t Move_GetSpeedL_Set(void);
uint8_t Move_GetSpeedR_Set(void);



// C ��ʹ�õ��ⲿ C ����

#define R_Speed_KP 300.0f
#define R_Speed_KI 5.0f
#define R_Speed_KD 15.0f
#define R_Speed_MaxOut 5000
#define R_Speed_IntegralLimit 1500.0f
#define R_Speed_BandI 100.0f

#define L_Speed_KP 300.0f
#define L_Speed_KI 5.0f
#define L_Speed_KD 15.0f
#define L_Speed_MaxOut 5000
#define L_Speed_IntegralLimit 1500.0f
#define L_Speed_BandI 100.0f

#define Turn_KP 0.50f
#define Turn_KI 0.001f
#define Turn_KD 0.7f
#define Turn_MaxOut 40
#define Turn_IntegralLimit 10.0f
#define Turn_BandI 1.0f

extern int16_t Encoder_Get_L(void);
extern int16_t Encoder_Get_R(void);
extern void Motor_SetPWM_L(int16_t Duty);
extern void Motor_SetPWM_R(int16_t Duty);

#ifdef __cplusplus
}
#endif

// ���� C++ �пɼ���ȫ�֣������� C �г�������/�������⣩
#ifdef __cplusplus
#include "algorithm_pid.h"
extern sPidTypeDef Speed_Pid[2];
extern sPidTypeDef Turn_Pid;
extern uint8_t Move_Speed;
#endif

#endif


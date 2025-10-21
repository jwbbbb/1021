#include "algorithm_pid.h"
#include <stdint.h>

// Forward declarations of C functions from Hardware/Encoder.c
extern "C" {
      int16_t Encoder_Get_L(void);
      int16_t Encoder_Get_R(void);
      void Motor_SetPWM_L(int16_t Duty);
      void Motor_SetPWM_R(int16_t Duty);
}
#include "move.h"
sPidTypeDef Speed_Pid[2];


#define R_Speed_KP 250.0f
#define R_Speed_KI 5.0f
#define R_Speed_KD 15.0f
#define R_Speed_MaxOut 7200
#define R_Speed_IntegralLimit 1000.0f
#define R_Speed_BandI 100.0f

#define L_Speed_KP 250.0f
#define L_Speed_KI 5.0f
#define L_Speed_KD 15.0f
#define L_Speed_MaxOut 7200
#define L_Speed_IntegralLimit 1000.0f
#define L_Speed_BandI 100.0f

sPidTypeDef Turn_Pid;

#define Turn_KP 0.50f
#define Turn_KI 0.001f
#define Turn_KD 0.3f
#define Turn_MaxOut 35
#define Turn_IntegralLimit 10.0f
#define Turn_BandI 1.0f


uint8_t Move_Speed =0;
extern int16_t Encoder_Get_L(void);
extern int16_t Encoder_Get_R(void);
extern void Motor_SetPWM_L(int16_t Duty);
extern void Motor_SetPWM_R(int16_t Duty);

float Fllow_Move;
extern float yaw;
extern float pitch;
uint8_t PWM_NewEnable=1;
void PID_Init(void)
{
      PID.Init(&Speed_Pid[0], POSITION, L_Speed_KP, L_Speed_KI, L_Speed_KD, L_Speed_MaxOut, L_Speed_IntegralLimit, L_Speed_BandI); //�����ٶȻ���ʼ��
      PID.Init(&Speed_Pid[1], POSITION, R_Speed_KP, R_Speed_KI, R_Speed_KD, R_Speed_MaxOut, R_Speed_IntegralLimit, R_Speed_BandI); //�����ٶȻ���ʼ��
      PID.Init(&Turn_Pid, POSITION, Turn_KP, Turn_KI, Turn_KD, Turn_MaxOut, Turn_IntegralLimit, Turn_BandI); //ת�򻷳�ʼ��
}

void move_task(void)
{
     Speed_Pid[0].ref = Encoder_Get_L(); //��ȡ���ֱ�����ֵ
     Speed_Pid[1].ref = -Encoder_Get_R(); //��ȡ���ֱ�����
     

     PID.Calc(&Turn_Pid, yaw, Fllow_Move); //����ת���������ʱ��Ϊ0

    Speed_Pid[0].set = Move_Speed - Turn_Pid.out; //���������ٶ�Ŀ��
     Speed_Pid[1].set = Move_Speed + Turn_Pid.out; //���������ٶ�Ŀ��

     PID.Calc(&Speed_Pid[0], Speed_Pid[0].ref, Speed_Pid[0].set); //���������ٶȻ����
     PID.Calc(&Speed_Pid[1], Speed_Pid[1].ref, -Speed_Pid[1].set); //���������ٶȻ����

     			if(PWM_NewEnable)
  		{
				Motor_SetPWM_L((int)Speed_Pid[0].out);
				Motor_SetPWM_R((int)Speed_Pid[1].out);
//			Motor_SetPWM_L(2000);
//			Motor_SetPWM_R(2000);
			}else if(!PWM_NewEnable){
				Speed_Pid[0].out =0;
				Speed_Pid[1].out =0;
				yaw =0;
				Fllow_Move=0;
				Motor_SetPWM_L(0);
				Motor_SetPWM_R(0);
			}
      if (pitch < -45)
      {
            PWM_NewEnable =0;
      }else{
            PWM_NewEnable =1;
      }
      
}

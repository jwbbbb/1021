#include "algorithm_pid.h"
#include <stdint.h>

// Forward declarations of C functions from Hardware/Encoder.c

//#define PL_1 (-0.06)
//#define PL_2 (-0.15)
//#define PR_1 (0.06)
//#define PR_2 (0.15)
//#define PL_1MAX	-2
//#define PL_2MAX -2
//#define PR_2MAX 11
//#define PR_1MAX 2
//#define PL_1MAX	-1
//#define PL_2MAX -1
//#define PR_2MAX 1
//#define PR_1MAX 1
#define PL_1MAX	-90.0f
#define PL_2MAX -25.6f
#define PR_2MAX 22.7f
#define PR_1MAX 82.5f


extern "C" {
      int16_t Encoder_Get_L(void);
      int16_t Encoder_Get_R(void);
      void Motor_SetPWM_L(int16_t Duty);
      void Motor_SetPWM_R(int16_t Duty);
}
#include "move.h"
sPidTypeDef Speed_Pid[2];
sPidTypeDef Turn_Pid;
uint8_t Move_Speed =30;

float Fllow_Move;
extern float yaw;
extern float pitch;
extern uint16_t vals[4];
uint8_t PWM_NewEnable=1;

float Total_T;

void PID_Init(void)
{
      PID.Init(&Speed_Pid[0], POSITION, L_Speed_KP, L_Speed_KI, L_Speed_KD, L_Speed_MaxOut, L_Speed_IntegralLimit, L_Speed_BandI); //左轮速度环初始化
      PID.Init(&Speed_Pid[1], POSITION, R_Speed_KP, R_Speed_KI, R_Speed_KD, R_Speed_MaxOut, R_Speed_IntegralLimit, R_Speed_BandI); //右轮速度环初始化
      PID.Init(&Turn_Pid, POSITION, Turn_KP, Turn_KI, Turn_KD, Turn_MaxOut, Turn_IntegralLimit, Turn_BandI); //转向环初始化
}

void move_task(void)
{
     Speed_Pid[0].ref = Encoder_Get_L(); //获取左轮编码器值
     Speed_Pid[1].ref = -Encoder_Get_R(); //获取右轮编码器
     
//      Total_T = vals[0] /PL_1MAX + vals[1] / PL_2MAX + vals[3] /PR_2MAX + vals[2] / PR_1MAX;
	      Total_T = vals[0] /PL_1MAX  + vals[2] / PR_1MAX;
//     if(vals[0]/PL_1MAX < 100 && vals[2]/PR_1MAX < 100) {Total_T = vals[0] /PL_1MAX + vals[1] / PL_2MAX + vals[3] /PR_2MAX + vals[2] / PR_1MAX;}
//	if(Total_T>100 ||Total_T < -100) {if (vals[0] <20 || vals[2] <20){Total_T =0;}};
	PID.Calc(&Turn_Pid, Total_T*3, 0); //计算转向环输出，暂时设为0
	if(Turn_Pid.out >5) Speed_Pid[0].set = Move_Speed - 2 *Turn_Pid.out; 
    else Speed_Pid[0].set = Move_Speed - 1.5 *Turn_Pid.out; //设置左轮速度目标
    if(Turn_Pid.out <-5) Speed_Pid[1].set = Move_Speed + 2 * Turn_Pid.out;  
	else Speed_Pid[1].set = Move_Speed +  Turn_Pid.out; //设置右轮速度目标

     PID.Calc(&Speed_Pid[0], Speed_Pid[0].ref, Speed_Pid[0].set); //计算左轮速度环输出
     PID.Calc(&Speed_Pid[1], Speed_Pid[1].ref, -Speed_Pid[1].set); //计算右轮速度环输出

     			if(PWM_NewEnable)
  		{
				Motor_SetPWM_L((int)Speed_Pid[0].out);
				Motor_SetPWM_R((int)Speed_Pid[1].out);
//			Motor_SetPWM_L(2000);
//			Motor_SetPWM_R(2000);
			}else if(!PWM_NewEnable){
				Speed_Pid[0].out =0;
				Speed_Pid[1].out =0;
				// yaw =0;
				// Fllow_Move=0;
				Motor_SetPWM_L(0);
				Motor_SetPWM_R(0);
			}
//      if (pitch < -45)
//      {

//// C 可调用 Getter 实现

//            PWM_NewEnable =0;
//      }else{
//            PWM_NewEnable =1;
//      }
      
}
extern "C" {
    void Move_SetSpeed(uint8_t v) { Move_Speed = v; }
    uint8_t Move_GetSpeed(void) { return Move_Speed; }
    uint8_t Move_GetSpeedL_Measure(void) { return Speed_Pid[0].ref; }
    uint8_t Move_GetSpeedR_Measure(void) { return Speed_Pid[1].ref; }
    uint8_t Move_GetSpeedL_Set(void) { return Speed_Pid[0].set; }
    uint8_t Move_GetSpeedR_Set(void) { return Speed_Pid[1].set; } 
    float Move_GetSpeedL_Out(void) { return Speed_Pid[0].out; }
    float Move_GetSpeedR_Out(void) { return Speed_Pid[1].out; }
}

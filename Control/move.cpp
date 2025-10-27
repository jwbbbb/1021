#include "algorithm_pid.h"
#include <stdint.h>

#include "move.h"
// 前向声明：这些 C 接口在 Hardware/Encoder.c 中实现，用于读编码器和设电机 PWM
//#define PL_1MAX	-88.5f 
//#define PL_2MAX -72.5f 
//#define PR_2MAX 100.3f
//#define PR_1MAX 76.5f 
#define PL_1MAX	-75.4f *2
#define PL_2MAX -72.5f 
#define PR_2MAX 100.3f
#define PR_1MAX 76.5f *2


// 编码器/电机接口（使用 C 链接，避免 C++ 名称重整）
extern "C" {
    int16_t Encoder_Get_L(void);   // 读取左编码器计数/速度（实现于 Hardware/Encoder.c）
    int16_t Encoder_Get_R(void);   // 读取右编码器计数/速度（实现于 Hardware/Encoder.c）
    void Motor_SetPWM_L(int16_t Duty); // 设置左电机 PWM 占空比/输出
    void Motor_SetPWM_R(int16_t Duty); // 设置右电机 PWM 占空比/输出
}
uint8_t temppppppp =120;
// 运动控制相关全局变量
sPidTypeDef Speed_Pid[2]; // 速度 PID: 0 = 左轮, 1 = 右轮
sPidTypeDef Turn_Pid;     // 转向 PID
uint8_t Move_Speed = 50;  // 基准移动速度（目标值）

float Fllow_Move;         // 未在本文件内使用的全局变量，保留以兼容其它模块
extern float yaw;         // 来自其他模块的姿态角（声明为 extern）
extern float pitch;       // 来自其他模块的俯仰角（声明为 extern）
extern uint16_t vals[4];  // 传感器或ADC采样数组（外部提供）
uint8_t PWM_NewEnable = 1; // PWM 输出使能标志（1: 允许输出, 0: 禁止输出）
int16_t temp_PWM =1;
float temp_angle;
float Total_T;            // 用于转向计算的总偏差/权重值
extern uint16_t Time_Cont;
void PID_Init(void)
{
    // 初始化 PID 参数：位置式 PID，分别为左右速度环与转向环
    PID.Init(&Speed_Pid[0], POSITION, L_Speed_KP, L_Speed_KI, L_Speed_KD, L_Speed_MaxOut, L_Speed_IntegralLimit, L_Speed_BandI); // 左轮速度环初始化
    PID.Init(&Speed_Pid[1], POSITION, R_Speed_KP, R_Speed_KI, R_Speed_KD, R_Speed_MaxOut, R_Speed_IntegralLimit, R_Speed_BandI); // 右轮速度环初始化
    PID.Init(&Turn_Pid, POSITION, Turn_KP, Turn_KI, Turn_KD, Turn_MaxOut, Turn_IntegralLimit, Turn_BandI); // 转向环初始化
}


void move_task(void)
{

	 if( Time_Cont <135){ Move_Speed =112;}
	 else if(Time_Cont  <1275) {Move_Speed =72;}
	 else  Move_Speed =51;
    // 根据 PWM_NewEnable 决定是否将计算出的 pid 输出下发到电机
    if (PWM_NewEnable)
    {
        Motor_Measure();

        Motor_Turn();
    
    // 计算速度 PID 输出：注意右轮的设定值被取负（与 ref 的符号约定保持一致）
    PID.Calc(&Speed_Pid[0], Speed_Pid[0].ref, Speed_Pid[0].set);
    PID.Calc(&Speed_Pid[1], Speed_Pid[1].ref, Speed_Pid[1].set);
	
        Motor_SetPWM_L((int)Speed_Pid[0].out);
        Motor_SetPWM_R((int)Speed_Pid[1].out);
		
		
		
		
        // 测试时可用固定 PWM 覆盖

    }
    else if (!PWM_NewEnable)
    {
        Motor_Measure();
        // 禁止输出时，把 PID 输出清零并断开电机输出
        Speed_Pid[0].out = 0;
        Speed_Pid[1].out = 0;
        Speed_Pid[0].set =0;
        Speed_Pid[1].set = 0;
        // yaw = 0; Fllow_Move = 0; // 保留的控制逻辑，可按需启用
        Motor_SetPWM_L(temp_PWM);
        Motor_SetPWM_R(temp_PWM);
    }

    // 根据俯仰角自动禁用 PWM 的逻辑被注释掉了，保留以便日后启用
    // if (pitch < -45) { PWM_NewEnable = 0; } else { PWM_NewEnable = 1; }
}

void Motor_Measure( void){
		    // 将实时编码器读数作为 PID 的被控量(ref)
    // 注意：右轮读数取负号，可能是因为编码器安装方向或正负约定不同
    
    Speed_Pid[0].ref = Encoder_Get_L();
    Speed_Pid[1].ref = Encoder_Get_R();


}
void Motor_Turn(void){
// 使用外部传感数组 vals 计算一个用于转向的合成量 Total_T。
    // PL_1MAX / PL_2MAX / PR_1MAX / PR_2MAX 是放大/缩放系数（在文件顶部定义）
    Total_T = vals[0] / PL_1MAX + (vals[1] / PL_2MAX + vals[3] / PR_2MAX) + vals[2] / PR_1MAX;
	
    // 下面是一些旧的或备选的计算方式被注释掉，保留以供调试
    // Total_T = vals[0] /PL_1MAX  + vals[2] / PR_1MAX;
    // if(vals[0]/PL_1MAX < 100 && vals[2]/PR_1MAX < 100) { ... }

    // 计算转向 PID（目标值这里是 Total_T*3，给出比例放大）
     PID.Calc(&Turn_Pid, Total_T * 4.0, 0);

    // 根据转向 PID 输出调整左右轮目标速度（简单的比例偏移）
    // 当转向较大时，放大差速量以提高响应（使用不同的倍率）
//	PID.Calc(&Turn_Pid, yaw, temp_angle);

//    if (Turn_Pid.out > 5)
//        Speed_Pid[0].set = Move_Speed - 2 * Turn_Pid.out;
//    else
        Speed_Pid[0].set = Move_Speed - Turn_Pid.out; // 左轮目标速度

//    if (Turn_Pid.out < -5)
//        Speed_Pid[1].set = Move_Speed + 2 * Turn_Pid.out;
//    else
        Speed_Pid[1].set = Move_Speed + Turn_Pid.out; // 右轮目标速度



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

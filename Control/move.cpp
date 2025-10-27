#include "algorithm_pid.h"
#include <stdint.h>

#include "move.h"
// ǰ����������Щ C �ӿ��� Hardware/Encoder.c ��ʵ�֣����ڶ������������� PWM
//#define PL_1MAX	-88.5f 
//#define PL_2MAX -72.5f 
//#define PR_2MAX 100.3f
//#define PR_1MAX 76.5f 
#define PL_1MAX	-75.4f *2
#define PL_2MAX -72.5f 
#define PR_2MAX 100.3f
#define PR_1MAX 76.5f *2


// ������/����ӿڣ�ʹ�� C ���ӣ����� C++ ����������
extern "C" {
    int16_t Encoder_Get_L(void);   // ��ȡ�����������/�ٶȣ�ʵ���� Hardware/Encoder.c��
    int16_t Encoder_Get_R(void);   // ��ȡ�ұ���������/�ٶȣ�ʵ���� Hardware/Encoder.c��
    void Motor_SetPWM_L(int16_t Duty); // �������� PWM ռ�ձ�/���
    void Motor_SetPWM_R(int16_t Duty); // �����ҵ�� PWM ռ�ձ�/���
}
uint8_t temppppppp =120;
// �˶��������ȫ�ֱ���
sPidTypeDef Speed_Pid[2]; // �ٶ� PID: 0 = ����, 1 = ����
sPidTypeDef Turn_Pid;     // ת�� PID
uint8_t Move_Speed = 50;  // ��׼�ƶ��ٶȣ�Ŀ��ֵ��

float Fllow_Move;         // δ�ڱ��ļ���ʹ�õ�ȫ�ֱ����������Լ�������ģ��
extern float yaw;         // ��������ģ�����̬�ǣ�����Ϊ extern��
extern float pitch;       // ��������ģ��ĸ����ǣ�����Ϊ extern��
extern uint16_t vals[4];  // ��������ADC�������飨�ⲿ�ṩ��
uint8_t PWM_NewEnable = 1; // PWM ���ʹ�ܱ�־��1: �������, 0: ��ֹ�����
int16_t temp_PWM =1;
float temp_angle;
float Total_T;            // ����ת��������ƫ��/Ȩ��ֵ
extern uint16_t Time_Cont;
void PID_Init(void)
{
    // ��ʼ�� PID ������λ��ʽ PID���ֱ�Ϊ�����ٶȻ���ת��
    PID.Init(&Speed_Pid[0], POSITION, L_Speed_KP, L_Speed_KI, L_Speed_KD, L_Speed_MaxOut, L_Speed_IntegralLimit, L_Speed_BandI); // �����ٶȻ���ʼ��
    PID.Init(&Speed_Pid[1], POSITION, R_Speed_KP, R_Speed_KI, R_Speed_KD, R_Speed_MaxOut, R_Speed_IntegralLimit, R_Speed_BandI); // �����ٶȻ���ʼ��
    PID.Init(&Turn_Pid, POSITION, Turn_KP, Turn_KI, Turn_KD, Turn_MaxOut, Turn_IntegralLimit, Turn_BandI); // ת�򻷳�ʼ��
}


void move_task(void)
{

	 if( Time_Cont <135){ Move_Speed =112;}
	 else if(Time_Cont  <1275) {Move_Speed =72;}
	 else  Move_Speed =51;
    // ���� PWM_NewEnable �����Ƿ񽫼������ pid ����·������
    if (PWM_NewEnable)
    {
        Motor_Measure();

        Motor_Turn();
    
    // �����ٶ� PID �����ע�����ֵ��趨ֵ��ȡ������ ref �ķ���Լ������һ�£�
    PID.Calc(&Speed_Pid[0], Speed_Pid[0].ref, Speed_Pid[0].set);
    PID.Calc(&Speed_Pid[1], Speed_Pid[1].ref, Speed_Pid[1].set);
	
        Motor_SetPWM_L((int)Speed_Pid[0].out);
        Motor_SetPWM_R((int)Speed_Pid[1].out);
		
		
		
		
        // ����ʱ���ù̶� PWM ����

    }
    else if (!PWM_NewEnable)
    {
        Motor_Measure();
        // ��ֹ���ʱ���� PID ������㲢�Ͽ�������
        Speed_Pid[0].out = 0;
        Speed_Pid[1].out = 0;
        Speed_Pid[0].set =0;
        Speed_Pid[1].set = 0;
        // yaw = 0; Fllow_Move = 0; // �����Ŀ����߼����ɰ�������
        Motor_SetPWM_L(temp_PWM);
        Motor_SetPWM_R(temp_PWM);
    }

    // ���ݸ������Զ����� PWM ���߼���ע�͵��ˣ������Ա��պ�����
    // if (pitch < -45) { PWM_NewEnable = 0; } else { PWM_NewEnable = 1; }
}

void Motor_Measure( void){
		    // ��ʵʱ������������Ϊ PID �ı�����(ref)
    // ע�⣺���ֶ���ȡ���ţ���������Ϊ��������װ���������Լ����ͬ
    
    Speed_Pid[0].ref = Encoder_Get_L();
    Speed_Pid[1].ref = Encoder_Get_R();


}
void Motor_Turn(void){
// ʹ���ⲿ�������� vals ����һ������ת��ĺϳ��� Total_T��
    // PL_1MAX / PL_2MAX / PR_1MAX / PR_2MAX �ǷŴ�/����ϵ�������ļ��������壩
    Total_T = vals[0] / PL_1MAX + (vals[1] / PL_2MAX + vals[3] / PR_2MAX) + vals[2] / PR_1MAX;
	
    // ������һЩ�ɵĻ�ѡ�ļ��㷽ʽ��ע�͵��������Թ�����
    // Total_T = vals[0] /PL_1MAX  + vals[2] / PR_1MAX;
    // if(vals[0]/PL_1MAX < 100 && vals[2]/PR_1MAX < 100) { ... }

    // ����ת�� PID��Ŀ��ֵ������ Total_T*3�����������Ŵ�
     PID.Calc(&Turn_Pid, Total_T * 4.0, 0);

    // ����ת�� PID �������������Ŀ���ٶȣ��򵥵ı���ƫ�ƣ�
    // ��ת��ϴ�ʱ���Ŵ�������������Ӧ��ʹ�ò�ͬ�ı��ʣ�
//	PID.Calc(&Turn_Pid, yaw, temp_angle);

//    if (Turn_Pid.out > 5)
//        Speed_Pid[0].set = Move_Speed - 2 * Turn_Pid.out;
//    else
        Speed_Pid[0].set = Move_Speed - Turn_Pid.out; // ����Ŀ���ٶ�

//    if (Turn_Pid.out < -5)
//        Speed_Pid[1].set = Move_Speed + 2 * Turn_Pid.out;
//    else
        Speed_Pid[1].set = Move_Speed + Turn_Pid.out; // ����Ŀ���ٶ�



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

#include "myfile.h"
#define oldTurn 0
#define newTurn 1
/************************ȫ�ֱ���***************************/
uint8_t Place_Enable,PWM_Enable=1;//��ת�򻷺��ٶȻ���ʹ�ܿ��أ�
int Speed_Out_L,Speed_Out_R,Place_Out=0;//�����������
int sensor_err,final_err=0;
float straight_err;

/************************PID������***************************/
int Basic_Speed=0;    //�����ٶȣ��������޸��ٶȣ�����Ԫ��Ҫ��ע�͵�
float Turn_factor=0.5;
int Left_Speed=0;
int	Right_Speed=0;
int Speed_PID[3] = {250,5,15}; 
float Move_St;
float tiaoshi_1;
#if oldTurn
float Place_PD[2] = {5.3,4.3};
#endif

#if newTurn
float Place_PD[3] = {0.5,0.001,0.3};
//float Place_PD[3] = {0.5,0.001,1};
//int max_pdOut = 20;
int max_pdOut = 20;
#endif

#if oldTuren
void Control()
{
			// ��ȡ��������Ȩ��ԭ���㷨��
			sensor_err = Error_Calcaulate(); 
	
			// �����ں����	
			final_err = get_fused_error(sensor_err,GZ);
	
			//ת��
			if(Place_Enable)
			{
				Place_Out=(int)Place_Control(final_err,0,Place_PD);
			}
			
			//Ԫ�ش���һ��ʼ������ʱ���ȹص�
			// Element_Process();
			//��ȡ������ֵ
			Encoder_Read();
			
			//ת����
			// Different_Speed();
			
			//�ٶȻ�
			Speed_Out_L=PID_Control(Speed_L,Left_Speed,Speed_PID); 
			Speed_Out_R=PID_Control(Speed_R,Right_Speed,Speed_PID);
	
			//����޷�
			Speed_Out_L=Min_Max( Speed_Out_L ,-Max_PWM, Max_PWM );
			Speed_Out_R=Min_Max( Speed_Out_R ,-Max_PWM, Max_PWM );
			
			//��pwm����������
			if(PWM_Enable)
  		{
				Motor_SetPWM_L(Speed_Out_L);
				Motor_SetPWM_R(Speed_Out_R);
//			Motor_SetPWM_L(2000);
//			Motor_SetPWM_R(2000);
			}
			 
}
#endif

#if 0
void Control()
{
			// // ��ȡ��������Ȩ��ԭ���㷨��
			// sensor_err = Error_Calcaulate(); 
	
			// // �����ں����	
			// final_err = get_fused_error(sensor_err,GZ);
	
			// //ת��
			// if(Place_Enable)
			// {
			// 	Place_Out=(int)Place_Control(final_err,0,Place_PD);
			// }
			
			//Ԫ�ش���һ��ʼ������ʱ���ȹص�
			// Element_Process();
			//��ȡ������ֵ
			Encoder_Read();
			
			//ת����
			// Different_Speed();
			
			//ת�����
			
			 tiaoshi_1 = Place_Control(yaw, Move_St*2, Place_PD);
			 
			Place_Out =(int)tiaoshi_1;
			if (tiaoshi_1 - Place_Out>0.5)
			{
				Place_Out += 1;
			}else if (Place_Out - tiaoshi_1 >0.5)
			{
				Place_Out -= 1;
			}
			{
			Left_Speed = Basic_Speed;
			Right_Speed =Basic_Speed;}

			//�ٶȻ�
			Speed_Out_L=PID_Control(Speed_L,Left_Speed - Place_Out,Speed_PID); 
			Speed_Out_R=PID_Control(Speed_R,Right_Speed + Place_Out,Speed_PID);
	
			//����޷�
			Speed_Out_L=  Min_Max( Speed_Out_L ,-Max_PWM, Max_PWM );
			Speed_Out_R= - Min_Max( Speed_Out_R ,-Max_PWM, Max_PWM );
			
			//��pwm����������
			if(PWM_Enable)
  		{
				Motor_SetPWM_L(Speed_Out_L);
				Motor_SetPWM_R(Speed_Out_R);
//			Motor_SetPWM_L(2000);
//			Motor_SetPWM_R(2000);
			}else if(!PWM_Enable){
				Speed_Out_L =0;
				Speed_Out_R =0;
				yaw =0;
				Move_St=0;
				Motor_SetPWM_L(0);
				Motor_SetPWM_R(0);
			}
			 
}
#endif

/************************λ��ʽת��pd***************************/
float Place_Control(float NowPoint, float SetPoint, float *TURN_PID) //PD����λ�û�
{
	static float LastError ,Integral_Turn= 0; // ��̬����������ʷֵ
	float KP, KI,KD; 
	float NowError, Out; 
	NowError = SetPoint - NowPoint; // ��ǰ��� 
	KP = *TURN_PID; 
	KI =*(TURN_PID+1);
	KD = *(TURN_PID+2); 
	Out = KP * NowError + KI *Integral_Turn +KD *(NowError-LastError); // PID ���ֵ 
	LastError = NowError; //������� 
	if (Out  > max_pdOut)
	{
		Out = max_pdOut;
	}
	if (Out  < -max_pdOut)
	{
		Out = -max_pdOut;
	}
	
	
	return Out; 
}

/************************λ��ʽ�ٶȻ�pd***************************/
int PID_Control(int NowPoint, int SetPoint, int *TURN_PID) //PI�����ٶȻ�
{
	  static int Integral,LastError = 0;      // �����ۻ�������̬������
		int KP,KI,KD,Out,NowError; 
		KP = *TURN_PID; 
		KI = *(TURN_PID+1); 
		KD = *(TURN_PID+2); 
	
    NowError = SetPoint - NowPoint;
    
   // �������ۼӣ����޷������ͣ�
   Integral += NowError;
   Integral = Min_Max(Integral, -INTEGRAL_MAX, INTEGRAL_MAX); // ʾ����INTEGRAL_MAX=1000
    
		//����pi��pd�Ҷ��Թ���pdЧ������һ��
    Out = KP * NowError + KI * Integral+KD *(NowError-LastError);
   
    return Out;
}

/************************���ټ���***************************/
void Different_Speed() 
{ 
 float k;  
 //ת�򻷵����
 //������ٶ�ȡ���ڻ����ٶȵĴ�С�������ٶȿ���֮��ת������Ҳ����ű��
 if(Place_Out >= 0) 
	 {
		 k = Place_Out * 0.01; 
		 Left_Speed = Basic_Speed * (1 - k); 
		 Right_Speed = Basic_Speed * (1 + k*Turn_factor); 
	 } 
	 else 
	{ 
	 k = -Place_Out * 0.01; 
	 Left_Speed = Basic_Speed * (1 + k*Turn_factor); 
	 Right_Speed = Basic_Speed * (1 - k); 
  } 
}

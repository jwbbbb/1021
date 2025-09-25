#include "myfile.h"
/************************ȫ�ֱ���***************************/
uint8_t Place_Enable,PWM_Enable=1;//��ת�򻷺��ٶȻ���ʹ�ܿ��أ�
int Speed_Out_L,Speed_Out_R,Place_Out=0;//�����������
int sensor_err,final_err=0;
float straight_err;

/************************PID������***************************/
int Basic_Speed=0;    //�����ٶȣ��������޸��ٶȣ�����Ԫ��Ҫ��ע�͵�
float Turn_factor=0.5;
int Left_Speed,Right_Speed=0;
int Speed_PID[3] = {250,0,30}; 
float Place_PD[2] = {5.3,4.3};

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
			Element_Process();
			//��ȡ������ֵ
			Encoder_Read();
			
			//ת����
			Different_Speed();
			
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


/************************λ��ʽת��pd***************************/
float Place_Control(float NowPoint, float SetPoint, float *TURN_PID) //PD����λ�û�
{
	static float LastError = 0; // ��̬����������ʷֵ
	float KP, KD; 
	float NowError, Out; 
	NowError = SetPoint - NowPoint; // ��ǰ��� 
	KP = *TURN_PID; 
	KD = *(TURN_PID+1); 
	Out = KP * NowError + KD *(NowError-LastError); // PID ���ֵ 
	LastError = NowError; //������� 
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
    
//    // �������ۼӣ����޷������ͣ�
//    Integral += NowError;
//    Integral = Min_Max(Integral, -INTEGRAL_MAX, INTEGRAL_MAX); // ʾ����INTEGRAL_MAX=1000
    
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

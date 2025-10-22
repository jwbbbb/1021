/***************************************************************************************
  * ��������bվ��ͷ��-��������ѿ�Դ����
  * ���������鿴��ʹ�ú��޸ģ���Ӧ�õ��Լ�����Ŀ֮��
  * �����Ȩ�齭Э�Ƽ����У��κ��˻���֯���ý����Ϊ����
  ***************************************************************************************
  */
#include "myfile.h"
#include "move.h"
volatile uint8_t data_ready = 0;

/************************������***************************/
int main(void)
{
		Serial_Init();
		Key_Init();
		MPU6050_Init();
		OLED_Init();
		Timer_Init();
		Encoder_Init(); 
		SENSOR_GPIO_Config(); //ѭ�����ų�ʼ��
		Motor_Init();					//�����ʼ��
		PWM_Init();						//ռ�ձȶ�ʱ��1��ʼ��
		PID_Init();						//�˶�����PID��ʼ����C++ʵ�֣�
		AD_Init();	
	while (1)
	{		  
		if(data_ready) 	//�������Ƿŵ���ѭ�����У��������Ƶ�����µ��жϿ��٣����������� 
					{	  
						//��ȡ������
						MPU6050_GetData(&AX, &AY, &AZ, &GX, &GY,&GZ);		
						
						
						data_ready = 0;
          }			
			Key_Num = Key_GetNum();
 	    AD_Test();
			menu_operation();																					//�˵����ú���
			Serial_Printf("%d,%d,%f,%f\r\n", 1,Speed_R,yaw,Location); //����������Լ��޸ı�����ӡ���������������ٶȻ���
			OLED_Update();	
	}
}

/************************�ж�***************************/
void TIM2_IRQHandler(void)
{

	if (TIM_GetITStatus(TIM2, TIM_IT_Update) == SET)
	{			 
		
		//move_task();
		data_ready = 1; //�����ǿ��Ʊ�־λ
		Key_Tick();			//��ȡ����ֵ�����ƴ�Ķ�ʱ��������
		TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
	}
}








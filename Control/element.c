#include "myfile.h"

/************************Ԫ��***************************/

//��������
uint8_t Element_Flag=0;
uint8_t Ten_Flag=0;
uint8_t Ring_Flag=0;
uint8_t Noline_Flag=0;
uint8_t Stop_Flag = 0;
uint8_t K=0;
int Speed_Choice[3]={110,0,130};

/************************Ԫ�ؿ���̨***************************/
//�������������߹ص�ĳ��Ԫ��

void Element_Process()
{
    Element_Normal();		//����״̬�£�Ĭ�ϴ�ת�򻷺��ٶȻ�
		Element_Ring();			//Բ��
		Element_Noline();		//���ߴ���
		Element_Stop();			//ͣ��
		Element_Ten();			//ʮ��ʶ�𣨵�δ����
}


void Element_Normal()
{
 if(Element_Flag==0)
 {
	 Place_Enable=1;
	 PWM_Enable=1;
	 Basic_Speed=Speed_Choice[0];
 }
}

//Բ��
void Element_Ring()
{
	if(Element_Flag==0&&Ring_Flag==0&&Noline_Flag==4&&Ten_Flag==1)	//����״̬�¿��Խ�&&Noline_Flag==4&&Ten_Flag==1�Ƴ����������Ǳ���Ϊ�˷�ֹ�󴥣�����һ��Ԫ��˳������
	{
			//��һ�μ���뻷
			if(R2==1&&M==1&&(L1==0||R1==0)&&L2==0)//���μ��
		{		
				Ring_Flag=1;	
				Clear_Location();			
		}
	}
		//��һ�μ�⵽���10cm�ٽ��м��Ȼ���뻷
	if(Ring_Flag==1&&Location>=10.5)
	{		
		if((L1==0&&M==1&&R1==0)||((L1==1||R1==1)&&M==1))
		{
			Ring_Flag=2;	
			Clear_Location();	
			Element_Flag=1;	
			Place_Enable=0;//�ص�ת��
			Place_Out=-15; //��һ���̶��Ĳ���
		}
		else{Ring_Flag=0;}
	}
		//�����뻷����������Ѳ��
	if(Ring_Flag==2&&Location>30)
	{
		Place_Enable=1;
		Ring_Flag=3;
		Basic_Speed=Speed_Choice[2];
		
	}
	//�����������뻷ͬ��
	if(Ring_Flag==3&&Location>380)
	{
		if(L2==1&&M==1&&R2==0)
		{
			Clear_Location();	
			Place_Enable=0;
			Place_Out=-15;	
			Ring_Flag=4;			
		}
	}
	//��������
	if(Ring_Flag==4&&Location>50)
	{
		Element_Flag=0;
		K=1;
		Ring_Flag=5;
		Clear_Location();
		
	}
}


//����   
void Element_Noline()
{
		if (Element_Flag == 0 &&Noline_Flag==0)
		{
			if (L2 == 0 && L1 == 0 && M == 0 && R1 == 0 && R2 == 0)  //���ߵ�ԭ��������е�Ϩ��
	      { 	  
					Noline_Flag = 1;
					Clear_Location();					//������ʶ����֮��ʼ����·��
	      }  								
    }
	if(Noline_Flag==1)	
	{
			if(L2==0&&L1==0&&M==0&&R1==0&&R2==0)
			{
				if(Location>19)							//���ۼ�·�̴���19����ʽ���붪�ߣ������������У������붪��
				{
				Element_Flag=2;
				Place_Enable=0;							
				Place_Out= -100;						//���ȴ����ת������Ҫ�ķ���
				Noline_Flag=2;	
				Clear_Location();						//�����������ͨ���������ж�ת�˶��٣�����������Ҳ���ԣ�
				}		
			}
			else
				{
					Clear_Location();	
					Noline_Flag=0;
				}	
	}
	if(Noline_Flag==2&&Location>25)		//ת����Ҫ�ķ������ֱ�߼��ٳ��ȥ
	{
		Noline_Flag=3;
		PWM_Enable=0;
		Motor_SetPWM_L(6000);						//���ù̶���ռ�ձ�
		Motor_SetPWM_R(6000);  	
	}	
	if(Noline_Flag==3&&Location>=50&&(L2==1||L1==1||M==1||R1==1||R2==1))//ʶ����֮�󣬻ָ�����Ѱ��
		{
			
			Noline_Flag=4;
			Element_Flag=0;
			Clear_Location();	
		}
}
//ͣ��  
void Element_Stop()
{
		if (Element_Flag == 0&&Stop_Flag==0&&Ring_Flag==5) 					//�ҵ�ͣ���������һ��Ԫ��Բ������������Ͷ�·����
		{
			if (L2 == 1 && L1 == 1 && M == 1 && R1 == 1 && R2 == 1)		//��Ȼ��ȫ��������ܺͶ����ص�����Ҫ���ñ�־λ������
	      { 
					Stop_Flag = 1;  
			  	Clear_Location();						
         }
		}
		if (Stop_Flag == 1&&Location >=30) 
		{
			Place_Enable=0;
			Basic_Speed=Speed_Choice[1];
		}
		
}
//ʮ��ʶ�𣬸����ж�Բ�����ҵ�Բ����ʮ��֮�󣬸�����������Լ�����
void Element_Ten()
{
 if(Element_Flag==0&&Ring_Flag==0&&Noline_Flag==4)
 {
	if (L2 == 1 && L1 == 1 && M == 1 && R1 == 1 && R2 == 1)
	{
		Ten_Flag=1;
	}

 }
}

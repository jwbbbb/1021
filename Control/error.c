#include "myfile.h"
#include "sensor.h"

/************************������������***************************/
// �����ṹ��ͺ궨�壨�����ļ���ͷ��
#define SENSOR_NUM 5									 //����������
#define SENSOR_MAX_ERR      50    		// ������������
#define GYRO_SCALE       	  3.5f 			// ���ٶȡ��������ϵ�� (ʾ��: 1��/s �� 0.08 error)
#define FUSION_ALPHA        0.92f		  // �����˲�ϵ��(0.9-0.98)
#define DT                  0.005f 	 	// ��������(5ms)
#define SCALE_FACTOR 10							  // ��������
//��·���ܵ�Ȩ��
#define L22  -2.8*L2
#define L11  -1.3*L1
#define MM   0*M
#define R11  1.3*R1
#define R22  2.8*R2

/* **************** �ṹ�� **************** */
typedef struct 
{
    float integrated_err; // �����ǻ������
    int last_sensor_err;  // �ϴδ��������
} FusionState;
typedef struct   //���ڶ���ṹ�����͡��ṹ����һ���û��Զ�����������ͣ����Խ�����������Ա�������һ�������typedef �ǽ��ṹ�嶨�����������Ϊ SensorMap���Ա����ʹ�á�
{
    float position;  // ������λ�����꣨���Էֲ���
    float channel;  // ��������Ӧ��ͨ����
} SensorMap;

/* **************** ȫ�ֱ��� **************** */
static FusionState fstate = {0, 0};
int i1,i2,i3,i4,i5,err=0;

// ���ݴ���������λ�ö������꣨�����������У�
static const SensorMap sensorMap[SENSOR_NUM] =   //������һ���������飬��ֻ���ڶ��������ļ��������ڷ��ʡ�const ���������е�Ԫ��ֵ�����޸ġ�
{
    {-2.8, 1}, // L2
    {-1.3, 2}, // L1	
    { 0, 3}, // M 
    { 1.3, 4}, // R1
    { 2.8, 5}  // R2	
};


/* ****************���㴫����ƫ�� **************** */
int Error_Calcaulate()
{		
	int active_sum = 0;
	int active_count = 0;
	 for (int i = 0; i < SENSOR_NUM; i++) 		
	{
            if (digital(sensorMap[i].channel))  		//digital �ĺ��������봫������ͨ���š�
						{
                active_sum += sensorMap[i].position; // �ۼӼ��������λ��
                active_count++; 										 //��ѭ���У�active_count ����ÿ������Ĵ�����ʱ���� 1��
						}
   }
	
				err = active_count ? (active_sum * SCALE_FACTOR) / active_count : 0;	//�������������err��Ȼ���޷���ֻ�ǱȽϸ��ӵ�д��
				err = err*Right_err();				//ֱ��ƫ��Ŵ�
	return err;           
}



/* **************** �����Ǹ������� **************** */
int get_fused_error(int sensor_err, float gyro_z) 
{
    /**** ����1�������ǻ��ִ��� ****/
	
    float delta_err = gyro_z*GYRO_SCALE;// �����ٶȷŴ�
    
    // ���ּ��㣨�����������Ϊdt�룩
    float dt = 0.01f; // 1ms���ڣ��ҵ�ʵ��������5ms������ϣ�������ֿ�һ�㣩
    fstate.integrated_err += delta_err * dt;
    
    /**** ����2�������˲��ں� ****/
    int fused_err = 0;
    if( sensor_err!=0||digital(3)==1) 
		{			
				// ���A���д������ź�ʱ�����û��֣����÷�ʽΪ0.4�Ĵ�����0.6�������ǡ�
        fstate.integrated_err = fstate.integrated_err * 0.6f + sensor_err * 0.4f;
//		fstate.last_sensor_err = sensor_err;		
		fused_err=(1-FUSION_ALPHA)* fstate.integrated_err + FUSION_ALPHA*sensor_err;
    } 
		
				// ���B���޴������ź�ʱ������������
		else
		{
				fused_err= fstate.integrated_err;			//û�д�������ʱ��������Ϊ���ϻ��֣����һֱ���
		}
		 /**** ����4���޷����� ****/
			//��if���㷨��һ����ֻ������д�����
			fused_err = (fused_err > SENSOR_MAX_ERR) ? SENSOR_MAX_ERR : ((fused_err < -SENSOR_MAX_ERR) ? -SENSOR_MAX_ERR : fused_err);

    return (int)fused_err;
}

/* **************** ֱ�ǷŴ�ƫ�� **************** */
float Right_err()
{
	if(R2==0&&M==1&&L1==1&&L2==1)//��ֱ��
		{return 27.0f;}
	else if(L2==0&&M==1&&R1==1&&R2==1)//��ֱ��
		{return 27.0f;}
	else 
		{return 1.0f;}
}













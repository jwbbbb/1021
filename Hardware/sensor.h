#ifndef __SENSOR_H
#define	__SENSOR_H

#include "sys.h"
#include "stm32f10x.h"	   

/*-------------���ֶ˿�----------------*/
#define L2 digital(1)
#define L1 digital(2)
#define M digital(3)
#define R1 digital(4)
#define R2 digital(5)

void SENSOR_GPIO_Config(void);

u8 digital(u8 channel);  	//��ȡXͨ������ֵ��0��1�� 1~5						  


#endif 

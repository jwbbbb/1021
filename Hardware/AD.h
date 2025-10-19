#ifndef __AD_H
#define __AD_H

#include "stm32f10x.h"

// adc_buffer ˳��PA2, PA3, PA4, PA5, PB0, PB1
extern volatile uint16_t adc_buffer[6];

void AD_Init(void);
void AD_GetValues(uint16_t *out_buf, uint8_t len);
// ��ȫ��ȡ���ڶ�ȡʱ��ʱ�ر� DMA �Ա�֤����һ����
void AD_GetValuesSafe(uint16_t *out_buf, uint8_t len);

// ���Ժ��������ڶ�ȡ��ͨ�� OLED �򴮿ڴ�ӡ����Ӧ�þ�����
void AD_Test(void);

#endif // __AD_H

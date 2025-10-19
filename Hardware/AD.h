#ifndef __AD_H
#define __AD_H

#include "stm32f10x.h"

// adc_buffer 顺序：PA2, PA3, PA4, PA5, PB0, PB1
extern volatile uint16_t adc_buffer[6];

void AD_Init(void);
void AD_GetValues(uint16_t *out_buf, uint8_t len);
// 安全读取：在读取时临时关闭 DMA 以保证数据一致性
void AD_GetValuesSafe(uint16_t *out_buf, uint8_t len);

// 测试函数：周期读取并通过 OLED 或串口打印（由应用决定）
void AD_Test(void);

#endif // __AD_H

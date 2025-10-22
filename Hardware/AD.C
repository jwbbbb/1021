/*
 * AD.C
 *
 * ADC1 配置：PA2 PA3 PA4 PA5 PB0 五路模拟输入，使用 ADC1 的通道和 DMA1_Channel1 循环采样
 */

#include "stm32f10x.h"
#include "myfile.h"

// ADC 采样缓冲区，顺序：PA2, PA3, PA4, PA5, PB0, PB1
volatile uint16_t adc_buffer[6];

// 安全读取：在读取时临时关闭 DMA 或禁中断以保证读取的一致性
void AD_GetValuesSafe(uint16_t *out_buf, uint8_t len)
{
	uint8_t i;
	if(len > 6) len = 6;
	// 方式1：临时关闭 DMA
	DMA_Cmd(DMA1_Channel1, DISABLE);
	for(i = 0; i < len; i++) out_buf[i] = adc_buffer[i];
	DMA_Cmd(DMA1_Channel1, ENABLE);
}

// 简单测试：读取 ADC 并把数值打印到 OLED（需项目中有 OLED_Printf/OLED_Update）
void AD_Test(void)
{
	uint16_t vals[4];
	AD_GetValues(vals,4);
	// 在 OLED 上显示六路值
	OLED_Printf(0, 0, OLED_8X16, "1:%05d", vals[0]);
	OLED_Printf(0, 16, OLED_8X16, "2:%05d", vals[1]);
	OLED_Printf(0, 32, OLED_8X16, "5:%05d", vals[3]);
	OLED_Printf(0, 48, OLED_8X16, "6:%05d", vals[2]);
	// 下一行显示第5和第6路（换页或覆盖显示，根据 OLED 高度调整）
	// OLED_Printf(64, 0, OLED_8X16, "5:%04d", vals[4]);
	// OLED_Printf(64, 16, OLED_8X16, "6:%04d", vals[5]);
	OLED_Update();
	

}

void ADC1_GPIO_Config(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	// 使能 GPIOA, GPIOB 时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB, ENABLE);

	// 仅配置下面列出的引脚为模拟输入（不会修改端口上其它未列出的引脚）
	// 将 PA2 PA3 PA4 PA5 配置为模拟输入
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	// 将 PB0 和 PB1 配置为模拟输入（注意：PB3/PB4 被 MyI2C 占用，未在此处配置）
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
}

void ADC1_DMA_Config(void)
{
	DMA_InitTypeDef DMA_InitStructure;

	// 使能 DMA 时钟
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

	DMA_DeInit(DMA1_Channel1);
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&ADC1->DR;
	DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)adc_buffer;
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC; // 外设到内存
	DMA_InitStructure.DMA_BufferSize = 6;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
	DMA_Init(DMA1_Channel1, &DMA_InitStructure);

	DMA_Cmd(DMA1_Channel1, ENABLE);
}

void ADC1_Config(void)
{
	ADC_InitTypeDef ADC_InitStructure;

	// 使能 ADC1 时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);

	ADC_DeInit(ADC1);

	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
	ADC_InitStructure.ADC_ScanConvMode = ENABLE; // 扫描模式
	ADC_InitStructure.ADC_ContinuousConvMode = ENABLE; // 连续转换
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
	ADC_InitStructure.ADC_NbrOfChannel = 6;
	ADC_Init(ADC1, &ADC_InitStructure);

	// 配置采样顺序与采样时间：通道对应关系
	// PA2 -> ADC_Channel_2
	ADC_RegularChannelConfig(ADC1, ADC_Channel_2, 1, ADC_SampleTime_55Cycles5);
	// PA3 -> ADC_Channel_3
	ADC_RegularChannelConfig(ADC1, ADC_Channel_3, 2, ADC_SampleTime_55Cycles5);
	// PA4 -> ADC_Channel_4
	ADC_RegularChannelConfig(ADC1, ADC_Channel_4, 3, ADC_SampleTime_55Cycles5);
	// PA5 -> ADC_Channel_5
	ADC_RegularChannelConfig(ADC1, ADC_Channel_5, 4, ADC_SampleTime_55Cycles5);
	// PB0 -> ADC_Channel_8
	ADC_RegularChannelConfig(ADC1, ADC_Channel_8, 5, ADC_SampleTime_55Cycles5);
	// PB1 -> ADC_Channel_9
	ADC_RegularChannelConfig(ADC1, ADC_Channel_9, 6, ADC_SampleTime_55Cycles5);

	// DMA 请求使能
	ADC_DMACmd(ADC1, ENABLE);

	// 校准 ADC
	ADC_Cmd(ADC1, ENABLE);
	ADC_ResetCalibration(ADC1);
	while(ADC_GetResetCalibrationStatus(ADC1));
	ADC_StartCalibration(ADC1);
	while(ADC_GetCalibrationStatus(ADC1));

	// 启动转换
	ADC_SoftwareStartConvCmd(ADC1, ENABLE);
}

// 对外接口：初始化 ADC + DMA
void AD_Init(void)
{
	ADC1_GPIO_Config();
	ADC1_DMA_Config();
	ADC1_Config();
}

// 获取当前 adc_buffer 的副本（线程安全：无中断保护，若需要可在调用侧禁中断）
void AD_GetValues(uint16_t *out_buf, uint8_t len)
{
	uint8_t i;
	if(len > 6) len = 6;
	for(i = 0; i < len; i++) out_buf[i] = adc_buffer[i];
}

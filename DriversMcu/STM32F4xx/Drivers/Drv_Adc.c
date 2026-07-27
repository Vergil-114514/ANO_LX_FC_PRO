#include "Drv_Adc.h"

uint16_t adcBuffer[8];

void DrvAdcInit(void)
{
    GPIO_InitTypeDef GPIO_InitStruct;
    // 使能 GPIOB 和 GPIOC 时钟
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB | RCC_AHB1Periph_GPIOC, ENABLE);
    // 配置 PB0 和 PB1
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AN;
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOB, &GPIO_InitStruct);
    // 配置 PC0-PC5
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5;
    GPIO_Init(GPIOC, &GPIO_InitStruct);
    ADC_InitTypeDef ADC_InitStruct;
    ADC_CommonInitTypeDef ADC_CommonInitStruct;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE); // 使能 ADC1 时钟
    // 通用 ADC 配置
    ADC_CommonInitStruct.ADC_Mode = ADC_Mode_Independent;
    ADC_CommonInitStruct.ADC_Prescaler = ADC_Prescaler_Div4; // ADC 时钟 = 84MHz / 4 = 21MHz
    ADC_CommonInitStruct.ADC_DMAAccessMode = ADC_DMAAccessMode_1;		 /*DMA失能*/
    ADC_CommonInitStruct.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles; /*两个采样阶段的延时5个时钟*/
    ADC_CommonInit( &ADC_CommonInitStruct);
    // ADC1 配置
    ADC_InitStruct.ADC_Resolution = ADC_Resolution_12b;
    ADC_InitStruct.ADC_ScanConvMode = ENABLE;              // 多通道扫描
    ADC_InitStruct.ADC_ContinuousConvMode = ENABLE;        // 连续转换
    ADC_InitStruct.ADC_ExternalTrigConv = 0; // 软件触发
    ADC_InitStruct.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
    ADC_InitStruct.ADC_DataAlign = ADC_DataAlign_Right;
    ADC_InitStruct.ADC_NbrOfConversion = 8;                // 8 个通道（PB0, PB1, PC0-PC5）
    ADC_Init(ADC1, &ADC_InitStruct);
    // 配置通道顺序和采样时间（480 周期适用于高阻抗信号）
    ADC_RegularChannelConfig(ADC1, ADC_Channel_8,  1, ADC_SampleTime_480Cycles); // PB0
    ADC_RegularChannelConfig(ADC1, ADC_Channel_9,  2, ADC_SampleTime_480Cycles); // PB1
    ADC_RegularChannelConfig(ADC1, ADC_Channel_10, 3, ADC_SampleTime_480Cycles); // PC0
    ADC_RegularChannelConfig(ADC1, ADC_Channel_11, 4, ADC_SampleTime_480Cycles); // PC1
    ADC_RegularChannelConfig(ADC1, ADC_Channel_12, 5, ADC_SampleTime_480Cycles); // PC2
    ADC_RegularChannelConfig(ADC1, ADC_Channel_13, 6, ADC_SampleTime_480Cycles); // PC3
    ADC_RegularChannelConfig(ADC1, ADC_Channel_14, 7, ADC_SampleTime_480Cycles); // PC4
    ADC_RegularChannelConfig(ADC1, ADC_Channel_15, 8, ADC_SampleTime_480Cycles); // PC5
    DMA_InitTypeDef DMA_InitStruct;
    DMA_StructInit( &DMA_InitStruct);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE); // 使能 DMA2 时钟

    while (DMA_GetCmdStatus(DMA2_Stream0) != DISABLE)
        ; /*等待DMA可以配置*/

    DMA_DeInit(DMA2_Stream0);
    DMA_InitStruct.DMA_Channel = DMA_Channel_0;          // ADC1 使用 DMA2 通道0
    DMA_InitStruct.DMA_PeripheralBaseAddr = (uint32_t) &ADC1->DR;
    DMA_InitStruct.DMA_Memory0BaseAddr = (uint32_t)adcBuffer;
    DMA_InitStruct.DMA_DIR = DMA_DIR_PeripheralToMemory;
    DMA_InitStruct.DMA_BufferSize = 8;                   // 8 个通道
    DMA_InitStruct.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStruct.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStruct.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
    DMA_InitStruct.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
    DMA_InitStruct.DMA_Mode = DMA_Mode_Circular;         // 循环模式
    DMA_InitStruct.DMA_Priority = DMA_Priority_High;
    DMA_InitStruct.DMA_FIFOMode = DMA_FIFOMode_Disable;
    DMA_InitStruct.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;			/*FIFO的值*/
    DMA_InitStruct.DMA_MemoryBurst = DMA_MemoryBurst_Single;					/*单次传输*/
    DMA_InitStruct.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;			/*单次传输*/
    DMA_Init(DMA2_Stream0, &DMA_InitStruct);
    DMA_Cmd(DMA2_Stream0, ENABLE); // 启动 DMA
    ADC_DMARequestAfterLastTransferCmd(ADC1, ENABLE);							 //源数据变化时开启DMA传输
    ADC_DMACmd(ADC1, ENABLE);  // 启用 ADC1 的 DMA 请求
    ADC_Cmd(ADC1, ENABLE);     // 启动 ADC1
    ADC_SoftwareStartConv(ADC1); // 软件触发 ADC 开始转换
}

float AdcVal_BatVot;
float AdcVal_BatCur;
void DrvAdcCal(void)
{
    if (PMU_TYPE == PT_NULL)
    {
#define UP_R 10 //10K
#define DW_R 1	//1K
        //
        AdcVal_BatVot = (float)adcBuffer[0] / 4096.0f * 3300.0f * (UP_R + DW_R) / DW_R * 0.001f;
        AdcVal_BatCur = (float)adcBuffer[1] / 4096.0f * 200.0f;
    }
    else if (PMU_TYPE == PT_FCS200)
    {
        AdcVal_BatVot = (float)adcBuffer[0] / 4096.0f * 3.3f * 15.65f;
        AdcVal_BatCur = (float)adcBuffer[1] / 4096.0f * 200.0f;
    }
}


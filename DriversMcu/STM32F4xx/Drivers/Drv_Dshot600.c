#include "Drv_Dshot600.h"

#define ESC_BIT_0 102
#define ESC_BIT_1 205
#define ESC_CMD_BUFFER_LEN 16
#define DMA_BUFFER_SIZE (ESC_CMD_BUFFER_LEN * 4 + 8)

static uint16_t dma_buffer[DMA_BUFFER_SIZE];
uint16_t add_checksum_and_telemetry(uint16_t packet, uint8_t telem)
{
	uint16_t packet_telemetry = (packet << 1) | (telem & 1);
	uint8_t i;
	uint16_t csum = 0;
	uint16_t csum_data = packet_telemetry;
	for (i = 0; i < 3; i++)
	{
		csum ^= csum_data; // xor data by nibbles
		csum_data >>= 4;
	}
	csum &= 0xf;
	return (packet_telemetry << 4) | csum; // append checksum
}
	
void DrvDshot600Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
	/* Enable GPIO clocks */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
	/* Enable DMA clocks */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);
	/* Enable TIM GPIO clocks */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_11 | GPIO_Pin_13 | GPIO_Pin_14;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOE, &GPIO_InitStructure);

    GPIO_PinAFConfig(GPIOE, GPIO_PinSource9, GPIO_AF_TIM1);
    GPIO_PinAFConfig(GPIOE, GPIO_PinSource11, GPIO_AF_TIM1);
    GPIO_PinAFConfig(GPIOE, GPIO_PinSource13, GPIO_AF_TIM1);
    GPIO_PinAFConfig(GPIOE, GPIO_PinSource14, GPIO_AF_TIM1);
	
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_OCInitTypeDef TIM_OCInitStructure;

	TIM_TimeBaseStructure.TIM_Period = 275;
	TIM_TimeBaseStructure.TIM_Prescaler = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);

	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Disable;
	TIM_OCInitStructure.TIM_Pulse = 0;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_Low;
	TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Reset;
    TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCNIdleState_Set;
	
	TIM_OC1Init(TIM1, &TIM_OCInitStructure);
	TIM_OC2Init(TIM1, &TIM_OCInitStructure);
	TIM_OC3Init(TIM1, &TIM_OCInitStructure);
	TIM_OC4Init(TIM1, &TIM_OCInitStructure);

	TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);
	TIM_OC2PreloadConfig(TIM1, TIM_OCPreload_Enable);
	TIM_OC3PreloadConfig(TIM1, TIM_OCPreload_Enable);
	TIM_OC4PreloadConfig(TIM1, TIM_OCPreload_Enable);

	TIM_DMAConfig(TIM1, TIM_DMABase_CCR1, TIM_DMABurstLength_4Transfers);
	TIM_DMACmd(TIM1, TIM_DMA_Update, ENABLE);
	TIM_ARRPreloadConfig(TIM1, ENABLE);
	TIM_CtrlPWMOutputs(TIM1, ENABLE);
	TIM_Cmd(TIM1, ENABLE);
	
	DMA_InitTypeDef DMA_InitStructure;
	DMA_StructInit(&DMA_InitStructure);
	
	DMA_DeInit(DMA2_Stream5);
	DMA_InitStructure.DMA_Channel = DMA_Channel_6;
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&TIM1->DMAR;
	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)&dma_buffer[0];
	DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;
	DMA_InitStructure.DMA_BufferSize = DMA_BUFFER_SIZE;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	DMA_Init(DMA2_Stream5, &DMA_InitStructure);
	DMA_Cmd(DMA2_Stream5, ENABLE);
}
uint16_t _1val[4] = {0};
void DrvDshot600Set(int16_t pwm[])
{
	//·¶Î§0-1000£¬×ª»»µ½DSHOTµÄ47-2047
	uint16_t motor_data[4];
	motor_data[0] = (uint16_t)((pwm[0] * 2) + 47);
	motor_data[1] = (uint16_t)((pwm[1] * 2) + 47);
	motor_data[2] = (uint16_t)((pwm[2] * 2) + 47);
	motor_data[3] = (uint16_t)((pwm[3] * 2) + 47);
	motor_data[0] = add_checksum_and_telemetry(motor_data[0], 0);
	motor_data[1] = add_checksum_and_telemetry(motor_data[1], 0);
	motor_data[2] = add_checksum_and_telemetry(motor_data[2], 0);
	motor_data[3] = add_checksum_and_telemetry(motor_data[3], 0);

	for (uint8_t i = 0; i < ESC_CMD_BUFFER_LEN; i++)
	{
		dma_buffer[i * 4] = (motor_data[0] & (0x01 << (ESC_CMD_BUFFER_LEN - i - 1))) ? ESC_BIT_1 : ESC_BIT_0;
		dma_buffer[i * 4 + 1] = (motor_data[1] & (0x01 << (ESC_CMD_BUFFER_LEN - i - 1))) ? ESC_BIT_1 : ESC_BIT_0;
		dma_buffer[i * 4 + 2] = (motor_data[2] & (0x01 << (ESC_CMD_BUFFER_LEN - i - 1))) ? ESC_BIT_1 : ESC_BIT_0;
		dma_buffer[i * 4 + 3] = (motor_data[3] & (0x01 << (ESC_CMD_BUFFER_LEN - i - 1))) ? ESC_BIT_1 : ESC_BIT_0;
	}

	DMA_ClearFlag(DMA2_Stream5, DMA_FLAG_TCIF5);
	DMA_Cmd(DMA2_Stream5, DISABLE);
	DMA_SetCurrDataCounter(DMA2_Stream5, DMA_BUFFER_SIZE);
	DMA_Cmd(DMA2_Stream5, ENABLE);
}


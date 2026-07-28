#include "Drv_RcIn.h"
#include <string.h>
//====PPM====
void DrvRcPpmInit(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    TIM_ICInitTypeDef TIM_ICInitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
    //==
    GPIO_StructInit(&GPIO_InitStructure);
    TIM_ICStructInit(&TIM_ICInitStructure);

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);

    NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = NVIC_PWMIN_P;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = NVIC_PWMIN_S;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    GPIO_PinAFConfig(GPIOC, GPIO_PinSource7, GPIO_AF_TIM3);

    TIM3->PSC = (168 / 2) - 1;

    TIM_ICInitStructure.TIM_Channel = TIM_Channel_2;
    TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_BothEdge;
    TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
    TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
    TIM_ICInitStructure.TIM_ICFilter = 0x0;
    TIM_ICInit(TIM3, &TIM_ICInitStructure);

    TIM_Cmd(TIM3, ENABLE);
    TIM_ITConfig(TIM3, TIM_IT_CC2, ENABLE);
}

void PPM_IRQH()
{

    static uint16_t  temp_cnt[2];
    //
    if (TIM3->SR & TIM_IT_CC2)
    {
        TIM3->SR = ~TIM_IT_CC2; //TIM_ClearITPendingBit(TIM3, TIM_IT_CC1);
        TIM3->SR = ~TIM_FLAG_CC2OF;
        //==
//		if (!(GPIOC->IDR & GPIO_Pin_7))
//		{
//			temp_cnt[0] = TIM_GetCapture2(TIM3);
//		}
//		else
//		{
//			temp_cnt[1] = TIM_GetCapture2(TIM3);
//			uint16_t  _tmp;
//			_tmp = temp_cnt[1] - temp_cnt[0];

//			DrvPpmGetOneCh(_tmp + 400);//×ª»»µ½1000-2000
//		}
        //==
        if (!(GPIOC->IDR & GPIO_Pin_7))
        {
            temp_cnt[0] = TIM_GetCapture2(TIM3);
            uint16_t  _tmp;
            _tmp = temp_cnt[0] - temp_cnt[1];
            //
            DrvPpmGetOneCh(_tmp);//
            //
            temp_cnt[1] = temp_cnt[0];
        }
        else
        {
            //temp_cnt[1] = TIM_GetCapture2(TIM3);

        }
    }
}
//====S-BUS====
void DrvRcSbusInit(void)
{
    USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
    GPIO_InitTypeDef GPIO_InitStructure;

    GPIO_StructInit(&GPIO_InitStructure);

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART6, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);

    NVIC_InitStructure.NVIC_IRQChannel = USART6_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = NVIC_UART6_P;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = NVIC_UART6_S;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    GPIO_PinAFConfig(GPIOC, GPIO_PinSource7, GPIO_AF_USART6);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL; //GPIO_PuPd_UP;//
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    USART_InitStructure.USART_BaudRate = 100000;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_2;
    USART_InitStructure.USART_Parity = USART_Parity_Even;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx;
    USART_Init(USART6, &USART_InitStructure);

    USART_ITConfig(USART6, USART_IT_RXNE, ENABLE);

    USART_Cmd(USART6, ENABLE);
}

void Sbus_IRQH(void)
{
    uint8_t com_data;

    if (USART_GetITStatus(USART6, USART_IT_RXNE))
    {
        USART_ClearITPendingBit(USART6, USART_IT_RXNE);
        //==
        com_data = USART6->DR;
        //
        DrvSbusGetOneByte(com_data);
    }
}
//====LORA3A22====
#define LORA3A22_AXIS_FULL_SCALE 2047

static uint8_t lora_rx_buffer[LORA3A22_FRAME_LEN];
static uint8_t lora_rx_index;
static uint16_t lora_link_age_ms;
static uint8_t lora_link_seen;

static uint8_t loraFrameChecksum(const uint8_t *frame)
{
    uint8_t checksum = 0;
    uint8_t i;

    for (i = 0; i < LORA3A22_FRAME_LEN; i++)
    {
        if (i != 1U)
        {
            checksum += frame[i];
        }
    }

    return checksum;
}

static int16_t loraReadInt16LE(const uint8_t *data)
{
    return (int16_t)((uint16_t)data[0] | ((uint16_t)data[1] << 8));
}

static int16_t loraAxisToChannel(int16_t raw, int32_t direction)
{
    int32_t channel;

    channel = 1500 + ((int32_t)raw * direction * 500) / LORA3A22_AXIS_FULL_SCALE;

    if (channel < 1000)
    {
        channel = 1000;
    }
    else if (channel > 2000)
    {
        channel = 2000;
    }

    return (int16_t)channel;
}

static void loraUpdateChannels(void)
{
    const uint8_t *frame = lora_rx_buffer;
    uint8_t switch0 = frame[14];
    uint8_t switch1 = frame[15];
    uint8_t i;

    // Mode 2: right horizontal/vertical -> roll/pitch, left vertical/horizontal -> throttle/yaw.
    rc_in.lora_ch[0] = loraAxisToChannel(loraReadInt16LE(&frame[6]), LORA3A22_ROLL_SIGN);
    rc_in.lora_ch[1] = loraAxisToChannel(loraReadInt16LE(&frame[8]), LORA3A22_PITCH_SIGN);
    rc_in.lora_ch[2] = loraAxisToChannel(loraReadInt16LE(&frame[4]), LORA3A22_THROTTLE_SIGN);
    rc_in.lora_ch[3] = loraAxisToChannel(loraReadInt16LE(&frame[2]), LORA3A22_YAW_SIGN);

    // switch_key[2]: high is normal; low requests the existing emergency-stop channel.
    rc_in.lora_ch[4] = (frame[16] != 0U) ? 1000 : 2000;

    if (switch0 != 0U && switch1 != 0U)
    {
        rc_in.lora_ch[5] = 1000;
    }
    else if (switch0 != 0U)
    {
        rc_in.lora_ch[5] = 1500;
    }
    else if (switch1 != 0U)
    {
        rc_in.lora_ch[5] = 2000;
    }
    else
    {
        rc_in.lora_ch[5] = 1000;
    }

    for (i = 0; i < 4U; i++)
    {
        rc_in.lora_ch[6U + i] = (frame[10U + i] != 0U) ? 2000 : 1000;
    }
}

void DrvRcLoraInit(void)
{
    lora_rx_index = 0;
    lora_link_age_ms = LORA3A22_LINK_TIMEOUT_MS;
    lora_link_seen = 0;
}

void DrvRcLoraRxOneByte(const uint8_t linktype, const uint8_t rx_byte)
{
    (void)linktype;

    if (lora_rx_index == 0U)
    {
        if (rx_byte == LORA3A22_FRAME_HEAD)
        {
            lora_rx_buffer[lora_rx_index++] = rx_byte;
        }
        return;
    }

    lora_rx_buffer[lora_rx_index++] = rx_byte;

    if (lora_rx_index < LORA3A22_FRAME_LEN)
    {
        return;
    }

    if (loraFrameChecksum(lora_rx_buffer) == lora_rx_buffer[1])
    {
        loraUpdateChannels();
        lora_link_age_ms = 0;
        lora_link_seen = 1;
        rc_in.signal_cnt_tmp++;
        rc_in.rc_in_mode_tmp = 3;
        rc_in.sig_mode = 3;
        rc_in.no_signal = 0;
    }

    lora_rx_index = 0;
}

uint8_t DrvRcLoraLinkAlive(float dT_s)
{
    uint16_t elapsed_ms;

    if (lora_link_seen == 0U || lora_link_age_ms >= LORA3A22_LINK_TIMEOUT_MS)
    {
        return 0;
    }

    elapsed_ms = (uint16_t)(dT_s * 1000.0f + 0.5f);
    if (elapsed_ms >= (LORA3A22_LINK_TIMEOUT_MS - lora_link_age_ms))
    {
        lora_link_age_ms = LORA3A22_LINK_TIMEOUT_MS;
    }
    else
    {
        lora_link_age_ms += elapsed_ms;
    }

    return (lora_link_age_ms < LORA3A22_LINK_TIMEOUT_MS) ? 1U : 0U;
}

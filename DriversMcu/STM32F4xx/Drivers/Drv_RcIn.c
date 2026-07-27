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

//			DrvPpmGetOneCh(_tmp + 400);//转换到1000-2000
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
//====CRSF====
CRSF_Frame crsf_frame;
void DrvRcCrsfInit(void)
{
	
}

static uint8_t crsf_crc8(uint8_t *data, uint8_t len)
{
    uint8_t crc = 0;
    for (uint8_t i = 0; i < len; i++)
    {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; j++)
        {
            if (crc & 0x80)
            {
                crc = (crc << 1) ^ 0xD5;
            }
            else
            {
                crc <<= 1;
            }
        }
    }
    return crc;
}

void DrvRcCrsfRxOneByte(const uint8_t linktype, const uint8_t rx_byte)
{
	static uint8_t crsf_rx_index = 0;
    static uint8_t crsf_rx_buffer[CRSF_MAX_PAYLOAD_LEN + 4];  // 接收缓冲区

    // 1. 检查是否是新帧开始（设备地址）
    if (crsf_rx_index == 0 && rx_byte == CRSF_ADDRESS_FLIGHT_CONTROLLER)
    {
        crsf_rx_buffer[crsf_rx_index++] = rx_byte;
    }
    // 2. 如果已经开始接收，存储数据
    else if (crsf_rx_index > 0)
    {
        crsf_rx_buffer[crsf_rx_index++] = rx_byte;

        // 3. 检查是否接收完整个帧
        if (crsf_rx_index >= 2)
        {
            uint8_t frame_len = crsf_rx_buffer[1];  // 第 2 字节是长度

            if (crsf_rx_index >= frame_len + 2)     // 完整帧 = Addr(1) + Len(1) + Data(Len) + CRC(1)
            {
                // 4. 校验 CRC
                uint8_t crc = crsf_crc8( &crsf_rx_buffer[2], frame_len - 1); // 计算 CRC（Addr + Len + Data）

                if (crc == crsf_rx_buffer[frame_len + 1])
                {
                    // 5. 解析帧
                    crsf_frame.device_addr = crsf_rx_buffer[0];
                    crsf_frame.frame_length = crsf_rx_buffer[1];
                    crsf_frame.frame_type = crsf_rx_buffer[2];
                    memcpy((uint8_t *)crsf_frame.payload, &crsf_rx_buffer[3], frame_len - 2); // 数据部分
                    crsf_frame.crc = crsf_rx_buffer[frame_len + 1];

                    if (crsf_frame.frame_type == CRSF_FRAMETYPE_RC_CHANNELS_PACKED)
                    {
                        crsf_channels_t *ch = (crsf_channels_t *) &crsf_rx_buffer[3];
                        rc_in.crsf_ch[0] = ch->ch0;
                        rc_in.crsf_ch[1] = ch->ch1;
                        rc_in.crsf_ch[2] = ch->ch2;
                        rc_in.crsf_ch[3] = ch->ch3;
                        rc_in.crsf_ch[4] = ch->ch4;
                        rc_in.crsf_ch[5] = ch->ch5;
                        rc_in.crsf_ch[6] = ch->ch6;
                        rc_in.crsf_ch[7] = ch->ch7;
                        rc_in.crsf_ch[8] = ch->ch8;
                        rc_in.crsf_ch[9] = ch->ch9;
                        rc_in.crsf_ch[10] = ch->ch10;
                        rc_in.crsf_ch[11] = ch->ch11;
                        rc_in.crsf_ch[12] = ch->ch12;
                        rc_in.crsf_ch[13] = ch->ch13;
                        rc_in.crsf_ch[14] = ch->ch14;
                        rc_in.crsf_ch[15] = ch->ch15;
						
						rc_in.signal_cnt_tmp++;
						rc_in.rc_in_mode_tmp = 3; //切换模式标记为crsf
                    }
                    else
                    {
                    }
                }

                crsf_rx_index = 0;  // 重置接收索引
            }
        }
    }
}



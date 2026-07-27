#include "Drv_Sys.h"

static uint64_t SysRunTimeMs = 0;

void SysTick_Init(void )
{
    RCC_ClocksTypeDef  rcc_clocks;
    uint32_t         cnts;

    RCC_GetClocksFreq ( &rcc_clocks );

    cnts = ( uint32_t ) rcc_clocks.HCLK_Frequency / TICK_PER_SECOND;
    cnts = cnts / 8;

    SysTick_Config ( cnts );
    SysTick_CLKSourceConfig ( SysTick_CLKSource_HCLK_Div8 );
}
void SysTick_Handler(void)
{
    SysRunTimeMs++;
}
uint32_t GetSysRunTimeMs(void)
{
    return SysRunTimeMs;
}
uint32_t GetSysRunTimeUs(void)
{
    register uint32_t ms;
    uint32_t value;
    do
    {
        ms = SysRunTimeMs;
        value = ms * TICK_US + ( SysTick->LOAD - SysTick->VAL ) * TICK_US / SysTick->LOAD;
    }
    while(ms != SysRunTimeMs);
    return value;
}

void MyDelayUs ( uint32_t us )
{
    uint32_t now = GetSysRunTimeUs();
    while ( GetSysRunTimeUs() - now < us );
}
void MyDelayMs(uint32_t time)
{
    while ( time-- )
        MyDelayUs ( 1000 );
}

void DrvSysInit(void)
{
    //中断优先级组别设置
    NVIC_PriorityGroupConfig(NVIC_GROUP);
    //滴答时钟
    SysTick_Init();
}

void UsbPortCtlInit(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_StructInit(&GPIO_InitStructure);

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);

    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
    GPIO_Init(GPIOD, &GPIO_InitStructure);

    GPIO_ResetBits(GPIOD, GPIO_Pin_11);
}

void UsbPortCtl(const uint8_t sel)
{
    if(sel)
    {
        //uart2连接至usb插座
        GPIO_SetBits(GPIOD, GPIO_Pin_11);
    }
    else
    {
        //mcu的dp\dm连接至usb插座
        GPIO_ResetBits(GPIOD, GPIO_Pin_11);
    }
}


/******************** (C) COPYRIGHT 2017 ANO Tech ********************************
 * 作者    ：匿名科创
 * 官网    ：www.anotc.com
 * 淘宝    ：anotc.taobao.com
 * 技术Q群 ：190169595
 * 描述    ：串口驱动
**********************************************************************************/
#include "Drv_Uart.h"
#include "AnoPTv8ExAPI.h"
#include "AnoPTv8.h"
#include "DataTransfer.h"
#include "Drv_UbloxGPS.h"
#include "Drv_AnoOf.h"
#include "DrvAnoOF_ptv7.h"
#include "Drv_RcIn.h"

void NoUse(const uint8_t type, const uint8_t data) {}
//串口接收发送快速定义，直接修改此处的函数名称宏，修改成自己的串口解析和发送函数名称即可，注意函数参数格式需统一
//Uart1：连接IMU
//Uart2：丝印UD		连接了内部CH343芯片
//Uart3：丝印UA		RL
//Uart4：丝印UC		GPS
//Uart5：丝印UB		OF
//Uart6：SBUS接口，只接了RX
//Uart7：丝印UF
//Uart8：丝印UG
#define U1GetOneByte	AnoPTv8HwRecvByte
#define U2GetOneByte	AnoPTv8HwRecvByte
#define U3GetOneByte	AnoPTv8HwRecvByte
#define U4GetOneByte	UBLOX_M8_GPS_Data_Receive
#define U5GetOneByte	DrvAnoOFGetOneByte_ptv7
#define U7GetOneByte	DrvRcCrsfRxOneByte
#define U8GetOneByte	AnoPTv8HwRecvByte
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//====uart1
#define U1RXBUFSIZE		256
#define U1TXBUFSIZE		2048
volatile uint8_t 	U1TxBuf[U1TXBUFSIZE];
volatile uint16_t 	U1TxInCnt = 0;
volatile uint16_t 	U1TxOutCnt = 0;
volatile uint8_t 	U1RxBuf[U1RXBUFSIZE];
volatile uint16_t 	U1RxInCnt = 0;
volatile uint16_t 	U1RxoutCnt = 0;

void DrvUart1Init(uint32_t br_num)
{
    RCC_APB2PeriphClockCmd(UART1_RCC, ENABLE); //开启UART时钟
    RCC_AHB1PeriphClockCmd(UART1_PORT_RCC, ENABLE);

    //串口中断优先级
    NVIC_InitTypeDef NVIC_InitStructure;
    NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = NVIC_UART1_P;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = NVIC_UART1_S;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    //配置IO
    GPIO_PinAFConfig(UART1_PORT, GPIO_PinSource9, GPIO_AF_USART1);
    GPIO_PinAFConfig(UART1_PORT, GPIO_PinSource10, GPIO_AF_USART1);
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Pin = UART1_PIN_TX;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(UART1_PORT, &GPIO_InitStructure);
    //
    GPIO_InitStructure.GPIO_Pin = UART1_PIN_RX;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(UART1_PORT, &GPIO_InitStructure);

    //配置串口
    USART_InitTypeDef USART_InitStructure;
    USART_StructInit(&USART_InitStructure);
    USART_DeInit(USART1);
    USART_InitStructure.USART_BaudRate = br_num;                                    //波特率可以通过地面站配置
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;                     //8位数据
    USART_InitStructure.USART_StopBits = USART_StopBits_1;                          //在帧结尾传输1个停止位
    USART_InitStructure.USART_Parity = USART_Parity_No;                             //禁用奇偶校验
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; //硬件流控制失能
    USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;                 //发送、接收使能

    //配置UART时钟
    USART_ClockInitTypeDef USART_ClockInitStruct;
    USART_ClockInitStruct.USART_Clock = USART_Clock_Disable;     //时钟低电平活动
    USART_ClockInitStruct.USART_CPOL = USART_CPOL_Low;           //SLCK引脚上时钟输出的极性->低电平
    USART_ClockInitStruct.USART_CPHA = USART_CPHA_2Edge;         //时钟第二个边沿进行数据捕获
    USART_ClockInitStruct.USART_LastBit = USART_LastBit_Disable; //最后一位数据的时钟脉冲不从SCLK输出

    USART_Init(USART1, &USART_InitStructure);
    USART_ClockInit(USART1, &USART_ClockInitStruct);

    //使能UART接收中断
    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
    //使能UART
    USART_Cmd(USART1, ENABLE);
}

void DrvUart1SendBuf(const uint8_t *DataToSend, const uint8_t data_num)
{
    uint8_t i;
    for (i = 0; i < data_num; i++)
    {
        U1TxBuf[U1TxInCnt++] = *(DataToSend + i);
        if(U1TxInCnt >= U1TXBUFSIZE)
            U1TxInCnt = 0;
    }

    if (!(USART1->CR1 & USART_CR1_TXEIE))
    {
        USART_ITConfig(USART1, USART_IT_TXE, ENABLE); //打开发送中断
    }
}

void drvU1GetByte(uint8_t data)
{
    U1RxBuf[U1RxInCnt++] = data;
    if(U1RxInCnt >= U1RXBUFSIZE)
        U1RxInCnt = 0;
}
void drvU1DataCheck(void)
{
    while(U1RxInCnt!=U1RxoutCnt)
    {
        U1GetOneByte(LT_U1, U1RxBuf[U1RxoutCnt++]);
        if(U1RxoutCnt >= U1RXBUFSIZE)
            U1RxoutCnt = 0;
    }
}
void Usart1_IRQ(void)
{
    uint8_t com_data;

    if (USART1->SR & USART_SR_ORE) //ORE中断
    {
        com_data = USART1->DR;
    }
    //接收中断
    if (USART_GetITStatus(USART1, USART_IT_RXNE))
    {
        USART_ClearITPendingBit(USART1, USART_IT_RXNE); //清除中断标志
        com_data = USART1->DR;
        drvU1GetByte(com_data);
    }
    //发送（进入移位）中断
    if (USART_GetITStatus(USART1, USART_IT_TXE))
    {
        USART1->DR = U1TxBuf[U1TxOutCnt++]; //写DR清除中断标志
		if(U1TxOutCnt >= U1TXBUFSIZE)
            U1TxOutCnt = 0;
        if (U1TxOutCnt == U1TxInCnt)
        {
            USART1->CR1 &= ~USART_CR1_TXEIE; //关闭TXE（发送中断）中断
        }
        
    }
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//====uart2
#define U2RXBUFSIZE		256
#define U2TXBUFSIZE		2048
volatile uint8_t 	U2TxBuf[U2TXBUFSIZE];
volatile uint16_t 	U2TxInCnt = 0;
volatile uint16_t 	U2TxOutCnt = 0;
volatile uint8_t 	U2RxBuf[U2RXBUFSIZE];
volatile uint16_t 	U2RxInCnt = 0;
volatile uint16_t 	U2RxoutCnt = 0;
void DrvUart2Init(uint32_t br_num)
{
    RCC_APB1PeriphClockCmd(UART2_RCC, ENABLE); //开启USART时钟
    RCC_AHB1PeriphClockCmd(UART2_PORT_RCC, ENABLE);

    //串口中断优先级
    NVIC_InitTypeDef NVIC_InitStructure;
    NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = NVIC_UART2_P;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = NVIC_UART2_S;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    //配置IO
    GPIO_PinAFConfig(UART2_PORT, GPIO_PinSource5, GPIO_AF_USART2);
    GPIO_PinAFConfig(UART2_PORT, GPIO_PinSource6, GPIO_AF_USART2);
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Pin = UART2_PIN_TX;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(UART2_PORT, &GPIO_InitStructure);
    //
    GPIO_InitStructure.GPIO_Pin = UART2_PIN_RX;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
    GPIO_Init(UART2_PORT, &GPIO_InitStructure);

    //配置串口
    USART_InitTypeDef USART_InitStructure;
    USART_StructInit(&USART_InitStructure);
    USART_DeInit(USART2);
    USART_InitStructure.USART_BaudRate = br_num;                                    //波特率可以通过地面站配置
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;                     //8位数据
    USART_InitStructure.USART_StopBits = USART_StopBits_1;                          //在帧结尾传输1个停止位
    USART_InitStructure.USART_Parity = USART_Parity_No;                             //禁用奇偶校验
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; //硬件流控制失能
    USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;                 //发送、接收使能

    //配置时钟
    USART_ClockInitTypeDef USART_ClockInitStruct;
    USART_ClockInitStruct.USART_Clock = USART_Clock_Disable;     //时钟低电平活动
    USART_ClockInitStruct.USART_CPOL = USART_CPOL_Low;           //SLCK引脚上时钟输出的极性->低电平
    USART_ClockInitStruct.USART_CPHA = USART_CPHA_2Edge;         //时钟第二个边沿进行数据捕获
    USART_ClockInitStruct.USART_LastBit = USART_LastBit_Disable; //最后一位数据的时钟脉冲不从SCLK输出

    USART_Init(USART2, &USART_InitStructure);
    USART_ClockInit(USART2, &USART_ClockInitStruct);

    //使能USART接收中断
    USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
    //使能USART
    USART_Cmd(USART2, ENABLE);
}

void DrvUart2SendBuf(const uint8_t *DataToSend, const uint8_t data_num)
{
    uint8_t i;
    for (i = 0; i < data_num; i++)
    {
        U2TxBuf[U2TxInCnt++] = *(DataToSend + i);
        if(U2TxInCnt >= U2TXBUFSIZE)
            U2TxInCnt = 0;
    }
    if (!(USART2->CR1 & USART_CR1_TXEIE))
    {
        USART_ITConfig(USART2, USART_IT_TXE, ENABLE); //打开发送中断
    }
}

void drvU2GetByte(uint8_t data)
{
    U2RxBuf[U2RxInCnt++] = data;
    if(U2RxInCnt >= U2RXBUFSIZE)
        U2RxInCnt = 0;
}
void drvU2DataCheck(void)
{
    while(U2RxInCnt!=U2RxoutCnt)
    {
        U2GetOneByte(LT_U2, U2RxBuf[U2RxoutCnt++]);
        if(U2RxoutCnt >= U2RXBUFSIZE)
            U2RxoutCnt = 0;
    }
}
void Usart2_IRQ(void)
{
    uint8_t com_data;
    if (USART2->SR & USART_SR_ORE) //ORE中断
    {
        com_data = USART2->DR;
    }
    //接收中断
    if (USART_GetITStatus(USART2, USART_IT_RXNE))
    {
        USART_ClearITPendingBit(USART2, USART_IT_RXNE); //清除中断标志
        com_data = USART2->DR;
        drvU2GetByte(com_data);
    }
    //发送（进入移位）中断
    if (USART_GetITStatus(USART2, USART_IT_TXE))
    {
        USART2->DR = U2TxBuf[U2TxOutCnt++]; //写DR清除中断标志
        if(U2TxOutCnt >= U2TXBUFSIZE)
            U2TxOutCnt = 0;
        if (U2TxOutCnt == U2TxInCnt)
        {
            USART2->CR1 &= ~USART_CR1_TXEIE; //关闭TXE（发送中断）中断
        }
    }
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//====uart3
#define U3RXBUFSIZE		256
#define U3TXBUFSIZE		256
uint8_t 	U3TxBuf[U3TXBUFSIZE];
uint16_t 	U3TxInCnt = 0;
uint16_t 	U3TxOutCnt = 0;
uint8_t 	U3RxBuf[U3RXBUFSIZE];
uint16_t 	U3RxInCnt = 0;
uint16_t 	U3RxoutCnt = 0;
void DrvUart3Init(uint32_t br_num)
{
    RCC_APB1PeriphClockCmd(UART3_RCC, ENABLE); //开启UART时钟
    RCC_AHB1PeriphClockCmd(UART3_PORT_RCC, ENABLE);

    //串口中断优先级
    NVIC_InitTypeDef NVIC_InitStructure;
    NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = NVIC_UART3_P;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = NVIC_UART3_S;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    //配置IO
    GPIO_PinAFConfig(UART3_PORT, GPIO_PinSource8, GPIO_AF_USART3);
    GPIO_PinAFConfig(UART3_PORT, GPIO_PinSource9, GPIO_AF_USART3);
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Pin = UART3_PIN_TX;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(UART3_PORT, &GPIO_InitStructure);
    //
    GPIO_InitStructure.GPIO_Pin = UART3_PIN_RX;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(UART3_PORT, &GPIO_InitStructure);

    //配置串口
    USART_InitTypeDef USART_InitStructure;
    USART_StructInit(&USART_InitStructure);
    USART_DeInit(USART3);
    USART_InitStructure.USART_BaudRate = br_num;                                    //波特率可以通过地面站配置
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;                     //8位数据
    USART_InitStructure.USART_StopBits = USART_StopBits_1;                          //在帧结尾传输1个停止位
    USART_InitStructure.USART_Parity = USART_Parity_No;                             //禁用奇偶校验
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; //硬件流控制失能
    USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;                 //发送、接收使能

    //配置时钟
    USART_ClockInitTypeDef USART_ClockInitStruct;
    USART_ClockInitStruct.USART_Clock = USART_Clock_Disable;     //时钟低电平活动
    USART_ClockInitStruct.USART_CPOL = USART_CPOL_Low;           //SLCK引脚上时钟输出的极性->低电平
    USART_ClockInitStruct.USART_CPHA = USART_CPHA_2Edge;         //时钟第二个边沿进行数据捕获
    USART_ClockInitStruct.USART_LastBit = USART_LastBit_Disable; //最后一位数据的时钟脉冲不从SCLK输出

    USART_Init(USART3, &USART_InitStructure);
    USART_ClockInit(USART3, &USART_ClockInitStruct);

    //使能UART接收中断
    USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);
    //使能UART
    USART_Cmd(USART3, ENABLE);
}

void DrvUart3SendBuf(const uint8_t *DataToSend, const uint8_t data_num)
{
    uint8_t i;
    for (i = 0; i < data_num; i++)
    {
        U3TxBuf[U3TxInCnt++] = *(DataToSend + i);
        if(U3TxInCnt >= U3TXBUFSIZE)
            U3TxInCnt = 0;
    }

    if (!(USART3->CR1 & USART_CR1_TXEIE))
    {
        USART_ITConfig(USART3, USART_IT_TXE, ENABLE); //打开发送中断
    }
}

void drvU3GetByte(uint8_t data)
{
    U3RxBuf[U3RxInCnt++] = data;
    if(U3RxInCnt >= U3RXBUFSIZE)
        U3RxInCnt = 0;
}
void drvU3DataCheck(void)
{
    while(U3RxInCnt!=U3RxoutCnt)
    {
        U3GetOneByte(LT_U3, U3RxBuf[U3RxoutCnt++]);
        if(U3RxoutCnt >= U3RXBUFSIZE)
            U3RxoutCnt = 0;
    }
}
void Usart3_IRQ(void)
{
    uint8_t com_data;

    if (USART3->SR & USART_SR_ORE) //ORE中断
    {
        com_data = USART3->DR;
    }
    //接收中断
    if (USART_GetITStatus(USART3, USART_IT_RXNE))
    {
        USART_ClearITPendingBit(USART3, USART_IT_RXNE); //清除中断标志
        com_data = USART3->DR;
        drvU3GetByte(com_data);
    }
    //发送（进入移位）中断
    if (USART_GetITStatus(USART3, USART_IT_TXE))
    {
        USART3->DR = U3TxBuf[U3TxOutCnt++]; //写DR清除中断标志
		if(U3TxOutCnt >= U3TXBUFSIZE)
            U3TxOutCnt = 0;
        if (U3TxOutCnt == U3TxInCnt)
        {
            USART3->CR1 &= ~USART_CR1_TXEIE; //关闭TXE（发送中断）中断
        }
    }
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//====uart4
#define U4RXBUFSIZE		256
#define U4TXBUFSIZE		256
uint8_t 	U4TxBuf[U4TXBUFSIZE];
uint16_t 	U4TxInCnt = 0;
uint16_t 	U4TxOutCnt = 0;
uint8_t 	U4RxBuf[U4RXBUFSIZE];
uint16_t 	U4RxInCnt = 0;
uint16_t 	U4RxoutCnt = 0;
void DrvUart4Init(uint32_t br_num)
{
    RCC_APB1PeriphClockCmd(UART4_RCC, ENABLE); //开启UART时钟
    RCC_AHB1PeriphClockCmd(UART4_PORT_RCC, ENABLE);

    //串口中断优先级
    NVIC_InitTypeDef NVIC_InitStructure;
    NVIC_InitStructure.NVIC_IRQChannel = UART4_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = NVIC_UART4_P;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = NVIC_UART4_S;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    //配置IO
    GPIO_PinAFConfig(UART4_PORT, GPIO_PinSource0, GPIO_AF_UART4);
    GPIO_PinAFConfig(UART4_PORT, GPIO_PinSource1, GPIO_AF_UART4);
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Pin = UART4_PIN_TX;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(UART4_PORT, &GPIO_InitStructure);
    //
    GPIO_InitStructure.GPIO_Pin = UART4_PIN_RX;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(UART4_PORT, &GPIO_InitStructure);

    //配置串口
    USART_InitTypeDef USART_InitStructure;
    USART_StructInit(&USART_InitStructure);
    USART_DeInit(UART4);
    USART_InitStructure.USART_BaudRate = br_num;                                    //波特率可以通过地面站配置
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;                     //8位数据
    USART_InitStructure.USART_StopBits = USART_StopBits_1;                          //在帧结尾传输1个停止位
    USART_InitStructure.USART_Parity = USART_Parity_No;                             //禁用奇偶校验
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; //硬件流控制失能
    USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;                 //发送、接收使能
    USART_Init(UART4, &USART_InitStructure);

    //使能UART接收中断
    USART_ITConfig(UART4, USART_IT_RXNE, ENABLE);
    //使能UART
    USART_Cmd(UART4, ENABLE);
}

void DrvUart4SendBuf(const uint8_t *DataToSend, const uint8_t data_num)
{
    uint8_t i;
    for (i = 0; i < data_num; i++)
    {
        U4TxBuf[U4TxInCnt++] = *(DataToSend + i);
        if(U4TxInCnt >= U4TXBUFSIZE)
            U4TxInCnt = 0;
    }

    if (!(UART4->CR1 & USART_CR1_TXEIE))
    {
        USART_ITConfig(UART4, USART_IT_TXE, ENABLE); //打开发送中断
    }
}

void drvU4GetByte(uint8_t data)
{
    U4RxBuf[U4RxInCnt++] = data;
    if(U4RxInCnt >= U4RXBUFSIZE)
        U4RxInCnt = 0;
}
void drvU4DataCheck(void)
{
    while(U4RxInCnt!=U4RxoutCnt)
    {
        U4GetOneByte(LT_U4, U4RxBuf[U4RxoutCnt++]);
        if(U4RxoutCnt >= U4RXBUFSIZE)
            U4RxoutCnt = 0;
    }
}
void Uart4_IRQ(void)
{
    uint8_t com_data;

    if (UART4->SR & USART_SR_ORE) //ORE中断
    {
        com_data = UART4->DR;
    }
    //接收中断
    if (USART_GetITStatus(UART4, USART_IT_RXNE))
    {
        USART_ClearITPendingBit(UART4, USART_IT_RXNE); //清除中断标志
        com_data = UART4->DR;
        drvU4GetByte(com_data);
    }
    //发送（进入移位）中断
    if (USART_GetITStatus(UART4, USART_IT_TXE))
    {
        UART4->DR = U4TxBuf[U4TxOutCnt++]; //写DR清除中断标志
		if(U4TxOutCnt >= U4TXBUFSIZE)
            U4TxOutCnt = 0;
        if (U4TxOutCnt == U4TxInCnt)
        {
            UART4->CR1 &= ~USART_CR1_TXEIE; //关闭TXE（发送中断）中断
        }
    }
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//====uart5
#define U5RXBUFSIZE		256
#define U5TXBUFSIZE		256
uint8_t 	U5TxBuf[U5TXBUFSIZE];
uint16_t 	U5TxInCnt = 0;
uint16_t 	U5TxOutCnt = 0;
uint8_t 	U5RxBuf[U5RXBUFSIZE];
uint16_t 	U5RxInCnt = 0;
uint16_t 	U5RxoutCnt = 0;
void DrvUart5Init(uint32_t br_num)
{
    RCC_APB1PeriphClockCmd(UART5_RCC, ENABLE); //开启UART时钟
    RCC_AHB1PeriphClockCmd(UART5_PORT1_RCC, ENABLE);
    RCC_AHB1PeriphClockCmd(UART5_PORT2_RCC, ENABLE);

    //串口中断优先级
    NVIC_InitTypeDef NVIC_InitStructure;
    NVIC_InitStructure.NVIC_IRQChannel = UART5_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = NVIC_UART5_P;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = NVIC_UART5_S;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    //配置IO
    GPIO_PinAFConfig(UART5_PORT1, GPIO_PinSource12, GPIO_AF_UART5);
    GPIO_PinAFConfig(UART5_PORT2, GPIO_PinSource2, GPIO_AF_UART5);
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Pin = UART5_PIN_TX;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(UART5_PORT1, &GPIO_InitStructure);
    //
    GPIO_InitStructure.GPIO_Pin = UART5_PIN_RX;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(UART5_PORT2, &GPIO_InitStructure);

    //配置串口
    USART_InitTypeDef USART_InitStructure;
    USART_StructInit(&USART_InitStructure);
    USART_DeInit(UART5);
    USART_InitStructure.USART_BaudRate = br_num;                                    //波特率可以通过地面站配置
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;                     //8位数据
    USART_InitStructure.USART_StopBits = USART_StopBits_1;                          //在帧结尾传输1个停止位
    USART_InitStructure.USART_Parity = USART_Parity_No;                             //禁用奇偶校验
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; //硬件流控制失能
    USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;                 //发送、接收使能
    USART_Init(UART5, &USART_InitStructure);

    //使能UART接收中断
    USART_ITConfig(UART5, USART_IT_RXNE, ENABLE);
    //使能UART
    USART_Cmd(UART5, ENABLE);
}

void DrvUart5SendBuf(const uint8_t *DataToSend, const uint8_t data_num)
{
    uint8_t i;
    for (i = 0; i < data_num; i++)
    {
        U5TxBuf[U5TxInCnt++] = *(DataToSend + i);
        if(U5TxInCnt >= U5TXBUFSIZE)
            U5TxInCnt = 0;
    }

    if (!(UART5->CR1 & USART_CR1_TXEIE))
    {
        USART_ITConfig(UART5, USART_IT_TXE, ENABLE); //打开发送中断
    }
}

void drvU5GetByte(uint8_t data)
{
    U5RxBuf[U5RxInCnt++] = data;
    if(U5RxInCnt >= U5RXBUFSIZE)
        U5RxInCnt = 0;
}
void drvU5DataCheck(void)
{
    while(U5RxInCnt!=U5RxoutCnt)
    {
        U5GetOneByte(LT_U5, U5RxBuf[U5RxoutCnt++]);
        if(U5RxoutCnt >= U4RXBUFSIZE)
            U5RxoutCnt = 0;
    }
}
void Uart5_IRQ(void)
{
    uint8_t com_data;

    if (UART5->SR & USART_SR_ORE) //ORE中断
    {
        com_data = UART5->DR;
    }
    //接收中断
    if (USART_GetITStatus(UART5, USART_IT_RXNE))
    {
        USART_ClearITPendingBit(UART5, USART_IT_RXNE); //清除中断标志
        com_data = UART5->DR;
        drvU5GetByte(com_data);
    }
    //发送（进入移位）中断
    if (USART_GetITStatus(UART5, USART_IT_TXE))
    {
        UART5->DR = U5TxBuf[U5TxOutCnt++]; //写DR清除中断标志
		if(U5TxOutCnt >= U5TXBUFSIZE)
            U5TxOutCnt = 0;
        if (U5TxOutCnt == U5TxInCnt)
        {
            UART5->CR1 &= ~USART_CR1_TXEIE; //关闭TXE（发送中断）中断
        }
    }
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//====uart7
#define U7RXBUFSIZE		256
#define U7TXBUFSIZE		256
uint8_t 	U7TxBuf[U7TXBUFSIZE];
uint16_t 	U7TxInCnt = 0;
uint16_t 	U7TxOutCnt = 0;
uint8_t 	U7RxBuf[U7RXBUFSIZE];
uint16_t 	U7RxInCnt = 0;
uint16_t 	U7RxoutCnt = 0;
void DrvUart7Init(uint32_t br_num)
{
    RCC_APB1PeriphClockCmd(UART7_RCC, ENABLE); //开启UART时钟
    RCC_AHB1PeriphClockCmd(UART7_PORT_RCC, ENABLE);

    //串口中断优先级
    NVIC_InitTypeDef NVIC_InitStructure;
    NVIC_InitStructure.NVIC_IRQChannel = UART7_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = NVIC_UART7_P;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = NVIC_UART7_S;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    //配置IO
    GPIO_PinAFConfig(UART7_PORT, GPIO_PinSource8, GPIO_AF_UART7);
    GPIO_PinAFConfig(UART7_PORT, GPIO_PinSource7, GPIO_AF_UART7);
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Pin = UART7_PIN_TX;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(UART7_PORT, &GPIO_InitStructure);
    //
    GPIO_InitStructure.GPIO_Pin = UART7_PIN_RX;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(UART7_PORT, &GPIO_InitStructure);

    //配置串口
    USART_InitTypeDef USART_InitStructure;
    USART_StructInit(&USART_InitStructure);
    USART_DeInit(UART7);
    USART_InitStructure.USART_BaudRate = br_num;                                    //波特率可以通过地面站配置
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;                     //8位数据
    USART_InitStructure.USART_StopBits = USART_StopBits_1;                          //在帧结尾传输1个停止位
    USART_InitStructure.USART_Parity = USART_Parity_No;                             //禁用奇偶校验
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; //硬件流控制失能
    USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;                 //发送、接收使能
    USART_Init(UART7, &USART_InitStructure);

    //使能UART接收中断
    USART_ITConfig(UART7, USART_IT_RXNE, ENABLE);
    //使能UART
    USART_Cmd(UART7, ENABLE);
}

void DrvUart7SendBuf(const uint8_t *DataToSend, const uint8_t data_num)
{
    uint8_t i;
    for (i = 0; i < data_num; i++)
    {
        U7TxBuf[U7TxInCnt++] = *(DataToSend + i);
        if(U7TxInCnt >= U7TXBUFSIZE)
            U7TxInCnt = 0;
    }

    if (!(UART7->CR1 & USART_CR1_TXEIE))
    {
        USART_ITConfig(UART7, USART_IT_TXE, ENABLE); //打开发送中断
    }
}

void drvU7GetByte(uint8_t data)
{
    U7RxBuf[U7RxInCnt++] = data;
    if(U7RxInCnt >= U7RXBUFSIZE)
        U7RxInCnt = 0;
}
void drvU7DataCheck(void)
{
    while(U7RxInCnt!=U7RxoutCnt)
    {
        U7GetOneByte(LT_U7, U7RxBuf[U7RxoutCnt++]);
        if(U7RxoutCnt >= U7RXBUFSIZE)
            U7RxoutCnt = 0;
    }
}
void Uart7_IRQ(void)
{
    uint8_t com_data;

    if (UART7->SR & USART_SR_ORE) //ORE中断
    {
        com_data = UART7->DR;
    }
    //接收中断
    if (USART_GetITStatus(UART7, USART_IT_RXNE))
    {
        USART_ClearITPendingBit(UART7, USART_IT_RXNE); //清除中断标志
        com_data = UART7->DR;
        drvU7GetByte(com_data);
    }
    //发送（进入移位）中断
    if (USART_GetITStatus(UART7, USART_IT_TXE))
    {
        UART7->DR = U7TxBuf[U7TxOutCnt++]; //写DR清除中断标志
		if(U7TxOutCnt >= U7TXBUFSIZE)
            U7TxOutCnt = 0;
        if (U7TxOutCnt == U7TxInCnt)
        {
            UART7->CR1 &= ~USART_CR1_TXEIE; //关闭TXE（发送中断）中断
        }
    }
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//====uart8
#define U8RXBUFSIZE		256
#define U8TXBUFSIZE		256
uint8_t 	U8TxBuf[U8TXBUFSIZE];
uint16_t 	U8TxInCnt = 0;
uint16_t 	U8TxOutCnt = 0;
uint8_t 	U8RxBuf[U8RXBUFSIZE];
uint16_t 	U8RxInCnt = 0;
uint16_t 	U8RxoutCnt = 0;
void DrvUart8Init(uint32_t br_num)
{
    RCC_APB1PeriphClockCmd(UART8_RCC, ENABLE); //开启UART时钟
    RCC_AHB1PeriphClockCmd(UART8_PORT_RCC, ENABLE);

    //串口中断优先级
    NVIC_InitTypeDef NVIC_InitStructure;
    NVIC_InitStructure.NVIC_IRQChannel = UART8_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = NVIC_UART8_P;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = NVIC_UART8_S;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    //配置IO
    GPIO_PinAFConfig(UART8_PORT, GPIO_PinSource1, GPIO_AF_UART8);
    GPIO_PinAFConfig(UART8_PORT, GPIO_PinSource0, GPIO_AF_UART8);
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Pin = UART8_PIN_TX;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(UART8_PORT, &GPIO_InitStructure);
    //
    GPIO_InitStructure.GPIO_Pin = UART8_PIN_RX;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(UART8_PORT, &GPIO_InitStructure);

    //配置串口
    USART_InitTypeDef USART_InitStructure;
    USART_StructInit(&USART_InitStructure);
    USART_DeInit(UART8);
    USART_InitStructure.USART_BaudRate = br_num;                                    //波特率可以通过地面站配置
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;                     //8位数据
    USART_InitStructure.USART_StopBits = USART_StopBits_1;                          //在帧结尾传输1个停止位
    USART_InitStructure.USART_Parity = USART_Parity_No;                             //禁用奇偶校验
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; //硬件流控制失能
    USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;                 //发送、接收使能
    USART_Init(UART8, &USART_InitStructure);

    //使能UART接收中断
    USART_ITConfig(UART8, USART_IT_RXNE, ENABLE);
    //使能UART
    USART_Cmd(UART8, ENABLE);
}

void DrvUart8SendBuf(const uint8_t *DataToSend, const uint8_t data_num)
{
    uint8_t i;
    for (i = 0; i < data_num; i++)
    {
        U8TxBuf[U8TxInCnt++] = *(DataToSend + i);
        if(U8TxInCnt >= U8TXBUFSIZE)
            U8TxInCnt = 0;
    }

    if (!(UART8->CR1 & USART_CR1_TXEIE))
    {
        USART_ITConfig(UART8, USART_IT_TXE, ENABLE); //打开发送中断
    }
}

void drvU8GetByte(uint8_t data)
{
    U8RxBuf[U8RxInCnt++] = data;
    if(U8RxInCnt >= U8RXBUFSIZE)
        U8RxInCnt = 0;
}
void drvU8DataCheck(void)
{
    while(U8RxInCnt!=U8RxoutCnt)
    {
        U8GetOneByte(LT_U8, U8RxBuf[U8RxoutCnt++]);
        if(U8RxoutCnt >= U8RXBUFSIZE)
            U8RxoutCnt = 0;
    }
}
void Uart8_IRQ(void)
{
    uint8_t com_data;

    if (UART8->SR & USART_SR_ORE) //ORE中断
    {
        com_data = UART8->DR;
    }
    //接收中断
    if (USART_GetITStatus(UART8, USART_IT_RXNE))
    {
        USART_ClearITPendingBit(UART8, USART_IT_RXNE); //清除中断标志
        com_data = UART8->DR;
        drvU8GetByte(com_data);
    }
    //发送（进入移位）中断
    if (USART_GetITStatus(UART8, USART_IT_TXE))
    {
        UART8->DR = U8TxBuf[U8TxOutCnt++]; //写DR清除中断标志
		if(U8TxOutCnt >= U8TXBUFSIZE)
            U8TxOutCnt = 0;
        if (U8TxOutCnt == U8TxInCnt)
        {
            UART8->CR1 &= ~USART_CR1_TXEIE; //关闭TXE（发送中断）中断
        }
    }
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void DrvUartDataCheck(void)
{
    drvU1DataCheck();
    drvU2DataCheck();
    drvU3DataCheck();
    drvU4DataCheck();
    drvU5DataCheck();
    drvU7DataCheck();
    drvU8DataCheck();
}
/******************* (C) COPYRIGHT 2014 ANO TECH *****END OF FILE************/

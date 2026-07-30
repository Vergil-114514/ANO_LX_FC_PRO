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
#include "Drv_UwbMini5.h"
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
#if NAV_INPUT_UWB_MINI5
#define U4GetOneByte	DrvUwbMini5RxOneByte
#else
#define U4GetOneByte	UBLOX_M8_GPS_Data_Receive
#endif
#define U5GetOneByte	DrvAnoOFGetOneByte_ptv7
#define U7GetOneByte	DrvRcLoraRxOneByte
#define U8GetOneByte	AnoPTv8HwRecvByte
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
volatile _drv_uart_stats_st DrvUartStats[9];

static uint8_t drvUartTxQueue(volatile uint8_t *txBuf, const uint16_t txBufSize,
                               volatile uint16_t *txInCnt, volatile uint16_t *txOutCnt,
                               const uint8_t *dataToSend, const uint8_t dataNum,
                               volatile _drv_uart_stats_st *stats)
{
    uint32_t primask;
    uint16_t txIn;
    uint16_t txOut;
    uint16_t used;
    uint16_t freeSize;

    if (dataNum == 0U)
    {
        return 1U;
    }

    primask = __get_PRIMASK();
    __disable_irq();

    txIn = *txInCnt;
    txOut = *txOutCnt;
    if (txIn >= txOut)
    {
        used = txIn - txOut;
    }
    else
    {
        used = txBufSize - txOut + txIn;
    }
    freeSize = txBufSize - used - 1U;

    if (dataNum > freeSize)
    {
        stats->tx_dropped_frames++;
        stats->tx_dropped_bytes += dataNum;
        __set_PRIMASK(primask);
        return 0U;
    }

    for (uint8_t i = 0U; i < dataNum; i++)
    {
        txBuf[txIn++] = dataToSend[i];
        if (txIn >= txBufSize)
        {
            txIn = 0U;
        }
    }
    *txInCnt = txIn;
    stats->tx_queued_bytes += dataNum;

    __set_PRIMASK(primask);
    return 1U;
}

static void drvUartRxStore(volatile uint8_t *rxBuf, const uint16_t rxBufSize,
                           volatile uint16_t *rxInCnt, volatile uint16_t *rxOutCnt,
                           const uint8_t data, volatile _drv_uart_stats_st *stats)
{
    uint16_t rxIn = *rxInCnt;
    uint16_t nextRxIn = rxIn + 1U;

    if (nextRxIn >= rxBufSize)
    {
        nextRxIn = 0U;
    }

    if (nextRxIn == *rxOutCnt)
    {
        stats->rx_dropped_bytes++;
        return;
    }

    rxBuf[rxIn] = data;
    *rxInCnt = nextRxIn;
    stats->rx_bytes++;
}

static uint8_t drvUartTxLoad(volatile uint8_t *txBuf, const uint16_t txBufSize,
                             volatile uint16_t *txInCnt, volatile uint16_t *txOutCnt,
                             uint8_t *data, volatile _drv_uart_stats_st *stats)
{
    uint16_t txOut;

    if (*txOutCnt == *txInCnt)
    {
        return 0U;
    }

    txOut = *txOutCnt;
    *data = txBuf[txOut++];
    if (txOut >= txBufSize)
    {
        txOut = 0U;
    }
    *txOutCnt = txOut;
    stats->tx_sent_bytes++;
    return 1U;
}

static void drvUartIRQ(USART_TypeDef *uart, volatile uint8_t *txBuf, const uint16_t txBufSize,
                       volatile uint16_t *txInCnt, volatile uint16_t *txOutCnt,
                       volatile uint8_t *rxBuf, const uint16_t rxBufSize,
                       volatile uint16_t *rxInCnt, volatile uint16_t *rxOutCnt,
                       volatile _drv_uart_stats_st *stats)
{
    const uint32_t status = uart->SR;
    uint8_t data;

    if (status & (USART_SR_ORE | USART_SR_NE | USART_SR_FE | USART_SR_PE))
    {
        data = (uint8_t)uart->DR;
        (void)data;
        stats->rx_hardware_errors++;
    }
    else if (status & USART_SR_RXNE)
    {
        data = (uint8_t)uart->DR;
        drvUartRxStore(rxBuf, rxBufSize, rxInCnt, rxOutCnt, data, stats);
    }

    if ((status & USART_SR_TXE) && (uart->CR1 & USART_CR1_TXEIE))
    {
        if (drvUartTxLoad(txBuf, txBufSize, txInCnt, txOutCnt, &data, stats))
        {
            uart->DR = data;
        }
        else
        {
            uart->CR1 &= ~USART_CR1_TXEIE;
        }
    }
}

//====uart1
#define U1RXBUFSIZE		512
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
    if (drvUartTxQueue(U1TxBuf, U1TXBUFSIZE, &U1TxInCnt, &U1TxOutCnt,
                       DataToSend, data_num, &DrvUartStats[1]) &&
        !(USART1->CR1 & USART_CR1_TXEIE))
    {
        USART_ITConfig(USART1, USART_IT_TXE, ENABLE); //打开发送中断
    }
}

void drvU1GetByte(uint8_t data)
{
    drvUartRxStore(U1RxBuf, U1RXBUFSIZE, &U1RxInCnt, &U1RxoutCnt,
                   data, &DrvUartStats[1]);
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
    drvUartIRQ(USART1, U1TxBuf, U1TXBUFSIZE, &U1TxInCnt, &U1TxOutCnt,
               U1RxBuf, U1RXBUFSIZE, &U1RxInCnt, &U1RxoutCnt, &DrvUartStats[1]);
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
    if (drvUartTxQueue(U2TxBuf, U2TXBUFSIZE, &U2TxInCnt, &U2TxOutCnt,
                       DataToSend, data_num, &DrvUartStats[2]) &&
        !(USART2->CR1 & USART_CR1_TXEIE))
    {
        USART_ITConfig(USART2, USART_IT_TXE, ENABLE); //打开发送中断
    }
}

void drvU2GetByte(uint8_t data)
{
    drvUartRxStore(U2RxBuf, U2RXBUFSIZE, &U2RxInCnt, &U2RxoutCnt,
                   data, &DrvUartStats[2]);
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
    drvUartIRQ(USART2, U2TxBuf, U2TXBUFSIZE, &U2TxInCnt, &U2TxOutCnt,
               U2RxBuf, U2RXBUFSIZE, &U2RxInCnt, &U2RxoutCnt, &DrvUartStats[2]);
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//====uart3
#define U3RXBUFSIZE		256
#define U3TXBUFSIZE		2048
volatile uint8_t 	U3TxBuf[U3TXBUFSIZE];
volatile uint16_t 	U3TxInCnt = 0;
volatile uint16_t 	U3TxOutCnt = 0;
volatile uint8_t 	U3RxBuf[U3RXBUFSIZE];
volatile uint16_t 	U3RxInCnt = 0;
volatile uint16_t 	U3RxoutCnt = 0;
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
    if (drvUartTxQueue(U3TxBuf, U3TXBUFSIZE, &U3TxInCnt, &U3TxOutCnt,
                       DataToSend, data_num, &DrvUartStats[3]) &&
        !(USART3->CR1 & USART_CR1_TXEIE))
    {
        USART_ITConfig(USART3, USART_IT_TXE, ENABLE); //打开发送中断
    }
}

void drvU3GetByte(uint8_t data)
{
    drvUartRxStore(U3RxBuf, U3RXBUFSIZE, &U3RxInCnt, &U3RxoutCnt,
                   data, &DrvUartStats[3]);
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
    drvUartIRQ(USART3, U3TxBuf, U3TXBUFSIZE, &U3TxInCnt, &U3TxOutCnt,
               U3RxBuf, U3RXBUFSIZE, &U3RxInCnt, &U3RxoutCnt, &DrvUartStats[3]);
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//====uart4
#define U4RXBUFSIZE		512
#define U4TXBUFSIZE		256
volatile uint8_t 	U4TxBuf[U4TXBUFSIZE];
volatile uint16_t 	U4TxInCnt = 0;
volatile uint16_t 	U4TxOutCnt = 0;
volatile uint8_t 	U4RxBuf[U4RXBUFSIZE];
volatile uint16_t 	U4RxInCnt = 0;
volatile uint16_t 	U4RxoutCnt = 0;
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
    if (drvUartTxQueue(U4TxBuf, U4TXBUFSIZE, &U4TxInCnt, &U4TxOutCnt,
                       DataToSend, data_num, &DrvUartStats[4]) &&
        !(UART4->CR1 & USART_CR1_TXEIE))
    {
        USART_ITConfig(UART4, USART_IT_TXE, ENABLE); //打开发送中断
    }
}

void drvU4GetByte(uint8_t data)
{
    drvUartRxStore(U4RxBuf, U4RXBUFSIZE, &U4RxInCnt, &U4RxoutCnt,
                   data, &DrvUartStats[4]);
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
    drvUartIRQ(UART4, U4TxBuf, U4TXBUFSIZE, &U4TxInCnt, &U4TxOutCnt,
               U4RxBuf, U4RXBUFSIZE, &U4RxInCnt, &U4RxoutCnt, &DrvUartStats[4]);
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//====uart5
#define U5RXBUFSIZE		256
#define U5TXBUFSIZE		256
volatile uint8_t 	U5TxBuf[U5TXBUFSIZE];
volatile uint16_t 	U5TxInCnt = 0;
volatile uint16_t 	U5TxOutCnt = 0;
volatile uint8_t 	U5RxBuf[U5RXBUFSIZE];
volatile uint16_t 	U5RxInCnt = 0;
volatile uint16_t 	U5RxoutCnt = 0;
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
    if (drvUartTxQueue(U5TxBuf, U5TXBUFSIZE, &U5TxInCnt, &U5TxOutCnt,
                       DataToSend, data_num, &DrvUartStats[5]) &&
        !(UART5->CR1 & USART_CR1_TXEIE))
    {
        USART_ITConfig(UART5, USART_IT_TXE, ENABLE); //打开发送中断
    }
}

void drvU5GetByte(uint8_t data)
{
    drvUartRxStore(U5RxBuf, U5RXBUFSIZE, &U5RxInCnt, &U5RxoutCnt,
                   data, &DrvUartStats[5]);
}
void drvU5DataCheck(void)
{
    while(U5RxInCnt!=U5RxoutCnt)
    {
        U5GetOneByte(LT_U5, U5RxBuf[U5RxoutCnt++]);
        if(U5RxoutCnt >= U5RXBUFSIZE)
            U5RxoutCnt = 0;
    }
}
void Uart5_IRQ(void)
{
    drvUartIRQ(UART5, U5TxBuf, U5TXBUFSIZE, &U5TxInCnt, &U5TxOutCnt,
               U5RxBuf, U5RXBUFSIZE, &U5RxInCnt, &U5RxoutCnt, &DrvUartStats[5]);
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//====uart7
#define U7RXBUFSIZE		256
#define U7TXBUFSIZE		256
volatile uint8_t 	U7TxBuf[U7TXBUFSIZE];
volatile uint16_t 	U7TxInCnt = 0;
volatile uint16_t 	U7TxOutCnt = 0;
volatile uint8_t 	U7RxBuf[U7RXBUFSIZE];
volatile uint16_t 	U7RxInCnt = 0;
volatile uint16_t 	U7RxoutCnt = 0;
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
    if (drvUartTxQueue(U7TxBuf, U7TXBUFSIZE, &U7TxInCnt, &U7TxOutCnt,
                       DataToSend, data_num, &DrvUartStats[7]) &&
        !(UART7->CR1 & USART_CR1_TXEIE))
    {
        USART_ITConfig(UART7, USART_IT_TXE, ENABLE); //打开发送中断
    }
}

void drvU7GetByte(uint8_t data)
{
    drvUartRxStore(U7RxBuf, U7RXBUFSIZE, &U7RxInCnt, &U7RxoutCnt,
                   data, &DrvUartStats[7]);
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
    drvUartIRQ(UART7, U7TxBuf, U7TXBUFSIZE, &U7TxInCnt, &U7TxOutCnt,
               U7RxBuf, U7RXBUFSIZE, &U7RxInCnt, &U7RxoutCnt, &DrvUartStats[7]);
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//====uart8
#define U8RXBUFSIZE		256
#define U8TXBUFSIZE		2048
volatile uint8_t 	U8TxBuf[U8TXBUFSIZE];
volatile uint16_t 	U8TxInCnt = 0;
volatile uint16_t 	U8TxOutCnt = 0;
volatile uint8_t 	U8RxBuf[U8RXBUFSIZE];
volatile uint16_t 	U8RxInCnt = 0;
volatile uint16_t 	U8RxoutCnt = 0;
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
    if (drvUartTxQueue(U8TxBuf, U8TXBUFSIZE, &U8TxInCnt, &U8TxOutCnt,
                       DataToSend, data_num, &DrvUartStats[8]) &&
        !(UART8->CR1 & USART_CR1_TXEIE))
    {
        USART_ITConfig(UART8, USART_IT_TXE, ENABLE); //打开发送中断
    }
}

void drvU8GetByte(uint8_t data)
{
    drvUartRxStore(U8RxBuf, U8RXBUFSIZE, &U8RxInCnt, &U8RxoutCnt,
                   data, &DrvUartStats[8]);
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
    drvUartIRQ(UART8, U8TxBuf, U8TXBUFSIZE, &U8TxInCnt, &U8TxOutCnt,
               U8RxBuf, U8RXBUFSIZE, &U8RxInCnt, &U8RxoutCnt, &DrvUartStats[8]);
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

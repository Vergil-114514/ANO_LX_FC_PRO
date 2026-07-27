#ifndef _USART_H
#define _USART_H

#include "SysConfig.h"

void DrvUart1Init(uint32_t br_num);
void Usart1_IRQ(void);
void DrvUart1SendBuf(const uint8_t *DataToSend, const uint8_t data_num);

void DrvUart2Init(uint32_t br_num);
void Usart2_IRQ(void);
void DrvUart2SendBuf(const uint8_t *DataToSend, const uint8_t data_num);

void DrvUart3Init(uint32_t br_num);
void Usart3_IRQ(void);
void DrvUart3SendBuf(const uint8_t *DataToSend, const uint8_t data_num);

void DrvUart4Init(uint32_t br_num);
void Uart4_IRQ(void);
void DrvUart4SendBuf(const uint8_t *DataToSend, const uint8_t data_num);

void DrvUart5Init(uint32_t br_num);
void Uart5_IRQ(void);
void DrvUart5SendBuf(const uint8_t *DataToSend, const uint8_t data_num);

void DrvUart7Init(uint32_t br_num);
void Uart7_IRQ(void);
void DrvUart7SendBuf(const uint8_t *DataToSend, const uint8_t data_num);

void DrvUart8Init(uint32_t br_num);
void Uart8_IRQ(void);
void DrvUart8SendBuf(const uint8_t *DataToSend, const uint8_t data_num);

void DrvUartDataCheck(void);
#endif

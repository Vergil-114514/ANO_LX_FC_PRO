#ifndef _DRV_UART_H_
#define _DRV_UART_H_
#include "sysconfig.h"

#define UartSendLXIMU	DrvUart5SendBuf

void DrvUart1Init(uint32_t baudrate);
void DrvUart1SendBuf(const uint8_t *data, const uint8_t len);
void DrvUart2Init(uint32_t baudrate);
void DrvUart2SendBuf(const uint8_t *data, const uint8_t len);
void DrvUart3Init(uint32_t baudrate);
void DrvUart3SendBuf(const uint8_t *data, const uint8_t len);
void DrvUart4Init(uint32_t baudrate);
void DrvUart4SendBuf(const uint8_t *data, const uint8_t len);
void DrvUart5Init(uint32_t baudrate);
void DrvUart5SendBuf(const uint8_t *data, const uint8_t len);

void DrvUartDataCheck(void);
#endif

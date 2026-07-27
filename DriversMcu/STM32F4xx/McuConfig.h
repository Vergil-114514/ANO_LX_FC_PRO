#ifndef _MCUCONFIG_H_
#define _MCUCONFIG_H_
#include "stm32f4xx.h"

//=======================================
/***************中断优先级******************/
#define NVIC_GROUP NVIC_PriorityGroup_3 //中断分组选择

#define NVIC_PWMIN_P 2 //接收机采集中断配置
#define NVIC_PWMIN_S 1

#define NVIC_TIME_P 7 //定时器中断配置
#define NVIC_TIME_S 1

#define NVIC_UART6_P 4 //串口6中断配置
#define NVIC_UART6_S 1

#define NVIC_UART8_P 4 //串口6中断配置
#define NVIC_UART8_S 1

#define NVIC_UART7_P 4 //串口6中断配置
#define NVIC_UART7_S 1

#define NVIC_UART5_P 1 //串口5中断配置
#define NVIC_UART5_S 0

#define NVIC_UART4_P 3 //串口4中断配置
#define NVIC_UART4_S 1

#define NVIC_UART3_P 4 //串口3中断配置
#define NVIC_UART3_S 1

#define NVIC_UART2_P 4 //串口2中断配置
#define NVIC_UART2_S 1

#define NVIC_UART1_P 3 //串口1中断配置 //gps
#define NVIC_UART1_S 0
/***********************************************/
//=========================================
#define UART1_RCC		RCC_APB2Periph_USART1
#define UART1_PORT_RCC	RCC_AHB1Periph_GPIOA
#define UART1_PORT		GPIOA
#define UART1_PIN_TX	GPIO_Pin_9
#define UART1_PIN_RX	GPIO_Pin_10
#define UART2_RCC		RCC_APB1Periph_USART2
#define UART2_PORT_RCC	RCC_AHB1Periph_GPIOD
#define UART2_PORT		GPIOD
#define UART2_PIN_TX	GPIO_Pin_5
#define UART2_PIN_RX	GPIO_Pin_6
#define UART3_RCC		RCC_APB1Periph_USART3
#define UART3_PORT_RCC	RCC_AHB1Periph_GPIOD
#define UART3_PORT		GPIOD
#define UART3_PIN_TX	GPIO_Pin_8
#define UART3_PIN_RX	GPIO_Pin_9
#define UART4_RCC		RCC_APB1Periph_UART4
#define UART4_PORT_RCC	RCC_AHB1Periph_GPIOA
#define UART4_PORT		GPIOA
#define UART4_PIN_TX	GPIO_Pin_0
#define UART4_PIN_RX	GPIO_Pin_1
#define UART5_RCC		RCC_APB1Periph_UART5
#define UART5_PORT1_RCC	RCC_AHB1Periph_GPIOC
#define UART5_PORT2_RCC	RCC_AHB1Periph_GPIOD
#define UART5_PORT1		GPIOC
#define UART5_PORT2		GPIOD
#define UART5_PIN_TX	GPIO_Pin_12
#define UART5_PIN_RX	GPIO_Pin_2
#define UART7_RCC		RCC_APB1Periph_UART7
#define UART7_PORT_RCC	RCC_AHB1Periph_GPIOE
#define UART7_PORT		GPIOE
#define UART7_PIN_TX	GPIO_Pin_8
#define UART7_PIN_RX	GPIO_Pin_7
#define UART8_RCC		RCC_APB1Periph_UART8
#define UART8_PORT_RCC	RCC_AHB1Periph_GPIOE
#define UART8_PORT		GPIOE
#define UART8_PIN_TX	GPIO_Pin_1
#define UART8_PIN_RX	GPIO_Pin_0

#define UartSendLXIMU 	DrvUart5SendBuf
#define UartSendRP		DrvUart2SendBuf
#endif


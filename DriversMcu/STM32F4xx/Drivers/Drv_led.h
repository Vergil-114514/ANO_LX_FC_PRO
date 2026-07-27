#ifndef _LED_H_
#define _LED_H_

#include "SysConfig.h"

#define DvrLedObOff         GPIO_ResetBits(GPIOE, GPIO_Pin_15)
#define DvrLedObOn          GPIO_SetBits(GPIOE, GPIO_Pin_15)

/***************LED GPIO定义******************/
#define ANO_RCC_LEDOB		RCC_AHB1Periph_GPIOE
#define ANO_GPIO_LEDOB		GPIOE
#define ANO_Pin_LEDOB		GPIO_Pin_15

void DvrLedInit(void);

#endif

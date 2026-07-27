/******************** (C) COPYRIGHT 2017 ANO Tech ********************************
 * 浣滆€�    锛氬尶鍚嶇鍒�
 * 瀹樼綉    锛歸ww.anotc.com
 * 娣樺疂    锛歛notc.taobao.com
 * 鎶€鏈疩缇� 锛�190169595
 * 鎻忚堪    锛歀ED椹卞姩
**********************************************************************************/
#include "Drv_led.h"

void DvrLedInit()
{
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_StructInit(&GPIO_InitStructure);

    RCC_AHB1PeriphClockCmd(ANO_RCC_LEDOB,ENABLE);

    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Pin   = ANO_Pin_LEDOB;
    GPIO_Init(ANO_GPIO_LEDOB, &GPIO_InitStructure);

    DvrLedObOff;
}

/******************* (C) COPYRIGHT 2016 ANO TECH *****END OF FILE************/

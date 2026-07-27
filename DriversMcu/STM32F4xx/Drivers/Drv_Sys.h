#ifndef _DRV_SYS_H_
#define _DRV_SYS_H_
#include "sysconfig.h"

void DrvSysInit(void);

uint32_t GetSysRunTimeMs(void);
uint32_t GetSysRunTimeUs(void);
void MyDelayMs(uint32_t time);

void UsbPortCtlInit(void);
void UsbPortCtl(const uint8_t sel);

#endif


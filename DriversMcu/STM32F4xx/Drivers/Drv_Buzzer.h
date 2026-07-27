#ifndef _DRVBUZZER_H_
#define _DRVBUZZER_H_
#include "SysConfig.h"


void DrvBuzzerInit(void);
void DrvBuzzerCtl(const uint8_t ena);

void DrvBuzzerRunTask(const float dT);
void DrvBuzzerAdvCtl1(const uint8_t tims, const uint16_t ontim_ms, const uint16_t offtim_ms);
void DrvBuzzerAdvCtl2(const uint16_t ontim1_ms, const uint16_t offtim1_ms, const uint16_t ontim2_ms, const uint16_t offtim2_ms, const uint16_t ontim3_ms, const uint16_t offtim3_ms);
void DrvBuzzerAdvCtlOK(void);
void DrvBuzzerAdvCtlERR(void);
#endif

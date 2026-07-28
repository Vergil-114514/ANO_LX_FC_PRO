#ifndef __DRV_RC_INPUT_H
#define __DRV_RC_INPUT_H

#include "SysConfig.h"
#define LORA3A22_FRAME_LEN         18U
#define LORA3A22_FRAME_HEAD        0xA3U
#define LORA3A22_LINK_TIMEOUT_MS   500U

/* Change only these signs after an unpropellered bench direction check. */
#ifndef LORA3A22_ROLL_SIGN
#define LORA3A22_ROLL_SIGN         1
#endif
#ifndef LORA3A22_PITCH_SIGN
#define LORA3A22_PITCH_SIGN        1
#endif
#ifndef LORA3A22_THROTTLE_SIGN
#define LORA3A22_THROTTLE_SIGN     1
#endif
#ifndef LORA3A22_YAW_SIGN
#define LORA3A22_YAW_SIGN          1
#endif

void DrvRcPpmInit(void);
void DrvRcSbusInit(void);
void DrvRcLoraInit(void);
void PPM_IRQH(void);
void Sbus_IRQH(void);
void DrvRcLoraRxOneByte(const uint8_t linktype, const uint8_t dat);
uint8_t DrvRcLoraLinkAlive(float dT_s);
#endif

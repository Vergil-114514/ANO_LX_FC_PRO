#ifndef __DRV_PAYLOAD_SERVO_H
#define __DRV_PAYLOAD_SERVO_H

#include "SysConfig.h"

#define PAYLOAD_SERVO_CLAMP_US 2166U
#define PAYLOAD_SERVO_RELEASE_US 1500U
#define PAYLOAD_SERVO_MIN_US 500U
#define PAYLOAD_SERVO_MAX_US 2500U

void DrvPayloadServoInit(void);
void DrvPayloadServoClamp(void);
void DrvPayloadServoRelease(void);
void DrvPayloadServoSetPulseUs(uint16_t pulseUs);
uint8_t DrvPayloadServoIsReleased(void);
void DrvPayloadServoIrqHandler(void);

#endif

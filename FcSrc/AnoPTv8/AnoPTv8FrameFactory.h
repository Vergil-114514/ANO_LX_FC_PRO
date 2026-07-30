#ifndef __ANOPTV8FRAMEFACTORY_H
#define __ANOPTV8FRAMEFACTORY_H
#include "AnoPTv8.h"
void AnoPTv8CalFrameCheck(_un_frame_v8 *p);

uint8_t AnoPTv8SendBuf(const uint16_t linktype, const uint8_t daddr, const uint8_t fid, const uint8_t priority, const uint8_t *buf, const uint16_t len);
uint8_t AnoPTv8SendBufFrom(const uint16_t linktype, const uint8_t saddr, const uint8_t daddr, const uint8_t fid, const uint8_t priority, const uint8_t *buf, const uint16_t len);

void AnoPTv8SendCheck(const uint16_t linktype, const uint8_t daddr, const uint8_t id, const uint8_t sc, const uint8_t ac);
void AnoPTv8SendDevInfo(const uint16_t linktype, const uint8_t daddr);
void AnoPTv8SendStr(const uint16_t linktype, const uint8_t daddr, const uint8_t string_color, const char *str);
void AnoPTv8SendValStr(const uint16_t linktype, const uint8_t daddr, const float val, const char *str);

void AnoPTv8SendParNum(const uint16_t linktype, const uint8_t daddr);
void AnoPTv8SendParVal(const uint16_t linktype, const uint8_t daddr, const uint16_t parid);
void AnoPTv8SendParInfo(const uint16_t linktype, const uint8_t daddr, const uint16_t parid);

void AnoPTv8SendCmdNum(const uint16_t linktype, const uint8_t daddr);
void AnoPTv8SendCmdInfo(const uint16_t linktype, const uint8_t daddr, const uint16_t cmd);


void AnoPTv8SendFrame0x0D(const uint16_t linktype, const uint8_t daddr, const float vbat, const float ibat, const float vfc, const float ifc);
#endif

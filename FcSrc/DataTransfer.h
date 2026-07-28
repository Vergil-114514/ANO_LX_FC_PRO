#ifndef __ANO_DT_LX_H
#define __ANO_DT_LX_H
//==引用
#include "SysConfig.h"
#include "AnoPTv8.h"

//==定义/声明
#define FUN_NUM_LEN 256

typedef struct
{
    uint8_t		fId;
    uint16_t 	cycleTime;
} _st_autoSendInfo;
typedef struct
{
    uint16_t 	timeCnt;
    uint8_t 	readyToSend;
} _st_autoSendSta;

void AnoDTLxRunTask1Ms(float dT_s);
void AnoDTLxFrameAnl(const uint8_t linktype, const _un_frame_v8 *p);
void AnoDTMotorTestFrameAnl(const uint8_t linktype, const _un_frame_v8 *p);
void AnoDTLxFrameSend(const uint8_t fid);
void AnoDTLxFrameSendTrigger(const uint8_t fid);
#endif

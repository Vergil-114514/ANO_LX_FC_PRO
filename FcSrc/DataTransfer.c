#include "DataTransfer.h"
#include "AnoPTv8ExAPI.h"
#include "LX_LowLevelFunc.h"
#include "Drv_RcIn.h"
#include "LX_ExtSensor.h"
#include "Drv_led.h"
#include "LX_FcState.h"
#include "Drv_Uart.h"
#include "AnoPTv8.h"
#include "AnoPTv8Run.h"
/*==========================================================================
 * 描述    ：凌霄飞控通信主程序
 * 更新时间：2024-12-25
 * 作者		 ：匿名科创-茶不思
 * 官网    ：www.anotc.com
 * 淘宝    ：anotc.taobao.com
 * 技术Q群 ：190169595
 * 项目合作：18084888982，18061373080
============================================================================
 * 匿名科创团队感谢大家的支持，欢迎大家进群互相交流、讨论、学习。
 * 若您觉得匿名有不好的地方，欢迎您拍砖提意见。
 * 若您觉得匿名好，请多多帮我们推荐，支持我们。
 * 匿名开源程序代码欢迎您的引用、延伸和拓展，不过在希望您在使用时能注明出处。
 * 君子坦荡荡，小人常戚戚，匿名坚决不会请水军、请喷子，也从未有过抹黑同行的行为。
 * 开源不易，生活更不容易，希望大家互相尊重、互帮互助，共同进步。
 * 只有您的支持，匿名才能做得更好。
===========================================================================*/
const _st_autoSendInfo ASFInfo[] =
{
    {0x40, 20},
    {0x30, 0},
    {0x33, 0},
    {0x34, 0},
    {0x41, 0},
    {0xE0, 0},
};
const uint16_t ASFCNT = sizeof(ASFInfo) / sizeof(_st_autoSendInfo);
_st_autoSendSta ASFSta[ASFCNT];

//===================================================================
void AutoSendFrameCheck(void)
{
    for(int i=0; i<ASFCNT; i++)
    {
        if(ASFInfo[i].cycleTime > 0)
        {
            ASFSta[i].timeCnt++;
            if(ASFSta[i].timeCnt >= ASFInfo[i].cycleTime)
            {
                ASFSta[i].timeCnt = 0;
                ASFSta[i].readyToSend = 1;
            }
        }
        if(ASFSta[i].readyToSend)
        {
			ASFSta[i].readyToSend = 0;
            AnoDTLxFrameSend(ASFInfo[i].fId);
            return;
        }
    }
}

void AnoDTLxRunTask1Ms(float dT_s)
{
    AutoSendFrameCheck();
}

void AnoDTLxFrameAnl(const uint8_t linktype, const _un_frame_v8 *p)
{
    uint8_t _fid = p->frame.frameid;
    switch(_fid)
    {
    case 0x00:
    {
        //校验数据
    }
    break;
    case 0x20:
    {
        //PWM数据
        pwm_to_esc.pwm_m1 = *((uint16_t  *)(p->frame.data));
        pwm_to_esc.pwm_m2 = *((uint16_t  *)(p->frame.data + 2));
        pwm_to_esc.pwm_m3 = *((uint16_t  *)(p->frame.data + 4));
        pwm_to_esc.pwm_m4 = *((uint16_t  *)(p->frame.data + 6));
        pwm_to_esc.pwm_m5 = *((uint16_t  *)(p->frame.data + 8));
        pwm_to_esc.pwm_m6 = *((uint16_t  *)(p->frame.data + 10));
        pwm_to_esc.pwm_m7 = *((uint16_t  *)(p->frame.data + 12));
        pwm_to_esc.pwm_m8 = *((uint16_t  *)(p->frame.data + 14));
    }
    break;
    case 0x0F:
    {
        //凌霄IMU发出的RGB灯光数据
    }
    break;
    case 0x06:
    {
        //凌霄飞控当前的运行状态
        fc_sta.fc_mode_sta = *(p->frame.data);
        fc_sta.unlock_sta = *(p->frame.data + 1);
        fc_sta.cmd_fun.CID = *(p->frame.data + 2);
        fc_sta.cmd_fun.CMD_0 = *(p->frame.data + 3);
        fc_sta.cmd_fun.CMD_1 = *(p->frame.data + 4);
    }
    break;
    case 0x07:
    {
        //飞行速度
        for(uint8_t i=0; i<6; i++)
        {
            fc_vel.byte_data[i] = *(p->frame.data + i);
        }
    }
    break;
    case 0x03:
    {
        //姿态角（需要在上位机凌霄IMU界面配置输出功能）
        for(uint8_t i=0; i<7; i++)
        {
            fc_att.byte_data[i] = *(p->frame.data + i);
        }
    }
    break;
    case 0x04:
    {
        //姿态四元数
        for(uint8_t i=0; i<9; i++)
        {
            fc_att_qua.byte_data[i] = *(p->frame.data + i);
        }
    }
    break;
    case 0x05:
    {
        //高度数据
        for(uint8_t i=0; i<9; i++)
        {
            fc_alt.byte_data[i] = *(p->frame.data + i);
        }
    }
    break;
    case 0x01:
    {
        //传感器数据
    }
    break;
    }
}
void AnoDTMotorTestFrameAnl(const uint8_t linktype, const _un_frame_v8 *p)
{
    uint16_t pulseUs;
    uint16_t durationMs;

    if (linktype != LT_U2 || p->frame.ddevid != ANOPTV8DEVID_MY || p->frame.frameid != 0xF1 || p->frame.datalen != 6U || p->frame.data[0] != 0xA5U)
    {
        return;
    }

    if (p->frame.data[1] == 0U && p->frame.data[2] == 0U && p->frame.data[3] == 0U && p->frame.data[4] == 0U && p->frame.data[5] == 0U)
    {
        LX_MotorTestStop();
        return;
    }

    pulseUs = (uint16_t)p->frame.data[2] | ((uint16_t)p->frame.data[3] << 8);
    durationMs = (uint16_t)p->frame.data[4] | ((uint16_t)p->frame.data[5] << 8);
    LX_MotorTestStart(p->frame.data[1], pulseUs, durationMs);
}

void AnoDTLxFrameSend(const uint8_t fid)
{
    switch (fid)
    {
    case 0x30: //GPS数据
    {
        uint8_t _sbuf[23];
        memcpy(_sbuf,ext_sens.fc_gps.byte,23);
        AnoPTv8SendBuf(LT_D_IMU, ANOPTV8DEVID_LXIMU, fid, ANOPTV8TXPRI_DATA, _sbuf, sizeof(_sbuf));
    }
    break;
    case 0x33: //通用速度测量数据
    {
        uint8_t _sbuf[6];
        memcpy(_sbuf,ext_sens.gen_vel.byte,6);
        AnoPTv8SendBuf(LT_D_IMU, ANOPTV8DEVID_LXIMU, fid, ANOPTV8TXPRI_DATA, _sbuf, sizeof(_sbuf));
    }
    break;
    case 0x34: //通用距离测量数据
    {
        uint8_t _sbuf[7];
        memcpy(_sbuf,ext_sens.gen_dis.byte,7);
        AnoPTv8SendBuf(LT_D_IMU, ANOPTV8DEVID_LXIMU, fid, ANOPTV8TXPRI_DATA, _sbuf, sizeof(_sbuf));
    }
    break;
    case 0x40: //遥控数据帧
    {
        uint8_t _sbuf[20];
        memcpy(_sbuf,rc_in.rc_ch.byte_data,20);
        AnoPTv8SendBuf(LT_D_IMU, ANOPTV8DEVID_LXIMU, fid, ANOPTV8TXPRI_DATA, _sbuf, sizeof(_sbuf));
    }
    break;
    case 0x41: //实时控制数据帧
    {
        uint8_t _sbuf[14];
        memcpy(_sbuf,rt_tar.byte_data,14);
        AnoPTv8SendBuf(LT_D_IMU, ANOPTV8DEVID_LXIMU, fid, ANOPTV8TXPRI_DATA, _sbuf, sizeof(_sbuf));
    }
    break;
    default:
        break;
    }
}

void AnoDTLxFrameSendTrigger(const uint8_t fid)
{
    for(int i=0; i<ASFCNT; i++)
    {
        if(fid == ASFInfo[i].fId)
        {
            ASFSta[i].readyToSend = 1;
        }
    }
}

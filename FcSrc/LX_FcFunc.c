#include "LX_FcFunc.h"
#include "LX_FcState.h"
#include "DataTransfer.h"

/*==========================================================================
 * 描述    ：凌霄飞控基本功能主程序
 * 更新时间：2020-03-31
 * 作者		 ：匿名科创-Jyoun
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

//////////////////////////////////////////////////////////////////////
//基本功能函数
//////////////////////////////////////////////////////////////////////
//
uint8_t FC_Unlock()
{
    //
    fc_sta.unlock_cmd = 1; //解锁
    //按协议发送指令
    if (AnoPTv8CmdSendIsInIdle()) //没有其他等待校验的CMD时才发送本CMD
    {
        uint8_t _sbuf[3] = {0x10, 0x00, 0x01};
        return AnoPTv8CmdSend(LT_D_IMU, ANOPTV8DEVID_LXIMU, _sbuf, sizeof(_sbuf));
    }
    else
    {
        return 0;
    }
}
//
uint8_t FC_Lock()
{
    //
    fc_sta.unlock_cmd = 0; //上锁
    //按协议发送指令
    if (AnoPTv8CmdSendIsInIdle()) //没有其他等待校验的CMD时才发送本CMD
    {
        uint8_t _sbuf[3] = {0x10, 0x00, 0x02};
        return AnoPTv8CmdSend(LT_D_IMU, ANOPTV8DEVID_LXIMU, _sbuf, sizeof(_sbuf));
    }
    else
    {
        return 0;
    }
}

//改变飞控模式(模式0-3)
uint8_t LX_Change_Mode(uint8_t new_mode)
{
    static uint8_t old_mode;
    if (old_mode != new_mode)
    {
        //
        if (AnoPTv8CmdSendIsInIdle()) //没有其他等待校验的CMD时才发送本CMD
        {
            old_mode = fc_sta.fc_mode_cmd = new_mode;
            //按协议发送指令
            uint8_t _sbuf[4] = {0x01, 0x01, 0x01, fc_sta.fc_mode_cmd};
            return AnoPTv8CmdSend(LT_D_IMU, ANOPTV8DEVID_LXIMU, _sbuf, sizeof(_sbuf));
        }
        else
        {
            return 0;
        }
    }
    else //已经在当前模式
    {
        return 1;
    }
}

//一键返航
//需要注意，程控模式下才能执行返航
uint8_t OneKey_Return_Home()
{
    //
    if (AnoPTv8CmdSendIsInIdle()) //没有其他等待校验的CMD时才发送本CMD
    {
        //按协议发送指令
        uint8_t _sbuf[3] = {0x10, 0x00, 0x07};
        return AnoPTv8CmdSend(LT_D_IMU, ANOPTV8DEVID_LXIMU, _sbuf, sizeof(_sbuf));
    }
    else
    {
        return 0;
    }
}

//一键起飞(高度cm)
uint8_t OneKey_Takeoff(uint16_t  height_cm)
{
    //
    if (AnoPTv8CmdSendIsInIdle()) //没有其他等待校验的CMD时才发送本CMD
    {
        //按协议发送指令
        uint8_t _sbuf[5] = {0x10, 0x00, 0x05, 0, 0};
        _sbuf[3] = BYTE0(height_cm);
        _sbuf[4] = BYTE1(height_cm);
        return AnoPTv8CmdSend(LT_D_IMU, ANOPTV8DEVID_LXIMU, _sbuf, sizeof(_sbuf));
    }
    else
    {
        return 0;
    }
}
//一键降落
uint8_t OneKey_Land()
{
    //
    if (AnoPTv8CmdSendIsInIdle()) //没有其他等待校验的CMD时才发送本CMD
    {
        //按协议发送指令
        uint8_t _sbuf[3] = {0x10, 0x00, 0x06};
        return AnoPTv8CmdSend(LT_D_IMU, ANOPTV8DEVID_LXIMU, _sbuf, sizeof(_sbuf));
    }
    else
    {
        return 0;
    }
}
//平移(距离cm，速度cmps，方向角度0-360度)
uint8_t Horizontal_Move(uint16_t  distance_cm, uint16_t  velocity_cmps, uint16_t  dir_angle_0_360)
{
    //
    if (AnoPTv8CmdSendIsInIdle()) //没有其他等待校验的CMD时才发送本CMD
    {
        //按协议发送指令
        uint8_t _sbuf[9];
        _sbuf[0] = 0X10;
        _sbuf[1] = 0X02;
        _sbuf[2] = 0X03;
        _sbuf[3] = BYTE0(distance_cm);
        _sbuf[4] = BYTE1(distance_cm);
        _sbuf[5] = BYTE0(velocity_cmps);
        _sbuf[6] = BYTE1(velocity_cmps);
        _sbuf[7] = BYTE0(dir_angle_0_360);
        _sbuf[8] = BYTE1(dir_angle_0_360);
        return AnoPTv8CmdSend(LT_D_IMU, ANOPTV8DEVID_LXIMU, _sbuf, sizeof(_sbuf));
    }
    else
    {
        return 0;
    }
}

//水平校准
uint8_t Horizontal_Calibrate()
{
    //
    if (AnoPTv8CmdSendIsInIdle()) //没有其他等待校验的CMD时才发送本CMD
    {
        //按协议发送指令
        uint8_t _sbuf[3] = {0x01, 0x00, 0x03};
        return AnoPTv8CmdSend(LT_D_IMU, ANOPTV8DEVID_LXIMU, _sbuf, sizeof(_sbuf));
    }
    else
    {
        return 0;
    }
}

//磁力计校准
uint8_t Mag_Calibrate()
{
    //
    if (AnoPTv8CmdSendIsInIdle()) //没有其他等待校验的CMD时才发送本CMD
    {
        //按协议发送指令
        uint8_t _sbuf[3] = {0x01, 0x00, 0x04};
        return AnoPTv8CmdSend(LT_D_IMU, ANOPTV8DEVID_LXIMU, _sbuf, sizeof(_sbuf));
    }
    else
    {
        return 0;
    }
}

//6面加速度校准
uint8_t ACC_Calibrate()
{
    //
    if (AnoPTv8CmdSendIsInIdle()) //没有其他等待校验的CMD时才发送本CMD
    {
        //按协议发送指令
        uint8_t _sbuf[3] = {0x01, 0x00, 0x05};
        return AnoPTv8CmdSend(LT_D_IMU, ANOPTV8DEVID_LXIMU, _sbuf, sizeof(_sbuf));
    }
    else
    {
        return 0;
    }
}

//陀螺仪校准
uint8_t GYR_Calibrate()
{
    //
    if (AnoPTv8CmdSendIsInIdle()) //没有其他等待校验的CMD时才发送本CMD
    {
        //按协议发送指令
        uint8_t _sbuf[3] = {0x01, 0x00, 0x02};
        return AnoPTv8CmdSend(LT_D_IMU, ANOPTV8DEVID_LXIMU, _sbuf, sizeof(_sbuf));
    }
    else
    {
        return 0;
    }
}

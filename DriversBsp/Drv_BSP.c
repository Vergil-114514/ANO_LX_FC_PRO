/******************** (C) COPYRIGHT 2017 ANO Tech ********************************
 * 作者    ：匿名科创
 * 官网    ：www.anotc.com
 * 淘宝    ：anotc.taobao.com
 * 技术Q群 ：190169595
 * 描述    ：飞控初始化
**********************************************************************************/
#include "Drv_BSP.h"
#include "Drv_PwmOut.h"
#include "Drv_Dshot600.h"
#include "Drv_led.h"
#include "Drv_RcIn.h"
#include "Drv_Timer.h"
#include "DataTransfer.h"
#include "Drv_UbloxGPS.h"
#include "Drv_UwbMini5.h"
#include "Drv_Uart.h"
#include "Drv_Timer.h"
#include "Drv_Usb.h"
#include "Drv_Adc.h"
#include "Drv_Buzzer.h"
#include "DrvIna228.h"

uint8_t EmergencyStopESC = 0;

uint8_t All_Init()
{
    DrvSysInit();
    //延时
    MyDelayMs(100);
    //USB插座选择
    UsbPortCtlInit();
    UsbPortCtl(0);
    DrvUsbInit();
    //LED功能初始化
    DvrLedInit();
    //初始化电调输出功能
	if(ESC_TYPE == 1)
		DrvDshot600Init();
	else 
		DrvPwmOutInit();
    MyDelayMs(100);
    //
    AnoPTv8ParInit();
    AnoPTv8CmdInit();
    //串口1初始化，函数参数为波特率
    DrvUart1Init(1000000);
    //串口2初始化，函数参数为波特率
    DrvUart2Init(500000);
    //串口3初始化
    DrvUart3Init(500000);
    //串口4初始化
#if NAV_INPUT_UWB_MINI5
    DrvUart4Init(115200);
#endif
    //串口5
    DrvUart5Init(500000);
    //串口7
    DrvUart7Init(115200);
    //串口8
    DrvUart8Init(500000);
    MyDelayMs(100);
    //SBUS输入采集初始化
    DrvRcInputInit();
    MyDelayMs(100);
    //UART4导航接口初始化
#if NAV_INPUT_UWB_MINI5
    DrvUwbMini5Init();
#elif NAV_INPUT_GPS
    Init_GPS();
#endif
    //初始化定时中断
    DrvTimerFcInit();
	//
	DrvAdcInit();
	//
	DrvBuzzerInit();
	DrvBuzzerAdvCtl1(2, 100, 50);
	DrvIna228Init();
    //初始化完成，返回1
    return (1);
}

_rc_input_st rc_in;
_rc_sbus_un sbus_in;

void DrvRcInputInit(void)
{
    //任意初始化一个模式
    DrvRcPpmInit();
    DrvRcLoraInit();
    //DrvRcSbusInit();
    //先标记位丢失
    rc_in.no_signal = 1;
    rc_in.fail_safe = 1;
}

void DrvPpmGetOneCh(uint16_t  data)
{
    static uint8_t ch_sta = 0;

    if ((data > 2500 && ch_sta > 3) || ch_sta == 10)
    {
        ch_sta = 0;
        rc_in.signal_cnt_tmp++;
        rc_in.rc_in_mode_tmp = 1; //切换模式标记为ppm
    }
    else if (data > 300 && data < 3000) //异常的脉冲过滤掉
    {
        //
        rc_in.ppm_ch[ch_sta] = data;
        ch_sta++;
    }
}

void DrvSbusGetOneByte(uint8_t data)
{
    /*
    sbus flags的结构如下所示：
    flags：
    bit7 = ch17 = digital channel (0x80)
    bit6 = ch18 = digital channel (0x40)
    bit5 = Frame lost, equivalent red LED on receiver (0x20)
    bit4 = failsafe activated (0x10) b: 0001 0000
    bit3 = n/a
    bit2 = n/a
    bit1 = n/a
    bit0 = n/a
    */
    const uint8_t frame_end[4] = {0x04, 0x14, 0x24, 0x34};
    static uint32_t sbus_time[2];
    static uint8_t datatmp[25];
    static uint8_t cnt = 0;
    static uint8_t frame_cnt;
    //
    sbus_time[0] = sbus_time[1];
    sbus_time[1] = GetSysRunTimeUs();

    if ((u32)(sbus_time[1] - sbus_time[0]) > 2500)
    {
        cnt = 0;
    }

    //
    if (cnt >= 25)
    {
        cnt = 0;
    }

    datatmp[cnt++] = data;

    //
    if (cnt == 25)
    {
        cnt = 24;

        if ((datatmp[0] == 0x0F && (datatmp[24] == 0x00 || datatmp[24] == frame_end[frame_cnt])))
        {
            cnt = 0;
            memcpy(sbus_in.rawdata, datatmp, 25);
            rc_in.sbus_ch[0] = sbus_in.stdata.ch0;
            rc_in.sbus_ch[1] = sbus_in.stdata.ch1;
            rc_in.sbus_ch[2] = sbus_in.stdata.ch2;
            rc_in.sbus_ch[3] = sbus_in.stdata.ch3;
            rc_in.sbus_ch[4] = sbus_in.stdata.ch4;
            rc_in.sbus_ch[5] = sbus_in.stdata.ch5;
            rc_in.sbus_ch[6] = sbus_in.stdata.ch6;
            rc_in.sbus_ch[7] = sbus_in.stdata.ch7;
            rc_in.sbus_ch[8] = sbus_in.stdata.ch8;
            rc_in.sbus_ch[9] = sbus_in.stdata.ch9;
            rc_in.sbus_ch[10] = sbus_in.stdata.ch10;
            rc_in.sbus_ch[11] = sbus_in.stdata.ch11;
            rc_in.sbus_ch[12] = sbus_in.stdata.ch12;
            rc_in.sbus_ch[13] = sbus_in.stdata.ch13;
            rc_in.sbus_ch[14] = sbus_in.stdata.ch14;
            rc_in.sbus_ch[15] = sbus_in.stdata.ch15;
            rc_in.sbus_flag = sbus_in.stdata.flag;

            //user
            //
            if (rc_in.sbus_flag & 0x08)
            {
                //如果有数据且能接收到有失控标记，则不处理，转嫁成无数据失控。
            }
            else
            {
                rc_in.signal_cnt_tmp++;
                rc_in.rc_in_mode_tmp = 2; //切换模式标记为sbus
            }

            //帧尾处理
            frame_cnt++;
            frame_cnt %= 4;
        }
        else
        {
            for (uint8_t i = 0; i < 24; i++)
            {
                datatmp[i] = datatmp[i + 1];
            }
        }
    }
}

static void rcSignalCheck(float *dT_s)
{
    //
    static uint8_t cnt_tmp;
    static uint16_t  time_dly;
    time_dly += ( *dT_s) * 1e3f;

    //==1000ms==
    if (time_dly > 1000)
    {
        time_dly = 0;
        //
        rc_in.signal_fre = rc_in.signal_cnt_tmp;

        //==判断信号是否丢失
        if (rc_in.signal_fre < 5)
        {
            rc_in.no_signal = 1;
        }
        else
        {
            rc_in.no_signal = 0;
        }

        //==判断是否切换输入方式
        if (rc_in.no_signal)
        {
            //初始0
            if (rc_in.sig_mode == 0)
            {
                cnt_tmp++;
                cnt_tmp %= 3;

                if (cnt_tmp == 1)
                {
                    DrvRcSbusInit();
                }
                else if (cnt_tmp == 2)
                {
                    DrvRcPpmInit();
                }
				else
				{
					DrvRcLoraInit();
				}
            }
        }
        else
        {
            rc_in.sig_mode = rc_in.rc_in_mode_tmp;
        }

        //==
        rc_in.signal_cnt_tmp = 0;
    }
}

#define RC_NO_CHECK 0  //0：监测遥控信号；1：不检测遥控信号
//
void DrvRcInputTask(float dT_s)
{
    static uint8_t failsafe;
    uint8_t lora_link_alive;
    //信号检测
    rcSignalCheck( &dT_s);
    lora_link_alive = DrvRcLoraLinkAlive(dT_s);

    //有信号
    if (rc_in.sig_mode == 3 && lora_link_alive == 0)
    {
        rc_in.no_signal = 1;
    }

    if (rc_in.no_signal == 0)
    {
        //ppm
        if (rc_in.sig_mode == 1)
        {
            for (uint8_t i = 0; i < 10; i++) //注意只有10个通道
            {
                rc_in.rc_ch.st_data.ch_[i] = rc_in.ppm_ch[i];
            }
        }
        //sbus
        else if (rc_in.sig_mode == 2)
        {
            for (uint8_t i = 0; i < 10; i++) //注意只有10个通道
            {
                rc_in.rc_ch.st_data.ch_[i] = 0.644f * (rc_in.sbus_ch[i] - 1024) + 1500; //248 --1024 --1800转换到1000-2000
            }
        }
		//lora
		else if (rc_in.sig_mode == 3)
        {
            for (uint8_t i = 0; i < 10; i++) //注意只有10个通道
            {
                rc_in.rc_ch.st_data.ch_[i] = rc_in.lora_ch[i];
            }
        }

        //检查失控保护设置
        if (
            (rc_in.rc_ch.st_data.ch_[RCCNANNELDEF_FLIGHTMODE] > 1200 && rc_in.rc_ch.st_data.ch_[RCCNANNELDEF_FLIGHTMODE] < 1400)
            || (rc_in.rc_ch.st_data.ch_[RCCNANNELDEF_FLIGHTMODE] > 1600 && rc_in.rc_ch.st_data.ch_[RCCNANNELDEF_FLIGHTMODE] < 1800))
        {
            //满足设置，标记为失控
            failsafe = 1;
        }
        else
        {
            failsafe = 0;
        }
    }
    //无信号
    else
    {
        //失控标记置位
        failsafe = 1;

        //
        for (uint8_t i = 0; i < 10; i++) //注意只有10个通道
        {
            rc_in.rc_ch.st_data.ch_[i] = 0; //
        }
    }

#if (RC_NO_CHECK == 0)
    //失控标记
    rc_in.fail_safe = failsafe;
#else

    //无信号或者检测到失控
    if (rc_in.no_signal != 0 || failsafe != 0)
    {
        for (uint8_t i = 0; i < 10; i++)
        {
            rc_in.rc_ch.st_data.ch_[i] = 1500;
        }
    }

    //不标记失控
    rc_in.fail_safe = 0;
#endif
}

/******************* (C) COPYRIGHT 2014 ANO TECH *****END OF FILE************/

#include "Drv_UbloxGPS.h"
#include "LX_ExtSensor.h"
#include "ANO_Math.h"
#include "DataTransfer.h"
/*==========================================================================
 * 描述    ：UBLOX_GPS数据解析
 * 更新时间：2018-11-08
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
#define UBX_BUF_NUM 500
uint8_t UbxRxBuf[UBX_BUF_NUM];
_UBXPVT_st *ubxPvtData;

///////////////////////////////////////////////////////////////
//匿名协议赋值
///////////////////////////////////////////////////////////////
uint8_t pvt_receive_updata;
uint8_t sat_receive_updata;
void GPS_Data_Prepare_Task(uint8_t dT_ms)
{
    if (pvt_receive_updata)
    {
        pvt_receive_updata = 0;
        //赋值
        ubxPvtData = (_UBXPVT_st *)(UbxRxBuf + 6);
        ext_sens.fc_gps.st_data.FIX_STA = ubxPvtData->fixType;
        ext_sens.fc_gps.st_data.S_NUM = ubxPvtData->numSV;
        ext_sens.fc_gps.st_data.LNG = ubxPvtData->lon;
        ext_sens.fc_gps.st_data.LAT = ubxPvtData->lat;
        ext_sens.fc_gps.st_data.ALT_GPS = ubxPvtData->hMSL / 10; //mm->cm
        ext_sens.fc_gps.st_data.N_SPE = ubxPvtData->velN / 10;	  //mm->cm
        ext_sens.fc_gps.st_data.E_SPE = ubxPvtData->velE / 10;	  //mm->cm
        ext_sens.fc_gps.st_data.D_SPE = ubxPvtData->velD / 10;	  //mm->cm
        //按协议处理赋值
        uint32_t tmp;
        tmp = ubxPvtData->pDOP * 0.01f;
        tmp = LIMIT(tmp, 0, 200);
        ext_sens.fc_gps.st_data.PDOP_001 = tmp;
        tmp = ubxPvtData->sAcc * 0.01f;
        tmp = LIMIT(tmp, 0, 200);
        ext_sens.fc_gps.st_data.SACC_001 = tmp;
        tmp = ubxPvtData->vAcc * 0.01f;
        tmp = LIMIT(tmp, 0, 200);
        ext_sens.fc_gps.st_data.VACC_001 = tmp;
        //标记置位，触发数据发送
        AnoDTLxFrameSendTrigger(0x30);
    }

    if (sat_receive_updata)
    {
        sat_receive_updata = 0;
        uint16_t _dlen = (uint16_t)UbxRxBuf[4] + ((uint16_t)UbxRxBuf[5] << 8);

        if ((_dlen >= 8U) && (((_dlen - 8U) % 12U) == 0U))
        {
            uint8_t _svn = (_dlen - 8U) / 12U;
            uint8_t numSVinView = UbxRxBuf[11];

            for (int i = 0; i < _svn; i++)
            {
                _UBXSAT_st *_psat = (_UBXSAT_st *) &UbxRxBuf[14 + 12 * i];
            }
        }
    }
}

///////////////////////////////////////////////////////////////
//以下为ublox 公司的UBX协议
///////////////////////////////////////////////////////////////

uint8_t Period_Out_H[] = {0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 0x64, 0x00, 0x01, 0x00, 0x01, 0x00, 0x7A, 0x12}; //

uint8_t Period_Out_L[] = {0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 0xC8, 0x00, 0x01, 0x00, 0x01, 0x00, 0xDE, 0x6A}; //

uint8_t Pulse_Period[] = {0xB5, 0x62, 0x06, 0x07, 0x14, 0x00, 0x40, 0x0D, 0x03, 0x00, 0xA0, 0x86, 0x01, 0x00, 0x01, 0x01,
                          0x00, 0x00, 0x34, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xD1, 0xCA
                         };

uint8_t Baud115200[] = {0xB5, 0x62, 0x06, 0x00, 0x14, 0x00, 0x01, 0x00, 0x00, 0x00, 0xD0, 0x08, 0x00, 0x00, 0x00, 0xC2,
                        0x01, 0x00, 0x01, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0xB8, 0x42
                       }; //设置波特率
uint8_t Baud921600[] = {0xB5, 0x62, 0x06, 0x00, 0x14, 0x00, 0x01, 0x00, 0x00, 0x00, 0xD0, 0x08, 0x00, 0x00, 0x00, 0x10,
                        0x0E, 0x00, 0x01, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x13, 0x1E
                       }; //设置波特率

uint8_t Save_Con[] = {0xB5, 0x62, 0x06, 0x09, 0x0D, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x1D, 0xAB}; //保存数据

//OFF
uint8_t POLLSH_SET[] = {0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0x01, 0x02, 0x00, 0x0D, 0x46};
uint8_t SOL_SET[] = {0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0x01, 0x06, 0x00, 0x11, 0x4E};
uint8_t VELNED_SET[] = {0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0x01, 0x12, 0x00, 0x1D, 0x66};
uint8_t TIMEUTC_SET[] = {0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0x01, 0x21, 0x00, 0x2C, 0x84};
uint8_t DOP_SET[] = {0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0x01, 0x04, 0x00, 0x0F, 0x4A};
//pvt
uint8_t PVT_SET[] = {0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0x01, 0x07, 0x01, 0x13, 0x51};
//nav_sat
uint8_t SAT_SET[] = {0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0x01, 0x35, 0x01, 0x41, 0xAD};	//已验证

char AnoGpsCmdDef_BUD[] = "$CFGPRT,,0,921600,,\r\n";
char AnoGpsCmdDef_SAT[] = "$CFGMOT,1,1,0\r\n";
//====
#include "Drv_Uart.h"
#define GPS_UART_INIT DrvUart4Init
#define GPS_PUT_BUFF DrvUart4SendBuf

void GPS_Rate_H()
{
    GPS_PUT_BUFF(Period_Out_H, sizeof(Period_Out_H));
}

void GPS_Rate_L()
{
    GPS_PUT_BUFF(Period_Out_L, sizeof(Period_Out_L));
}

//尝试用4种波特率初始化GPS
void Init_GPS()
{
#ifdef GPS_USE_UBLOX_M8
	GPS_UART_INIT(115200);
	MyDelayMs(10);
	GPS_PUT_BUFF((uint8_t*)AnoGpsCmdDef_BUD, sizeof(AnoGpsCmdDef_BUD)); 
	MyDelayMs(200);
    GPS_UART_INIT(9600);
	MyDelayMs(10);
    GPS_PUT_BUFF(Baud921600, sizeof(Baud921600)); //9600 改变波特率
    MyDelayMs(200);
    GPS_UART_INIT(19200);
    MyDelayMs(10);
    GPS_PUT_BUFF(Baud921600, sizeof(Baud921600)); //19200 改变波特率
    MyDelayMs(200);
    GPS_UART_INIT(38400);
    MyDelayMs(10);
    GPS_PUT_BUFF(Baud921600, sizeof(Baud921600)); //38400 改变波特率
    MyDelayMs(200);
    GPS_UART_INIT(115200);
	MyDelayMs(10);
    GPS_PUT_BUFF(Baud921600, sizeof(Baud921600)); //115200 改变波特率
	MyDelayMs(200);
    GPS_UART_INIT(921600);
    //设置完波特率后，要有长延时等待生效
    MyDelayMs(800);
    GPS_PUT_BUFF(POLLSH_SET, sizeof(POLLSH_SET));
    MyDelayMs(20);
    GPS_PUT_BUFF(SOL_SET, sizeof(SOL_SET));
    MyDelayMs(20);
    GPS_PUT_BUFF(VELNED_SET, sizeof(VELNED_SET));
    MyDelayMs(20);
    GPS_PUT_BUFF(TIMEUTC_SET, sizeof(TIMEUTC_SET));
    MyDelayMs(20);
    GPS_PUT_BUFF(DOP_SET, sizeof(DOP_SET));
    MyDelayMs(20);
    GPS_PUT_BUFF(PVT_SET, sizeof(PVT_SET));
    MyDelayMs(20);
    GPS_PUT_BUFF(SAT_SET, sizeof(SAT_SET));
    MyDelayMs(20);
	GPS_PUT_BUFF((uint8_t*)AnoGpsCmdDef_SAT, sizeof(AnoGpsCmdDef_SAT));
    MyDelayMs(20);
    GPS_Rate_H();
#else
#ifdef GPS_USE_RTK
    //57600波特率用于RTK
    GPS_UART_INIT(57600);
#endif
#endif
}

//====================================

//#define UBX_BUF_NUM 100
//static uint8_t ubx_buf[UBX_BUF_NUM];
static uint8_t protocol_class, protocol_id, protocol_length_t;
static uint16_t  protocol_length;
uint8_t pvt_recerve_ok_cnt;
void UBLOX_M8_GPS_Data_Receive(const uint8_t linktype, const uint8_t Data)
{
    static uint16_t state = 0;

    if (state == 0 && Data == 0xB5)
    {
        UbxRxBuf[state] = Data;
        state = 1;
    }
    else if (state == 1 && Data == 0x62)
    {
        UbxRxBuf[state] = Data;
        state = 2;
    }
    else if (state == 2)
    {
        UbxRxBuf[state] = Data;
        state = 3;
        protocol_class = Data;

        if (protocol_class != 0x01)
        {
            state = 0;
        }
    }
    else if (state == 3)
    {
        UbxRxBuf[state] = Data;
        state = 4;
        protocol_id = Data;

        if (protocol_id != 0x07 && protocol_id != 0x35 && protocol_id != 0x30)
        {
            state = 0;
        }
    }
    else if (state == 4)
    {
        UbxRxBuf[state] = Data;
        state = 5;
        protocol_length_t = Data;
    }
    else if (state == 5)
    {
        UbxRxBuf[state] = Data;
        state = 6;
        protocol_length = (uint16_t)protocol_length_t + ((uint16_t)Data << 8);

        if (protocol_length > (UBX_BUF_NUM - 8U))
        {
            state = 0;
            return;
        }

        if (protocol_id == 0x07)
        {
            if ((protocol_length > 100U) || (protocol_length < sizeof(_UBXPVT_st)))
            {
                state = 0;
            }

            if (pvt_receive_updata)
            {
                state = 0;
            }
        }
        else if (protocol_id == 0x35 || protocol_id == 0x30)
        {
            if (sat_receive_updata)
            {
                state = 0;
            }
        }
        else
        {
            state = 0;
        }
    }
    else if (state >= 6)
    {
		UbxRxBuf[state] = Data;
        state++;
		
		if(state >= (protocol_length + 8))
		{
			uint8_t CK_A = 0, CK_B = 0;

            for (int i = 2; i < (protocol_length + 6); i++)
            {
                CK_A += UbxRxBuf[i];
                CK_B += CK_A;
            }

            if (CK_A == UbxRxBuf[protocol_length + 6] && CK_B == UbxRxBuf[protocol_length + 7])
            {
                //
                if (protocol_class == 0x01 && protocol_id == 0x07)
                {
                    pvt_receive_updata = 1;
                }
                else if (protocol_class == 0x01 && protocol_id == 0x35)
                {
                    sat_receive_updata = 1;
                }
                else if (protocol_class == 0x01 && protocol_id == 0x30)
                {
                    sat_receive_updata = 1;
                }
            }
            state = 0;
		}
    }
    else
    {
        state = 0;
    }
}

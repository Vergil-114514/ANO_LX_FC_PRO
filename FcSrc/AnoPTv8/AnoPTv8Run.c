#include "AnoPTv8Run.h"
//接收缓冲区
_recvST recvBuf[LT_COUNT];
//发送缓冲区
_sendST AnoPTv8TxBuf[ANOPTV8_SENDBUFLEN];
/*************************************************************************************************************************************************
1毫秒周期执行函数，用来检查是否有数据需要发送以及其他需要定期检查的功能
*************************************************************************************************************************************************/
void AnoPTv8RunThread1ms(void)
{
	int _priority = 256;	//最低优先级
    int _needSendIndex = -1;

    for (int _sendCntOneTime = 0; _sendCntOneTime < 5; _sendCntOneTime++)
    {
		_needSendIndex = -1;
		_priority = 255;
        //根据优先级搜索一遍
        for (int i = 0; i < ANOPTV8_SENDBUFLEN; i++)
        {
            if (AnoPTv8TxBuf[i].used == 1 && AnoPTv8TxBuf[i].readyToSend == 1)
            {
                if (AnoPTv8TxBuf[i].priority < _priority)
                {
                    _priority = AnoPTv8TxBuf[i].priority;
                    _needSendIndex = i;
                }
                else if (AnoPTv8TxBuf[i].priority == _priority)
                {
                    //同优先级的，先发送sequence小的（先进入缓冲区的）
                    if (AnoPTv8TxBuf[i].sequence < AnoPTv8TxBuf[_needSendIndex].sequence)
                    {
                        _needSendIndex = i;
                    }
                }
            }
        }

        if (_needSendIndex >= 0)
        {
            //有需要发送的数据
            //执行发送函数并将该缓冲区标记为空
            AnoPTv8HwSendBytes(AnoPTv8TxBuf[_needSendIndex].linkType, AnoPTv8TxBuf[_needSendIndex].dataBuf.rawBytes, AnoPTv8TxBuf[_needSendIndex].dataBuf.frame.datalen + ANOPTV8_FRAME_HEADLEN + 2);
            AnoPTv8TxBuf[_needSendIndex].used = 0;
            AnoPTv8TxBuf[_needSendIndex].readyToSend = 0;
        }
        else
        {
            break;
        }
    }
	AnoPTv8CmdRunThread1ms();
}
/*************************************************************************************************************************************************
获取下一个空闲的发送缓冲区
*************************************************************************************************************************************************/
int AnoPTv8GetFreeTxBufIndex(const uint8_t priority)
{
	static uint16_t lastSequence = 0;

    int ret = -1;
	//先搜索是否有空余，如有空余，直接返回空余
    for (int i = 0; i < ANOPTV8_SENDBUFLEN; i++)
    {
        if (AnoPTv8TxBuf[i].used == 0)
        {
            AnoPTv8TxBuf[i].used = 1;
            AnoPTv8TxBuf[i].sequence = lastSequence++;
            ret = i;
            return ret;
        }
    }
	//没有空余，则选取优先级最低的写入
	int _lowstPri = -1;
	for (int i = 0; i < ANOPTV8_SENDBUFLEN; i++)
    {
        if (AnoPTv8TxBuf[i].priority > _lowstPri)
		{
			_lowstPri = AnoPTv8TxBuf[i].priority;
			ret = i;
		}
		else if(AnoPTv8TxBuf[i].priority == _lowstPri)
		{
			if(AnoPTv8TxBuf[i].sequence < AnoPTv8TxBuf[ret].sequence)
			{
				ret = i;
			}
		}
    }
	//如果最低优先级的优先级还要高于priority，则返回-1
	if(AnoPTv8TxBuf[ret].priority < priority)
		ret = -1;
	
	if(ret >=0 && ret<ANOPTV8_SENDBUFLEN)
		return ret;
	else
		return -1;
}
/*************************************************************************************************************************************************
对接收到的帧进行校验
*************************************************************************************************************************************************/
static int8_t anoPTv8FrameCheck(const _un_frame_v8 *p)
{
    uint8_t sumcheck = 0, addcheck = 0;
    if (p->frame.head != ANOPTV8_FRAME_HEAD)
        return 0;
    for (uint16_t i = 0; i < (ANOPTV8_FRAME_HEADLEN + p->frame.datalen); i++)
    {
        sumcheck += p->rawBytes[i];
        addcheck += sumcheck;
    }
    if (sumcheck == p->frame.sumcheck && addcheck == p->frame.addcheck)
        return 1;
    else
        return 0;
}
/*************************************************************************************************************************************************
对校验通过的数据帧进行解析，执行相应的功能
*************************************************************************************************************************************************/
static void anoPTv8FrameAnl(const uint16_t linktype, const _un_frame_v8 *p)
{
    if(p->frame.ddevid != ANOPTV8DEVID_MY && p->frame.ddevid != ANOPTV8DEVID_ALL)
        return;

    //根据帧ID执行不同的功能
    switch (p->frame.frameid)
    {
    case 0x00:
        if(p->frame.data[0] == 0xC0)
            AnoPTv8CmdRecvCheck(linktype, p);
        AnoPTv8FrameAnl(linktype, p);
        break;
    case 0xC0:
    case 0xC1:
        //命令相关帧
        AnoPTv8CmdFrameAnl(linktype, p);
        break;
    case 0xE0:
    case 0xE1:
        //参数读写相关帧
        AnoPTv8ParFrameAnl(linktype, p);
        break;
    default:
        AnoPTv8FrameAnl(linktype, p);
        break;
    }
}
/*************************************************************************************************************************************************
串口等通信方式下，每收到1字节数据，调用本函数一次，将接收到的数据传入进行数据解析
*************************************************************************************************************************************************/
void AnoPTv8RecvOneByte(const uint16_t linktype, const uint8_t dat)
{
    //本连接方式下接收缓存的index
    uint8_t _bufInx = 0;
    //搜索用的临时变量，每次搜索左移1位
    uint16_t _searchInx = 1<<0;
    //根据连接方式进行搜索，当linktype某一位非0时，退出搜索，记录第几位，从而获得对应的接收缓存的index
    while(((linktype&_searchInx) == 0) && (_searchInx>0))
    {
        _searchInx = _searchInx<<1;
        _bufInx++;
    }
    //如果接收缓存index大于连接数，退出，否则越界，因为定义的接收缓存数量=连接方式数量
    if(_bufInx >= LT_COUNT)
        return;
    //将接收缓存地址赋值给解析用的指针
    _recvST * pRxBuf = &recvBuf[_bufInx];

    if (pRxBuf->recvSta == 0)
    {
        //第一步，搜索帧头
        if (dat == ANOPTV8_FRAME_HEAD)
        {
            pRxBuf->recvSta = 1;
            pRxBuf->recvDataLen = 1;
            pRxBuf->dataBuf.rawBytes[0] = dat;
        }
    }
    else if (pRxBuf->recvSta == 1)
    {
        //第二步，接收地址字节、帧ID、帧长度等数据
        pRxBuf->dataBuf.rawBytes[pRxBuf->recvDataLen++] = dat;
        if (pRxBuf->recvDataLen >= ANOPTV8_FRAME_HEADLEN)
        {
            //如果数据长度超过定义的最大数据长度，重新开始接收
            if(pRxBuf->dataBuf.frame.datalen >= ANOPTV8_FRAME_MAXDATALEN)
                pRxBuf->recvSta = 0;
            else
                pRxBuf->recvSta = 2;
        }
    }
    else if (pRxBuf->recvSta == 2)
    {
        //第三步，接收数据内容
        pRxBuf->dataBuf.rawBytes[pRxBuf->recvDataLen++] = dat;
        if (pRxBuf->recvDataLen >= ANOPTV8_FRAME_HEADLEN + pRxBuf->dataBuf.frame.datalen)
            pRxBuf->recvSta = 3;
    }
    else if (pRxBuf->recvSta == 3)
    {
        //第四步，接收和校验字节
        pRxBuf->dataBuf.rawBytes[pRxBuf->recvDataLen++] = dat;
        pRxBuf->dataBuf.frame.sumcheck = dat;
        pRxBuf->recvSta = 4;
    }
    else if (pRxBuf->recvSta == 4)
    {
        //第5步，接收附加校验字节
        pRxBuf->dataBuf.rawBytes[pRxBuf->recvDataLen++] = dat;
        pRxBuf->dataBuf.frame.addcheck = dat;
        pRxBuf->recvSta = 0;
        //一帧数据接收完毕，进行校验，如果校验通过则进行帧解析
        if (anoPTv8FrameCheck(&pRxBuf->dataBuf))
        {
            //目的设备地址等于本模块地址，直接进行解析,如果是广播地址，也进行解析
            if(ANOPTV8DEVID_MY == pRxBuf->dataBuf.frame.ddevid || ANOPTV8DEVID_ALL == pRxBuf->dataBuf.frame.ddevid)
                anoPTv8FrameAnl(linktype, &pRxBuf->dataBuf);
            //如果目标地址不是本模块地址，则对数据进行转发，这里不要用else if，因为广播帧也要转发
            if(ANOPTV8DEVID_MY != pRxBuf->dataBuf.frame.ddevid)
            {
                AnoPTv8FrameExchange(linktype, &pRxBuf->dataBuf);
            }
        }
    }
    else
    {
        pRxBuf->recvSta = 0;
    }
}


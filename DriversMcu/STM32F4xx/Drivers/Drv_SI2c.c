#include "Drv_SI2c.h"

/***************I2C GPIO定义******************/
#define ANO_GPIO_I2C	GPIOB
#define I2C_Pin_SCL		GPIO_Pin_10
#define I2C_Pin_SDA		GPIO_Pin_11
#define ANO_RCC_I2C		RCC_AHBPeriph_GPIOB

#define SCL_H         ANO_GPIO_I2C->BSRRL = I2C_Pin_SCL
#define SCL_L         ANO_GPIO_I2C->BSRRH = I2C_Pin_SCL
#define SDA_H         ANO_GPIO_I2C->BSRRL = I2C_Pin_SDA
#define SDA_L         ANO_GPIO_I2C->BSRRH = I2C_Pin_SDA
#define SCL_read      ANO_GPIO_I2C->IDR  & I2C_Pin_SCL
#define SDA_read      ANO_GPIO_I2C->IDR  & I2C_Pin_SDA

static uint8_t mySelfInitAlready = 0;

static void I2c_Soft_delay()
{
    __nop();
    __nop();
    __nop();
    __nop();
    __nop();
    __nop();
    __nop();
    __nop();
    __nop();
    __nop();
    //
    __nop();
    __nop();
    __nop();
    __nop();
    __nop();
    __nop();
    __nop();
    __nop();
    __nop();
    __nop();
	//
    __nop();
    __nop();
    __nop();
    __nop();
    __nop();
    __nop();
    __nop();
    __nop();
    __nop();
    __nop();
	//
    __nop();
    __nop();
    __nop();
    __nop();
    __nop();
    __nop();
    __nop();
    __nop();
    __nop();
    __nop();
	//
    __nop();
    __nop();
    __nop();
    __nop();
    __nop();
    __nop();
    __nop();
    __nop();
    __nop();
    __nop();
	//
    __nop();
    __nop();
    __nop();
    __nop();
    __nop();
    __nop();
    __nop();
    __nop();
    __nop();
    __nop();
	//
    __nop();
    __nop();
    __nop();
    __nop();
    __nop();
    __nop();
    __nop();
    __nop();
    __nop();
    __nop();
	//
    __nop();
    __nop();
    __nop();
    __nop();
    __nop();
    __nop();
    __nop();
    __nop();
    __nop();
    __nop();
	//
    __nop();
    __nop();
    __nop();
    __nop();
    __nop();
    __nop();
    __nop();
    __nop();
    __nop();
    __nop();
	//
    __nop();
    __nop();
    __nop();
    __nop();
    __nop();
    __nop();
    __nop();
    __nop();
    __nop();
    __nop();
	//
    __nop();
    __nop();
    __nop();
    __nop();
    __nop();
    __nop();
    __nop();
    __nop();
    __nop();
    __nop();
	//
    __nop();
    __nop();
    __nop();
    __nop();
    __nop();
    __nop();
    __nop();
    __nop();
    __nop();
    __nop();
	//
    __nop();
    __nop();
    __nop();
    __nop();
    __nop();
    __nop();
    __nop();
    __nop();
    __nop();
    __nop();
}

void DrvSI2cInit()
{
	if(mySelfInitAlready)
		return;
	mySelfInitAlready = 1;
    GPIO_InitTypeDef  GPIO_InitStructure;
    GPIO_StructInit( &GPIO_InitStructure);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
    GPIO_InitStructure.GPIO_Pin =  I2C_Pin_SCL | I2C_Pin_SDA;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(ANO_GPIO_I2C, &GPIO_InitStructure);
	SDA_H;
    SCL_H;
	I2c_Soft_delay();
}

static u8 I2c_Soft_Start()
{
    SDA_H;
    SCL_H;
    I2c_Soft_delay();

    if (!SDA_read)
    {
        return 0;	//SDA线为低电平则总线忙,退出
    }

    SDA_L;
    I2c_Soft_delay();

    if (SDA_read)
    {
        return 0;	//SDA线为高电平则总线出错,退出
    }

    SCL_L;
    I2c_Soft_delay();
    return 1;
}

static void I2c_Soft_Stop()
{
    SDA_L;
    SCL_H;
    I2c_Soft_delay();
    SDA_H;
}

static void I2c_Soft_Ack()
{
    SDA_L;
    I2c_Soft_delay();
    SCL_H;
    I2c_Soft_delay();
    SCL_L;
    I2c_Soft_delay();
    SDA_H;//释放总线
}

static void I2c_Soft_NoAck()
{
    SDA_H;
    I2c_Soft_delay();
    SCL_H;
    I2c_Soft_delay();
    SCL_L;
}

static void I2c_Soft_SendByte(u8 SendByte) //数据从高位到低位//
{
    u8 i = 8;

    while (i--)
    {
        if (SendByte & 0x80)
        {
            SDA_H;
        }
        else
        {
            SDA_L;
        }

        I2c_Soft_delay();//# ###
        SCL_H;
        I2c_Soft_delay();
        SCL_L;
        I2c_Soft_delay();
        SendByte <<= 1;
    }

    //释放总线
    SDA_H;
}

//====
u16 i2c_ack_err_cnt;
u16 ErrTime = 0;
//====
static u8 I2c_Soft_WaitAck(void) 	 //返回为:=1有ASK,=0无ASK
{
    SCL_H;
    I2c_Soft_delay();

    while (SDA_read) //while(SDA_read)
    {
        ErrTime++;

        if (ErrTime > 10)
        {
            i2c_ack_err_cnt ++;
            I2c_Soft_Stop();
            return 1;
        }
    }

    SCL_L;
    I2c_Soft_delay();
    return 0;
}

//读1个字节
static u8 I2c_Soft_ReadByte()  //数据从高位到低位//
{
    u8 i = 8;
    u8 ReceiveByte = 0;
    SDA_H;
    I2c_Soft_delay();

    while (i--)
    {
        ReceiveByte <<= 1;
        SCL_H;
        I2c_Soft_delay();

        if (SDA_read)
        {
            ReceiveByte |= 0x01;
        }

        SCL_L;
        I2c_Soft_delay();
    }

    return ReceiveByte;
}

//====
//====

// IIC写一个字节数据
uint8_t DrvSI2c_Write1Byte(uint8_t SlaveAddress, uint8_t REG_Address, uint8_t REG_data)
{
    //
    I2c_Soft_Start();
    //
    I2c_Soft_SendByte(SlaveAddress << 1);

    if (I2c_Soft_WaitAck())
    {
        I2c_Soft_Stop();
        return 1;
    }

    //
    I2c_Soft_SendByte(REG_Address);

    if (I2c_Soft_WaitAck())
    {
        return 1;
    }

    //
    I2c_Soft_SendByte(REG_data);

    if (I2c_Soft_WaitAck())
    {
        return 1;
    }

    //
    I2c_Soft_Stop();
    return 0;
}

// IIC读1字节数据
uint8_t DrvSI2c_Read1Byte(uint8_t SlaveAddress, uint8_t REG_Address, uint8_t *REG_data)
{
    //
    I2c_Soft_Start();
    //
    I2c_Soft_SendByte(SlaveAddress << 1);

    if (I2c_Soft_WaitAck())
    {
        I2c_Soft_Stop();
        return 1;
    }

    //
    I2c_Soft_SendByte(REG_Address);

    if (I2c_Soft_WaitAck())
    {
        return 1;
    }

    //
    I2c_Soft_Start();
    //
    I2c_Soft_SendByte(SlaveAddress << 1 | 0x01);

    if (I2c_Soft_WaitAck())
    {
        return 1;
    }

    //
    *REG_data = I2c_Soft_ReadByte();
    //
    I2c_Soft_NoAck();
    //
    I2c_Soft_Stop();
    return 0;
}

// IIC写n字节数据
uint8_t DrvSI2c_WriteNByte(uint8_t SlaveAddress, uint8_t REG_Address, uint8_t len, uint8_t *buf)
{
    I2c_Soft_Start();
    I2c_Soft_SendByte(SlaveAddress << 1);

    if (I2c_Soft_WaitAck())
    {
        return 1;
    }

    I2c_Soft_SendByte(REG_Address);

    if (I2c_Soft_WaitAck())
    {
        return 1;
    }

    while (len--)
    {
        I2c_Soft_SendByte( *buf++);

        if (I2c_Soft_WaitAck())
        {
            return 1;
        }
    }

    I2c_Soft_Stop();
    return 0;
}

// IIC读n字节数据
uint8_t DrvSI2c_ReadNByte(uint8_t SlaveAddress, uint8_t REG_Address, uint8_t len, uint8_t *buf)
{
    I2c_Soft_Start();
    //
    I2c_Soft_SendByte(SlaveAddress << 1);

    if (I2c_Soft_WaitAck())
    {
        return 1;
    }

    //
    I2c_Soft_SendByte(REG_Address);

    if (I2c_Soft_WaitAck())
    {
        return 1;
    }

    //
    I2c_Soft_Start();
    //
    I2c_Soft_SendByte(SlaveAddress << 1 | 0x01);

    if (I2c_Soft_WaitAck())
    {
        return 1;
    }

    //
    while (len)
    {
        //
        *buf = I2c_Soft_ReadByte();

        //
        if (len == 1)
        {
            I2c_Soft_NoAck();
        }
        else
        {
            I2c_Soft_Ack();
        }

        buf++;
        len--;
    }

    //
    I2c_Soft_Stop();
    return 0;
}


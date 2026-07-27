#ifndef _DRVSI2C_H_
#define _DRVSI2C_H_
#include "SysConfig.h"


void DrvSI2cInit(void);
uint8_t DrvSI2c_Write1Byte(uint8_t SlaveAddress, uint8_t REG_Address, uint8_t REG_data);
uint8_t DrvSI2c_Read1Byte(uint8_t SlaveAddress, uint8_t REG_Address, uint8_t *REG_data);
uint8_t DrvSI2c_WriteNByte(uint8_t SlaveAddress, uint8_t REG_Address, uint8_t len, uint8_t *buf);
uint8_t DrvSI2c_ReadNByte(uint8_t SlaveAddress, uint8_t REG_Address, uint8_t len, uint8_t *buf);

#endif

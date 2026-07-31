#include "Drv_CarLink.h"
#include "SysConfig.h"
#include "Drv_Sys.h"

#define CAR_LINK_FRAME_LENGTH 11U
#define CAR_LINK_HEADER_0 0xAAU
#define CAR_LINK_HEADER_1 0x55U
#define CAR_LINK_PROTOCOL_VERSION 0x01U

_car_link_st CarLink;

static uint8_t carLinkFrame[CAR_LINK_FRAME_LENGTH];
static uint8_t carLinkFrameLength;

static uint16_t carLinkCrc16(const uint8_t *data, const uint8_t length)
{
    uint16_t crc = 0xFFFFU;
    uint8_t i;
    uint8_t bit;

    for (i = 0U; i < length; i++)
    {
        crc ^= (uint16_t)data[i] << 8;
        for (bit = 0U; bit < 8U; bit++)
        {
            if ((crc & 0x8000U) != 0U)
            {
                crc = (uint16_t)((crc << 1) ^ 0x1021U);
            }
            else
            {
                crc <<= 1;
            }
        }
    }

    return crc;
}

static void carLinkParseFrame(void)
{
    uint16_t crc;
    uint8_t sequence;
    uint8_t delta;

    if (carLinkFrame[2] != CAR_LINK_PROTOCOL_VERSION)
    {
        CarLink.rejected_frame_count++;
        return;
    }

    crc = carLinkCrc16(carLinkFrame, 9U);
    if ((carLinkFrame[9] != (uint8_t)(crc & 0xFFU)) ||
        (carLinkFrame[10] != (uint8_t)(crc >> 8)))
    {
        CarLink.rejected_frame_count++;
        return;
    }

    sequence = carLinkFrame[3];
    if (CarLink.sequence_valid != 0U)
    {
        delta = (uint8_t)(sequence - CarLink.sequence);
        if ((delta == 0U) || (delta >= 128U))
        {
            CarLink.rejected_frame_count++;
            return;
        }
    }

    CarLink.yaw_x100 = (int32_t)((uint32_t)carLinkFrame[5] |
                                 ((uint32_t)carLinkFrame[6] << 8) |
                                 ((uint32_t)carLinkFrame[7] << 16) |
                                 ((uint32_t)carLinkFrame[8] << 24));
    CarLink.flags = carLinkFrame[4] & CAR_LINK_FLAG_MASK;
    if ((CarLink.flags & CAR_LINK_FLAG_ABORT) != 0U)
    {
        CarLink.abort_pending = 1U;
    }
    if ((CarLink.flags & CAR_LINK_FLAG_CAR_AT_A) != 0U)
    {
        CarLink.car_at_a_pending = 1U;
    }
    CarLink.sequence = sequence;
    CarLink.sequence_valid = 1U;
    CarLink.valid = 1U;
    CarLink.age_ms = 0U;
    CarLink.last_update_ms = GetSysRunTimeMs();
    CarLink.valid_frame_count++;
}

void DrvCarLinkInit(void)
{
    CarLink.yaw_x100 = 0;
    CarLink.last_update_ms = 0U;
    CarLink.valid_frame_count = 0U;
    CarLink.rejected_frame_count = 0U;
    CarLink.age_ms = 0U;
    CarLink.flags = 0U;
    CarLink.sequence = 0U;
    CarLink.valid = 0U;
    CarLink.abort_pending = 0U;
    CarLink.car_at_a_pending = 0U;
    CarLink.sequence_valid = 0U;
    carLinkFrameLength = 0U;
}

void DrvCarLinkRxOneByte(const uint8_t data)
{
    if (carLinkFrameLength == 0U)
    {
        if (data == CAR_LINK_HEADER_0)
        {
            carLinkFrame[0] = data;
            carLinkFrameLength = 1U;
        }
        return;
    }

    if ((carLinkFrameLength == 1U) && (data != CAR_LINK_HEADER_1))
    {
        carLinkFrameLength = (data == CAR_LINK_HEADER_0) ? 1U : 0U;
        if (carLinkFrameLength != 0U)
        {
            carLinkFrame[0] = data;
        }
        return;
    }

    carLinkFrame[carLinkFrameLength++] = data;
    if (carLinkFrameLength >= CAR_LINK_FRAME_LENGTH)
    {
        carLinkParseFrame();
        carLinkFrameLength = 0U;
    }
}

void DrvCarLinkTask1ms(void)
{
    if (CarLink.valid == 0U)
    {
        return;
    }

    if (CarLink.age_ms < 0xFFFFU)
    {
        CarLink.age_ms++;
    }

    if (CarLink.age_ms > CAR_LINK_TIMEOUT_MS)
    {
        CarLink.valid = 0U;
        CarLink.flags = 0U;
        CarLink.sequence_valid = 0U;
    }
}

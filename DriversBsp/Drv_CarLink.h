#ifndef __DRV_CAR_LINK_H__
#define __DRV_CAR_LINK_H__

#include <stdint.h>

#define CAR_LINK_FLAG_START        0x01U
#define CAR_LINK_FLAG_ABORT        0x02U
#define CAR_LINK_FLAG_TASK_DYNAMIC 0x04U
#define CAR_LINK_FLAG_CAR_AT_A     0x08U
#define CAR_LINK_FLAG_MASK         (CAR_LINK_FLAG_START | CAR_LINK_FLAG_ABORT | \
                                    CAR_LINK_FLAG_TASK_DYNAMIC | CAR_LINK_FLAG_CAR_AT_A)

typedef struct
{
    int32_t yaw_x100;
    uint32_t last_update_ms;
    uint32_t valid_frame_count;
    uint32_t rejected_frame_count;
    uint16_t age_ms;
    uint8_t flags;
    uint8_t sequence;
    uint8_t valid;
    uint8_t abort_pending;
    uint8_t car_at_a_pending;
    uint8_t sequence_valid;
} _car_link_st;

extern _car_link_st CarLink;

void DrvCarLinkInit(void);
void DrvCarLinkRxOneByte(const uint8_t data);
void DrvCarLinkTask1ms(void);

#endif

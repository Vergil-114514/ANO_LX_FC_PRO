#ifndef __DRV_UWB_MINI5_H__
#define __DRV_UWB_MINI5_H__

#include <stdint.h>

typedef struct
{
    int32_t position_mm[3];
    uint32_t residual_mm;
    uint32_t last_update_ms;
    uint32_t valid_frame_count;
    uint32_t rejected_frame_count;
    uint16_t age_ms;
    uint8_t valid;
    uint8_t anchor_mask;
    uint8_t position_update_cnt;
    uint8_t last_rseq;
    uint8_t rseq_valid;
} _uwb_mini5_st;

extern _uwb_mini5_st UwbMini5;

void DrvUwbMini5Init(void);
void DrvUwbMini5RxOneByte(const uint8_t linktype, const uint8_t data);
void DrvUwbMini5Task1ms(void);

#endif

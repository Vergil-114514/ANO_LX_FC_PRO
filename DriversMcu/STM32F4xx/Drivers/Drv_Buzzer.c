#include "Drv_Buzzer.h"

/*************** GPIO Definition ******************/
#define BUZZER_RCC        RCC_AHB1Periph_GPIOC
#define BUZZER_GPIO       GPIOC
#define BUZZER_PIN        GPIO_Pin_2

/*************** Advanced Control States ******************/
typedef enum {
    BUZZER_IDLE = 0,
    BUZZER_ADV1_CTRL,
    BUZZER_ADV2_CTRL
} BuzzerCtrlState;

/*************** Static Variables ******************/
static BuzzerCtrlState s_ctrlState = BUZZER_IDLE;
static uint8_t s_adv1Times = 0;
static uint16_t s_adv1OnTime = 0;
static uint16_t s_adv1OffTime = 0;
static uint16_t s_adv2Timings[6] = {0};
static uint16_t s_timMsCnt = 0;
static uint8_t s_step = 0;

/*************** Private Function Prototypes ******************/
static void BuzzerSetState(uint8_t state);
static void HandleAdv1Ctrl(void);
static void HandleAdv2Ctrl(void);
static void ResetBuzzerState(void);
/*************** Public Functions ******************/
void DrvBuzzerInit(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {
        .GPIO_Pin = BUZZER_PIN,
        .GPIO_Mode = GPIO_Mode_OUT,
        .GPIO_Speed = GPIO_Speed_2MHz,
        .GPIO_OType = GPIO_OType_OD,
        .GPIO_PuPd = GPIO_PuPd_UP
    };
    
    RCC_AHB1PeriphClockCmd(BUZZER_RCC, ENABLE);
    GPIO_Init(BUZZER_GPIO, &GPIO_InitStruct);
    BuzzerSetState(0);
}

void DrvBuzzerCtl(uint8_t enable)
{
    BuzzerSetState(enable);
}

void DrvBuzzerRunTask(float dT)
{
    if (s_ctrlState == BUZZER_IDLE) {
        return;
    }

    s_timMsCnt += (uint16_t)(dT * 1000);

    switch (s_ctrlState) {
        case BUZZER_ADV1_CTRL:
            HandleAdv1Ctrl();
            break;
            
        case BUZZER_ADV2_CTRL:
            HandleAdv2Ctrl();
            break;
            
        default:
            ResetBuzzerState();
            break;
    }
}

void DrvBuzzerAdvCtl1(uint8_t times, uint16_t onTimeMs, uint16_t offTimeMs)
{
    if (s_ctrlState == BUZZER_IDLE) {
        s_adv1Times = times;
        s_adv1OnTime = onTimeMs;
        s_adv1OffTime = offTimeMs;
        s_ctrlState = BUZZER_ADV1_CTRL;
        s_step = 0;
        s_timMsCnt = 0;
    }
}

void DrvBuzzerAdvCtl2(uint16_t onTime1, uint16_t offTime1, 
                      uint16_t onTime2, uint16_t offTime2,
                      uint16_t onTime3, uint16_t offTime3)
{
    if (s_ctrlState == BUZZER_IDLE) {
        s_adv2Timings[0] = onTime1;
        s_adv2Timings[1] = offTime1;
        s_adv2Timings[2] = onTime2;
        s_adv2Timings[3] = offTime2;
        s_adv2Timings[4] = onTime3;
        s_adv2Timings[5] = offTime3;
        s_ctrlState = BUZZER_ADV2_CTRL;
        s_step = 0;
        s_timMsCnt = 0;
    }
}

void DrvBuzzerAdvCtlOK(void)
{
    DrvBuzzerAdvCtl1(3, 100, 100);
}

void DrvBuzzerAdvCtlERR(void)
{
    DrvBuzzerAdvCtl2(1000, 500, 100, 100, 100, 100);
}

/*************** Private Functions ******************/
static void BuzzerSetState(uint8_t state)
{
    if (state) {
        GPIO_ResetBits(BUZZER_GPIO, BUZZER_PIN);
    } else {
        GPIO_SetBits(BUZZER_GPIO, BUZZER_PIN);
    }
}

static void HandleAdv1Ctrl(void)
{
    switch (s_step) {
        case 0: // Start sequence
            if (s_adv1Times == 0) {
                ResetBuzzerState();
                return;
            }
            s_step++;
            BuzzerSetState(1);
            break;
            
        case 1: // On time
            if (s_timMsCnt >= s_adv1OnTime) {
                s_step++;
                s_timMsCnt = 0;
                BuzzerSetState(0);
            }
            break;
            
        case 2: // Off time
            if (s_timMsCnt >= s_adv1OffTime) {
                s_step = 0;
                s_timMsCnt = 0;
                if (s_adv1Times > 0) {
                    s_adv1Times--;
                }
            }
            break;
            
        default:
            ResetBuzzerState();
            break;
    }
}

static void HandleAdv2Ctrl(void)
{
    if (s_step == 0) {
        // Initial step - turn buzzer on
        s_step++;
        s_timMsCnt = 0;
        BuzzerSetState(1);
        return;
    }

    uint8_t timingIndex = s_step - 1;
    if (timingIndex >= sizeof(s_adv2Timings)/sizeof(s_adv2Timings[0])) {
        ResetBuzzerState();
        return;
    }

    if (s_timMsCnt >= s_adv2Timings[timingIndex]) {
        s_step++;
        s_timMsCnt = 0;
        BuzzerSetState(s_step % 2); // Alternate between on and off
    }
}

static void ResetBuzzerState(void)
{
    s_step = 0;
    s_timMsCnt = 0;
    s_ctrlState = BUZZER_IDLE;
    BuzzerSetState(0);
}

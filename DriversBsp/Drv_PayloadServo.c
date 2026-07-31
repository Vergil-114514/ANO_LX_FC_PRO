#include "Drv_PayloadServo.h"

#define PAYLOAD_SERVO_GPIO_RCC RCC_AHB1Periph_GPIOE
#define PAYLOAD_SERVO_GPIO GPIOE
#define PAYLOAD_SERVO_PIN GPIO_Pin_9
#define PAYLOAD_SERVO_TIMER_RCC RCC_APB1Periph_TIM2
#define PAYLOAD_SERVO_TIMER TIM2
#define PAYLOAD_SERVO_TIMER_PRESCALER 84U
#define PAYLOAD_SERVO_TIMER_PERIOD 0xFFFFFFFFUL
#define PAYLOAD_SERVO_FRAME_US 20000U
#define PAYLOAD_SERVO_IRQ_P 2U
#define PAYLOAD_SERVO_IRQ_S 0U

static volatile uint16_t payloadServoRequestedUs = PAYLOAD_SERVO_CLAMP_US;
static volatile uint16_t payloadServoActiveUs = PAYLOAD_SERVO_CLAMP_US;
static volatile uint8_t payloadServoOutputHigh;
static volatile uint8_t payloadServoReleased;

static uint16_t DrvPayloadServoClampPulse(const uint16_t pulseUs)
{
    if (pulseUs < PAYLOAD_SERVO_MIN_US)
    {
        return PAYLOAD_SERVO_MIN_US;
    }
    if (pulseUs > PAYLOAD_SERVO_MAX_US)
    {
        return PAYLOAD_SERVO_MAX_US;
    }
    return pulseUs;
}

void DrvPayloadServoInit(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    RCC_AHB1PeriphClockCmd(PAYLOAD_SERVO_GPIO_RCC, ENABLE);
    RCC_APB1PeriphClockCmd(PAYLOAD_SERVO_TIMER_RCC, ENABLE);

    GPIO_StructInit(&GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = PAYLOAD_SERVO_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(PAYLOAD_SERVO_GPIO, &GPIO_InitStructure);

    payloadServoRequestedUs = PAYLOAD_SERVO_CLAMP_US;
    payloadServoActiveUs = PAYLOAD_SERVO_CLAMP_US;
    payloadServoOutputHigh = 1U;
    payloadServoReleased = 0U;

    TIM_DeInit(PAYLOAD_SERVO_TIMER);
    TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
    TIM_TimeBaseStructure.TIM_Period = PAYLOAD_SERVO_TIMER_PERIOD;
    TIM_TimeBaseStructure.TIM_Prescaler = PAYLOAD_SERVO_TIMER_PRESCALER - 1U;
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(PAYLOAD_SERVO_TIMER, &TIM_TimeBaseStructure);
    TIM_SetCounter(PAYLOAD_SERVO_TIMER, 0U);
    TIM_SetCompare1(PAYLOAD_SERVO_TIMER, PAYLOAD_SERVO_CLAMP_US);
    TIM_ClearITPendingBit(PAYLOAD_SERVO_TIMER, TIM_IT_CC1);
    TIM_ITConfig(PAYLOAD_SERVO_TIMER, TIM_IT_CC1, ENABLE);

    NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = PAYLOAD_SERVO_IRQ_P;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = PAYLOAD_SERVO_IRQ_S;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    GPIO_SetBits(PAYLOAD_SERVO_GPIO, PAYLOAD_SERVO_PIN);
    TIM_Cmd(PAYLOAD_SERVO_TIMER, ENABLE);
}

void DrvPayloadServoSetPulseUs(const uint16_t pulseUs)
{
    if (payloadServoReleased != 0U)
    {
        return;
    }
    payloadServoRequestedUs = DrvPayloadServoClampPulse(pulseUs);
}

void DrvPayloadServoClamp(void)
{
    DrvPayloadServoSetPulseUs(PAYLOAD_SERVO_CLAMP_US);
}

void DrvPayloadServoRelease(void)
{
    if (payloadServoReleased == 0U)
    {
        payloadServoRequestedUs = PAYLOAD_SERVO_RELEASE_US;
        payloadServoReleased = 1U;
    }
}

uint8_t DrvPayloadServoIsReleased(void)
{
    return payloadServoReleased;
}

void DrvPayloadServoIrqHandler(void)
{
    if (TIM_GetITStatus(PAYLOAD_SERVO_TIMER, TIM_IT_CC1) != RESET)
    {
        TIM_ClearITPendingBit(PAYLOAD_SERVO_TIMER, TIM_IT_CC1);

        if (payloadServoOutputHigh != 0U)
        {
            GPIO_ResetBits(PAYLOAD_SERVO_GPIO, PAYLOAD_SERVO_PIN);
            payloadServoOutputHigh = 0U;
            PAYLOAD_SERVO_TIMER->CCR1 += PAYLOAD_SERVO_FRAME_US - payloadServoActiveUs;
        }
        else
        {
            payloadServoActiveUs = payloadServoRequestedUs;
            GPIO_SetBits(PAYLOAD_SERVO_GPIO, PAYLOAD_SERVO_PIN);
            payloadServoOutputHigh = 1U;
            PAYLOAD_SERVO_TIMER->CCR1 += payloadServoActiveUs;
        }
    }
}

#include "bsp_delay.h"
#include "main.h"

static uint8_t fac_us = 0;
static uint32_t fac_ms = 0;
BSP_Delay bsp_delay;

void BSP_Delay::Timer::DWT_Init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;     // ʹ�� DWT �Ĵ���
    DWT->CYCCNT = 0;                                     // ���������
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;                // ʹ�����ڼ�����
}

uint32_t BSP_Delay::Timer::DWT_GetUs(void)
{
    return DWT->CYCCNT / (HAL_RCC_GetHCLKFreq() / 1000000);
}

void BSP_Delay::Timer::Timer_Start(TimerNonBlocking *timer, uint32_t delay_us) {
    timer->start_time_us = bsp_delay.timer.DWT_GetUs();
    timer->delay_us = delay_us;
    timer->active = 1;
}

uint8_t BSP_Delay::Timer::Timer_Expired(TimerNonBlocking *timer) {
    if (!timer->active) return 0;

    uint32_t now = bsp_delay.timer.DWT_GetUs();
    // �����ȫ�ж�
    if ((uint32_t)(now - timer->start_time_us) >= timer->delay_us) {
        timer->active = 0;
        return 1;
    }
    return 0;
}//������΢����ʱ

void BSP_Delay::Delay::delay_init(void)
{
    fac_us = SystemCoreClock / 1000000;
    fac_ms = SystemCoreClock / 1000;

}

void BSP_Delay::Delay::delay_us(uint16_t nus)
{
    uint32_t ticks = 0;
    uint32_t told = 0;
    uint32_t tnow = 0;
    uint32_t tcnt = 0;
    uint32_t reload = 0;
    reload = SysTick->LOAD;
    ticks = nus * fac_us;
    told = SysTick->VAL;
    while (1)
    {
        tnow = SysTick->VAL;
        if (tnow != told)
        {
            if (tnow < told)
            {
                tcnt += told - tnow;
            }
            else
            {
                tcnt += reload - tnow + told;
            }
            told = tnow;
            if (tcnt >= ticks)
            {
                break;
            }
        }
    }
}


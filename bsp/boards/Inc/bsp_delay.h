#ifndef BSP_DELAY_H
#define BSP_DELAY_H


#ifdef __cplusplus
extern "C" 
{
#endif

#include <stdint.h>

typedef struct {
    uint32_t start_time_us;
    uint32_t delay_us;
    uint8_t active;
} TimerNonBlocking;


class BSP_Delay
{
	public:
		class Timer
        {
            public:
                void DWT_Init(void);
                uint32_t DWT_GetUs(void);
                void Timer_Start(TimerNonBlocking *timer, uint32_t delay_us);
                uint8_t Timer_Expired(TimerNonBlocking *timer);

        }timer;
		class Delay
		{
			public:
			    void delay_init(void);
			    void delay_us(uint16_t nus);
			    void delay_ms(uint16_t nms);
		}delay;
		
/*******对象*******/
extern BSP_Delay bsp_delay;
#ifdef __cplusplus
}
#endif
// extern void delay_init(void);
// extern void delay_us(uint16_t nus);
// extern void delay_ms(uint16_t nms);
#endif


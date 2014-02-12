#ifndef __TIMER_H
#define __TIMER_H

#ifdef __cplusplus
 extern "C" {
#endif
   
extern void Timer_Init(void);   
extern void Delay_ms(uint32_t time);
extern uint32_t GetSysTick(void);
#ifdef __cplusplus
}
#endif

#endif /* __TIMER_H */
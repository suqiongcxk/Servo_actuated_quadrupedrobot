#ifndef    MYDWT_H
#define		MYDWT_H

#include "stm32h7xx_hal.h"
void Enable_DWT_CycleCounter(void);



extern  uint32_t start_cycle;
extern  uint32_t end_cycle ;
extern  uint32_t delta_cycles;


//计时宏定义
#define DWT_TIME_START() \
    uint32_t start_cycles = DWT->CYCCNT  // 记录起始周期数

#define DWT_TIME_END(label) \
    do { \
        uint32_t end_cycles = DWT->CYCCNT; \
        /* 处理32位计数器溢出，保证计时结果正确 */ \
        uint32_t elapsed_cycles = (end_cycles >= start_cycles) ? (end_cycles - start_cycles) : (0xFFFFFFFF - start_cycles + end_cycles + 1); \
        /* 仅打印标签和周期数，%lu 匹配 uint32_t 类型 */ \
        printf("[TIME] %s: %lu cycles\n", label, elapsed_cycles); \
    } while(0)

		
		
#endif

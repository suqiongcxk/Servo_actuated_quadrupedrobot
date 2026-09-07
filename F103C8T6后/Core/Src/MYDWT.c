#include "MYDWT.h"
#include "main.h"
#include "core_cm3.h" 

// 全局计时相关变量
uint32_t start_cycle = 0;    // 计时起始周期数（存储DWT->CYCCNT的起始值）
uint32_t end_cycle = 0;      // 计时结束周期数（存储DWT->CYCCNT的结束值）
uint32_t delta_cycles = 0;   // 耗时总周期数（end_cycle - start_cycle，需后续处理溢出）
float time_us;               // 耗时微秒数（后续可通过周期数和主频转换得到）

/**
 * @brief  使能DWT循环计数器（用于高精度硬件计时）
 * @note   该函数仅需在程序入口处调用一次，初始化后DWT->CYCCNT开始自增计数
 * @param  无
 * @retval 无
 */
void Enable_DWT_CycleCounter(void) {
    // 使能内核调试跟踪功能（TRCENA），为DWT计数器工作提供基础
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;  
    
    // 将DWT循环计数器（CYCCNT）清零，重置计时起点
    DWT->CYCCNT = 0;                                
                                                    
    // 使能DWT循环计数器（CYCCNTENA），开启后CYCCNT会随内核时钟自增
    DWT->CTRL  |= DWT_CTRL_CYCCNTENA_Msk;          
}

// 以下是示例计时代码片段（注释补全）
//		start_cycle = DWT->CYCCNT;  // 记录当前DWT计数器值，作为计时起始点
//		
//		// 此处放置需要计时的业务代码（如函数调用、数据处理等）
//		
//		end_cycle = DWT->CYCCNT;    // 记录当前DWT计数器值，作为计时结束点
//		
//		// 打印耗时周期数（注：未处理32位计数器溢出，短时间计时可直接使用）
//		printf("%d\n", end_cycle - start_cycle);



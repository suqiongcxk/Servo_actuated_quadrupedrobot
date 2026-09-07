#include "main.h"
#include "core_cm7.h" //ȡ������� Cortex-M ��

uint64_t start_cycle = 0;
uint64_t end_cycle = 0;
uint64_t delta_cycles = 0;
float time_us;

void Enable_DWT_CycleCounter(void) {
	
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL  |= DWT_CTRL_CYCCNTENA_Msk;
	
}



//		start_cycle = DWT->CYCCNT; 初始计时
//		
//		
//	     我的函数
//		
//		end_cycle = DWT->CYCCNT;结束始计时
//		
//		printf("%d\n", end_cycle - start_cycle);



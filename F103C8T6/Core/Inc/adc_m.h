#ifndef _ADC_M_H
#define _ADC_M_H

#include "main.h"
#include "adc.h"
#define ADC1_CHANNEL_CNT 6// ADC1通道数量
#define ADC1_CHANNEL_FRE 1// 采样频率/次数

extern uint16_t dma_i;

extern uint16_t adc1_val_buf[ADC1_CHANNEL_CNT*ADC1_CHANNEL_FRE];// 存放DMA多通道采样值

extern uint32_t adc1_aver_val[ADC1_CHANNEL_CNT];// 各通道平均值的缓存数组

extern uint16_t value[ADC1_CHANNEL_CNT];// 最终采样值

void ADC_Sample_Start(void);

void ADC_Process(void);

#endif

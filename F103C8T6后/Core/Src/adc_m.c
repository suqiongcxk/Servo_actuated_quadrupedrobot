#include "adc_m.h"
#include "string.h"
#include <stdio.h>
#include <stdint.h>
#include "stm32f1xx_hal.h"
#include "Servo_control.h"
uint16_t dma_i;

uint16_t adc1_val_buf[ADC1_CHANNEL_CNT*ADC1_CHANNEL_FRE];// 存放DMA多通道采样值

uint32_t adc1_aver_val[ADC1_CHANNEL_CNT]={0};// 各通道平均值的缓存数组

uint16_t value[ADC1_CHANNEL_CNT]={0};// 最终采样值


 int point[3]={0};
 
int lenth=0;// 数据长度
 
void ADC_Sample_Start(void)
{
	HAL_ADCEx_Calibration_Start(&hadc1);
	
	if(HAL_ADC_Start_DMA(&hadc1,(uint32_t*)&adc1_val_buf,(ADC1_CHANNEL_CNT*ADC1_CHANNEL_FRE))!=HAL_OK)
	{
		Error_Handler();
	}
}



void ADC_Process(void)
{
	// 清空DMA缓存值
	for(dma_i=0;dma_i<ADC1_CHANNEL_CNT;dma_i++)
	{
		adc1_aver_val[dma_i]=0;
	}
	for(dma_i=0;dma_i<ADC1_CHANNEL_FRE;dma_i++)
	{
		adc1_aver_val[0]+=adc1_val_buf[dma_i*ADC1_CHANNEL_CNT+0];
		adc1_aver_val[1]+=adc1_val_buf[dma_i*ADC1_CHANNEL_CNT+1];
		adc1_aver_val[2]+=adc1_val_buf[dma_i*ADC1_CHANNEL_CNT+2];
		adc1_aver_val[3]+=adc1_val_buf[dma_i*ADC1_CHANNEL_CNT+3];
		adc1_aver_val[4]+=adc1_val_buf[dma_i*ADC1_CHANNEL_CNT+4];
		adc1_aver_val[5]+=adc1_val_buf[dma_i*ADC1_CHANNEL_CNT+5];
	}
	for(dma_i=0;dma_i<ADC1_CHANNEL_CNT;dma_i++)
	{
		value[dma_i]=adc1_aver_val[dma_i]/ADC1_CHANNEL_FRE;
//		printf("%d ",value[dma_i]);
	}
//		printf("\n ");
}


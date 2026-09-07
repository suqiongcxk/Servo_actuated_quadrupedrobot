#ifndef data_table_H_
#define data_table_H_

#include "stm32h7xx_hal.h"
#include "geometric_method.h"

float MY_DUOJIcosf(float progress);
float MY_GAITsinf(uint16_t progress);
float MY_IK_acos(float progress);
float MY_GAITCOSf(uint16_t progress);
float MY_GAITCOSf_0_1(uint16_t progress);


// 双曲余弦数组
extern const float duoji_cos_arr[100];
// 步态正弦数组
extern  const float GAIT_SIN_ARR[200];
// 逆余弦数组
extern const float IK_acos[2048];
// 步态余弦数组
extern const float GAIT_COS_ARR[200];
// 步态余弦数组,这个表的数值先慢后快再慢更天然
extern const float GAIT_COS_ARR_0_1[200];
#endif



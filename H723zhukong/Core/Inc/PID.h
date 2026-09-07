#ifndef  PID_h
#define  PID_h
#include "stm32h7xx_hal.h"

typedef struct 
{
float p1;
float i1;
float d1;
float out1;
float error1;
float error_last1;
float errorint_max1;//积分限幅最高
float errorint_min1;//积分限幅最低
float output_rate_max1;//微分限幅最高
float output_rate_min1;//微分限幅最低
float out_max1;
float out_min1;
float actural1;//实际值
float target1;//目标值
float errorint1;


}PID_struct;




extern PID_struct IMU_PID ;
void 	 IMU_pid_init (PID_struct *motor_pid );//PID初始化
float  LV1_PID_BACK(PID_struct *motor_pid, float NOW,float target);



#endif



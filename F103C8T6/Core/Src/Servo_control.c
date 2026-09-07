#include "Servo_control.h"
#include "math.h"
#include "data_table.h"
#include "tim.h"
#include "adc_m.h"
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
float TARGET_ANGLE_PRE[12] = {-1,-1,-1,-1,-1,-1,-1,-1-1,-1,-1,-1};  //目标角度预设值
float STEP[12] = {0.0f};              //步长
float Start_angle[12] = {0.0f};       //起始角度
int   calibrate_flag[ADC1_CHANNEL_CNT] = {0};    //校准值，舵机零位
float  Servo_zero_angle[6] = {0};  
//存储舵机目标角度
float  MOVE_TARget[6] = {0.0f};
//当前占空比
uint16_t pulse[6] = {0.0f};
//存储当前角度
float  CURRENT_ANGLE[6] = {0.0f};
//存储转动时间
uint16_t  MOVE_TIME[6] = {0.0f};
//髋关节校准完成标志位
int Hip_calibrate_flag = 0;
//大腿关节校准完成标志位
int Thigh_calibrate_flag = 0;
//小腿关节校准完成标志位
int Lower_leg_calibrate_flag = 0;
//小腿SW测到的校准后的关节角
const float    RIGHT_hip_SW_angle  = 45.0f;
const float    RIGHT_Lower_leg_SW_angle  = 80.51f;
const float    RIGHT_thigh_SW_angle  = 65.94f;
const float    LEFT_hip_SW_angle  = 45.0f;
const float    LEFT_thigh_SW_angle  = 65.94f;
const float    LEFT_Lower_leg_SW_angle   = 80.51f;

//定义当前步长
float 	 DUOJI_progress[12] = {0,0,0 ,0,0,0 ,0,0,0, 0,0,0};
//@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@角度定义@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
float  					Left_thigh_angle  				    =					0;    //左大腿  
float  					Left_lower_leg_angle    		    =					0;	  //左小腿
float  					Right_thigh_angle     	 		    =					0;	  //右大腿		
float  					Right_lower_leg_angle   	 	    =					0;	  //右小腿    
float  					Left_hip_angle     	 	            =					0;	  //左髋	
float  					Right_hip_angle     	 	        =					0;	  //右髋		



//@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@通道定义@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
uint16_t			Left_thigh_channel   				    =					0x0103;     //左大腿舵机通道
uint16_t			Left_lower_leg_channel    		        =					0x0102;		//左小腿舵机通道
uint16_t			Left_hip_channel     	 	            =					0x0104;		//左髋舵机通道		
uint16_t			Right_thigh_channel     	 		    =					0x0202;		//右大腿舵机通道		
uint16_t			Right_lower_leg_channel   	 	        =					0x0101;		//右小腿舵机通道		
uint16_t			Right_hip_channel     			        =					0x0201;		//右髋舵机通道		





/*
@@@   角度转换函数
*/
void Serial_turn ( uint8_t Servo_ID,  float angle)
{
//	ASSERT( angle< 0 ||  angle > 180 );
	uint16_t set_compare = angle*( ( My_Arr * 0.1 )/180)+500;
	
	switch  (Servo_ID)
	{
		case 1 :   __HAL_TIM_SET_COMPARE(&htim1 , TIM_CHANNEL_1 , set_compare);			 break ;
		case 2 :   __HAL_TIM_SET_COMPARE(&htim1 , TIM_CHANNEL_2 , set_compare);          break ;
		case 3 :   __HAL_TIM_SET_COMPARE(&htim1 , TIM_CHANNEL_3 , set_compare);          break ;
		case 4 :   __HAL_TIM_SET_COMPARE(&htim1 , TIM_CHANNEL_4 , set_compare);          break ;
		case 5 :   __HAL_TIM_SET_COMPARE(&htim2 , TIM_CHANNEL_1 , set_compare);          break ;
		case 6 :   __HAL_TIM_SET_COMPARE(&htim2 , TIM_CHANNEL_2 , set_compare);          break ;
	} 
}





/**
 * @brief 舵机低通滤波平滑移动控制函数
 * @details 采用低通滤波算法实现舵机从当前角度平滑移动到目标角度，
 *          目标角度会被映射到新的数值范围，同时将角度范围限制在0~180度，并转换为对应PWM占空比
 * @param TIM_CHAnnel 舵机对应的定时器通道编码（高8位为定时器编号，低8位为通道编号）
 * @param target_angle 舵机目标角度（范围为0.0~180.0度）
 * @param current_angle 指向当前角度的指针，函数会更新该值
 * @param filter_gain 低通滤波系数（用于控制角度变化的平滑程度）
 * @param max_speed 舵机最大转动速度（用于限制角度变化的最大步长）
 * @return 更新后的当前角度值（范围为0.0~180.0度）
 */
float Servo_LowPass_Control(uint16_t TIM_CHAnnel, float target_angle, 
                           float *current_angle, float filter_gain, float max_speed ) {
    // 计算目标角度与当前角度的差值
    float delta = target_angle - *current_angle;
    
    // 限制舵机最大转动速度（防止角度突变）
    if (max_speed > 0) {
        float max_delta = max_speed * 0.01f;  // 按10ms周期计算最大允许角度变化量
        delta = fmaxf(-max_delta, fminf(max_delta, delta));
    }
    
    // 低通滤波更新当前角度
    *current_angle += filter_gain * delta;
    // 限制角度在0~180度范围内
    *current_angle = fmaxf(0.0f, fminf(180.0f, *current_angle));
    
    // 将角度转换为对应的PWM脉冲值
    uint16_t pulse = (uint16_t)(*current_angle * ANGLE_TO_DUTY) + 500;
    
		
    // 输出PWM信号到对应定时器通道
    uint8_t timer_num = (TIM_CHAnnel >> 8) & 0xF;  // 提取高8位的定时器编号
    uint8_t channel_num = TIM_CHAnnel & 0xFF;      // 提取低8位的通道编号
    
    switch (timer_num) {
        case 0x01:  // TIM1
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1 + (channel_num - 1) * 4, pulse);
            break;
        case 0x03:  // TIM3
            __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1 + (channel_num - 1) * 4, pulse);
            break;
    }
    
    return *current_angle;
}


//直接PWM驱动
float Servo_Direct_Control(uint16_t TIM_CHAnnel, float target_angle, 
                          float *current_angle) {
    // 1. 直接更新当前角度为目标角度 (无过渡)
    *current_angle = target_angle;
    
    // 2. 限制角度范围 [0, 180]
    *current_angle = fmaxf(0.0f, fminf(180.0f, *current_angle));
    
    // 3. 转换为 PWM 占空比 (500-2500us)
    uint16_t pulse = (uint16_t)(*current_angle * ANGLE_TO_DUTY) + 500;
    
    // 4. 解析定时器和通道
    uint8_t timer_num = (TIM_CHAnnel >> 8) & 0xF;
    uint8_t channel_num = TIM_CHAnnel & 0xFF;
    
    // 5. 立即设置 PWM
    switch (timer_num) {
        case 0x01:
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1 + (channel_num - 1) * 4, pulse);
            break;
        case 0x02:
            __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1 + (channel_num - 1) * 4, pulse);
            break;
    }


    return *current_angle;
}


/**
 * @brief 舵机S曲线平滑移动控制函数
 * @details 采用S曲线算法，实现舵机从当前角度平滑移动到目标角度，
 *          目标角度会被映射到新的数值范围，同时将角度范围限制在0~180度，并转换为对应PWM占空比
 * @param TIM_CHAnnel 舵机对应的定时器通道编码（高8位为定时器编号，低8位为通道编号）
 * @param target_angle 舵机目标角度（范围为0.0~180.0度）
 * @param current_angle 指向当前角度的指针，函数会更新该值
 * @param total_time 舵机从当前角度到目标角度的总时长（单位为毫秒）
 * @return 更新后的当前角度值（范围为0.0~180.0度）
 */
float Servo_SCurve_Control(uint16_t TIM_CHAnnel, float target_angle,
                          float *current_angle, float total_time,int ID) {
    float delta = target_angle - *current_angle;
    if(TARGET_ANGLE_PRE[ID-1] != target_angle)
    {
        STEP[ID-1] = delta;
        Start_angle[ID-1] = *current_angle;
        TARGET_ANGLE_PRE[ID-1] = target_angle;  // 更新目标角度预设值
				DUOJI_progress[ID-1]= 0;
    }
//		printf("%.2f\n",STEP[RIGHT_THIGH_ID]);
//		printf("%.2f\n",Start_angle[RIGHT_THIGH_ID]);
//		printf("%.2f\n",TARGET_ANGLE_PRE[RIGHT_THIGH_ID]);
    if (fabsf(delta) > 0) {
        DUOJI_progress[ID-1] += 0.001f / (total_time * 0.001f);//将时间间隔统一转换为以毫秒为单位的进度增量
        if (DUOJI_progress[ID-1] > 1.0f) DUOJI_progress[ID-1] = 1.0f;
//			  printf("%.2f\n",total_time);
        // S曲线缓动计算
        float eased_progress = 0.5f * (1.0f - MY_DUOJIcosf(DUOJI_progress[ID-1]));
        *current_angle = Start_angle[ID-1] + STEP[ID-1] * eased_progress;
			
    }else {
        DUOJI_progress[ID-1] = 0;
    }
    
    // 限制角度范围
    *current_angle = fmaxf(0.0f, fminf(180.0f, *current_angle));
    // 转换为PWM占空比 
     pulse[ID-1] = (uint16_t)(*current_angle * ANGLE_TO_DUTY) + 500;
    
    // 获取对应的定时器和通道
    uint8_t timer_num = (TIM_CHAnnel >> 8) & 0xF;
    uint8_t channel_num = TIM_CHAnnel & 0xFF;
    
    switch (timer_num) {
        case 0x01:
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1 + (channel_num - 1) * 4, pulse[ID-1]);
            break;
        case 0x02:
            __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1 + (channel_num - 1) * 4, pulse[ID-1]);
            break;
    }
    

    //调试信息输出
    // printf("ID:%d",ID);
    //printf("progress:%.3f,current_angle:%.3f,target_angle:%.3f,delta:%.3f\n",progress,*current_angle,target_angle,delta);
//     printf("%.2f\n",progress[RIGHT_THIGH_ID]);
    // printf("%d\n",channel_num);
//		printf("%.2f\n",total_time);
//			printf ("%.3f\n",progress);
//printf("%.2f\n",delta);
    return *current_angle;

}





//舵机零位校准
void Servo_zero_angle_Calibrate(void )
{  

    if(Hip_calibrate_flag==0)
    {
    //先初始化大腿跟髋关节的PWM通道
    MOVE_TARget[RIGHT_HIP_ID] = 1.0f;
    MOVE_TIME[RIGHT_HIP_ID]   = 5000;
    MOVE_TARget[LEFT_HIP_ID] = 180.0f;
    MOVE_TIME[LEFT_HIP_ID]   = 5000;

        if (calibrate_flag[RIGHT_HIP_ID] )
    {
        //校准完成
        Servo_zero_angle[RIGHT_HIP_ID] = CURRENT_ANGLE[RIGHT_HIP_ID] -4 ; 
        MOVE_TARget[RIGHT_HIP_ID] = Servo_zero_angle[RIGHT_HIP_ID] + RIGHT_hip_SW_angle; //回到水平位置
        MOVE_TIME[RIGHT_HIP_ID] = 1000.0f;
        osDelay(1000); //等待1秒，确保校准完成	
        Hip_calibrate_flag = 1; 
			while(1);
    }

//       if (calibrate_flag[LEFT_HIP_ID] )
//   {
//       //校准完成    
//       Servo_zero_angle[LEFT_HIP_ID] = CURRENT_ANGLE[LEFT_HIP_ID] -5 ;    
//       MOVE_TARget[LEFT_HIP_ID] = Servo_zero_angle[LEFT_HIP_ID] - LEFT_hip_SW_angle; //回到水平位置
//       MOVE_TIME[LEFT_HIP_ID] = 500.0f;   
//       osDelay(1000); //等待1秒，确保校准完成	
//       Hip_calibrate_flag = 1;    
//			while(1);   
//   }  
    
	
    }

    //髋关节校准结束，初始化大腿PWM通道，大腿校准
    //大腿校准完成之后小腿也会校准完成
    if( Thigh_calibrate_flag==0 && Hip_calibrate_flag == 1)
    {
    // MOVE_TARget[RIGHT_THIGH_ID] = 0.0f;
    // MOVE_TIME[RIGHT_THIGH_ID] = 3000.0f;
    // if (calibrate_flag[RIGHT_THIGH_ID] )
    // {
    //     //校准完成
    //     Servo_zero_angle[RIGHT_THIGH_ID] = CURRENT_ANGLE[RIGHT_THIGH_ID] + 5; 
    //     MOVE_TARget[RIGHT_THIGH_ID] = CURRENT_ANGLE[RIGHT_THIGH_ID] + 5 +RIGHT_thigh_SW_angle; //回到水平位置
    //     MOVE_TIME[RIGHT_THIGH_ID] = 1000.0f;
    //     osDelay(1000); //等待1秒，确保校准完成		
	  // HAL_TIM_PWM_Start(&htim1,Right_lower_leg_PWMchannel); //初始化过一次的话就会自动退出
	  // 	Thigh_calibrate_flag = 1; 		
    // }




    MOVE_TARget[LEFT_THIGH_ID] = 180.0f;
    MOVE_TIME[LEFT_THIGH_ID] = 3000.0f;

        if (calibrate_flag[LEFT_THIGH_ID] )
    {  
        //校准完成
        Servo_zero_angle[LEFT_THIGH_ID] = CURRENT_ANGLE[LEFT_THIGH_ID] -13 ; 
        MOVE_TARget[LEFT_THIGH_ID] = Servo_zero_angle[LEFT_THIGH_ID] - LEFT_thigh_SW_angle; //回到水平位置
        MOVE_TIME[LEFT_THIGH_ID] = 1000.0f;
        osDelay(1000); //等待1秒，确保校准完成	
			  HAL_TIM_PWM_Start(&htim1,Left_lower_leg_PWMchannel); //初始化小腿 
				MOVE_TARget[LEFT_LOWER_LEG_ID]  = 180.0f;
				MOVE_TIME[LEFT_LOWER_LEG_ID] = 3000.0f;
				osDelay(2000);
        Thigh_calibrate_flag = 1; 
    }


    

    }

//     //大腿校准完成，初始化小腿PWM通道，小腿校准
//     if(Thigh_calibrate_flag ==1 && Lower_leg_calibrate_flag==0)
//    {
//				
//		// 		MOVE_TARget[RIGHT_LOWER_LEG_ID]  = 5.0f;
//		// 		MOVE_TIME[RIGHT_LOWER_LEG_ID] = 3000.0f;
//        //      osDelay(2000);
//        //    if (calibrate_flag[RIGHT_LOWER_LEG_ID] )
//        // {
//        //     //校准完成
//        //     Servo_zero_angle[RIGHT_LOWER_LEG_ID] = CURRENT_ANGLE[RIGHT_LOWER_LEG_ID] + 9; 
//        //     MOVE_TARget[RIGHT_LOWER_LEG_ID] = CURRENT_ANGLE[RIGHT_LOWER_LEG_ID] + 9 ; //回到水平位置
//        //     MOVE_TIME[RIGHT_LOWER_LEG_ID] = 1000.0f; 
//        //     osDelay(1000);
//        //     Lower_leg_calibrate_flag = 1; 
//        // }
//			
//			
//				MOVE_TARget[LEFT_LOWER_LEG_ID]  = 180.0f;
//				MOVE_TIME[LEFT_LOWER_LEG_ID] = 3000.0f;
//				
//           if (calibrate_flag[LEFT_LOWER_LEG_ID] )
//        {
//            //校准完成
//            Servo_zero_angle[LEFT_LOWER_LEG_ID] = CURRENT_ANGLE[LEFT_LOWER_LEG_ID] -9 ; 
//            MOVE_TARget[LEFT_LOWER_LEG_ID] = CURRENT_ANGLE[LEFT_LOWER_LEG_ID]  -9; //回到水平位置
//            MOVE_TIME[LEFT_LOWER_LEG_ID] = 1000.0f; 
//            osDelay(1000);
//            Lower_leg_calibrate_flag = 1; 
//        }


//    }
		
		
    // //髋关节校准未完成
    // if(!Hip_calibrate_flag)
    // {

    // /*髋关节校准*/
    //     if (calibrate_flag[RIGHT_HIP_ID] )
    //     {
    //         //校准完成
    //         Hip_calibrate_flag = 1;
    //     }
    // }

}


//舵机参数初始化
void Servo_Parameter_Init(void)
{
	
	    CURRENT_ANGLE[RIGHT_HIP_ID] =Right_hip_ZERO_angle + RIGHT_hip_SW_angle  ;
      CURRENT_ANGLE[RIGHT_LOWER_LEG_ID] =80.51 - 30  + Right_lower_leg_ZERO_angle + 0; 
			CURRENT_ANGLE[RIGHT_THIGH_ID]  = Right_thigh_ZERO_angle + 65.94 + 0;


      CURRENT_ANGLE[LEFT_HIP_ID] =Left_hip_ZERO_angle  - LEFT_hip_SW_angle;

      CURRENT_ANGLE[LEFT_LOWER_LEG_ID] = 30 - 80.51f  + Left_lower_leg_ZERO_angle - 0; 
			CURRENT_ANGLE[LEFT_THIGH_ID]  = Left_thigh_ZERO_angle - 65.94 - 0;
	

}




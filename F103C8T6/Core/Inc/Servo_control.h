#ifndef SERVO_CONTROL_H
#define SERVO_CONTROL_H

#include "main.h"
#include "string.h"
#include "stdio.h"
#include "stm32f1xx_hal.h"
#include "tim.h"
#include "adc_m.h"

#define My_Arr    															20000
#define ANGLE_TO_DUTY														11.111f
extern  int      calibrate_flag[ADC1_CHANNEL_CNT] ;    //校准值，舵机零位
extern     float       Left_thigh_angle  				   				    ;    //左大腿  
extern     float       Left_lower_leg_angle    		    					;	  //左小腿
extern     float       Right_thigh_angle     	 		    				;	  //右大腿		
extern     float       Right_lower_leg_angle   	 	    					;	  //右小腿    
extern     float       Left_hip_angle     	 	            				;	  //左髋	
extern     float       Right_hip_angle     	 	       				        ;	  //右髋


extern uint16_t			Left_thigh_channel              ;       //左大腿舵机通道
extern uint16_t			Left_lower_leg_channel    		;		//左小腿舵机通道
extern uint16_t			Right_thigh_channel             ;		//右大腿舵机通道		
extern uint16_t			Right_lower_leg_channel   	 	;		//右小腿舵机通道		
extern uint16_t			Right_hip_channel     			;		//右髋舵机通道		
extern uint16_t			Left_hip_channel     	 	    ;		//左髋舵机通道	



//@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@ADC采样ID定义@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
#define       AIN_Channel     	 	   					0				
#define       BIN_Channel     	 	   					1	
#define       CIN_Channel     	 	   					2	
#define       DIN_Channel     	 	   					3	
#define       EIN_Channel     	 	   				    4	
#define       FIN_Channel     	 	   				    5

//@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@舵机ID定义@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
#define       LEFT_LOWER_LEG_ID     	 	   				DIN_Channel			
#define       LEFT_THIGH_ID     	   				    	EIN_Channel
#define       LEFT_HIP_ID     	 	       					FIN_Channel

#define       RIGHT_THIGH_ID     	 	   					  BIN_Channel	
#define       RIGHT_LOWER_LEG_ID     	   					CIN_Channel	
#define       RIGHT_HIP_ID     	 	       					AIN_Channel	

/////////////////////////////////////手动保存舵机零角度/////////////////////////////////////
#define     Left_thigh_ZERO_angle  	              152.146759
#define     Left_lower_leg_ZERO_angle             171
#define     Right_thigh_ZERO_angle                34      //大腿向上与水平面夹角为65.94度
#define     Right_lower_leg_ZERO_angle            18      //大小腿之间的夹角是80.51度
#define     Left_hip_ZERO_angle                   134
#define     Right_hip_ZERO_angle                  43.5        

    	
extern const float    RIGHT_hip_SW_angle  ;
extern const float    RIGHT_Lower_leg_SW_angle  ;
extern const float    RIGHT_thigh_SW_angle  ;
extern const float    LEFT_hip_SW_angle  ;
extern const float    LEFT_thigh_SW_angle ;
extern const float    LEFT_Lower_leg_SW_angle ;
 	
    	 	
    	 	    	 	   			    	
 

#define			  Left_hip_PWMchannel     	 	            		  	TIM_CHANNEL_4		//左髋舵机通道
#define				Left_thigh_PWMchannel   				    				    TIM_CHANNEL_3   //左大腿舵机通道
#define				Left_lower_leg_PWMchannel    		        				TIM_CHANNEL_2		//左小腿舵机通道

#define				Right_thigh_PWMchannel     	 		    				    TIM_CHANNEL_2		//右大腿舵机通道		
#define				Right_lower_leg_PWMchannel   	 	        				TIM_CHANNEL_1		//右小腿舵机通道		
#define				Right_hip_PWMchannel     			        					TIM_CHANNEL_1		//右髋舵机通道		

//存储舵机目标角度
extern  float  MOVE_TARget[6] ;
//存储当前角度
extern  float  CURRENT_ANGLE[6] ;
//存储转动时间
extern  uint16_t  MOVE_TIME[6] ;
// 初始零角度
extern  float Servo_zero_angle[6] ;
//当前占空比
extern uint16_t pulse[6] ;


void Serial_turn ( uint8_t ID,  float angle);                           //串口控制舵机转动
float Servo_SCurve_Control(uint16_t TIM_CHAnnel, float target_angle,
                          float *current_angle, float total_time,int ID);    //S曲线平滑转动
void Servo_Parameter_Init(void);
//舵机零位校准
void Servo_zero_angle_Calibrate(void );
float Servo_Direct_Control(uint16_t TIM_CHAnnel, float target_angle, 
                          float *current_angle);

#endif


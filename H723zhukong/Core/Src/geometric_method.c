#include "geometric_method.h"
#include "math.h"
#include "data_table.h"
#include <math.h>
#include  "stdio.h"
#include "arm_math.h"


gait_angle  LEG1_angle = {0};
gait_angle  LEG2_angle = {0};
gait_angle  LEG3_angle = {0};
gait_angle  LEG4_angle = {0};
gait_XY     LEG1_XY     = {0};
gait_XY     LEG2_XY     = {0};
gait_XY     LEG3_XY     = {0};
gait_XY     LEG4_XY     = {0};
gait_YZ     LEG1_YZ     = {0};
gait_YZ     LEG2_YZ     = {0};
gait_YZ     LEG3_YZ     = {0};
gait_YZ     LEG4_YZ     = {0};
SIMPLE_NOW_GAIT_XY  NOW_GAIT_XY_LEG1 = {0};
SIMPLE_NOW_GAIT_XY  NOW_GAIT_XY_LEG2 = {0};
SIMPLE_NOW_GAIT_XY  NOW_GAIT_XY_LEG3 = {0};
SIMPLE_NOW_GAIT_XY  NOW_GAIT_XY_LEG4 = {0};
virtual_force       VIRTUAL_FORCE_LEG1 = {0};
virtual_force       VIRTUAL_FORCE_LEG2 = {0};
virtual_force       VIRTUAL_FORCE_LEG3 = {0};
virtual_force       VIRTUAL_FORCE_LEG4 = {0};



float BIG_lenth = 0;        //大腿长度即L1       厘米为单位
float small_lenth = 0;      //小腿长度即L2       厘米为单位
float BODY_height = 0;        //身体离地高度     厘米为单位    以大腿舵机中心为起点
float foot_top_height = 0;    //足端离地高度     厘米为单位    以足中心为起点
float MOVE_DISTANCE  = 0;     //拖动距离
float MOVE_DISTENCE_START = 0;
float MOVE_DISTENCE_END   = 0;

float BIG_lenth2   = 0;    //L1的平方        
float small_lenth2 = 0;    //L2的平方


//实际控制的时候要考虑占空比
float  raise_step = 1.5;
float  translation_step = 0.7;



void SIMPLE_NOW_GAIT_XY_Init(SIMPLE_NOW_GAIT_XY*  NOW_gait_XY)
{
    NOW_gait_XY->foot_top_X = 0;
    NOW_gait_XY->foot_top_Y = 0;
    NOW_gait_XY->last_error_X = 0;
    NOW_gait_XY->last_error_Z = 0;
    NOW_gait_XY->foot_top_Z   = 0;
    NOW_gait_XY->last_error_Y = 0;
}


/**
 * @brief 步态参数初始化
 * @details 初始化步态参数，控制行走特征
 * @param 步态模式，迈步距离X，抬起最高点Y，步态分辨率
 */

void  GAIT_Init( float  X_start ,float X_end , float Y, float resolution ,  float height, float  L1 , float L2 ,
    float translate_BEGAIN , float translate_END ,float translate_HIGH ,float Hip_lenth)   //三自由度中begain是正end是负，而8自由度开始是负结束是正
{

	BODY_height  = height;
	MOVE_DISTENCE_START = X_start ;
	MOVE_DISTENCE_END   = X_end;
	MOVE_DISTANCE = X_end - X_start;
		
	foot_top_height = Y;
	BIG_lenth    = L1;
	small_lenth  = L2;
    BIG_lenth2   = L1*L1;
    small_lenth2 = L2*L2;


    LEG1_XY.resolution = resolution;
    LEG1_XY.thase_LEG  =0;
    LEG1_XY.distance_from_foot_end_to_thigh  = height;


    LEG2_XY.resolution = resolution;
    LEG2_XY.thase_LEG  =0;
    LEG2_XY.distance_from_foot_end_to_thigh  = height;



    LEG3_XY.resolution = resolution;
    LEG3_XY.thase_LEG  =0;
    LEG3_XY.distance_from_foot_end_to_thigh  = height;



    LEG4_XY.resolution = resolution;//最小为1
    LEG4_XY.thase_LEG  =0;
    LEG4_XY.distance_from_foot_end_to_thigh  = height;



    LEG1_YZ.resolution = resolution;
    LEG1_YZ.thase_LEG  =0;
    LEG1_YZ.translate_BEGAIN = translate_BEGAIN;
    LEG1_YZ.translate_END = translate_END;
    LEG1_YZ.translate_step_length =  translate_END - translate_BEGAIN;
    LEG1_YZ.translate_high = translate_HIGH;
    LEG1_YZ.hip_lenth = Hip_lenth;
    LEG1_YZ.hip_lenth_2 = Hip_lenth*Hip_lenth;
		LEG1_YZ.LEG_BODY_height = height;

    LEG2_YZ.resolution = resolution;
    LEG2_YZ.thase_LEG  =0;
    LEG2_YZ.translate_BEGAIN = translate_BEGAIN;
    LEG2_YZ.translate_END = translate_END;
    LEG2_YZ.translate_step_length = translate_END - translate_BEGAIN;
    LEG2_YZ.translate_high = translate_HIGH;
    LEG2_YZ.hip_lenth = Hip_lenth;
    LEG2_YZ.hip_lenth_2 = Hip_lenth*Hip_lenth;
		LEG2_YZ.LEG_BODY_height = height;

    LEG3_YZ.resolution = resolution;
    LEG3_YZ.thase_LEG  =0;
    LEG3_YZ.translate_BEGAIN = translate_BEGAIN;
    LEG3_YZ.translate_END = translate_END;
    LEG3_YZ.translate_step_length =  (translate_END - translate_BEGAIN);
    LEG3_YZ.translate_high = translate_HIGH;
    LEG3_YZ.hip_lenth = Hip_lenth;
    LEG3_YZ.hip_lenth_2 = Hip_lenth*Hip_lenth;
		LEG3_YZ.LEG_BODY_height = height;


    LEG4_YZ.resolution = resolution;
    LEG4_YZ.thase_LEG  =0;
    LEG4_YZ.translate_BEGAIN = translate_BEGAIN;
    LEG4_YZ.translate_END = translate_END;
    LEG4_YZ.translate_step_length =  (translate_END - translate_BEGAIN);
    LEG4_YZ.translate_high = translate_HIGH;
    LEG4_YZ.hip_lenth = Hip_lenth;
    LEG4_YZ.hip_lenth_2 = Hip_lenth*Hip_lenth;
		LEG4_YZ.LEG_BODY_height = height;
}







//////////////////////////////////////VMC_CONTROL//////////////////////////////////////

// 固定参数定义
#define L1 0.21f      // 大腿长度 (m)
#define L2 0.21f      // 小腿长度 (m)
#define DENSITY 1000 // 连杆密度 (kg/m^3)
#define WIDTH 0.015f  // 连杆宽度 (m)
#define HEIGHT 0.0075f // 连杆高度 (m)
#define G 9.81f       // 重力加速度 (m/s^2)

// 计算连杆质量
#define M1 0.05f  // 大腿质量(kg)0.05
#define M2 0.05f  // 小腿质量(kg)0.05 
float   VMC_theta[2] = {0};
/**
 * @brief 雅可比矩阵计算
 * @param theta 关节角度数组 [theta1, theta2]
 * @param fx 虚拟力在x方向的分量
 * @param fz 虚拟力在z方向的分量
 * @param toa1 输出：关节1的力矩
 * @param toa2 输出：关节2的力矩
 */
void Jocob(float theta[], float fx, float fz, float *toa1, float *toa2 ) {
//    float theta1 = theta[0];
//    float theta2 = theta[1];
//    // 计算雅可比矩阵
//    float J[2][2];
//    J[0][0] = -L1 * sinf(theta1) - L2 * arm_sin_f32(theta1 + theta2);
//    J[0][1] = -L2 * arm_sin_f32(theta1 + theta2);
//    J[1][0] = L1 * arm_cos_f32(theta1) + L2 * arm_cos_f32(theta1 + theta2);
//    J[1][1] = L2 * arm_cos_f32(theta1 + theta2);
//    
//    // 计算重力补偿力矩
//    float tau_g1 = (M1 * L1 / 2.0f + M2 * L1) * G * arm_cos_f32(theta1) + M2 * L2 / 2.0f * G * arm_cos_f32(theta1 + theta2);
//    float tau_g2 = M2 * L2 / 2.0f * G * arm_cos_f32(theta1 + theta2);
//    
//    // 计算VMC力矩映射
//    float F[2] = {fx, fz};
//    float tau_vmc[2];
//    tau_vmc[0] = J[0][0] * F[0] + J[1][0] * F[1];  // J' * F
//    tau_vmc[1] = J[0][1] * F[0] + J[1][1] * F[1];
//    
//    // 最终输出力矩
//    *toa1 = tau_vmc[0] + tau_g1;
//    *toa2 = tau_vmc[1] + tau_g2;
}







/**
 * @brief 虚拟力PD控制
 * @param ANY_gait_XY  规划得到的足端坐标
 * @param NOW_gait_XY  当前足端坐标
 * @param VIRTUAL_FORCE  虚拟力
 */
void PD_CONTROL(gait_XY*  ANY_gait_XY , SIMPLE_NOW_GAIT_XY*  NOW_gait_XY  ,virtual_force*  VIRTUAL_FORCE) 
{
    //转换成m为单位
    ANY_gait_XY->foot_top_X *= 0.01f;
    ANY_gait_XY->foot_top_Y *= 0.01f;
    //y方向转换成坐标系单位
    ANY_gait_XY->foot_top_Y += -0.300f;


    float Kp = 16;//2.0
    float Kd = 1.4f; //0.8
    float dt = 0.004f; // 假设时间步长为5ms
    float   d_error_X = 0;
    float   d_error_Z = 0;
    float error_X = ANY_gait_XY->foot_top_X - NOW_gait_XY->foot_top_X;
    float error_Z = ANY_gait_XY->foot_top_Y - NOW_gait_XY->foot_top_Z;
    if(NOW_gait_XY->last_error_X!= 0)
    {
     d_error_X = (error_X - NOW_gait_XY->last_error_X) / dt;
     d_error_Z = (error_Z - NOW_gait_XY->last_error_Z) / dt;

    }else 
    {
        d_error_X = 0;
        d_error_Z = 0;
    }
    float force_X = Kp * error_X + Kd * d_error_X;
    float force_Z = Kp * error_Z + Kd * d_error_Z;
    NOW_gait_XY->last_error_X = error_X;
    NOW_gait_XY->last_error_Z = error_Z;

    VIRTUAL_FORCE->force_X = force_X;
    VIRTUAL_FORCE->force_Z = force_Z;
 
    //调试信息区
    // printf("error_X:%.3f,error_Z:%.3f\n",error_X,error_Z);
    // printf("d_error_X:%.3f,d_error_Z:%.3f\n",d_error_X,d_error_Z);
    // printf("force_X:%.3f,force_Z:%.3f\n",force_X,force_Z);
    // printf("foot_top_X:%.3f,foot_top_Z:%.3f\n",NOW_gait_XY->foot_top_X,NOW_gait_XY->foot_top_Z);
    // printf("%.3f,%.3f\n",ANY_gait_XY->foot_top_X,ANY_gait_XY->foot_top_Y);

}








/**
 * 二自由度运动学正解
 * @param theta1 第一个关节的角度 (弧度)
 * @param theta2 第二个关节的角度 (弧度)
 * @param NOW_gait_XY 输出参数：步态坐标结构体指针，用于存储计算得到的脚部位置
 * 
 * 功能：根据两个关节的角度计算机器人脚部的笛卡尔坐标位置
 * 应用场景：用于机器人腿部运动学计算，获取脚部末端位置
 * 其中：
 * L1 - 第一根连杆的长度 (大腿长度，单位：米)
 * L2 - 第二根连杆的长度 (小腿长度，单位：米)
 * foot_top_X - 脚部在X轴方向的位置 (单位：米)
 * foot_top_Z - 脚部在Z轴方向的位置 (单位：米)
 */

 
void two_degree_of_freedom_forward_kinematics(float theta1, float theta2, SIMPLE_NOW_GAIT_XY*  NOW_gait_XY) 
{
//    NOW_gait_XY->foot_top_X = L1 * arm_cos_f32(theta1) + L2 * arm_cos_f32(theta1 + theta2);
//    NOW_gait_XY->foot_top_Z = L1 * sinf(theta1) + L2 * sinf(theta1 + theta2);
}


                 


//////////////////////////////////////运动学逆解//////////////////////////////////////


//通过足端坐标，计算角度

void   Inverse_Kinematics( gait_angle*  ANY_gait_angle , gait_XY*  ANY_gait_XY  )	//运动学逆解
{
    ANY_gait_angle->angle_fai = MY_IK_acos(
                                            (BIG_lenth2 
                                            +(ANY_gait_XY->foot_top_X2  
                                            +(ANY_gait_XY->foot_top_h_Y_2)) 
                                            - small_lenth2)
                                            /(2*BIG_lenth *sqrtf((ANY_gait_XY->foot_top_X2 
                                            +ANY_gait_XY->foot_top_h_Y_2) ) 
                                             )
                                        );


	//小腿是angle_P
    ANY_gait_angle->angle_P  = MY_IK_acos(    (BIG_lenth2 + small_lenth2 -(ANY_gait_XY->foot_top_X2 + ANY_gait_XY->foot_top_h_Y_2)  )
                                           /(2 * BIG_lenth * small_lenth)
                                          )   ;      
 
	
    //大腿是theata_1
    ANY_gait_angle->theata_1 = 180 - ANY_gait_angle->angle_fai -     atan2f( (ANY_gait_XY->distance_from_foot_end_to_thigh -  ANY_gait_XY->foot_top_Y),  ANY_gait_XY->foot_top_X ) *57.296f        ;


//    printf("%f,%f,%f,%f,%f\n" , BIG_lenth,small_lenth, ANY_gait_angle->theata_1  , ANY_gait_angle->angle_P  ,  ANY_gait_XY.distance_from_foot_end_to_thigh );  //


}


void Inverse_Kinematics_add_YZ( gait_angle*  ANY_gait_angle , gait_YZ*  ANY_gait_YZ ,gait_XY*  ANY_gait_XY )	//运动学逆解第三自由度补充
{
    float angle_R1 = 0;
    float angle_R2 = 0;
	ANY_gait_XY->distance_from_foot_end_to_thigh = sqrtf(ANY_gait_YZ->foot_top_h_Z_2  + 2*ANY_gait_YZ->foot_top_Y *ANY_gait_YZ->hip_lenth   + ANY_gait_YZ->foot_top_Y *ANY_gait_YZ->foot_top_Y ) ;
    angle_R1 = -atan2f(ANY_gait_YZ->hip_lenth,ANY_gait_XY->distance_from_foot_end_to_thigh ) ; //横向移动的角度，正数表示向右移动，负数表示向左移动
    angle_R2 = -atan2f(ANY_gait_YZ->foot_top_Y + ANY_gait_YZ->hip_lenth, ANY_gait_YZ->LEG_BODY_height - ANY_gait_YZ->foot_top_Z) ; //横向移动的角度，正数表示向右移动，负数表示向左移动
    ANY_gait_angle->translate_angle_R = angle_R2 - angle_R1; //横向移动的角度，正数表示向右移动，负数表示向左移动 
    ANY_gait_angle->translate_angle_R =ANY_gait_angle->translate_angle_R *57.296f;
//		printf("%.2f,%.2f,%.2f\n",ANY_gait_angle->translate_angle_R ,angle_R1,angle_R2 );
}



/// @brief  占空比清零
/// @return 
void GET_GAIT_theta_CLEAR( void )
{
    LEG1_XY.thase_LEG = 0;
    LEG2_XY.thase_LEG = 0;
    LEG3_XY.thase_LEG = 0;
    LEG4_XY.thase_LEG = 0; 

    LEG1_YZ.thase_LEG = 0;
    LEG2_YZ.thase_LEG = 0;
    LEG3_YZ.thase_LEG = 0;
    LEG4_YZ.thase_LEG = 0; 

}



/**
 * @brief   计算步态的X-Y轴轨迹坐标（脚步端点位置）
 * @param   ANY_gait_XY: 步态参数与轨迹结果结构体指针
 *          - 输入参数：thase_LEG（当前步态相位）、resolution（相位步长）、foot_top_height（抬脚高度）、MOVE_DISTANCE（单步移动距离）
 *          - 输出参数：foot_top_X（脚步X轴坐标）、foot_top_Y（脚步Y轴坐标）
 * @note    步态周期划分为两个阶段：抬起阶段（0~T_2_4）和拖动阶段（T_2_4~T_4_4）
 *          - T_Start：步态起始相位（默认0）
 *          - T_2_4：抬起阶段结束相位（周期的1/2，对应200单位，与代码中200.0f匹配）
 *          - T_4_4：步态完整周期相位（周期的1，对应400单位）
 *          - MY_GAITCOSf：自定义正弦函数（用于生成平滑的抬起轨迹）
 */

 void GET_GAIT_XY(gait_XY*  ANY_gait_XY   , int dirction )
 {
    if (dirction == forward)
    { 
     if (ANY_gait_XY->First_step_flag  ==  0)//还在第一步，只有第一步要特殊处理
     {
        if (ANY_gait_XY->thase_LEG  >= T_2_4)   
        {
         ANY_gait_XY->thase_LEG      +=  translation_step *  ANY_gait_XY->resolution;
        }else
        {
             ANY_gait_XY->thase_LEG      +=  raise_step *  ANY_gait_XY->resolution;
        }

         if(ANY_gait_XY->thase_LEG   < T_2_4)                                     //还在抬起周期
         {
             ANY_gait_XY->foot_top_Y = 0.7f*foot_top_height* MY_GAITCOSf(ANY_gait_XY->thase_LEG);
             ANY_gait_XY->foot_top_X = MOVE_DISTENCE_END * MY_GAITCOSf_0_1(ANY_gait_XY->thase_LEG) ;//此时位于原点，要向前迈一个end
         }else                                                                     //在拖动周期
         {
             ANY_gait_XY->foot_top_Y = 0;
             ANY_gait_XY->foot_top_X = MOVE_DISTANCE * (1.0f - (ANY_gait_XY->thase_LEG - T_2_4) /200.0f)  +  MOVE_DISTENCE_START;//拖动回来的时候就是正常的拖动了
         }
         ANY_gait_XY->foot_top_X2    = ANY_gait_XY->foot_top_X * ANY_gait_XY->foot_top_X;
         ANY_gait_XY->foot_top_h_Y_2 = (ANY_gait_XY->distance_from_foot_end_to_thigh - ANY_gait_XY->foot_top_Y) *(ANY_gait_XY->distance_from_foot_end_to_thigh- ANY_gait_XY->foot_top_Y) ;
		
				 if (ANY_gait_XY->thase_LEG  >= T_4_4)   
         {
             ANY_gait_XY->thase_LEG  = T_Start;//完成了一个周期
             ANY_gait_XY->foot_top_X = MOVE_DISTENCE_START;
             ANY_gait_XY->foot_top_Y = 0;
             ANY_gait_XY->First_step_flag  =  1;
         }
     }else  
		 
     {
         if (ANY_gait_XY->thase_LEG  >= T_4_4)   
         {
             ANY_gait_XY->thase_LEG  = T_Start;//完成了一个周期
             ANY_gait_XY->foot_top_X = MOVE_DISTENCE_START;
             ANY_gait_XY->foot_top_Y = 0;
         }

        if (ANY_gait_XY->thase_LEG  >= T_2_4)   
        {
         ANY_gait_XY->thase_LEG      +=  translation_step *  ANY_gait_XY->resolution;
        }else
        {
             ANY_gait_XY->thase_LEG      +=  raise_step *  ANY_gait_XY->resolution;
        }
         if(ANY_gait_XY->thase_LEG   < T_2_4)                                     //还在抬起周期
         {
             ANY_gait_XY->foot_top_Y = foot_top_height* MY_GAITCOSf(ANY_gait_XY->thase_LEG);
             ANY_gait_XY->foot_top_X = MOVE_DISTANCE * MY_GAITCOSf_0_1(ANY_gait_XY->thase_LEG)   + MOVE_DISTENCE_START ;
         }else                                                                     //在拖动周期
         {
             ANY_gait_XY->foot_top_Y = 0;
             ANY_gait_XY->foot_top_X = MOVE_DISTANCE * (1.0f - (ANY_gait_XY->thase_LEG - T_2_4) /200.0f)   +  MOVE_DISTENCE_START;
         }
         ANY_gait_XY->foot_top_X2    = ANY_gait_XY->foot_top_X * ANY_gait_XY->foot_top_X;
         ANY_gait_XY->foot_top_h_Y_2 = (ANY_gait_XY->distance_from_foot_end_to_thigh - ANY_gait_XY->foot_top_Y) *(ANY_gait_XY->distance_from_foot_end_to_thigh- ANY_gait_XY->foot_top_Y) ;
     }
 }else if (dirction == backward)
 {
     if (ANY_gait_XY->First_step_flag  ==  0)//还在第一步，只有第一步要特殊处理
     {

        if (ANY_gait_XY->thase_LEG  >= T_2_4)   
        {
         ANY_gait_XY->thase_LEG      +=  raise_step *  ANY_gait_XY->resolution;
        }else
        {
             ANY_gait_XY->thase_LEG      +=  translation_step *  ANY_gait_XY->resolution;
        }
         ANY_gait_XY->backward_thase_LEG = T_4_4 - ANY_gait_XY->thase_LEG ;
         if(ANY_gait_XY->backward_thase_LEG   < T_2_4)                                     //还在抬起周期
         {
             ANY_gait_XY->foot_top_Y = 0.7f*foot_top_height* MY_GAITCOSf(ANY_gait_XY->backward_thase_LEG);
             ANY_gait_XY->foot_top_X = MOVE_DISTANCE * MY_GAITCOSf_0_1(ANY_gait_XY->backward_thase_LEG)  + MOVE_DISTENCE_START ;//此时位于原点，要向前迈一个end
         }else                                                                     //在拖动周期
         {
             ANY_gait_XY->foot_top_Y = 0;
             ANY_gait_XY->foot_top_X = MOVE_DISTENCE_END * (1.0f - (ANY_gait_XY->backward_thase_LEG - T_2_4) /200.0f)  ;//拖动回来的时候就是正常的拖动了
         }
         ANY_gait_XY->foot_top_X2    = ANY_gait_XY->foot_top_X * ANY_gait_XY->foot_top_X;
         ANY_gait_XY->foot_top_h_Y_2 = (ANY_gait_XY->distance_from_foot_end_to_thigh - ANY_gait_XY->foot_top_Y) *(ANY_gait_XY->distance_from_foot_end_to_thigh- ANY_gait_XY->foot_top_Y) ;
				 
		if (ANY_gait_XY->thase_LEG  >= T_4_4)   
         {
             ANY_gait_XY->thase_LEG  = T_Start;//完成了一个周期
             ANY_gait_XY->foot_top_X = MOVE_DISTENCE_START;
             ANY_gait_XY->foot_top_Y = 0;
             ANY_gait_XY->First_step_flag  =  1;
         }
     }else  
		 
     {
         if (ANY_gait_XY->thase_LEG  >= T_4_4)   
         {
             ANY_gait_XY->thase_LEG  = T_Start;//完成了一个周期
             ANY_gait_XY->foot_top_X = MOVE_DISTENCE_START;
             ANY_gait_XY->foot_top_Y = 0;
         }
        if (ANY_gait_XY->thase_LEG  >= T_2_4)   
        {
         ANY_gait_XY->thase_LEG      +=  raise_step *  ANY_gait_XY->resolution;
        }else
        {
             ANY_gait_XY->thase_LEG      +=  translation_step *  ANY_gait_XY->resolution;
        }
         ANY_gait_XY->backward_thase_LEG = T_4_4 - ANY_gait_XY->thase_LEG ;
         if(ANY_gait_XY->backward_thase_LEG   < T_2_4)                                     //还在抬起周期
         {
             ANY_gait_XY->foot_top_Y = foot_top_height* MY_GAITCOSf(ANY_gait_XY->backward_thase_LEG);
             ANY_gait_XY->foot_top_X = MOVE_DISTANCE * MY_GAITCOSf_0_1(ANY_gait_XY->backward_thase_LEG)  + MOVE_DISTENCE_START ;
         }else                                                                     //在拖动周期
         {
             ANY_gait_XY->foot_top_Y = 0;
             ANY_gait_XY->foot_top_X = MOVE_DISTANCE * (1.0f - (ANY_gait_XY->backward_thase_LEG - T_2_4) /200.0f)   +  MOVE_DISTENCE_START;
         }
         ANY_gait_XY->foot_top_X2    = ANY_gait_XY->foot_top_X * ANY_gait_XY->foot_top_X;
         ANY_gait_XY->foot_top_h_Y_2 = (ANY_gait_XY->distance_from_foot_end_to_thigh - ANY_gait_XY->foot_top_Y) *(ANY_gait_XY->distance_from_foot_end_to_thigh - ANY_gait_XY->foot_top_Y) ;
     }
 }
 
 
 
 
 }









/**
 * @brief   计算步态的X-Y-Z轴轨迹坐标（脚步端点位置）
 * @param   ANY_gait_XY: 步态参数与轨迹结果结构体指针
 *          - 输入参数：thase_LEG（当前步态相位）、resolution（相位步长）、foot_top_height（抬脚高度）、MOVE_DISTANCE（单步移动距离）
 *          - 输出参数：foot_top_X（脚步X轴坐标）、foot_top_Y（脚步Y轴坐标）
 * @note    步态周期划分为两个阶段：抬起阶段（0~T_2_4）和拖动阶段（T_2_4~T_4_4）
 *          - T_Start：步态起始相位（默认0）
 *          - T_2_4：抬起阶段结束相位（周期的1/2，对应200单位，与代码中200.0f匹配）
 *          - T_4_4：步态完整周期相位（周期的1，对应400单位）
 *          - MY_GAITCOSf：自定义正弦函数（用于生成平滑的抬起轨迹）
 */

 void GET_GAIT_YZ(gait_YZ*  ANY_gait_XYZ   , int dirction )
 {
    if (dirction == LEFT) 
    {
     if (ANY_gait_XYZ->First_step_flag  ==  0)//还在第一步，只有第一步要特殊处理
     {

         if (ANY_gait_XYZ->LEFT_translate_thase_LEG  >= T_2_4)   
        {
         ANY_gait_XYZ->LEFT_translate_thase_LEG      +=  translation_step *  ANY_gait_XYZ->resolution;
        }else
        {
             ANY_gait_XYZ->LEFT_translate_thase_LEG  +=  raise_step *  ANY_gait_XYZ->resolution;
        }
         if(ANY_gait_XYZ->LEFT_translate_thase_LEG   < T_2_4)                                     //还在抬起周期
         {
             ANY_gait_XYZ->foot_top_Z = ANY_gait_XYZ->translate_high* MY_GAITCOSf(ANY_gait_XYZ->LEFT_translate_thase_LEG);
             ANY_gait_XYZ->foot_top_Y = ANY_gait_XYZ->translate_END * MY_GAITCOSf_0_1(ANY_gait_XYZ->LEFT_translate_thase_LEG)   ;//此时位于原点，要向前迈一个end
         }else                                                                     //在拖动周期
         {
             ANY_gait_XYZ->foot_top_Z = 0;
             ANY_gait_XYZ->foot_top_Y = ANY_gait_XYZ->translate_step_length * (1.0f - (ANY_gait_XYZ->LEFT_translate_thase_LEG - T_2_4) /200.0f)  +  ANY_gait_XYZ->translate_BEGAIN;//拖动回来的时候就是正常的拖动了
         }
         ANY_gait_XYZ->foot_top_h_Z_2 = (ANY_gait_XYZ->LEG_BODY_height - ANY_gait_XYZ->foot_top_Z) *(ANY_gait_XYZ->LEG_BODY_height - ANY_gait_XYZ->foot_top_Z)  ;
				 
				 if (ANY_gait_XYZ->LEFT_translate_thase_LEG  >= T_4_4) 
         {
             ANY_gait_XYZ->LEFT_translate_thase_LEG  = T_Start;//完成了一个周期
             ANY_gait_XYZ->foot_top_Y = ANY_gait_XYZ->translate_BEGAIN;
             ANY_gait_XYZ->foot_top_Z = 0;
             ANY_gait_XYZ->First_step_flag  =  1;
         }
     }else  
	
     {
         if (ANY_gait_XYZ->LEFT_translate_thase_LEG  >= T_4_4)   
         {
             ANY_gait_XYZ->LEFT_translate_thase_LEG  = T_Start;//完成了一个周期
             ANY_gait_XYZ->foot_top_Y =  ANY_gait_XYZ->translate_BEGAIN;
             ANY_gait_XYZ->foot_top_Z = 0;
         }


         if (ANY_gait_XYZ->LEFT_translate_thase_LEG  >= T_2_4)   
        {
         ANY_gait_XYZ->LEFT_translate_thase_LEG      +=  translation_step *  ANY_gait_XYZ->resolution;
        }else
        {
             ANY_gait_XYZ->LEFT_translate_thase_LEG  +=  raise_step *  ANY_gait_XYZ->resolution;
        }
         if(ANY_gait_XYZ->LEFT_translate_thase_LEG   < T_2_4)                                     //还在抬起周期
         {
             ANY_gait_XYZ->foot_top_Z = ANY_gait_XYZ->translate_high* MY_GAITCOSf(ANY_gait_XYZ->LEFT_translate_thase_LEG);
             ANY_gait_XYZ->foot_top_Y = ANY_gait_XYZ->translate_step_length * MY_GAITCOSf_0_1(ANY_gait_XYZ->LEFT_translate_thase_LEG)  + ANY_gait_XYZ->translate_BEGAIN ;
         }else                                                                     //在拖动周期
         {
             ANY_gait_XYZ->foot_top_Z = 0;
             ANY_gait_XYZ->foot_top_Y = ANY_gait_XYZ->translate_step_length * (1.0f - (ANY_gait_XYZ->LEFT_translate_thase_LEG - T_2_4) /200.0f)   +  ANY_gait_XYZ->translate_BEGAIN ;
         }
         ANY_gait_XYZ->foot_top_h_Z_2 = (ANY_gait_XYZ->LEG_BODY_height - ANY_gait_XYZ->foot_top_Z) *(ANY_gait_XYZ->LEG_BODY_height - ANY_gait_XYZ->foot_top_Z)  ;
     }
 }else if (dirction == RIGHT)
 {
     if (ANY_gait_XYZ->First_step_flag  ==  0)//还在第一步，只有第一步要特殊处理
     {
        
        if (ANY_gait_XYZ->thase_LEG  >= T_2_4)   
        {
         ANY_gait_XYZ->thase_LEG      +=  raise_step *  ANY_gait_XYZ->resolution;
        }else
        {
             ANY_gait_XYZ->thase_LEG      +=  translation_step *  ANY_gait_XYZ->resolution;
        }
         ANY_gait_XYZ->RIGHT_translate_thase_LEG = T_4_4 - ANY_gait_XYZ->thase_LEG ;


         if(ANY_gait_XYZ->RIGHT_translate_thase_LEG   < T_2_4)                                     //还在抬起周期
         {
             ANY_gait_XYZ->foot_top_Z = ANY_gait_XYZ->translate_high* MY_GAITCOSf(ANY_gait_XYZ->RIGHT_translate_thase_LEG);
             ANY_gait_XYZ->foot_top_Y = ANY_gait_XYZ->translate_step_length * MY_GAITCOSf_0_1(ANY_gait_XYZ->RIGHT_translate_thase_LEG)  + ANY_gait_XYZ->translate_BEGAIN  ;//此时位于原点，要向前迈一个end
         }else                                                                     //在拖动周期
         {
             ANY_gait_XYZ->foot_top_Z = 0;
             ANY_gait_XYZ->foot_top_Y = ANY_gait_XYZ->translate_END * (1.0f - (ANY_gait_XYZ->RIGHT_translate_thase_LEG - T_2_4) /200.0f)  ;//拖动回来的时候就是正常的拖动了
         }
         ANY_gait_XYZ->foot_top_h_Z_2 = (ANY_gait_XYZ->LEG_BODY_height - ANY_gait_XYZ->foot_top_Z) *(ANY_gait_XYZ->LEG_BODY_height - ANY_gait_XYZ->foot_top_Z)  ;
				 
		if (ANY_gait_XYZ->thase_LEG  >= T_4_4)   
         {
             ANY_gait_XYZ->thase_LEG  = T_Start;//完成了一个周期
             ANY_gait_XYZ->foot_top_Y =  ANY_gait_XYZ->translate_BEGAIN;
             ANY_gait_XYZ->foot_top_Z = 0;
             ANY_gait_XYZ->First_step_flag  =  1;
         }
     }else  
		 
     {
         if (ANY_gait_XYZ->thase_LEG  >= T_4_4)   
         {
             ANY_gait_XYZ->thase_LEG  = T_Start;//完成了一个周期
             ANY_gait_XYZ->foot_top_Y =  ANY_gait_XYZ->translate_BEGAIN;
             ANY_gait_XYZ->foot_top_Z = 0;
         }
        if (ANY_gait_XYZ->thase_LEG  >= T_2_4)   
        {
         ANY_gait_XYZ->thase_LEG      +=  raise_step *  ANY_gait_XYZ->resolution;
        }else
        {
             ANY_gait_XYZ->thase_LEG      +=  translation_step *  ANY_gait_XYZ->resolution;
        }
         ANY_gait_XYZ->RIGHT_translate_thase_LEG = T_4_4 - ANY_gait_XYZ->thase_LEG ;
         if(ANY_gait_XYZ->RIGHT_translate_thase_LEG   < T_2_4)                                     //还在抬起周期
         {
             ANY_gait_XYZ->foot_top_Z = ANY_gait_XYZ->translate_high* MY_GAITCOSf(ANY_gait_XYZ->RIGHT_translate_thase_LEG);
             ANY_gait_XYZ->foot_top_Y = ANY_gait_XYZ->translate_step_length * MY_GAITCOSf_0_1(ANY_gait_XYZ->RIGHT_translate_thase_LEG)  + ANY_gait_XYZ->translate_BEGAIN ;
         }else                                                                     //在拖动周期
         {
             ANY_gait_XYZ->foot_top_Z = 0;
             ANY_gait_XYZ->foot_top_Y = ANY_gait_XYZ->translate_step_length  * (1.0f - (ANY_gait_XYZ->RIGHT_translate_thase_LEG - T_2_4) /200.0f)   +   ANY_gait_XYZ->translate_BEGAIN;
         }
         ANY_gait_XYZ->foot_top_h_Z_2 = (ANY_gait_XYZ->LEG_BODY_height - ANY_gait_XYZ->foot_top_Z) *(ANY_gait_XYZ->LEG_BODY_height - ANY_gait_XYZ->foot_top_Z);
     }
 }
}




#ifndef     geometric_method_H
#define	    geometric_method_H
#include "stm32h7xx_hal.h"
#include "data_table.h"
#include <stdint.h>
//周期定义
#define  T_Start  0
#define  T_1_4    100
#define  T_2_4    200
#define  T_3_4    300
#define  T_4_4    400
#define  forward    1
#define  backward  -1
#define  LEFT      -2
#define  RIGHT      2
#define  left_side_of_the_body  0
#define  right_side_of_the_body 1
#define _constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))

extern float BODY_height ;
typedef  struct 
{
	float   angle_fai  ;  //足端与大腿中心连线跟大腿的夹角
    float   angle_P    ;  //大腿跟小腿的夹角
    float   theata_1   ;  //大腿与水平线的夹角
    float   translate_angle_R  ;  //横向移动的角度
}gait_angle;

typedef  struct 
{
    float   foot_top_X  ;
    float   foot_top_Y  ;
    float   foot_top_Z  ;
    float   last_error_X  ;
    float   last_error_Y  ;
    float   last_error_Z  ;

}SIMPLE_NOW_GAIT_XY;

typedef  struct 
{
    float   force_X  ;
    float   force_Y  ;
    float   force_Z  ;

}virtual_force;

typedef  struct 
{
    int     gait_mode   ;
    float   foot_top_X  ;
    float   foot_top_Y  ;
    float   thase_LEG   ;        //一个完整周期改成0，100，200，300, 400四个区段,使用时转换为T
    uint16_t   resolution  ;        //步态分辨率，越高越丝滑
    float   foot_top_X2  ;          //足端坐标X的平方
    float   foot_top_h_Y_2  ;       //足端坐标（h-y）的平方
    int First_step_flag  ;
    uint16_t backward_thase_LEG   ;
    float  distance_from_foot_end_to_thigh  ;  //足端到大腿中心的距离
}gait_XY;


typedef  struct 
{
    int     gait_mode   ;
    float   foot_top_X  ;
    float   foot_top_Y  ;
    float   thase_LEG   ;        //一个完整周期改成0，100，200，300, 400四个区段,使用时转换为T
    uint16_t   resolution  ;        //步态分辨率，越高越丝滑
    float   foot_top_X2  ;          //足端坐标X的平方
    float   foot_top_h_Y_2  ;       //足端坐标（h-y）的平方
    float   foot_top_h_Z_2  ;       //足端坐标（h-Z）的平方
    int     First_step_flag  ;
    uint16_t backward_thase_LEG   ;
    float   foot_top_Z   ;     //横向移动坐标
    float   LEG_L        ;     //侧视图中腿的长度，和八自由度中的离地高度是一个概念
    float   hip_lenth    ;     //大腿根部长度
    float   hip_angle    ;     //大腿根部与水平线的夹角
    float   hip_lenth_2  ;     //大腿根部长度的平方
    float   translate_BEGAIN ; //横向移动的起始位置
    float   translate_END   ;  //横向移动的结束位置
    float   translate_step_length    ;       //横向移动的步长
    float   translate_high  ;  //横向移动的抬脚高度
		float   LEG_BODY_height ; 
    uint16_t LEFT_translate_thase_LEG       ;
    uint16_t RIGHT_translate_thase_LEG      ;
}gait_YZ;



//结构体参数  
extern  gait_angle  LEG1_angle ;
extern  gait_angle  LEG2_angle ;
extern  gait_angle  LEG3_angle ;
extern  gait_angle  LEG4_angle ;
extern  gait_XY     LEG1_XY    ;
extern  gait_XY     LEG2_XY    ;
extern  gait_XY     LEG3_XY    ;
extern  gait_XY     LEG4_XY    ;
extern  gait_YZ     LEG1_YZ    ;
extern  gait_YZ     LEG2_YZ    ;    
extern  gait_YZ     LEG3_YZ    ;
extern  gait_YZ     LEG4_YZ    ;
extern SIMPLE_NOW_GAIT_XY  NOW_GAIT_XY_LEG1 ;
extern SIMPLE_NOW_GAIT_XY  NOW_GAIT_XY_LEG2 ;
extern SIMPLE_NOW_GAIT_XY  NOW_GAIT_XY_LEG3 ;
extern SIMPLE_NOW_GAIT_XY  NOW_GAIT_XY_LEG4 ;
extern virtual_force      VIRTUAL_FORCE_LEG1;
extern virtual_force      VIRTUAL_FORCE_LEG2;
extern virtual_force      VIRTUAL_FORCE_LEG3;
extern virtual_force      VIRTUAL_FORCE_LEG4;
extern float distance_from_foot_end_to_thigh;
extern float  raise_step ;
extern float  translation_step ;
extern float   VMC_theta[2];
extern float MOVE_DISTANCE  ;     //拖动距离
extern float MOVE_DISTENCE_START ; //拖动的起始位置
extern float MOVE_DISTENCE_END   ; //拖动的结束位置
extern float  foot_top_height ;
void Jocob(float theta[], float fx, float fz, float *toa1, float *toa2 )  ;

void  Inverse_Kinematics( gait_angle*  ANY_gait_angle , gait_XY*  ANY_gait_XY  );	//运动学逆解
void GET_GAIT_XY(gait_XY*  ANY_gait_XY   , int dirction );
void  GAIT_Init( float  X_start ,float X_end , float Y, float resolution ,  float height, float  L1 , float L2 ,
float translate_BEGAIN , float translate_END ,float translate_HIGH ,float Hip_lenth);
void GET_GAIT_YZ(gait_YZ*  ANY_gait_XYZ   , int dirction );
void GET_GAIT_theta_CLEAR( void );
void  Inverse_Kinematics_add_YZ( gait_angle*  ANY_gait_angle , gait_YZ*  ANY_gait_YZ ,gait_XY*  ANY_gait_XY );  //运动学逆解，增加了横向移动的角度计算
void two_degree_of_freedom_forward_kinematics(float theta1, float theta2, SIMPLE_NOW_GAIT_XY*  NOW_gait_XY) ;
void PD_CONTROL(gait_XY*  ANY_gait_XY , SIMPLE_NOW_GAIT_XY*  NOW_gait_XY  ,virtual_force*  VIRTUAL_FORCE) ;
void SIMPLE_NOW_GAIT_XY_Init(SIMPLE_NOW_GAIT_XY*  NOW_gait_XY);
#endif 










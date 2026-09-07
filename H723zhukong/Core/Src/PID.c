#include "PID.h"
#include "stdio.h"
#include "math.h"
PID_struct IMU_PID;
int error_flag = 0;

/*********************************************************************************
 * Function:  		PID计算
* Description：   输入参数得出PID参数
* Parameters:   	PID结构体，当前数据
 * Return:       无返回
 * Others:				
**********************************************************************************/
float LV1_PID_BACK(PID_struct *motor_pid, float NOW, float target) {

	
	  //============== 更新目标值和实际值 ==============
    motor_pid->target1 = target; 
    motor_pid->actural1 = NOW;

    //============== 误差计算 ==============
    float previous_error = motor_pid->error1;  // 保存上一次误差（用于微分和调试）
	
    motor_pid->error1 = motor_pid->target1 - motor_pid->actural1;

    //============== 积分抗饱和 ==============
    if (fabs(motor_pid->error1) < motor_pid->errorint_max1 * 2.0f) {
        motor_pid->errorint1 += motor_pid->error1;
    }
    motor_pid->errorint1 = fmaxf(fminf(motor_pid->errorint1, motor_pid->errorint_max1), motor_pid->errorint_min1);

    //============== 微分计算（使用前次误差） ==============
    float delta_error = motor_pid->error1 - previous_error;
    motor_pid->error_last1 = motor_pid->error1;  // 更新历史误差

    // 微分低通滤波
    static float alpha = 0.2f;
    float filtered_delta = alpha * delta_error + (1 - alpha) * motor_pid->error_last1;

    //============== PID输出 ==============
    motor_pid->out1 = motor_pid->p1 * motor_pid->error1 +
                      motor_pid->i1 * motor_pid->errorint1 +
											(error_flag > 0 ?  (motor_pid->d1 * filtered_delta  ) :0)  ;
		if( error_flag == 0){error_flag = 1; }
    //============== 输出限幅 ==============
    motor_pid->out1 = fmaxf(fminf(motor_pid->out1, motor_pid->out_max1), motor_pid->out_min1);
    return motor_pid->out1;

}




/*********************************************************************************
 * Function:  		陀螺仪PID初始化
* Description：   给PID各项参数赋值
* Parameters:   	PID结构体
 * Return:        无返回
 * Others:				
**********************************************************************************/
void IMU_pid_init (PID_struct *motor_pid )//PID初始化
{
	motor_pid->p1=25;//65
	motor_pid->i1=0;//10
	motor_pid->d1=3;
	
  motor_pid->errorint_max1= 								0;//积分限幅最高
	motor_pid->errorint_min1 = 								0;//积分限幅最低
  motor_pid->out_max1= 											500;
	motor_pid->out_min1=											-500;
	
}





#include "Impedance_Servo_Control.h"
#include "Servo_control.h"
#include "math.h"
#include "data_table.h"
#include "tim.h"

// ************************* 阻抗控制参数配置 *************************
#define IMPEDANCE_CONTROL_PERIOD 0.001f  // 固定控制周期 1ms（和你的舵机调用周期一致）
#define ANGLE_MAX 180.0f
#define ANGLE_MIN 0.0f

// 虚拟阻抗参数（调参指南：先调K，再调B）
// K: 刚度系数，范围0.05~1.0，越小越软，越大越硬；四足腿部建议0.1~0.3
// B: 阻尼系数，范围0.01~0.1，一般为K的1/10~1/5，抑制抖动
static float K_impedance = 0.2f;  
static float B_impedance = 0.03f; 

// 全局静态变量，保存上一次的误差，用于计算速度项
static float e_pos_prev = 0.0f;

/**
 * @brief 舵机阻抗控制函数，模拟柔顺力矩输出
 * @param TIM_CHAnnel 舵机通道编号（和你原有代码兼容）
 * @param target_ref 期望参考角度（硬目标角度）
 * @param current_angle 指向当前角度的指针（和你原有代码兼容，函数内更新）
 * @return 最终输出给舵机的指令角度
 */
float Servo_Impedance_Control(uint16_t TIM_CHAnnel, float target_ref, float *current_angle)
{
    // 1. 计算位置误差（参考角度 - 当前角度）
    float e_pos = target_ref - *current_angle;
    
    // 2. 计算速度误差（误差的变化率，对应角速度差）
    float e_vel = (e_pos - e_pos_prev) / IMPEDANCE_CONTROL_PERIOD;
    
    // 3. 保存本次误差，用于下一次计算
    e_pos_prev = e_pos;
    
    // 4. 阻抗模型核心计算：修正指令角度
    float theta_cmd = target_ref - (K_impedance * e_pos + B_impedance * e_vel);
    
    // 5. 角度限幅（和你原有代码逻辑完全一致）
    theta_cmd = fmaxf(ANGLE_MIN, fminf(ANGLE_MAX, theta_cmd));
    
    // 6. 更新当前角度（和你原有低通/S曲线控制逻辑兼容）
    *current_angle = theta_cmd;
//    printf("%.2f\n",*current_angle);
    // 7. 转换为PWM脉冲值（复用你原有宏定义）
    uint16_t pulse = (uint16_t)(*current_angle * ANGLE_TO_DUTY) + 500;
    
    // 8. 输出PWM到对应定时器通道（和你原有代码完全一致，无修改）
    uint8_t timer_num = (TIM_CHAnnel >> 8) & 0xF;
    uint8_t channel_num = TIM_CHAnnel & 0xFF;
    
    switch (timer_num) {
        case 0x01:
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1 + (channel_num - 1) * 4, pulse);
            break;
        case 0x02:
            __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1 + (channel_num - 1) * 4, pulse);
            break;
    }
    
    return theta_cmd;
}




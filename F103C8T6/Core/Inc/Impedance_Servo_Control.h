#ifndef Impedance_Servo_Contol_H
#define Impedance_Servo_Contol_H
#include "Impedance_Servo_Control.h"
#include "Servo_control.h"
#include "math.h"
#include "data_table.h"
#include "tim.h"

float Servo_Impedance_Control(uint16_t TIM_CHAnnel, float target_ref, float *current_angle);


#endif



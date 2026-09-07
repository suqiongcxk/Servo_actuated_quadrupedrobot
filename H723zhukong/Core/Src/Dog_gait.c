#include "Dog_gait.h"
#include <math.h>

static uint8_t posture_settling = 0;

/* Always recompute the error: changing the target cannot retain an old sign. */
static float Posture_Approach(float current, float target)
{
    float error = target - current;
    float step = POSTURE_APPROACH_GAIN * error;
    if (fabsf(error) <= POSTURE_SNAP_CM) return target;
    if (step > POSTURE_MAX_STEP_CM) step = POSTURE_MAX_STEP_CM;
    if (step < -POSTURE_MAX_STEP_CM) step = -POSTURE_MAX_STEP_CM;
    return current + step;
}

/* Preserve the last commanded foot coordinates, including a lifted foot.
 * Settle them towards a neutral pose without resetting phase/height first.
 * GAIT_Init in main must have initialized link lengths before this runs.
 */
static uint8_t Posture_UpdatePose(float front_height, float rear_height,
                                 float left_front_x, float left_front_lift,
                                 float twist_offset)
{
    gait_XY *xy[4] = {&LEG1_XY, &LEG2_XY, &LEG3_XY, &LEG4_XY};
    gait_YZ *yz[4] = {&LEG1_YZ, &LEG2_YZ, &LEG3_YZ, &LEG4_YZ};
    gait_angle *angle[4] = {&LEG1_angle, &LEG2_angle, &LEG3_angle, &LEG4_angle};
    uint8_t done = 1;
    uint8_t i;
    for (i = 0; i < 4; ++i)
    {
        float target = (i < 2) ? front_height : rear_height;
        float target_x = (i == 0) ? left_front_x : 0.0f;
        float target_y = (i == 0) ? left_front_lift : 0.0f;
        /* Same front/rear sign convention as REMOTE_V_Set's turn command. */
        float lateral = (i < 2) ? twist_offset : -twist_offset;
        float vertical;
        xy[i]->resolution = 0;
        yz[i]->resolution = 0;
        yz[i]->LEG_BODY_height = Posture_Approach(yz[i]->LEG_BODY_height, target);
        xy[i]->foot_top_X = Posture_Approach(xy[i]->foot_top_X, target_x);
        xy[i]->foot_top_Y = Posture_Approach(xy[i]->foot_top_Y, target_y);
        yz[i]->foot_top_Y = Posture_Approach(yz[i]->foot_top_Y, lateral);
        yz[i]->foot_top_Z = Posture_Approach(yz[i]->foot_top_Z, 0.0f);
        vertical = yz[i]->LEG_BODY_height - yz[i]->foot_top_Z;
        yz[i]->foot_top_h_Z_2 = vertical * vertical;
        Inverse_Kinematics_add_YZ(angle[i], yz[i], xy[i]);
        xy[i]->foot_top_X2 = xy[i]->foot_top_X * xy[i]->foot_top_X;
        vertical = xy[i]->distance_from_foot_end_to_thigh - xy[i]->foot_top_Y;
        xy[i]->foot_top_h_Y_2 = vertical * vertical;
        Inverse_Kinematics(angle[i], xy[i]);
        if (yz[i]->LEG_BODY_height != target || xy[i]->foot_top_X != target_x ||
            xy[i]->foot_top_Y != target_y || yz[i]->foot_top_Y != lateral ||
            yz[i]->foot_top_Z != 0.0f) done = 0;
    }
    return done;
}

static uint8_t Posture_UpdateTargets(float front_height, float rear_height,
                                    float left_front_x, float left_front_lift)
{
    return Posture_UpdatePose(front_height, rear_height, left_front_x, left_front_lift, 0.0f);
}

static uint8_t Posture_Update(float front_height, float rear_height)
{
    return Posture_UpdateTargets(front_height, rear_height, 0.0f, 0.0f);
}

static uint8_t wave_stage = 0;
static volatile float turn_requested = 0.0f;
static uint8_t turn_running = 0;
static float turn_phase = 0.0f;
static float turn_amplitude = 0.0f;
static float turn_envelope = 0.0f;
static float turn_direction = 0.0f;
static uint32_t turn_last_ms = 0;

void Self_Rotation_Request(float offset_cm)
{
    if (!isfinite(offset_cm)) offset_cm = 0.0f;
    if (offset_cm > TURN_MAX_OFFSET_CM) offset_cm = TURN_MAX_OFFSET_CM;
    if (offset_cm < -TURN_MAX_OFFSET_CM) offset_cm = -TURN_MAX_OFFSET_CM;
    if (fabsf(offset_cm) < 0.03f) offset_cm = 0.0f;
    turn_requested = offset_cm;
    GAIT_MODE = GAIT_MODE_SELF_ROTATION;
}

/* One phase and one lift per leg. Turns use the existing front/rear lateral
 * sign convention; physical turn radius still depends on the chassis geometry.
 * On reversal, stop or height change, settle the actual foot pose first.
 */
void Self_Rotation_move(void)
{
    gait_XY *xy[4] = {&LEG1_XY, &LEG2_XY, &LEG3_XY, &LEG4_XY};
    gait_YZ *yz[4] = {&LEG1_YZ, &LEG2_YZ, &LEG3_YZ, &LEG4_YZ};
    gait_angle *angle[4] = {&LEG1_angle, &LEG2_angle, &LEG3_angle, &LEG4_angle};
    float requested = turn_requested;
    float target_height = Tort_height;
    uint32_t now = HAL_GetTick();
    uint32_t dt;
    uint8_t i;
    if (GAIT_MODE_LAST != GAIT_MODE_SELF_ROTATION) turn_running = 0;
    GAIT_MODE_LAST = GAIT_MODE_SELF_ROTATION;
    posture_settling = 0;
    if (requested == 0.0f || requested * turn_direction < 0.0f ||
        yz[0]->LEG_BODY_height != target_height || yz[1]->LEG_BODY_height != target_height ||
        yz[2]->LEG_BODY_height != target_height || yz[3]->LEG_BODY_height != target_height)
        turn_running = 0;
    if (!turn_running)
    {
        uint8_t done = Posture_Update(target_height, target_height);
        height_above_ground = (yz[0]->LEG_BODY_height + yz[1]->LEG_BODY_height +
                               yz[2]->LEG_BODY_height + yz[3]->LEG_BODY_height) * 0.25f;
        if (done)
        {
            turn_phase = turn_amplitude = turn_envelope = 0.0f;
            turn_last_ms = now;
            turn_direction = requested;
            if (requested != 0.0f) turn_running = 1;
            else
            {
                GAIT_MODE_LAST = 0; /* Release ownership even with continuous RX. */
                GAIT_MODE = (forward_start != 0.0f || forward_end != 0.0f ||
                              translate_BEGAIN != 0.0f || translate_END != 0.0f)
                             ? GAIT_MODE_TORT : GAIT_MODE_STAND;
            }
        }
        return;
    }
    dt = now - turn_last_ms;
    turn_last_ms = now;
    if (dt > 20U) dt = 20U; /* Do not jump through a cycle after a delayed task. */
    if (dt == 0U) return;
    turn_amplitude = Posture_Approach(turn_amplitude, requested);
    turn_envelope += (float)dt / 1000.0f;
    if (turn_envelope > 1.0f) turn_envelope = 1.0f;
    turn_phase += (float)dt / TURN_PERIOD_MS;
    if (turn_phase >= 1.0f) turn_phase -= 1.0f;
    for (i = 0; i < 4; ++i)
    {
        float phase = turn_phase + ((i == 1 || i == 3) ? 0.5f : 0.0f);
        float s, blend, lateral, lift, vertical;
        if (phase >= 1.0f) phase -= 1.0f;
        s = (phase < 0.5f) ? phase * 2.0f : (phase - 0.5f) * 2.0f;
        blend = s*s*s*(10.0f + s*(-15.0f + 6.0f*s));
        lateral = turn_amplitude * turn_envelope *
                  ((phase < 0.5f) ? (1.0f - 2.0f*blend) : (-1.0f + 2.0f*blend));
        lift = (phase < 0.5f) ? TURN_LIFT_CM * turn_envelope *
               16.0f*s*s*(1.0f-s)*(1.0f-s) : 0.0f;
        xy[i]->resolution = yz[i]->resolution = 0;
        xy[i]->thase_LEG = yz[i]->thase_LEG = phase * 400.0f;
        xy[i]->foot_top_X = xy[i]->foot_top_X2 = 0.0f;
        xy[i]->foot_top_Y = lift;
        yz[i]->foot_top_Y = (i < 2) ? lateral : -lateral;
        yz[i]->foot_top_Z = 0.0f; /* Lift is represented only in XY. */
        yz[i]->foot_top_h_Z_2 = target_height * target_height;
        Inverse_Kinematics_add_YZ(angle[i], yz[i], xy[i]);
        vertical = xy[i]->distance_from_foot_end_to_thigh - lift;
        xy[i]->foot_top_h_Y_2 = vertical * vertical;
        Inverse_Kinematics(angle[i], xy[i]);
    }
}

static uint8_t twist_stage = 0;
static uint32_t twist_start_ms = 0;

void Body_Twist_Start(void)
{
    if (GAIT_MODE != GAIT_MODE_BODY_TWIST || twist_stage == 2)
        twist_stage = 0;
    GAIT_MODE = GAIT_MODE_BODY_TWIST;
}

/* Stand, sway front/rear in opposite directions without stepping, then center.
 * Six seconds after preparation. The sine-squared envelope eases both ends.
 */
void Body_Twist_move(void)
{
    uint32_t elapsed;
    float offset = 0.0f;
    if (GAIT_MODE_LAST != GAIT_MODE_BODY_TWIST) twist_stage = 0;
    GAIT_MODE_LAST = GAIT_MODE_BODY_TWIST;
    posture_settling = 0;
    if (twist_stage == 0)
    {
        if (Posture_Update(TWIST_BODY_HEIGHT_CM, TWIST_BODY_HEIGHT_CM))
        {
            twist_start_ms = HAL_GetTick();
            twist_stage = 1;
        }
    }
    else
    {
        elapsed = (uint32_t)(HAL_GetTick() - twist_start_ms);
        if (twist_stage == 1 && elapsed < TWIST_SWAY_MS)
        {
            float t = (float)elapsed / TWIST_SWAY_MS;
            float envelope = sinf(3.14159265359f * t);
            offset = TWIST_OFFSET_CM * envelope * envelope *
                     sinf(6.28318530718f * TWIST_CYCLES * t);
        }
        if (Posture_UpdatePose(TWIST_BODY_HEIGHT_CM, TWIST_BODY_HEIGHT_CM,
                               0.0f, 0.0f, offset) && elapsed >= TWIST_DURATION_MS)
            twist_stage = 2;
    }
    height_above_ground = (LEG1_YZ.LEG_BODY_height + LEG2_YZ.LEG_BODY_height +
                           LEG3_YZ.LEG_BODY_height + LEG4_YZ.LEG_BODY_height) * 0.25f;
}

static uint8_t dance_stage = 0;
static uint32_t dance_start_ms = 0;

void Dance_Start(void)
{
    if (GAIT_MODE != GAIT_MODE_DANCE || dance_stage == 2)
        dance_stage = 0;
    GAIT_MODE = GAIT_MODE_DANCE;
}

/* Ten-second routine after settling into the starting stance.
 * Front/rear height offsets: squat, rise, bow, rise, rear dip, rise,
 * squat, rise, then a two-second neutral finish. All feet stay down.
 */
void Dance_move(void)
{
    static const float offsets[9][2] = {
        {0, 0}, {-1, -1}, {0, 0}, {-1, 0}, {0, 0},
        {0, -1}, {0, 0}, {-1, -1}, {0, 0}
    };
    uint32_t elapsed;
    float front = DANCE_BODY_HEIGHT_CM;
    float rear = DANCE_BODY_HEIGHT_CM;
    if (GAIT_MODE_LAST != GAIT_MODE_DANCE) dance_stage = 0;
    GAIT_MODE_LAST = GAIT_MODE_DANCE;
    posture_settling = 0;
    if (dance_stage == 0)
    {
        if (Posture_Update(front, rear))
        {
            dance_start_ms = HAL_GetTick();
            dance_stage = 1;
        }
    }
    else
    {
        elapsed = (uint32_t)(HAL_GetTick() - dance_start_ms);
        if (dance_stage == 1 && elapsed < DANCE_DURATION_MS * 8U / 10U)
        {
            float position = (float)elapsed / ((float)DANCE_DURATION_MS / 10.0f);
            uint8_t segment = (uint8_t)position;
            float t = position - segment;
            /* Quintic easing: zero velocity and acceleration at each keyframe. */
            float blend = t * t * t * (10.0f + t * (-15.0f + 6.0f * t));
            front += DANCE_BOB_CM * (offsets[segment][0] + blend *
                     (offsets[segment + 1][0] - offsets[segment][0]));
            rear += DANCE_BOB_CM * (offsets[segment][1] + blend *
                    (offsets[segment + 1][1] - offsets[segment][1]));
        }
        if (Posture_Update(front, rear) && elapsed >= DANCE_DURATION_MS)
            dance_stage = 2; /* Hold standing until another command arrives. */
    }
    height_above_ground = (LEG1_YZ.LEG_BODY_height + LEG2_YZ.LEG_BODY_height +
                           LEG3_YZ.LEG_BODY_height + LEG4_YZ.LEG_BODY_height) * 0.25f;
}

static uint16_t wave_updates = 0;

void Wave_Left_Front_Start(void)
{
    /* Repeated start requests during an action do not restart its progress. */
    if (GAIT_MODE != GAIT_MODE_WAVE_LEFT_FRONT || wave_stage == 4)
        wave_stage = 0;
    GAIT_MODE = GAIT_MODE_WAVE_LEFT_FRONT;
}

/* LEG1 maps to the front F103's target_angle_leftleg. No direct servo writes. */
void Wave_Left_Front_move(void)
{
    float x;
    if (GAIT_MODE_LAST != GAIT_MODE_WAVE_LEFT_FRONT)
        wave_stage = 0;
    GAIT_MODE_LAST = GAIT_MODE_WAVE_LEFT_FRONT;
    posture_settling = 0;
    switch (wave_stage)
    {
    case 0: /* Sit and settle all four feet first. */
        if (Posture_Update(SIT_FRONT_HEIGHT_CM, SIT_REAR_HEIGHT_CM))
            wave_stage = 1;
        break;
    case 1: /* Lift only the left front foot. */
        if (Posture_UpdateTargets(SIT_FRONT_HEIGHT_CM, SIT_REAR_HEIGHT_CM,
                                  WAVE_REACH_CM, WAVE_LIFT_CM))
        {
            wave_updates = 0;
            wave_stage = 2;
        }
        break;
    case 2:
        x = WAVE_REACH_CM + 0.5f * WAVE_SWING_CM *
            (1.0f - cosf(6.28318530718f *
             (float)(wave_updates % WAVE_PERIOD_UPDATES) / WAVE_PERIOD_UPDATES));
        Posture_UpdateTargets(SIT_FRONT_HEIGHT_CM, SIT_REAR_HEIGHT_CM, x, WAVE_LIFT_CM);
        if (++wave_updates >= WAVE_PERIOD_UPDATES * WAVE_REPEAT_COUNT)
            wave_stage = 3;
        break;
    case 3: /* Lower smoothly, retaining the actual last commanded position. */
        if (Posture_Update(SIT_FRONT_HEIGHT_CM, SIT_REAR_HEIGHT_CM))
            wave_stage = 4;
        break;
    default: /* Hold seated; a held Bluetooth button must not repeat the action. */
        Posture_Update(SIT_FRONT_HEIGHT_CM, SIT_REAR_HEIGHT_CM);
        break;
    }
    height_above_ground = (LEG1_YZ.LEG_BODY_height + LEG2_YZ.LEG_BODY_height +
                           LEG3_YZ.LEG_BODY_height + LEG4_YZ.LEG_BODY_height) * 0.25f;
}

static uint8_t Posture_BeforeWalk(float target, uint8_t mode)
{
    if (GAIT_MODE_LAST != mode || LEG1_YZ.LEG_BODY_height != target ||
        LEG2_YZ.LEG_BODY_height != target || LEG3_YZ.LEG_BODY_height != target ||
        LEG4_YZ.LEG_BODY_height != target) posture_settling = 1;
    if (!posture_settling) return 0;

    if (!Posture_Update(target, target))
    {
        height_above_ground = (LEG1_YZ.LEG_BODY_height + LEG2_YZ.LEG_BODY_height +
                               LEG3_YZ.LEG_BODY_height + LEG4_YZ.LEG_BODY_height) * 0.25f;
        return 1;
    }
    height_above_ground = target;
    posture_settling = 0;
    GAIT_MODE_LAST = 0; /* Reinitialize gait only after all feet have settled. */
    return 0;
}

void Walk_Forward_Start(void)
{
    GAIT_MODE = GAIT_MODE_FORWARD_TEST;
}

/* Non-blocking forward test, using the existing 1+3 / 2+4 leg pairing.
 * Angles are published and sent by the existing FreeRTOS/RS485 tasks.
 * Intended for power-on testing from the initial symmetric foot position.
 */
void Walk_Forward(void)
{
    gait_XY *xy[4] = {&LEG1_XY, &LEG2_XY, &LEG3_XY, &LEG4_XY};
    gait_YZ *yz[4] = {&LEG1_YZ, &LEG2_YZ, &LEG3_YZ, &LEG4_YZ};
    gait_angle *angles[4] = {&LEG1_angle, &LEG2_angle, &LEG3_angle, &LEG4_angle};
    static uint8_t second_pair_started = 0;
    uint8_t ready = 0;
    uint8_t i;

    if (GAIT_MODE_LAST != GAIT_MODE_FORWARD_TEST)
    {
        forward_start = -FORWARD_TEST_HALF_STRIDE_CM;
        forward_end = FORWARD_TEST_HALF_STRIDE_CM;
        forward_height = FORWARD_TEST_LIFT_CM;
        resolution = FORWARD_TEST_RESOLUTION;
        translate_BEGAIN = 0.0f;
        translate_END = 0.0f;
        translate_HIGH = 0.0f;
        GAIT_Init(forward_start, forward_end, forward_height, resolution,
                  height_above_ground, L1, L2, 0.0f, 0.0f, 0.0f, Hip_lenth);
        SET_GAIT_duty_cycle(0.5f);
        Clear_Start_Flag();
        GET_GAIT_theta_CLEAR();
        second_pair_started = 0;
        GAIT_MODE_LAST = GAIT_MODE_FORWARD_TEST;
    }

    /* Raise/lower the body by at most 0.01 cm per update before stepping. */
    if (height_above_ground < FORWARD_TEST_BODY_HEIGHT_CM - 0.01f)
        height_above_ground += 0.01f;
    else if (height_above_ground > FORWARD_TEST_BODY_HEIGHT_CM + 0.01f)
        height_above_ground -= 0.01f;
    else
    {
        height_above_ground = FORWARD_TEST_BODY_HEIGHT_CM;
        ready = 1;
    }

    /* Start the other pair exactly when the first pair reaches support. */
    if (ready && LEG1_XY.thase_LEG >= T_2_4)
        second_pair_started = 1;

    for (i = 0; i < 4; ++i)
    {
        xy[i]->resolution = (ready && ((i == 0) || (i == 2) || second_pair_started))
                            ? FORWARD_TEST_RESOLUTION : 0;
        yz[i]->resolution = 0;
        yz[i]->LEG_BODY_height = height_above_ground;
        GET_GAIT_YZ(yz[i], RIGHT);
        Inverse_Kinematics_add_YZ(angles[i], yz[i], xy[i]);
        GET_GAIT_XY(xy[i], forward);
        Inverse_Kinematics(angles[i], xy[i]);
    }
}

/*步态重要参数
1.步态类型变更时一定要重新初始化步态参数并且先回到站立姿态
2.遥控的摇杆初期调整步态的步长后面可以改变分辨率
*/
float height_above_ground = 10.0f;
float forward_start       = 0.0f;//负数
float forward_end         = 0.0f;//正数
float forward_height      = 3.5f;
float resolution          = 10.0f;
float L1                  = 10.0f;
float L2                  = 10.0f;
float translate_BEGAIN    = 0;//正数
float translate_END       = 0;//负数
float translate_HIGH      = 0.5;
float Hip_lenth           = 4.5;

//步态特别参数
float Crawl_height        = 15; 
float Tort_height         = 16; 
float Crawl_resolution    = 12; 
float Tort_resolution     = 8; //8	

/// @brief 步态模式记录
uint8_t GAIT_MODE         = GAIT_MODE_SIT_DOWN ;
uint8_t GAIT_MODE_LAST    = 0 ;


//遥控方向
int remote_dir = 0;
  

//开始运动标志位
int LEG1_start = 0;
int LEG2_start = 0;
int LEG3_start = 0;
int LEG4_start = 0;


//启动标志清零
void Clear_Start_Flag(void)
{
    LEG1_start = 0;
    LEG2_start = 0;
    LEG3_start = 0;
    LEG4_start = 0;

    LEG1_XY.First_step_flag = 0;
    LEG2_XY.First_step_flag = 0;
    LEG3_XY.First_step_flag = 0;
    LEG4_XY.First_step_flag = 0;

    LEG1_YZ.First_step_flag = 0;
    LEG2_YZ.First_step_flag = 0;
    LEG3_YZ.First_step_flag = 0;
    LEG4_YZ.First_step_flag = 0;
}



//设置步态占空比,占空比范围0-1
void SET_GAIT_duty_cycle(float duty_cycle)
{
    raise_step = 1.0f*(1 - duty_cycle);
    translation_step = 1.0f*duty_cycle;
}

/*
单腿占空比0.25 ,以一号腿为参考系
 3号腿的相位差为四分之一个周期
四号腿的相位差为四分之二个周期
二号腿的相位差为四分之三个周期
*/
void Crawl_move(void )
{
    if (Posture_BeforeWalk(Crawl_height, GAIT_MODE_CRAWL)) return;
    if(GAIT_MODE_LAST != GAIT_MODE_CRAWL)
    {
        // 更改步态参数
				forward_start = -2;
				forward_end   =  1.5;
				forward_height = 7;
				resolution = Crawl_resolution;
        GAIT_Init(forward_start,forward_end, forward_height, resolution,
         height_above_ground, L1, L2,translate_BEGAIN, translate_END, translate_HIGH,Hip_lenth);
        SET_GAIT_duty_cycle(0.2f);
        Clear_Start_Flag();    //清除启动及第一步标志位
        GET_GAIT_theta_CLEAR();
        /*
        回到站立姿态,和清除步态参数代码
        */

        //更新步态模式记录
        GAIT_MODE_LAST = GAIT_MODE_CRAWL;
    }else
    {
			
			
        #ifdef ZIYOUDU_8

        /*
        重心转移与平衡部分
        */
       
        if (LEG1_start == 1)GET_GAIT_XY(&LEG1_XY,remote_dir);
        if (LEG2_start == 1)GET_GAIT_XY(&LEG2_XY,remote_dir);
        if (LEG3_start == 1)GET_GAIT_XY(&LEG3_XY,remote_dir);
        if (LEG4_start == 1)GET_GAIT_XY(&LEG4_XY,remote_dir);
        if(LEG1_XY.thase_LEG >= 200){LEG3_start = 1;} //1 3 4 2
        if(LEG3_XY.thase_LEG >= 200){LEG4_start = 1;}      
        if(LEG4_XY.thase_LEG >= 200){LEG2_start = 1;}
        #endif
        LEG1_start = 1;


				
        #ifdef ZIYOUDU_12
			
			
			
        if (LEG1_start == 1)			{LEG1_XY.resolution = resolution;  LEG1_YZ.resolution = resolution;}
				else                      {LEG1_XY.resolution = 0;  LEG1_YZ.resolution = 0;}
				
        if (LEG2_start == 1)			{LEG2_XY.resolution = resolution;  LEG2_YZ.resolution = resolution;}
				else                      {LEG2_XY.resolution = 0;  LEG2_YZ.resolution = 0;}
				
        if (LEG3_start == 1)			{LEG3_XY.resolution = resolution;  LEG3_YZ.resolution = resolution;}
				else                      {LEG3_XY.resolution = 0;  LEG3_YZ.resolution = 0;}	
				
        if (LEG4_start == 1)		  {LEG4_XY.resolution = resolution;  LEG4_YZ.resolution = resolution;}
				else                      {LEG4_XY.resolution = 0;  LEG4_YZ.resolution = 0;}	  

				
		LEG1_YZ.LEG_BODY_height = height_above_ground;
		GET_GAIT_YZ(&LEG1_YZ , RIGHT);
		Inverse_Kinematics_add_YZ(&LEG1_angle , &LEG1_YZ, &LEG1_XY);
		GET_GAIT_XY(&LEG1_XY , forward);
		Inverse_Kinematics(&LEG1_angle , &LEG1_XY);
					
	
	
		LEG2_YZ.LEG_BODY_height = height_above_ground;
		GET_GAIT_YZ(&LEG2_YZ , RIGHT);
		Inverse_Kinematics_add_YZ(&LEG2_angle , &LEG2_YZ, &LEG2_XY);
		GET_GAIT_XY(&LEG2_XY , forward);
		Inverse_Kinematics(&LEG2_angle , &LEG2_XY);
		
		
		LEG3_YZ.LEG_BODY_height = height_above_ground;
		GET_GAIT_YZ(&LEG3_YZ , RIGHT);
		Inverse_Kinematics_add_YZ(&LEG3_angle , &LEG3_YZ, &LEG3_XY);
		GET_GAIT_XY(&LEG3_XY , forward);
		Inverse_Kinematics(&LEG3_angle , &LEG3_XY);					
		

		LEG4_YZ.LEG_BODY_height = height_above_ground;
		GET_GAIT_YZ(&LEG4_YZ , RIGHT);
		Inverse_Kinematics_add_YZ(&LEG4_angle , &LEG4_YZ, &LEG4_XY);
		GET_GAIT_XY(&LEG4_XY , forward);
		Inverse_Kinematics(&LEG4_angle , &LEG4_XY);	
		
        if(LEG1_XY.thase_LEG >= 200){LEG3_start = 1;} //1 3 4 2
        if(LEG3_XY.thase_LEG >= 200){LEG4_start = 1;}      
        if(LEG4_XY.thase_LEG >= 200){LEG2_start = 1;}
        #endif 
			
			
    }


}


/*
单腿占空比0.5 ,以一号腿为参考系
3号腿同步
四号腿与二号腿同步，相差半个周期
*/
void Tortoise_move(void )
{
    if (Posture_BeforeWalk(Tort_height, GAIT_MODE_TORT)) return;
    if(GAIT_MODE_LAST != GAIT_MODE_TORT)
    {
        // 更改步态参数
			
                /* Keep the latest remote stride/lift after a height change. */
                forward_start = MOVE_DISTENCE_START;
                forward_end = MOVE_DISTENCE_END;
                forward_height = foot_top_height;
				resolution = Tort_resolution;
			
        GAIT_Init(forward_start,forward_end, forward_height, resolution,
         height_above_ground, L1, L2,translate_BEGAIN, translate_END, translate_HIGH,Hip_lenth);
        SET_GAIT_duty_cycle(0.5f);
        Clear_Start_Flag();    //清除启动及第一步标志位
        GET_GAIT_theta_CLEAR();
        /*
        回到站立姿态,和清除步态参数代码
        */

        //更新步态模式记录
        GAIT_MODE_LAST = GAIT_MODE_TORT;
    }else
    {
        #ifdef ZIYOUDU_8

        /*
        重心转移与平衡部分
        */
       
        if (LEG1_start == 1)GET_GAIT_XY(&LEG1_XY,remote_dir);
        if (LEG2_start == 1)GET_GAIT_XY(&LEG2_XY,remote_dir);
        if (LEG3_start == 1)GET_GAIT_XY(&LEG3_XY,remote_dir);
        if (LEG4_start == 1)GET_GAIT_XY(&LEG4_XY,remote_dir);
        if(LEG1_XY.thase_LEG >= 200){LEG2_start = 1;} //1和3 4和2
        if(LEG3_XY.thase_LEG >= 200){LEG4_start = 1;}      
        #endif
        LEG1_start = 1;
        LEG3_start = 1;


			
        #ifdef ZIYOUDU_12
        if (LEG1_start == 1)			{LEG1_XY.resolution = resolution;  LEG1_YZ.resolution = resolution;}
				else                      {LEG1_XY.resolution = 0;  LEG1_YZ.resolution = 0;  LEG1_XY.thase_LEG = 0; LEG1_YZ.thase_LEG = 0; }
				
        if (LEG2_start == 1)			{LEG2_XY.resolution = resolution;  LEG2_YZ.resolution = resolution;}
				else                      {LEG2_XY.resolution = 0;  LEG2_YZ.resolution = 0;  LEG2_XY.thase_LEG = 0; LEG2_YZ.thase_LEG = 0; }
				
        if (LEG3_start == 1)			{LEG3_XY.resolution = resolution;  LEG3_YZ.resolution = resolution;}
				else                      {LEG3_XY.resolution = 0;  LEG3_YZ.resolution = 0;  LEG3_XY.thase_LEG = 0; LEG3_YZ.thase_LEG = 0; }	
				
        if (LEG4_start == 1)		  {LEG4_XY.resolution = resolution;  LEG4_YZ.resolution = resolution;}
				else                      {LEG4_XY.resolution = 0;  LEG4_YZ.resolution = 0;  LEG4_XY.thase_LEG = 0; LEG4_YZ.thase_LEG = 0; }	
				
				
		LEG1_YZ.LEG_BODY_height = height_above_ground;
		GET_GAIT_YZ(&LEG1_YZ , RIGHT);
		Inverse_Kinematics_add_YZ(&LEG1_angle , &LEG1_YZ, &LEG1_XY);
    GET_GAIT_XY(&LEG1_XY , forward);
		Inverse_Kinematics(&LEG1_angle , &LEG1_XY);
					
	
	
		LEG2_YZ.LEG_BODY_height = height_above_ground;
		GET_GAIT_YZ(&LEG2_YZ , RIGHT);
		Inverse_Kinematics_add_YZ(&LEG2_angle , &LEG2_YZ, &LEG2_XY);
		GET_GAIT_XY(&LEG2_XY , forward);
		Inverse_Kinematics(&LEG2_angle , &LEG2_XY);
		
		
		LEG3_YZ.LEG_BODY_height = height_above_ground;
		GET_GAIT_YZ(&LEG3_YZ , RIGHT);
		Inverse_Kinematics_add_YZ(&LEG3_angle , &LEG3_YZ, &LEG3_XY);
		GET_GAIT_XY(&LEG3_XY , forward);
		Inverse_Kinematics(&LEG3_angle , &LEG3_XY);					
		
    
		LEG4_YZ.LEG_BODY_height = height_above_ground;
		GET_GAIT_YZ(&LEG4_YZ , RIGHT);
		Inverse_Kinematics_add_YZ(&LEG4_angle , &LEG4_YZ, &LEG4_XY);
		GET_GAIT_XY(&LEG4_XY , forward);
		Inverse_Kinematics(&LEG4_angle , &LEG4_XY);	
		
        if(LEG1_XY.thase_LEG >= 200){LEG2_start = 1;} //1和3 4和2
        if(LEG3_XY.thase_LEG >= 200){LEG4_start = 1;}   				
        #endif 
        
    }

}




/*
单腿占空比0.5 ,以一号腿为参考系
4号腿同步
三号腿与二号腿同步，相差半个周期
*/
void PACE_move(void )
{
    if(GAIT_MODE_LAST != GAIT_MODE_PACE)
    {
        // 更改步态参数
        GAIT_Init(forward_start,forward_end, forward_height, resolution,
         height_above_ground, L1, L2,translate_BEGAIN, translate_END, translate_HIGH,Hip_lenth);
        SET_GAIT_duty_cycle(0.5f);
        Clear_Start_Flag();    //清除启动及第一步标志位
        GET_GAIT_theta_CLEAR();
        LEG1_start = 1;
        LEG4_start = 1;
        /*
        回到站立姿态,和清除步态参数代码
        */
        //更新步态模式记录
        GAIT_MODE_LAST = GAIT_MODE_PACE;
    }else
    {
        #ifdef ZIYOUDU_8

        /*
        重心转移与平衡部分
        */

        if (LEG1_start == 1)GET_GAIT_XY(&LEG1_XY,remote_dir);
        if (LEG2_start == 1)GET_GAIT_XY(&LEG2_XY,remote_dir);
        if (LEG3_start == 1)GET_GAIT_XY(&LEG3_XY,remote_dir);
        if (LEG4_start == 1)GET_GAIT_XY(&LEG4_XY,remote_dir);
        if(LEG1_XY.thase_LEG >= 200){LEG2_start = 1;} //1和3 4和2
        if(LEG4_XY.thase_LEG >= 200){LEG3_start = 1;}      
        #endif 

        #ifdef ZIYOUDU_12
        
        /*
        重心转移与平衡部分
        */
        if (LEG1_start == 1)			{LEG1_XY.resolution = resolution;  LEG1_YZ.resolution = resolution;}
				else                      {LEG1_XY.resolution = 0;  LEG1_YZ.resolution = 0;}
				
        if (LEG2_start == 1)			{LEG2_XY.resolution = resolution;  LEG2_YZ.resolution = resolution;}
				else                      {LEG2_XY.resolution = 0;  LEG2_YZ.resolution = 0;}
				
        if (LEG3_start == 1)			{LEG3_XY.resolution = resolution;  LEG3_YZ.resolution = resolution;}
				else                      {LEG3_XY.resolution = 0;  LEG3_YZ.resolution = 0;}	
				
        if (LEG4_start == 1)		  {LEG4_XY.resolution = resolution;  LEG4_YZ.resolution = resolution;}
				else                      {LEG4_XY.resolution = 0;  LEG4_YZ.resolution = 0;}	

		GET_GAIT_YZ(&LEG1_YZ , RIGHT);
		Inverse_Kinematics_add_YZ(&LEG1_angle , &LEG1_YZ, &LEG1_XY);
		GET_GAIT_XY(&LEG1_XY , forward);
		Inverse_Kinematics(&LEG1_angle , &LEG1_XY);
					
	
	

		GET_GAIT_YZ(&LEG2_YZ , RIGHT);
		Inverse_Kinematics_add_YZ(&LEG2_angle , &LEG2_YZ, &LEG2_XY);
		GET_GAIT_XY(&LEG2_XY , forward);
		Inverse_Kinematics(&LEG2_angle , &LEG2_XY);
		
		

		GET_GAIT_YZ(&LEG3_YZ , RIGHT);
		Inverse_Kinematics_add_YZ(&LEG3_angle , &LEG3_YZ, &LEG3_XY);
		GET_GAIT_XY(&LEG3_XY , forward);
		Inverse_Kinematics(&LEG3_angle , &LEG3_XY);					
		


		GET_GAIT_YZ(&LEG4_YZ , RIGHT);
		Inverse_Kinematics_add_YZ(&LEG4_angle , &LEG4_YZ, &LEG4_XY);
		GET_GAIT_XY(&LEG4_XY , forward);
		Inverse_Kinematics(&LEG4_angle , &LEG4_XY);	
		

        if(LEG1_XY.thase_LEG >= 200){LEG2_start = 1;} //1和3 4和2
        if(LEG4_XY.thase_LEG >= 200){LEG3_start = 1;}      
        #endif 
        
    }   
}   



/*
单腿占空比0.5 ,以一号腿为参考系
2号腿同步
三号腿与四号腿同步，相差半个周期
*/
void Bound_move(void )
{
    if(GAIT_MODE_LAST != GAIT_MODE_BOUND)
    {
        // 更改步态参数
        GAIT_Init(forward_start,forward_end, forward_height, resolution,
         height_above_ground, L1, L2,translate_BEGAIN, translate_END, translate_HIGH,Hip_lenth);
        SET_GAIT_duty_cycle(0.5f);
        Clear_Start_Flag();    //清除启动及第一步标志位
        GET_GAIT_theta_CLEAR();
        LEG1_start = 1;
        LEG4_start = 1;
        /*
        回到站立姿态,和清除步态参数代码
        */
        //更新步态模式记录
        GAIT_MODE_LAST = GAIT_MODE_BOUND;
    }else
    {
        #ifdef ZIYOUDU_8

        /*
        重心转移与平衡部分
        */

        if (LEG1_start == 1)GET_GAIT_XY(&LEG1_XY,remote_dir);
        if (LEG2_start == 1)GET_GAIT_XY(&LEG2_XY,remote_dir);
        if (LEG3_start == 1)GET_GAIT_XY(&LEG3_XY,remote_dir);
        if (LEG4_start == 1)GET_GAIT_XY(&LEG4_XY,remote_dir);
        if(LEG1_XY.thase_LEG >= 200){LEG2_start = 1;} //1和3 4和2
        if(LEG4_XY.thase_LEG >= 200){LEG3_start = 1;} 
        #endif 

        #ifdef ZIYOUDU_12
        
        if (LEG1_start == 1)			{LEG1_XY.resolution = resolution;  LEG1_YZ.resolution = resolution;}
				else                      {LEG1_XY.resolution = 0;  LEG1_YZ.resolution = 0;}
				
        if (LEG2_start == 1)			{LEG2_XY.resolution = resolution;  LEG2_YZ.resolution = resolution;}
				else                      {LEG2_XY.resolution = 0;  LEG2_YZ.resolution = 0;}
				
        if (LEG3_start == 1)			{LEG3_XY.resolution = resolution;  LEG3_YZ.resolution = resolution;}
				else                      {LEG3_XY.resolution = 0;  LEG3_YZ.resolution = 0;}	
				
        if (LEG4_start == 1)		  {LEG4_XY.resolution = resolution;  LEG4_YZ.resolution = resolution;}
				else                      {LEG4_XY.resolution = 0;  LEG4_YZ.resolution = 0;}	
		GET_GAIT_YZ(&LEG1_YZ , RIGHT);
		Inverse_Kinematics_add_YZ(&LEG1_angle , &LEG1_YZ, &LEG1_XY);
		GET_GAIT_XY(&LEG1_XY , forward);
		Inverse_Kinematics(&LEG1_angle , &LEG1_XY);
			
	
	

		GET_GAIT_YZ(&LEG2_YZ , RIGHT);
		Inverse_Kinematics_add_YZ(&LEG2_angle , &LEG2_YZ, &LEG2_XY);
		GET_GAIT_XY(&LEG2_XY , forward);
		Inverse_Kinematics(&LEG2_angle , &LEG2_XY);
		

		GET_GAIT_YZ(&LEG3_YZ , RIGHT);
		Inverse_Kinematics_add_YZ(&LEG3_angle , &LEG3_YZ, &LEG3_XY);
		GET_GAIT_XY(&LEG3_XY , forward);
		Inverse_Kinematics(&LEG3_angle , &LEG3_XY);					
		

		GET_GAIT_YZ(&LEG4_YZ , RIGHT);
		Inverse_Kinematics_add_YZ(&LEG4_angle , &LEG4_YZ, &LEG4_XY);
		GET_GAIT_XY(&LEG4_XY , forward);
		Inverse_Kinematics(&LEG4_angle , &LEG4_XY);	
		
        if(LEG1_XY.thase_LEG >= 200){LEG2_start = 1;} //1和3 4和2
        if(LEG4_XY.thase_LEG >= 200){LEG3_start = 1;} 
        #endif 
        
    }   
}   

void Stand_move(void )
{
    posture_settling = 0;
    /* In stand mode height_above_ground is the requested symmetric height. */
    Posture_Update(height_above_ground, height_above_ground);
    GAIT_MODE_LAST = GAIT_MODE_STAND;
}



//原地踏步
void March_move(void )
{
    if (GAIT_MODE_LAST != GAIT_MODE_MARCH)
    {
        // 更改步态参数
        GAIT_Init(forward_start,forward_end, forward_height, resolution,
         height_above_ground, L1, L2,translate_BEGAIN, translate_END, translate_HIGH,Hip_lenth);
        SET_GAIT_duty_cycle(0.5f);
        Clear_Start_Flag();    //清除启动及第一步标志位
        GET_GAIT_theta_CLEAR();
        LEG1_start = 1;
        LEG3_start = 1;
        /*
        回到站立姿态,和清除步态参数代码
        */
        //更新步态模式记录
        GAIT_MODE_LAST = GAIT_MODE_MARCH;
    }else{
        #ifdef ZIYOUDU_8

        /*
        重心转移与平衡部分
        */
       
        if (LEG1_start == 1)GET_GAIT_XY(&LEG1_XY,remote_dir);
        if (LEG2_start == 1)GET_GAIT_XY(&LEG2_XY,remote_dir);
        if (LEG3_start == 1)GET_GAIT_XY(&LEG3_XY,remote_dir);
        if (LEG4_start == 1)GET_GAIT_XY(&LEG4_XY,remote_dir);
        if(LEG1_XY.thase_LEG >= 200){LEG2_start = 1;} //1和3 4和2
        if(LEG3_XY.thase_LEG >= 200){LEG4_start = 1;}      
        #endif 

        #ifdef ZIYOUDU_12

        if (LEG1_start == 1)			{LEG1_XY.resolution = resolution;  LEG1_YZ.resolution = resolution;}
				else                      {LEG1_XY.resolution = 0;  LEG1_YZ.resolution = 0;}
				
        if (LEG2_start == 1)			{LEG2_XY.resolution = resolution;  LEG2_YZ.resolution = resolution;}
				else                      {LEG2_XY.resolution = 0;  LEG2_YZ.resolution = 0;}
				
        if (LEG3_start == 1)			{LEG3_XY.resolution = resolution;  LEG3_YZ.resolution = resolution;}
				else                      {LEG3_XY.resolution = 0;  LEG3_YZ.resolution = 0;}	
				
        if (LEG4_start == 1)		  {LEG4_XY.resolution = resolution;  LEG4_YZ.resolution = resolution;}
				else                      {LEG4_XY.resolution = 0;  LEG4_YZ.resolution = 0;}		
		GET_GAIT_YZ(&LEG1_YZ , RIGHT);
		Inverse_Kinematics_add_YZ(&LEG1_angle , &LEG1_YZ, &LEG1_XY);
		GET_GAIT_XY(&LEG1_XY , forward);
		Inverse_Kinematics(&LEG1_angle , &LEG1_XY);
	
		GET_GAIT_YZ(&LEG2_YZ , RIGHT);
		Inverse_Kinematics_add_YZ(&LEG2_angle , &LEG2_YZ, &LEG2_XY);
		GET_GAIT_XY(&LEG2_XY , forward);
		Inverse_Kinematics(&LEG2_angle , &LEG2_XY);
	
		GET_GAIT_YZ(&LEG3_YZ , RIGHT);
		Inverse_Kinematics_add_YZ(&LEG3_angle , &LEG3_YZ, &LEG3_XY);
		GET_GAIT_XY(&LEG3_XY , forward);
		Inverse_Kinematics(&LEG3_angle , &LEG3_XY);					
		
		GET_GAIT_YZ(&LEG4_YZ , RIGHT);
		Inverse_Kinematics_add_YZ(&LEG4_angle , &LEG4_YZ, &LEG4_XY);
		GET_GAIT_XY(&LEG4_XY , forward);
		Inverse_Kinematics(&LEG4_angle , &LEG4_XY);	

        if(LEG1_XY.thase_LEG >= 200){LEG2_start = 1;} //1和3 4和2
        if(LEG3_XY.thase_LEG >= 200){LEG4_start = 1;}   				
        #endif 
        
    }
}



void sit_down(void )
{
    posture_settling = 0;
    Posture_Update(SIT_FRONT_HEIGHT_CM, SIT_REAR_HEIGHT_CM);
    height_above_ground = (LEG1_YZ.LEG_BODY_height + LEG2_YZ.LEG_BODY_height +
                           LEG3_YZ.LEG_BODY_height + LEG4_YZ.LEG_BODY_height) * 0.25f;
    GAIT_MODE_LAST = GAIT_MODE_SIT_DOWN;
}







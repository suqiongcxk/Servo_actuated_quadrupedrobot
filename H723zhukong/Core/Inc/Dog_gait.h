#ifndef __DOG_GAIT_H__
#define __DOG_GAIT_H__  
#include "stdio.h"
#include "geometric_method.h"


//步态自由度开关
//#define ZIYOUDU_8                1
 #define ZIYOUDU_12            1


//步态模式及宏定义
extern  uint8_t GAIT_MODE     ;
extern  uint8_t GAIT_MODE_LAST;
#define GAIT_MODE_TORT        1
#define GAIT_MODE_CRAWL       2
#define GAIT_MODE_PACE        3
#define GAIT_MODE_BOUND       4
#define GAIT_MODE_STAND       5
#define GAIT_MODE_MARCH       6
#define GAIT_MODE_SELF_ROTATION 7
#define TURN_PERIOD_MS          1600U
#define TURN_MAX_OFFSET_CM       3.0f
#define TURN_LIFT_CM             2.5f
void Self_Rotation_Request(float offset_cm);
void Self_Rotation_move(void);
#define GAIT_MODE_SIT_DOWN    8
#define GAIT_MODE_FORWARD_TEST 9
#define GAIT_MODE_WAVE_LEFT_FRONT 10
#define GAIT_MODE_DANCE          11
#define GAIT_MODE_BODY_TWIST     12
#define TWIST_DURATION_MS        7500U
#define TWIST_SWAY_MS            6000U
#define TWIST_BODY_HEIGHT_CM     12.0f
#define TWIST_OFFSET_CM           3.0f
#define TWIST_FOLLOW_SPEED_SCALE  2.0f /* Lateral tracking only. */
#define TWIST_CYCLES              4U
void Body_Twist_Start(void);
void Body_Twist_move(void);
#define DANCE_DURATION_MS         6500U
#define DANCE_BODY_HEIGHT_CM     12.0f
#define DANCE_BOB_CM              4.0f
void Dance_Start(void);
void Dance_move(void);
#define WAVE_LIFT_CM             5.0f
#define WAVE_REACH_CM            2.0f
#define WAVE_SWING_CM            6.0f
#define SIT_STANCE_SPREAD_CM     1.0f /* Move each foot outward from the body. */
#define WAVE_BODY_SHIFT_RIGHT_CM 1.5f /* Shift weight away from the lifted left-front leg. */
#define WAVE_PERIOD_UPDATES      360U /* 1.8 s at 5 ms/update. */
#define WAVE_REPEAT_COUNT        2U
void Wave_Left_Front_Start(void);
void Wave_Left_Front_move(void);

/* Forward test: cm; phase increment per 5 ms update (larger = faster). */
#define FORWARD_TEST_HALF_STRIDE_CM  1.0f
#define FORWARD_TEST_LIFT_CM         2.0f
#define FORWARD_TEST_BODY_HEIGHT_CM 13.0f
#define FORWARD_TEST_RESOLUTION      4U

/* Trot cadence follows the requested foot excursion.
 * At a 5 ms gait update and 0.5 duty cycle, resolution 4/8 is about 1/2 Hz.
 */
#define TORT_RESOLUTION_MIN           4.0f
#define TORT_RESOLUTION_MAX           8.0f
#define TORT_CADENCE_START_CM         0.20f
#define TORT_CADENCE_FULL_CM          2.50f
#define TORT_CADENCE_FILTER_GAIN      0.05f

/* Posture transition, called every 5 ms. Distances are in cm. */
#define POSTURE_MAX_STEP_CM          0.08f
#define POSTURE_APPROACH_GAIN        0.08f
#define POSTURE_SNAP_CM              0.001f
#define SIT_FRONT_HEIGHT_CM         12.0f
#define SIT_REAR_HEIGHT_CM           7.0f

/* IMU attitude feedback. Positive roll means left side high; positive pitch
 * means front high. Flip a DIRECTION value if the physical correction is wrong.
 */
#define BALANCE_ENABLE                  1
#define BALANCE_ROLL_ZERO_DEG          0.0f
#define BALANCE_PITCH_ZERO_DEG         0.0f
#define BALANCE_ROLL_DIRECTION         1.0f
#define BALANCE_PITCH_DIRECTION        1.0f
#define BALANCE_ANGLE_DEADBAND_DEG      0.30f
#define BALANCE_MAX_ANGLE_DEG          15.0f
#define BALANCE_MAX_RATE_DPS          100.0f
#define BALANCE_KP_CM_PER_DEG           0.06f
#define BALANCE_KD_CM_PER_DPS           0.004f
#define BALANCE_MAX_LEG_CM              1.00f
#define BALANCE_OUTPUT_FILTER_GAIN      0.15f
#define BALANCE_WALK_SCALE              0.55f
#define BALANCE_RESET_GAP_MS           100U

/* Call Start once after GAIT_Init, before osKernelStart.
 * Walk_Forward is called by the existing gait task every 5 ms.
 * This mode owns gait parameters; Bluetooth enable/disable (5/6) still works.
 */
void Walk_Forward_Start(void);
void Walk_Forward(void);


extern float height_above_ground;
extern float forward_start;
extern float forward_end;
extern float forward_height;
extern float resolution;
extern float L1;
extern float L2;
extern float translate_BEGAIN;
extern float translate_END;
extern float translate_HIGH;
extern float Hip_lenth;
extern float Tort_height   ;


//函数声明

void Clear_Start_Flag(void);
void SET_GAIT_duty_cycle(float duty_cycle);
void Crawl_move(void );
void Tortoise_move(void );
void PACE_move(void );
void Bound_move(void );
void Stand_move(void );
void March_move(void );
void sit_down(void );
#endif

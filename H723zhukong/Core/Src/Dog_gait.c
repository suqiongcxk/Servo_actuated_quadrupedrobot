#include "Dog_gait.h"
#include "JY901S.h"
#include <math.h>

static uint8_t posture_settling = 0;

/* Always recompute the error: changing the target cannot retain an old sign. */
static float Posture_ApproachScaled(float current, float target, float speed_scale)
{
    float error = target - current;
    float step = POSTURE_APPROACH_GAIN * speed_scale * error;
    float max_step = POSTURE_MAX_STEP_CM * speed_scale;
    if (fabsf(error) <= POSTURE_SNAP_CM) return target;
    if (step > max_step) step = max_step;
    if (step < -max_step) step = -max_step;
    return current + step;
}

static float Posture_Approach(float current, float target)
{
    return Posture_ApproachScaled(current, target, 1.0f);
}

static float balance_roll_cm = 0.0f;
static float balance_pitch_cm = 0.0f;
static uint32_t balance_last_call_ms = 0U;

static float Balance_Clamp(float value, float limit)
{
    if (value > limit) return limit;
    if (value < -limit) return -limit;
    return value;
}

static float Balance_Deadband(float value)
{
    if (value > BALANCE_ANGLE_DEADBAND_DEG)
        return value - BALANCE_ANGLE_DEADBAND_DEG;
    if (value < -BALANCE_ANGLE_DEADBAND_DEG)
        return value + BALANCE_ANGLE_DEADBAND_DEG;
    return 0.0f;
}

/* roll_cm/pitch_cm describe measured tilt. Leg height signs below generate
 * the opposite body motion. Outputs decay safely to zero if IMU data is stale.
 */
static void Balance_GetCorrections(float mode_scale, float *roll_cm, float *pitch_cm)
{
    JY901S_Snapshot sample;
    uint32_t now = HAL_GetTick();
    float roll_target = 0.0f;
    float pitch_target = 0.0f;
    float total;

    if ((uint32_t)(now - balance_last_call_ms) > BALANCE_RESET_GAP_MS)
        balance_roll_cm = balance_pitch_cm = 0.0f;
    balance_last_call_ms = now;

    if (BALANCE_ENABLE && JY901S_GetLatest(&sample) &&
        isfinite(sample.angles.roll) && isfinite(sample.angles.pitch) &&
        isfinite(sample.rates.roll_rate) && isfinite(sample.rates.pitch_rate))
    {
        float roll_error = BALANCE_ROLL_DIRECTION *
                           (sample.angles.roll - BALANCE_ROLL_ZERO_DEG);
        float pitch_error = BALANCE_PITCH_DIRECTION *
                            (sample.angles.pitch - BALANCE_PITCH_ZERO_DEG);
        float roll_rate = BALANCE_ROLL_DIRECTION * sample.rates.roll_rate;
        float pitch_rate = BALANCE_PITCH_DIRECTION * sample.rates.pitch_rate;
        roll_error = Balance_Deadband(Balance_Clamp(roll_error, BALANCE_MAX_ANGLE_DEG));
        pitch_error = Balance_Deadband(Balance_Clamp(pitch_error, BALANCE_MAX_ANGLE_DEG));
        roll_rate = Balance_Clamp(roll_rate, BALANCE_MAX_RATE_DPS);
        pitch_rate = Balance_Clamp(pitch_rate, BALANCE_MAX_RATE_DPS);
        roll_target = BALANCE_KP_CM_PER_DEG * roll_error +
                      BALANCE_KD_CM_PER_DPS * roll_rate;
        pitch_target = BALANCE_KP_CM_PER_DEG * pitch_error +
                       BALANCE_KD_CM_PER_DPS * pitch_rate;
        /* Keep the sum applied to any one leg inside the configured limit. */
        total = fabsf(roll_target) + fabsf(pitch_target);
        if (total > BALANCE_MAX_LEG_CM)
        {
            float scale = BALANCE_MAX_LEG_CM / total;
            roll_target *= scale;
            pitch_target *= scale;
        }
    }

    balance_roll_cm += BALANCE_OUTPUT_FILTER_GAIN *
                       (roll_target - balance_roll_cm);
    balance_pitch_cm += BALANCE_OUTPUT_FILTER_GAIN *
                        (pitch_target - balance_pitch_cm);
    *roll_cm = mode_scale * balance_roll_cm;
    *pitch_cm = mode_scale * balance_pitch_cm;
}

static float Balance_LegHeight(uint8_t leg, float roll_cm, float pitch_cm)
{
    float roll_sign = (leg == 0 || leg == 3) ? -1.0f : 1.0f;
    float pitch_sign = (leg < 2) ? -1.0f : 1.0f;
    return roll_sign * roll_cm + pitch_sign * pitch_cm;
}

/* Preserve the last commanded foot coordinates, including a lifted foot.
 * Settle them towards a neutral pose without resetting phase/height first.
 * GAIT_Init in main must have initialized link lengths before this runs.
 */
static uint8_t Posture_UpdatePose(float front_height, float rear_height,
                                 float left_front_x, float left_front_lift,
                                 float twist_offset, float lateral_offset,
                                 float stance_spread, float lateral_speed_scale,
                                 float roll_height_cm, float pitch_height_cm)
{
    gait_XY *xy[4] = {&LEG1_XY, &LEG2_XY, &LEG3_XY, &LEG4_XY};
    gait_YZ *yz[4] = {&LEG1_YZ, &LEG2_YZ, &LEG3_YZ, &LEG4_YZ};
    gait_angle *angle[4] = {&LEG1_angle, &LEG2_angle, &LEG3_angle, &LEG4_angle};
    uint8_t done = 1;
    uint8_t i;
    for (i = 0; i < 4; ++i)
    {
        float target = ((i < 2) ? front_height : rear_height) +
                       Balance_LegHeight(i, roll_height_cm, pitch_height_cm);
        float target_x = (i == 0) ? left_front_x : 0.0f;
        float target_y = (i == 0) ? left_front_lift : 0.0f;
        /* LEG1/4 are left; LEG2/3 are right.  Opposite signs widen the stance. */
        float side = (i == 0 || i == 3) ? -1.0f : 1.0f;
        /* Keep the existing front/rear sign convention for body twist. */
        float lateral = lateral_offset + side * stance_spread +
                        ((i < 2) ? twist_offset : -twist_offset);
        float vertical;
        xy[i]->resolution = 0;
        yz[i]->resolution = 0;
        yz[i]->LEG_BODY_height = Posture_Approach(yz[i]->LEG_BODY_height, target);
        xy[i]->foot_top_X = Posture_Approach(xy[i]->foot_top_X, target_x);
        xy[i]->foot_top_Y = Posture_Approach(xy[i]->foot_top_Y, target_y);
        yz[i]->foot_top_Y = Posture_ApproachScaled(yz[i]->foot_top_Y, lateral,
                                                  lateral_speed_scale);
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
    return Posture_UpdatePose(front_height, rear_height, left_front_x, left_front_lift,
                              0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f);
}

static uint8_t Posture_Update(float front_height, float rear_height)
{
    return Posture_UpdateTargets(front_height, rear_height, 0.0f, 0.0f);
}

/* A positive body shift moves the body right by commanding all feet left. */
static uint8_t Posture_UpdateSit(float left_front_x, float left_front_lift,
                                float body_shift_right)
{
    return Posture_UpdatePose(SIT_FRONT_HEIGHT_CM, SIT_REAR_HEIGHT_CM,
                              left_front_x, left_front_lift, 0.0f,
                              -body_shift_right, SIT_STANCE_SPREAD_CM, 1.0f,
                              0.0f, 0.0f);
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
 * 7.5 seconds after preparation. The sine-squared envelope eases both ends.
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
                               0.0f, 0.0f, offset, 0.0f, 0.0f,
                               TWIST_FOLLOW_SPEED_SCALE, 0.0f, 0.0f) &&
            elapsed >= TWIST_DURATION_MS)
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

/* 6.5-second routine after settling into the starting stance.
 * Front/rear height offsets: squat, rise, bow, rise, rear dip, rise,
 * squat, rise, then a 1.3-second neutral finish. All feet stay down.
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
    if (GAIT_MODE != GAIT_MODE_WAVE_LEFT_FRONT || wave_stage == 6)
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
    case 0: /* Sit with a wider stance and settle all four feet first. */
        if (Posture_UpdateSit(0.0f, 0.0f, 0.0f))
            wave_stage = 1;
        break;
    case 1: /* Shift weight to the right before removing left-front support. */
        if (Posture_UpdateSit(0.0f, 0.0f, WAVE_BODY_SHIFT_RIGHT_CM))
            wave_stage = 2;
        break;
    case 2: /* Lift only the left front foot while holding the weight shift. */
        if (Posture_UpdateSit(WAVE_REACH_CM, WAVE_LIFT_CM,
                              WAVE_BODY_SHIFT_RIGHT_CM))
        {
            wave_updates = 0;
            wave_stage = 3;
        }
        break;
    case 3:
        x = WAVE_REACH_CM + 0.5f * WAVE_SWING_CM *
            (1.0f - cosf(6.28318530718f *
             (float)(wave_updates % WAVE_PERIOD_UPDATES) / WAVE_PERIOD_UPDATES));
        Posture_UpdateSit(x, WAVE_LIFT_CM, WAVE_BODY_SHIFT_RIGHT_CM);
        if (++wave_updates >= WAVE_PERIOD_UPDATES * WAVE_REPEAT_COUNT)
            wave_stage = 4;
        break;
    case 4: /* Put the left-front foot down before moving the body back. */
        if (Posture_UpdateSit(0.0f, 0.0f, WAVE_BODY_SHIFT_RIGHT_CM))
            wave_stage = 5;
        break;
    case 5: /* Return to the centered, widened sitting pose. */
        if (Posture_UpdateSit(0.0f, 0.0f, 0.0f))
            wave_stage = 6;
        break;
    default: /* Hold seated; a held Bluetooth button must not repeat the action. */
        Posture_UpdateSit(0.0f, 0.0f, 0.0f);
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
float Tort_resolution     = TORT_RESOLUTION_MIN;

/// @brief 步态模式记录
uint8_t GAIT_MODE         = GAIT_MODE_STAND ; // 上电默认静止站立
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
static float Tortoise_TargetResolution(void)
{
    float forward_command = fmaxf(fabsf(forward_start), fabsf(forward_end));
    float lateral_command = fmaxf(fabsf(translate_BEGAIN), fabsf(translate_END));
    float motion_command = fmaxf(forward_command, lateral_command);
    float speed_ratio;

    speed_ratio = (motion_command - TORT_CADENCE_START_CM) /
                  (TORT_CADENCE_FULL_CM - TORT_CADENCE_START_CM);
    if (speed_ratio < 0.0f) speed_ratio = 0.0f;
    if (speed_ratio > 1.0f) speed_ratio = 1.0f;
    return TORT_RESOLUTION_MIN +
           (TORT_RESOLUTION_MAX - TORT_RESOLUTION_MIN) * speed_ratio;
}

void Tortoise_move(void )
{
    float target_resolution;
    float balance_roll;
    float balance_pitch;
    if (Posture_BeforeWalk(Tort_height, GAIT_MODE_TORT)) return;
    target_resolution = Tortoise_TargetResolution();
    if(GAIT_MODE_LAST != GAIT_MODE_TORT)
    {
        // 更改步态参数
			
                /* Keep the latest remote stride/lift after a height change. */
                forward_start = MOVE_DISTENCE_START;
                forward_end = MOVE_DISTENCE_END;
                forward_height = foot_top_height;
				Tort_resolution = target_resolution;
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
        Balance_GetCorrections(BALANCE_WALK_SCALE, &balance_roll, &balance_pitch);
        /* Change cadence continuously with command size without a phase jump. */
        Tort_resolution += TORT_CADENCE_FILTER_GAIN *
                           (target_resolution - Tort_resolution);
        resolution = Tort_resolution;
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
				
				
		LEG1_YZ.LEG_BODY_height = height_above_ground +
                                      Balance_LegHeight(0, balance_roll, balance_pitch);
		GET_GAIT_YZ(&LEG1_YZ , RIGHT);
		Inverse_Kinematics_add_YZ(&LEG1_angle , &LEG1_YZ, &LEG1_XY);
    GET_GAIT_XY(&LEG1_XY , forward);
		Inverse_Kinematics(&LEG1_angle , &LEG1_XY);
					
	
	
		LEG2_YZ.LEG_BODY_height = height_above_ground +
                                      Balance_LegHeight(1, balance_roll, balance_pitch);
		GET_GAIT_YZ(&LEG2_YZ , RIGHT);
		Inverse_Kinematics_add_YZ(&LEG2_angle , &LEG2_YZ, &LEG2_XY);
		GET_GAIT_XY(&LEG2_XY , forward);
		Inverse_Kinematics(&LEG2_angle , &LEG2_XY);
		
		
		LEG3_YZ.LEG_BODY_height = height_above_ground +
                                      Balance_LegHeight(2, balance_roll, balance_pitch);
		GET_GAIT_YZ(&LEG3_YZ , RIGHT);
		Inverse_Kinematics_add_YZ(&LEG3_angle , &LEG3_YZ, &LEG3_XY);
		GET_GAIT_XY(&LEG3_XY , forward);
		Inverse_Kinematics(&LEG3_angle , &LEG3_XY);					
		
    
		LEG4_YZ.LEG_BODY_height = height_above_ground +
                                      Balance_LegHeight(3, balance_roll, balance_pitch);
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
    float balance_roll;
    float balance_pitch;
    posture_settling = 0;
    /* In stand mode height_above_ground is the requested symmetric height. */
    Balance_GetCorrections(1.0f, &balance_roll, &balance_pitch);
    Posture_UpdatePose(height_above_ground, height_above_ground,
                       0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f,
                       balance_roll, balance_pitch);
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
    Posture_UpdateSit(0.0f, 0.0f, 0.0f);
    height_above_ground = (LEG1_YZ.LEG_BODY_height + LEG2_YZ.LEG_BODY_height +
                           LEG3_YZ.LEG_BODY_height + LEG4_YZ.LEG_BODY_height) * 0.25f;
    GAIT_MODE_LAST = GAIT_MODE_SIT_DOWN;
}







#ifndef JOINT_MIT_H
#define JOINT_MIT_H

#include <stdint.h>

typedef struct {
    /* raw / zero */
    int32_t raw_zero;
    int32_t raw_rel;

    /* state */
    float q;        // rad (joint side)
    float q_prev;
    float qd;       // rad/s (filtered)

    /* command */
    float q_des;
    float qd_des;

    /* gains */
    float kp;
    float kd;
    float tau_ff;
    float tau;
    float tau_max;

    /* transmission */
    float gear_ratio;     // e.g. 10.0

    /* deadband & backlash handling */
    float q_db;           // rad, position deadband (backlash)
    float q_hold_band;    // rad, hold zone
    float qd_db;          // rad/s

    /* filters */
    float qd_alpha;       // 0~1

    /* torque shaping */
    float tau_rate_limit; // Nm/s
    float tau_prev;
    int   torque_sign;    // +1 / -1

    /* raw mapping */
    int32_t raw_min;
    int32_t raw_max;

    uint8_t zero_valid;

    float q_target;     // [rad] 期望关节角

} JointMIT;

/* API */
void joint_mit_init(JointMIT *j);
void joint_mit_config_raw(JointMIT *j,
                          int32_t raw_min,
                          int32_t raw_max,
                          float   gear_ratio);

void joint_mit_zero_here(JointMIT *j, int32_t raw_now);
void joint_mit_step_1khz(JointMIT *j, int32_t raw_abs);

#endif

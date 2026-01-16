#include "joint_mit.h"
#include <math.h>

#define TWO_PI 6.283185307f
#define ENC_RES 16384.0f   // AS5047P 14-bit

static inline float clampf(float x, float lo, float hi)
{
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

static inline float deadband(float x, float db)
{
    if (x >  db) return x - db;
    if (x < -db) return x + db;
    return 0.0f;
}

void joint_mit_init(JointMIT *j)
{
    j->raw_zero = 0;
    j->raw_rel  = 0;

    j->q = j->q_prev = 0.0f;
    j->qd = 0.0f;

    j->q_des = 0.0f;
    j->qd_des = 0.0f;

    j->kp = 0.1f;
    j->kd = 0.05f;
    j->tau_ff = 0.0f;
    j->tau = 0.0f;
    j->tau_max = 0.5f;

    j->gear_ratio = 1.0f;

    /* backlash handling defaults */
    j->q_db        = 0.02f;   // ~1.1 deg
    j->q_hold_band = 0.01f;
    j->qd_db       = 0.1f;

    j->qd_alpha = 0.08f;

    j->tau_rate_limit = 20.0f; // Nm/s
    j->tau_prev = 0.0f;

    j->torque_sign = 1;

    j->raw_min = 0;
    j->raw_max = 16383;

    j->zero_valid = 0;
}

void joint_mit_config_raw(JointMIT *j,
                          int32_t raw_min,
                          int32_t raw_max,
                          float   gear_ratio)
{
    j->raw_min = raw_min;
    j->raw_max = raw_max;
    j->gear_ratio = gear_ratio;
}

void joint_mit_zero_here(JointMIT *j, int32_t raw_now)
{
    j->raw_zero = raw_now;
    j->q = 0.0f;
    j->q_prev = 0.0f;
    j->qd = 0.0f;
    j->tau_prev = 0.0f;
    j->zero_valid = 1;
}

void joint_mit_step_1khz(JointMIT *j, int32_t raw_abs)
{
    if (!j->zero_valid) {
        j->tau = 0.0f;
        return;
    }

    /* ---------- position ---------- */
    /* Handle 14-bit encoder wrap-around: find minimal signed difference */
    int32_t diff = (int32_t)raw_abs - (int32_t)j->raw_zero;
    if (diff > 8192) {
        diff -= 16384;
    } else if (diff < -8192) {
        diff += 16384;
    }
    j->raw_rel = diff;

    /* motor side angle */
    float q_m = (float)j->raw_rel * (TWO_PI / ENC_RES);

    /* joint side */
    j->q = q_m / j->gear_ratio;

    /* ---------- velocity ---------- */
    float qd_raw = (j->q - j->q_prev) * 1000.0f;
    j->q_prev = j->q;

    j->qd += j->qd_alpha * (qd_raw - j->qd);

    /* ---------- MIT-style control ---------- */
    float pos_err = j->q_des - j->q;
    float vel_err = j->qd_des - j->qd;

    /* backlash deadband */
    float e  = deadband(pos_err, j->q_db);
    float ed = deadband(vel_err, j->qd_db);

    float tau_cmd;

    /* hold zone: don't fight backlash */
    if (fabsf(pos_err) < j->q_hold_band) {
        tau_cmd = j->tau_prev * 0.98f;   // decay
    } else {
        tau_cmd = j->kp * e + j->kd * ed + j->tau_ff;
    }

    /* clamp */
    tau_cmd = clampf(tau_cmd, -j->tau_max, j->tau_max);

    /* torque slew-rate limit */
    float max_step = j->tau_rate_limit * 0.001f;
    float tau = clampf(tau_cmd,
                        j->tau_prev - max_step,
                        j->tau_prev + max_step);

    j->tau_prev = tau;
    j->tau = tau * (float)j->torque_sign;
}

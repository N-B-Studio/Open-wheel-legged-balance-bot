#include "joint_hw.h"
#include <string.h>

#define AXIS_STATE_CLOSED_LOOP  8

#define CONTROL_MODE_TORQUE     1
#define INPUT_MODE_PASSTHROUGH  1

static void apply_cfg_to_mit(Joint* j)
{
    joint_mit_init(&j->mit);

    joint_mit_config_raw(&j->mit, j->cfg.raw_min, j->cfg.raw_max, j->cfg.gear_ratio);

    j->mit.kp = j->cfg.kp;
    j->mit.kd = j->cfg.kd;
    j->mit.tau_max = j->cfg.tau_max;

    j->mit.q_db = j->cfg.q_db;
    j->mit.q_hold_band = j->cfg.q_hold_band;
    j->mit.qd_alpha = j->cfg.qd_alpha;
    j->mit.tau_rate_limit = j->cfg.tau_rate_limit;

    j->mit.torque_sign = j->cfg.torque_sign;
}

void joint_init(Joint* j, const JointConfig* cfg)
{
    memset(j, 0, sizeof(*j));
    j->cfg = *cfg;

    // Init encoder
    j->enc.hspi = cfg->hspi;
    j->enc.cs_port = cfg->cs_port;
    j->enc.cs_pin  = cfg->cs_pin;
    as5047p_init(&j->enc);

    // Init odrive
    j->od.hcan = cfg->hcan;
    j->od.node_id = cfg->node_id;

    apply_cfg_to_mit(j);
}

void joint_odrive_enable_torque_mode(Joint* j)
{
    // 你 main.c 里一直在写的那几步，集中起来
    odrive_can_clear_errors(&j->od);
    HAL_Delay(20);

    odrive_can_set_axis_state(&j->od, AXIS_STATE_CLOSED_LOOP);
    HAL_Delay(100);

    odrive_can_set_controller_mode(&j->od, CONTROL_MODE_TORQUE, INPUT_MODE_PASSTHROUGH);
    HAL_Delay(100);
}

uint8_t joint_zero_to_home(Joint* j)
{
    uint8_t e = 0;
    uint16_t raw = as5047p_read_angle_raw14(&j->enc, &e);

    j->last_raw = raw;
    j->last_err = e;

    if (e != 0) {
        // encoder read fail -> do not arm
        j->mit.zero_valid = 0;
        odrive_can_set_input_torque(&j->od, 0.0f);
        return 0;
    }

    // Set zero point from raw abs config value.
    joint_mit_zero_here(&j->mit, j->cfg.home_raw_abs);
    return 1;
}

void joint_update_1khz(Joint* j)
{
    uint8_t e = 0;
    uint16_t raw = as5047p_read_angle_raw14(&j->enc, &e);

    j->last_raw = raw;
    j->last_err = e;

    if (e == 0 && j->mit.zero_valid) {

    	// key controll line
        j->mit.q_des = j->q_target;

        joint_mit_step_1khz(&j->mit, (int32_t)raw);
        odrive_can_set_input_torque(&j->od, j->mit.tau);
    } else {
        odrive_can_set_input_torque(&j->od, 0.0f);
    }
}

#pragma once
#include "stm32h7xx_hal.h"
#include <stdint.h>
#include "as5047p.h"
#include "odrive_can.h"
#include "joint_mit.h"

typedef struct {
    // hardware
    SPI_HandleTypeDef*   hspi;
    GPIO_TypeDef*        cs_port;
    uint16_t             cs_pin;

    FDCAN_HandleTypeDef* hcan;
    uint8_t              node_id;

    // MIT mapping & limits
    int32_t raw_min;
    int32_t raw_max;
    float   gear_ratio;     // 你现在输出轴装编码器 -> 通常设 1.0

    // fixed “power-on home” (absolute raw on output shaft)
    int32_t home_raw_abs;

    // gains
    float kp;
    float kd;
    float tau_max;

    // backlash/friction helpers
    float q_db;
    float q_hold_band;
    float qd_alpha;
    float tau_rate_limit;

    int8_t torque_sign;     // +1 or -1
} JointConfig;

typedef struct {
    AS5047P   enc;
    ODriveCan od;
    JointMIT  mit;

    JointConfig cfg;

    uint16_t last_raw;
    uint8_t  last_err;
} Joint;

void joint_init(Joint* j, const JointConfig* cfg);
void joint_odrive_enable_torque_mode(Joint* j);
uint8_t joint_zero_to_home(Joint* j);      // return 1 ok, 0 fail
void joint_update_1khz(Joint* j);          // read encoder -> MIT -> send torque

// optional helpers
static inline float joint_get_q(const Joint* j)  { return j->mit.q;  }
static inline float joint_get_qd(const Joint* j) { return j->mit.qd; }
static inline float joint_get_tau(const Joint* j){ return j->mit.tau;}

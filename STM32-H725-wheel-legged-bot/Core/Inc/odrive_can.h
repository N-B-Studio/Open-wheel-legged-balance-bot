#pragma once
#include "fdcan.h"
#include <stdint.h>

typedef struct {
    FDCAN_HandleTypeDef *hcan;
    uint8_t node_id;
} ODriveCan;

void odrive_can_clear_errors(ODriveCan *od);
void odrive_can_set_axis_state(ODriveCan *od, uint32_t state);
void odrive_can_set_controller_mode(ODriveCan *od, uint32_t control_mode, uint32_t input_mode);
void odrive_can_set_input_torque(ODriveCan *od, float tau_nm);

#include "odrive_can.h"
#include "bsp_fdcan.h"

#define CMD_SET_AXIS_STATE       0x07
#define CMD_SET_CONTROLLER_MODE  0x0B
#define CMD_SET_INPUT_TORQUE     0x0E
#define CMD_CLEAR_ERRORS         0x18

static inline uint16_t odrive_build_id(uint8_t node_id, uint8_t cmd_id) {
    return ((uint16_t)node_id << 5) | (cmd_id & 0x1F);
}

void odrive_can_clear_errors(ODriveCan *od) {
    uint8_t data[8] = {0};
    uint16_t id = odrive_build_id(od->node_id, CMD_CLEAR_ERRORS);
    // DLC=0 才是正确的“空 payload”命令
    fdcanx_send_data(od->hcan, id, data, 0);
}

void odrive_can_set_axis_state(ODriveCan *od, uint32_t state) {
    uint8_t data[8] = {0};
    data[0] = (uint8_t)(state & 0xFF);
    data[1] = (uint8_t)((state >> 8) & 0xFF);
    data[2] = (uint8_t)((state >> 16) & 0xFF);
    data[3] = (uint8_t)((state >> 24) & 0xFF);

    uint16_t id = odrive_build_id(od->node_id, CMD_SET_AXIS_STATE);
    fdcanx_send_data(od->hcan, id, data, 4);
}

void odrive_can_set_controller_mode(ODriveCan *od, uint32_t control_mode, uint32_t input_mode) {
    uint8_t data[8];

    data[0] = (uint8_t)(control_mode & 0xFF);
    data[1] = (uint8_t)((control_mode >> 8) & 0xFF);
    data[2] = (uint8_t)((control_mode >> 16) & 0xFF);
    data[3] = (uint8_t)((control_mode >> 24) & 0xFF);

    data[4] = (uint8_t)(input_mode & 0xFF);
    data[5] = (uint8_t)((input_mode >> 8) & 0xFF);
    data[6] = (uint8_t)((input_mode >> 16) & 0xFF);
    data[7] = (uint8_t)((input_mode >> 24) & 0xFF);

    uint16_t id = odrive_build_id(od->node_id, CMD_SET_CONTROLLER_MODE);
    fdcanx_send_data(od->hcan, id, data, 8);
}

void odrive_can_set_input_torque(ODriveCan *od, float tau_nm) {
    uint8_t data[8] = {0};
    union { float f; uint8_t b[4]; } u;
    u.f = tau_nm;

    data[0] = u.b[0];
    data[1] = u.b[1];
    data[2] = u.b[2];
    data[3] = u.b[3];

    uint16_t id = odrive_build_id(od->node_id, CMD_SET_INPUT_TORQUE);
    fdcanx_send_data(od->hcan, id, data, 4);
}

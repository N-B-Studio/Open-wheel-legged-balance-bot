#pragma once
#include <stdint.h>

typedef struct {
    float l1; // thigh
    float l2; // calf
} LegGeom;

// 2D FK/IK：先做 hip_pitch + knee_pitch 的平面
void leg_fk_2d(float q_hip, float q_knee, const LegGeom* g, float* x, float* z);
int  leg_ik_2d(float x, float z, const LegGeom* g, float* q_hip, float* q_knee);

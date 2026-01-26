#pragma once
#include <stdint.h>

/* Leg geometry: two-link serial kinematic chain */
typedef struct {
    float l1;  /* Thigh link length (m) */
    float l2;  /* Calf link length (m) */
} LegGeom;

/* 2D Forward Kinematics: (q_hip, q_knee) -> (x, z)
 * Computes end-effector position from joint angles in the sagittal plane
 * Coordinate system: x = forward, z = -down (negative downward) */
void leg_fk_2d(float q_hip, float q_knee, const LegGeom* g, float* x, float* z);

/* 2D Inverse Kinematics: (x, z) -> (q_hip, q_knee)
 * Computes joint angles from desired end-effector position in sagittal plane */
int leg_ik_2d(float x, float z, const LegGeom* g, float* q_hip, float* q_knee);

/* 3D Inverse Kinematics: (x, y, z) -> (q_roll, q_hip, q_knee)
 * Decomposes 3D position into hip_roll (frontal) and 2D IK (sagittal)
 * Algorithm: Rotate 3D point by roll angle to project onto sagittal plane */
int leg_ik_3d(float x, float y, float z, const LegGeom* g, float* q_roll, float* q_hip, float* q_knee);

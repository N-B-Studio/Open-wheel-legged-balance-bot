#include "leg_kin.h"
#include <math.h>

/* ========== 2D FORWARD KINEMATICS ==========
 * Input:  q_hip (hip pitch), q_knee (knee pitch)
 * Output: (x, z) end-effector position
 * Model:  Serial 2-link: hip -> thigh (l1) -> knee -> calf (l2)
 * Coords:  x = horizontal (forward positive), z = vertical (downward negative) */
void leg_fk_2d(float q_hip, float q_knee, const LegGeom* g, float* x, float* z)
{
    /* Compute absolute angles and lengths for each segment */
    *x = g->l1 * sinf(q_hip) + g->l2 * sinf(q_hip + q_knee);
    *z = -g->l1 * cosf(q_hip) - g->l2 * cosf(q_hip + q_knee);
}

/* ========== 2D INVERSE KINEMATICS ==========
 * Input:  (x, z) desired end-effector position
 * Output: (q_hip, q_knee) joint angles
 * Method: Law of cosines + geometric decomposition */
int leg_ik_2d(float x, float z,
              const LegGeom *g,
              float *q_hip,
              float *q_knee)
{
    float l1 = g->l1;  /* Thigh length */
    float l2 = g->l2;  /* Calf length */

    /* --- Section 1: Reach validation --- */
    /* Distance from origin to target */
    float d = sqrtf(x*x + z*z);

    /* Check if target is within workspace
     * Reachable if: (l1 - l2) <= d <= (l1 + l2) */
    if (d > (l1 + l2) || d < fabsf(l1 - l2)) {
        return -1;  /* Target unreachable */
    }

    /* --- Section 2: Knee angle (law of cosines) --- */
    /* d² = l1² + l2² - 2·l1·l2·cos(π - q_knee) */
    /* Therefore: cos(q_knee) = (l1² + l2² - d²) / (2·l1·l2) */
    float cos_knee = (l1*l1 + l2*l2 - d*d) / (2.0f*l1*l2);
    cos_knee = fmaxf(-1.0f, fminf(1.0f, cos_knee));  /* Clamp for numerical stability */
    *q_knee = M_PI - acosf(cos_knee);

    /* --- Section 3: Hip angle (geometric decomposition) --- */
    /* alpha = angle of target vector (x, -z) from horizontal */
    float alpha = atan2f(x, -z);
    /* beta  = angle between thigh and target vector (law of cosines) */
    float beta = acosf((l1*l1 + d*d - l2*l2) / (2.0f*l1*d));
    /* Hip angle = alpha - beta */
    *q_hip = alpha - beta;

    return 0;
}

/* ========== 3D INVERSE KINEMATICS ==========
 * Input:  (x, y, z) desired 3D end-effector position
 * Output: (q_roll, q_hip, q_knee) three joint angles
 * Method: Decouple into (1) hip_roll from frontal plane, (2) 2D IK on sagittal plane
 *
 * Coordinate system:
 *   x = forward (sagittal), y = lateral (frontal), z = vertical (downward negative)
 * Algorithm:
 *   1. Compute roll angle from y-z projection
 *   2. Rotate (y,z) by roll to eliminate lateral component (y' = 0)
 *   3. Apply 2D IK to (x, z_projected) */
int leg_ik_3d(float x, float y, float z,
              const LegGeom* g,
              float* q_roll, float* q_hip, float* q_knee)
{
    /* --- Section 1: Estimate hip roll from frontal plane (y, -z) --- */
    /* Standard standing: z < 0 (leg extends downward)
     * Small roll angles: atan2(y, -z) stays in reasonable range [-π/2, π/2] */
    float qr = atan2f(y, -z);

    /* --- Section 2: Project 3D point onto sagittal plane by rotating (y,z) by qr --- */
    /* Rotation matrix around X-axis (roll rotation):
     *   [1    0      0   ] [x]     [x]
     *   [0  cos(qr) sin(qr)] [y]  =  [y·cos(qr) - z·sin(qr)]
     *   [0 -sin(qr) cos(qr)] [z]     [y·sin(qr) + z·cos(qr)]
     *
     * After rotation, new y' should be 0, and z_projected = y·sin(qr) + z·cos(qr)
     * This z_projected is the effective height in the sagittal plane */
    float c = cosf(qr);
    float s = sinf(qr);
    float z_p = y*s + z*c;  /* Projected z in sagittal plane (negative when standing) */

    /* --- Section 3: Solve 2D IK on projected sagittal plane --- */
    float qh = 0.0f, qk = 0.0f;
    int ok = leg_ik_2d(x, z_p, g, &qh, &qk);
    if (ok != 0) return ok;

    *q_roll = qr;
    *q_hip  = qh;
    *q_knee = qk;
    return 0;
}

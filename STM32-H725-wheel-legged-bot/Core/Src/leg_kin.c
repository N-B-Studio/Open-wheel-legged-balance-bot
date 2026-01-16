#include "leg_kin.h"
#include <math.h>

void leg_fk_2d(float q_hip, float q_knee, const LegGeom* g, float* x, float* z)
{
    *x = g->l1 * sinf(q_hip) + g->l2 * sinf(q_hip + q_knee);
    *z = -g->l1 * cosf(q_hip) - g->l2 * cosf(q_hip + q_knee);
}

int leg_ik_2d(float x, float z,
              const LegGeom *g,
              float *q_hip,
              float *q_knee)
{
    float l1 = g->l1;
    float l2 = g->l2;

    float d = sqrtf(x*x + z*z);

    // reachability check
    if (d > (l1 + l2) || d < fabsf(l1 - l2)) {
        return -1; // unreachable
    }

    // knee
    float cos_knee = (l1*l1 + l2*l2 - d*d) / (2.0f*l1*l2);
    cos_knee = fmaxf(-1.0f, fminf(1.0f, cos_knee));
    *q_knee = M_PI - acosf(cos_knee);

    // hip
    float alpha = atan2f(x, -z);
    float beta  = acosf((l1*l1 + d*d - l2*l2) / (2.0f*l1*d));
    *q_hip = alpha - beta;

    return 0;
}

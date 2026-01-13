#include "leg_kin.h"
#include <math.h>

void leg_fk_2d(float q_hip, float q_knee, const LegGeom* g, float* x, float* z)
{
    *x = g->l1 * sinf(q_hip) + g->l2 * sinf(q_hip + q_knee);
    *z = -g->l1 * cosf(q_hip) - g->l2 * cosf(q_hip + q_knee);
}

int leg_ik_2d(float x, float z, const LegGeom* g, float* q_hip, float* q_knee)
{
    float l1 = g->l1, l2 = g->l2;
    float D = (x*x + z*z - l1*l1 - l2*l2) / (2.0f*l1*l2);
    if (D < -1.0f || D > 1.0f) return 0;

    float qk = acosf(D);
    float qh = atan2f(x, -z) - atan2f(l2*sinf(qk), l1 + l2*cosf(qk));

    *q_knee = qk;
    *q_hip  = qh;
    return 1;
}

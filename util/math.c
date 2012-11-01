

#include "math.h"


void quat_rot_vec(vec3_t *v_out, vec3_t *v_in, quat_t *quat)
{
   float r = quat->q0;
   float i = quat->q1;
   float j = quat->q2;
   float k = quat->q3;
   v_out->x = 2 * (r * v_in->z * j + i * v_in->z * k - r * v_in->y * k + i * v_in->y * j) + v_in->x * (r * r + i * i - j * j - k * k);
   v_out->y = 2 * (r * v_in->x * k + i * v_in->x * j - r * v_in->z * i + j * v_in->z * k) + v_in->y * (r * r - i * i + j * j - k * k);
   v_out->z = 2 * (r * v_in->y * i - r * v_in->x * j + i * v_in->x * k + j * v_in->y * k) + v_in->z * (r * r - i * i - j * j + k * k);
}


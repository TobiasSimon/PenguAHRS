

#ifndef __AHRS_UTIL_H__
#define __AHRS_UTIL_H__


#include "../util/math.h"


/* fast inverse square-root
   see: http://en.wikipedia.org/wiki/Fast_inverse_square_root */
float inv_sqrt(float x);

void quaternion_init(quat_t *quat, float ax, float ay, float az, float mx, float my, float mz);

void quat_to_euler(euler_t *euler, quat_t *quat);


#endif /* __AHRS_UTIL_H__ */


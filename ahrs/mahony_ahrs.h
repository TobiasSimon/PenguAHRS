
/*
 * Madgwick's implementation of Mayhony's AHRS algorithm.
 * See: http://www.x-io.co.uk/node/8#open_source_ahrs_and_imu_algorithms
 *
 * (C) SOH Madgwick, 2011
 * (C) Tobias Simon, 2012
 */

#ifndef __MAHONY_AHRS_H__
#define __MAHONY_AHRS_H__


typedef struct
{
   float twoKp; /* 2 * proportional gain (Kp) */
   float twoKi; /* 2 * integral gain (Ki) */
   float integralFBx;
   float integralFBy;
   float integralFBz;
   float q0, q1, q2, q3; /* quaternion of sensor frame relative to auxiliary frame */
}
mahony_ahrs_t;


void mahony_ahrs_init(mahony_ahrs_t *ahrs, float Kp, float Ki);

void mahony_ahrs_update(mahony_ahrs_t *ahrs,
                        float gx, float gy, float gz,
                        float ax, float ay, float az,
                        float mx, float my, float mz, float dt);


#endif /* __MAHONY_AHRS_H__ */


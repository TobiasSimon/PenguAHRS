//=====================================================================================================
// MadgwickAHRS.h
//=====================================================================================================
//
// Implementation of Madgwick's IMU and AHRS algorithms.
// See: http://www.x-io.co.uk/node/8#open_source_ahrs_and_imu_algorithms
//
// Date			Author          Notes
// 29/09/2011	SOH Madgwick    Initial release
// 02/10/2011	SOH Madgwick	Optimised for reduced CPU load
//
//=====================================================================================================

#ifndef __MADGWICK_AHRS_H__
#define __MADGWICK_AHRS_H__


typedef struct
{
   float beta; /* 2 * beta (Kp) */
   int initialized;
   float q0, q1, q2, q3; /* quaternion of sensor frame relative to auxiliary frame */
}
madgwick_ahrs_t;


void madgwick_ahrs_update(madgwick_ahrs_t *ahrs,
                          float gx, float gy, float gz,
                          float ax, float ay, float az,
                          float mx, float my, float mz,
                          float accelCutoff, float dt);


#endif /* __MADGWICK_AHRS_H__ */

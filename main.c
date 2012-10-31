

#include "chips/itg3200/itg3200.h"
#include "chips/bma180/bma180.h"
#include "chips/hmc5883/hmc5883.h"

#include "i2c/i2c.h"
#include <stdio.h>
#include <string.h>
#include <time.h>

#include "ahrs/mahony_ahrs.h"
#include "ahrs/madgwick_ahrs.h"
#include "ahrs/ekf.h"
#include "ahrs/util.h"

#include <math.h>


int64_t ts_diff(struct timespec *timeA_p, struct timespec *timeB_p)
{
   return ((timeA_p->tv_sec * 1000000000) + timeA_p->tv_nsec) -
          ((timeB_p->tv_sec * 1000000000) + timeB_p->tv_nsec);
}


int main(void)
{
   i2c_bus_t bus;
   i2c_bus_open(&bus, "/dev/i2c-0");

   /* ITG: */
   itg3200_dev_t itg;
   itg3200_init(&itg, &bus, ITG3200_DLPF_42HZ);

   /* BMA: */
   bma180_dev_t bma;
   bma180_init(&bma, &bus, BMA180_RANGE_4G, BMA180_BW_40HZ);
   bma180_avg_acc(&bma);

   /* HMC: */
   hmc5883_dev_t hmc;
   hmc5883_init(&hmc, &bus);

   
   /* estimator init using sensor readings: */
   bma180_read_acc(&bma);
   hmc5883_read(&hmc);

   mahony_ahrs_t mahony_ahrs;
   mahony_ahrs_init(&mahony_ahrs, 10.5, 0.0f);
   //quaternion_init(&mahony_ahrs.quat, bma.acc.x, bma.acc.y, bma.acc.z, hmc.raw.x, hmc.raw.y, hmc.raw.z);
    
   madgwick_ahrs_t madgwick_ahrs;
   madgwick_ahrs_init(&madgwick_ahrs, 1.0);
   quaternion_init(&madgwick_ahrs.quat, bma.acc.x, bma.acc.y, bma.acc.z, hmc.raw.x, hmc.raw.y, hmc.raw.z);

   struct timespec curr, prev;
   clock_gettime(CLOCK_MONOTONIC, &prev);
   while (1)
   {
      clock_gettime(CLOCK_MONOTONIC, &curr);
      float dt = (float)ts_diff(&curr, &prev) / 1000000000.0;
      prev = curr; 
      
      /* sensor data acquisition: */
      itg3200_read_gyro(&itg);
      bma180_read_acc(&bma);
      hmc5883_read(&hmc);
      
      /* state estimates and output: */
      euler_t euler;
      
      //madgwick_ahrs_update(&madgwick_ahrs, itg.gyro.x, itg.gyro.y, itg.gyro.z, bma.raw.x, bma.raw.y, bma.raw.z, hmc.raw.x, hmc.raw.y, hmc.raw.z, 11.0, dt);
      madgwick_ahrs_update(&madgwick_ahrs, 0, 0, 0, bma.raw.x, bma.raw.y, bma.raw.z, hmc.raw.x, hmc.raw.y, hmc.raw.z, 11.0, dt);
      printf("%f %f %f %f\n", madgwick_ahrs.quat.q0, madgwick_ahrs.quat.q1, madgwick_ahrs.quat.q2, madgwick_ahrs.quat.q3);

      //quat_to_euler(&euler, &madgwick_ahrs.quat);
      //printf("%f %f %f %f %f\n", fmod(euler.yaw * 180.0 / M_PI + 360.0, 360.0), euler.pitch * 180.0 / M_PI, euler.roll * 180.0 / M_PI, bma.acc.x, bma.acc.y);
      
      //mahony_ahrs_update(&mahony_ahrs, 0, 0, 0, bma.acc.x, bma.acc.y, -bma.acc.z, hmc.raw.x, hmc.raw.y, hmc.raw.z, dt);
      //quat_to_euler(&euler, &mahony_ahrs.quat);
      //printf("%f %f %f\n", euler.yaw, euler.pitch, euler.roll);
      //printf("%f %f %f\n", hmc.mag.x, hmc.mag.y, hmc.mag.z);
      fflush(stdout);
   }
   return 0;
}


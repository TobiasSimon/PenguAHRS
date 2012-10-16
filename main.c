

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

#include <math.h>


struct
{
   float x;
   float y;
   float z;
}
euler;


void euler_angles(float q0, float q1, float q2, float q3)
{
   float s = q0;
   struct
   {
      float x;
      float y;
      float z;
   }
   v;
   v.x = q1;
   v.y = q2;
   v.z = q3;  

   float sqw = s*s;    
   float sqx = v.x*v.x;    
   float sqy = v.y*v.y;    
   float sqz = v.z*v.z;    

   euler.x = atan2f(2.f * (v.x*v.y + v.z*s), sqx - sqy - sqz + sqw);          
   euler.y = asinf(-2.f * (v.x*v.z - v.y*s));
   euler.z = atan2f(2.f * (v.y*v.z + v.x*s), -sqx - sqy + sqz + sqw);    
}



int64_t ts_diff(struct timespec *timeA_p, struct timespec *timeB_p)
{
   return ((timeA_p->tv_sec * 1000000000) + timeA_p->tv_nsec) -
          ((timeB_p->tv_sec * 1000000000) + timeB_p->tv_nsec);
}


int main(void)
{
   i2c_bus_t bus;
   i2c_bus_open(&bus, "/dev/i2c-0");

   itg3200_dev_t itg;
   itg3200_init(&itg, &bus, ITG3200_DLPF_42HZ);

   bma180_dev_t bma;
   bma180_init(&bma, &bus, BMA180_RANGE_4G, BMA180_BW_40HZ);
   bma180_avg_acc(&bma);

   printf("%f %f %f\n", bma.avg.x, bma.avg.y, bma.avg.z);

   hmc5883_dev_t hmc;
   hmc5883_init(&hmc, &bus);

   struct timespec curr, prev;
   clock_gettime(CLOCK_MONOTONIC, &prev);

   mahony_ahrs_t mahony_ahrs;
   mahony_ahrs_init(&mahony_ahrs, 0.5f, 0.0f);
    
   bma180_read_acc(&bma);
   hmc5883_read(&hmc);
   madgwick_ahrs_t madgwick_ahrs;
   madgwick_ahrs_init(&madgwick_ahrs, bma.acc.x, bma.acc.y, bma.acc.z, hmc.raw.x, hmc.raw.y, hmc.raw.z, 0.1);
   
   memset(&gConfig, 0, sizeof(gConfig));

   /* set-up reference vectors: */
   gConfig.acc_ref.x = bma.avg.x;
   gConfig.acc_ref.y = bma.avg.y;
   gConfig.acc_ref.z = bma.avg.z;

   gConfig.mag_ref.x = 1000;
   gConfig.mag_ref.y = 0;
   gConfig.mag_ref.z = 0;

   /* set-up cross-axis alignment: */
   identity_3x3(&gConfig.gyro_alignment);
   identity_3x3(&gConfig.acc_alignment);
   identity_3x3(&gConfig.mag_cal);

   /* set-up scales: */
   gConfig.gyro_scales.x = 1.0;
   gConfig.gyro_scales.y = 1.0;
   gConfig.gyro_scales.z = 1.0;

   /* set-up covariances: */
   gConfig.process_covariance = 1.0;
   gConfig.acc_covariance = 100000.0;
   gConfig.mag_covariance = 1000.0;

   ekf_init();

   raw_sensor_data_t sensor_data;
   
   
   while (1)
   {
      clock_gettime(CLOCK_MONOTONIC, &curr);
      float dt = (float)ts_diff(&curr, &prev) / 1000000000.0;
      prev = curr; 
      
      itg3200_read_gyro(&itg);
      bma180_read_acc(&bma);
      hmc5883_read(&hmc);
    
      sensor_data.gyro.x = itg.gyro.x;
      sensor_data.gyro.y = itg.gyro.y;
      sensor_data.gyro.z = itg.gyro.z;

      sensor_data.acc.x = bma.acc.x;
      sensor_data.acc.y = bma.acc.y;
      sensor_data.acc.z = bma.acc.z;

      sensor_data.mag.x = hmc.raw.x;
      sensor_data.mag.y = hmc.raw.y;
      sensor_data.mag.z = hmc.raw.z;

     
      mahony_ahrs_update(&mahony_ahrs, itg.gyro.x, itg.gyro.y, itg.gyro.z, bma.acc.x, bma.acc.y, bma.acc.z, 0.0, 0.0, 0.0, /*hmc.raw.x, hmc.raw.y, hmc.raw.z,*/ dt);
      euler_angles(mahony_ahrs.q0, mahony_ahrs.q1, mahony_ahrs.q2, mahony_ahrs.q3);

      //printf("%f %f %f\n", itg.gyro.x, itg.gyro.y, itg.gyro.z);
      //printf("%f %f %f\n", bma.acc.x, bma.acc.y, bma.acc.z - 9.81);
      /*printf("(%.1f, %.1f, %.1f); ", hmc.raw.x, hmc.raw.y, hmc.raw.z);*/
      //printf("y: %.1f, p: %.1f, r: %.1f\n", euler.x, euler.y, euler.z);
      
      sensor_data.new_acc_data = 1;
      sensor_data.new_mag_data = 1;
	  ekf_run(&sensor_data, dt);
      //printf("\rphi = %f, psi = %f, theta = %f    ", ekf_state.phi, ekf_state.psi, ekf_state.theta);
      printf("%f %f %f 0 0 0 0 0 0 0 0 0 0 0 0 0\n", ekf_state.theta, ekf_state.phi, ekf_state.psi);
      ///printf("%f %f %f 0 0 0 0 0 0 0 0 0 0 0 0 0\n", euler.y, euler.z, euler.x);
	  fflush(stdout);
   }
   return 0;
}


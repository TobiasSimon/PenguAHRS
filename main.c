

#include "chips/itg3200/itg3200.h"
#include "chips/bma180/bma180.h"
#include "i2c/i2c.h"
#include "ekf/ekf.h"
#include <stdio.h>
#include <string.h>
#include <time.h>


int64_t ts_diff(struct timespec *timeA_p, struct timespec *timeB_p)
{
   return ((timeA_p->tv_sec * 1000000000) + timeA_p->tv_nsec) -
          ((timeB_p->tv_sec * 1000000000) + timeB_p->tv_nsec);
}


int main(void)
{

   memset(&gConfig, 0, sizeof(gConfig));

   /* set-up reference vectors: */
   gConfig.acc_ref.x = 0;
   gConfig.acc_ref.y = 0;
   gConfig.acc_ref.z = -1;

   gConfig.mag_ref.x = 1;
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
   gConfig.process_covariance = 10.0;
   gConfig.acc_covariance = 1000.0;
   gConfig.mag_covariance = 1000.0;

   ekf_init();

   raw_sensor_data_t sensor_data;
   sensor_data.new_acc_data = 0;
   sensor_data.acc.x = 0.0;
   sensor_data.acc.y = 0.0;
   sensor_data.acc.z = 1.0;

   sensor_data.new_mag_data = 0;
   sensor_data.mag.x = 1.0;
   sensor_data.mag.y = 0.0;
   sensor_data.mag.z = 0.0;

   i2c_bus_t bus;
   i2c_bus_open(&bus, "/dev/i2c-4");
   itg3200_dev_t itg;
   itg3200_init(&itg, &bus, ITG3200_DLPF_42HZ);
   bma180_dev_t bma;
   bma180_init(&bma, &bus, BMA180_RANGE_4G, BMA180_BW_10HZ);


   struct timespec curr, prev;
   clock_gettime(CLOCK_MONOTONIC, &prev);
   while (1)
   {
      clock_gettime(CLOCK_MONOTONIC, &curr);
      float dt = (float)ts_diff(&curr, &prev) / 1000000000.0;
      prev = curr; 
      
      itg3200_read_gyro(&itg);
      sensor_data.gyro.x = itg.gyro.x;
      sensor_data.gyro.y = itg.gyro.y;
      sensor_data.gyro.z = itg.gyro.z;
      
      bma180_read_acc(&bma);
      sensor_data.acc.x = bma.acc.x;
      sensor_data.acc.y = bma.acc.y;
      sensor_data.acc.z = bma.acc.z;

      ekf_run(&sensor_data, dt);
      printf("phi = %.1f, psi = %.1f, theta = %.1f\n", ekf_state.phi, ekf_state.psi, ekf_state.theta);
      
      /*printf("\rgyro_xyz: %f %f %f, temperature: %f  ",
         dev.gyro.x, dev.gyro.y, dev.gyro.z,
         dev.temperature);*/
   }
   return 0;
}


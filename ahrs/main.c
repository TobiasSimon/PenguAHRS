

#include "ekf.h"
#include <string.h>
#include <stdio.h>
#include <fenv.h>



int main(void)
{
   feenableexcept(FE_DIVBYZERO | FE_INVALID | FE_OVERFLOW);
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
   sensor_data.gyro.x = 0.01;
   sensor_data.gyro.y = 0.01;
   sensor_data.gyro.z = 0.0;
   
   sensor_data.acc.x = 0.0;
   sensor_data.acc.y = 0.0;
   sensor_data.acc.z = 1.0;
  
   sensor_data.mag.x = 1.0;
   sensor_data.mag.y = 0.0;
   sensor_data.mag.z = 0.0;

   int i = 0;
   while (1) //for (i = 0; i < 10; i++)
   {
      sensor_data.new_acc_data = 1;
      sensor_data.new_mag_data = 1;
      ekf_run(&sensor_data, 0.01);
      printf("phi = %f, psi = %f, theta = %f\n", ekf_state.phi, ekf_state.psi, ekf_state.theta);
   }
   return 0;
}



#include "chips/itg3200/itg3200.h"
#include "chips/bma180/bma180.h"
#include "chips/hmc5883/hmc5883.h"

#include "i2c/i2c.h"
#include <stdio.h>
#include <string.h>
#include <time.h>

#include "ahrs/madgwick_ahrs.h"
#include "ahrs/util.h"
#include "util/udp4.h"
#include "util/interval.h"

#include <stdlib.h>
#include <math.h>


#define STANDARD_BETA 0.5
#define START_BETA STANDARD_BETA
#define BETA_STEP  0.01
#define FINAL_BETA 0.01


int main(void)
{
   i2c_bus_t bus;
   int ret = i2c_bus_open(&bus, "/dev/i2c-4");
   if (ret < 0)
   {
      return EXIT_FAILURE;
   }

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

   madgwick_ahrs_t madgwick_ahrs;
   madgwick_ahrs_init(&madgwick_ahrs, STANDARD_BETA);

   interval_t interval;
   interval_init(&interval);
   float init = START_BETA;

   udp_socket_t *socket = udp_socket_create("127.0.0.1", 5005, 0, 0);

   while (1)
   {
      float dt = interval_measure(&interval);
      init -= BETA_STEP;
      if (init < FINAL_BETA)
      {
         init = FINAL_BETA;
      }
      madgwick_ahrs.beta = init;
      
      /* sensor data acquisition: */
      itg3200_read_gyro(&itg);
      bma180_read_acc(&bma);
      hmc5883_read(&hmc);
      
      /* state estimates and output: */
      euler_t euler;
      
      madgwick_ahrs_update(&madgwick_ahrs, itg.gyro.x, itg.gyro.y, itg.gyro.z, bma.raw.x, bma.raw.y, bma.raw.z, hmc.raw.x, hmc.raw.y, hmc.raw.z, 11.0, dt);
      char buffer[1024];
      int len = sprintf(buffer, "%f %f %f %f", madgwick_ahrs.quat.q0, madgwick_ahrs.quat.q1, madgwick_ahrs.quat.q2, madgwick_ahrs.quat.q3);
      udp_socket_send(socket, buffer, len);

      quat_to_euler(&euler, &madgwick_ahrs.quat);
      printf("%f %f %f\n", fmod(euler.yaw * 180.0 / M_PI + 360.0, 360.0), euler.pitch * 180.0 / M_PI, euler.roll * 180.0 / M_PI);
      fflush(stdout);
   }
   return 0;
}


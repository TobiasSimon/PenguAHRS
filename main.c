
/*
   PenguAHRS - A Linux-based Attitude and Heading Reference System

   Copyright (C) 2012 Tobias Simon

   This program is free software; you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation; either version 2 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.
*/


#include "kalman.h"
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
#include "util/math.h"

#include <stdlib.h>
#include <math.h>


#define STANDARD_BETA 0.5
#define START_BETA STANDARD_BETA
#define BETA_STEP  0.001
#define FINAL_BETA 0.01


void die(char *msg, int code)
{
   int index = -code;
   if (index < sys_nerr)
   {
      fprintf(stderr, "fatal error: %s, code %d (%s)\n", msg, code, sys_errlist[index]);
   }
   else
   {
      fprintf(stderr, "fatal error: %s, code %d\n", msg, code);
   }
   exit(EXIT_FAILURE);
}


int main(void)
{
   i2c_bus_t bus;
   int ret = i2c_bus_open(&bus, "/dev/i2c-14");
   if (ret < 0)
   {
      die("could not open i2c bus", ret);
   }

   /* ITG: */
   itg3200_dev_t itg;
   ret = itg3200_init(&itg, &bus, ITG3200_DLPF_42HZ);
   if (ret < 0)
   {
      die("could not inizialize ITG3200", ret);
   }

   /* BMA: */
   bma180_dev_t bma;
   bma180_init(&bma, &bus, BMA180_RANGE_4G, BMA180_BW_40HZ);

   /* HMC: */
   hmc5883_dev_t hmc;
   hmc5883_init(&hmc, &bus);

   /* initialize AHRS filter: */
   madgwick_ahrs_t madgwick_ahrs;
   madgwick_ahrs_init(&madgwick_ahrs, STANDARD_BETA);

   interval_t interval;
   interval_init(&interval);
   float init = START_BETA;
   udp_socket_t *socket = udp_socket_create("127.0.0.1", 5005, 0, 0);

   /* accelerometer low-pass filter: */
   int lp_init[3] = {1, 1, 1};
   vec3_t lp;

   kalman_t kalman1, kalman2, kalman3;
   kalman_init(&kalman1, 1.0, 1.0e1, 0, 0);
   kalman_init(&kalman2, 1.0, 1.0e1, 0, 0);
   kalman_init(&kalman3, 1.0, 1.0e1, 0, 0);
   vec3_t global_acc; /* x = N, y = E, z = D */
   int kalman_init = 500;
   while (1)
   {
      int i;
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
      
      float alpha = 0.01;
      quat_t q_body_to_world;
      quat_copy(&q_body_to_world, &madgwick_ahrs.quat);
      quat_rot_vec(&global_acc, &bma.raw, &q_body_to_world);
      for (i = 0; i < 3; i++)
      {
         if (lp_init[i])
         {
            lp_init[i] = 0;
            lp.vec[i] = global_acc.vec[i];
         }
         else
         {
            lp.vec[i] = (1.0 - alpha) * lp.vec[i] + alpha * global_acc.vec[i];
         }
         global_acc.vec[i] = global_acc.vec[i] - lp.vec[i];
      }
      if (--kalman_init == 0)
      {
         kalman_init = 1;
         kalman_in_t kalman_in;
         kalman_in.dt = dt;
         kalman_in.pos = 0;
         kalman_out_t kalman_out;

         kalman_in.acc = global_acc.x;
         kalman_run(&kalman_out, &kalman1, &kalman_in);
         printf("%f ", kalman_out.pos);
         kalman_in.acc = global_acc.y;
         kalman_run(&kalman_out, &kalman2, &kalman_in);
         printf("%f ", kalman_out.pos);
         kalman_in.acc = -global_acc.z;
         kalman_run(&kalman_out, &kalman3, &kalman_in);
         printf("%f\n", kalman_out.pos);
      }

      //printf("%f %f %f\n", global_acc.x, global_acc.y, global_acc.z);
      
      char buffer[1024];
      int len = sprintf(buffer, "%f %f %f %f %f %f %f", madgwick_ahrs.quat.q0, madgwick_ahrs.quat.q1, madgwick_ahrs.quat.q2, madgwick_ahrs.quat.q3,
                                                        global_acc.x, global_acc.y, global_acc.z);
      udp_socket_send(socket, buffer, len);
      fflush(stdout);
   }
   return 0;
}


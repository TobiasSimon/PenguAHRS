
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
#include "chips/ms5611/ms5611.h"
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
#include "util/sliding_avg.h"

#include <errno.h>
#include <stdlib.h>
#include <math.h>


#define STANDARD_BETA 0.5
#define START_BETA STANDARD_BETA
#define BETA_STEP  0.001
#define FINAL_BETA 0.05


void fatal(char *msg, int code)
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
}

#include <pthread.h>

float alt_start = 0.0;
float alt_rel = 0.0;
pthread_mutex_t mutex = PTHREAD_MUTEX_INITIALIZER;


void *ms5611_reader(void *arg)
{
   ms5611_dev_t *ms = (ms5611_dev_t *)arg;
   ms5611_measure(ms);
   alt_start = ms->c_a;
   float last_alt_rel = 0.0;
   while (1)
   {
      ms5611_measure(ms);
      pthread_mutex_lock(&mutex);
      alt_rel = ms->c_a - alt_start;
      if (fabs(alt_rel - last_alt_rel) > 10.0)
      {
         alt_rel = last_alt_rel;
      }
      pthread_mutex_unlock(&mutex);
   }
}


int main(void)
{
   i2c_bus_t bus;
   int ret = i2c_bus_open(&bus, "/dev/i2c-3");
   if (ret < 0)
   {
      fatal("could not open i2c bus", ret);
      return EXIT_FAILURE;
   }

   /* ITG: */
   itg3200_dev_t itg;
itg_again:
   ret = itg3200_init(&itg, &bus, ITG3200_DLPF_42HZ);
   if (ret < 0)
   {
      fatal("could not inizialize ITG3200", ret);
      if (ret == -EAGAIN)
      {
         goto itg_again;   
      }
      return EXIT_FAILURE;
   }

   /* BMA: */
   bma180_dev_t bma;
   bma180_init(&bma, &bus, BMA180_RANGE_4G, BMA180_BW_10HZ);

   /* HMC: */
   hmc5883_dev_t hmc;
   hmc5883_init(&hmc, &bus);
   
   /* MS: */
   ms5611_dev_t ms;
   ret = ms5611_init(&ms, &bus, MS5611_OSR4096, MS5611_OSR4096);
   if (ret < 0)
   {
      fatal("could not inizialize MS5611", ret);
      return EXIT_FAILURE;
   }
   pthread_t thread;
   pthread_create(&thread, NULL, ms5611_reader, &ms);

   /* initialize AHRS filter: */
   madgwick_ahrs_t madgwick_ahrs;
   madgwick_ahrs_init(&madgwick_ahrs, STANDARD_BETA);

   interval_t interval;
   interval_init(&interval);
   float init = START_BETA;
   udp_socket_t *socket = udp_socket_create("10.0.0.100", 5005, 0, 0);

   /* kalman filter: */
   kalman_t kalman1, kalman2, kalman3;
   kalman_init(&kalman1, 1.0e-6, 1.0e-2, 0, 0);
   kalman_init(&kalman2, 1.0e-6, 1.0e-2, 0, 0);
   kalman_init(&kalman3, 1.0e-6, 1.0e-2, 0, 0);
   vec3_t global_acc; /* x = N, y = E, z = D */
   int init_done = 0;
   int converged = 0;
   sliding_avg_t *avg[3];
   avg[0] = sliding_avg_create(1000, 0.0);
   avg[1] = sliding_avg_create(1000, 0.0);
   avg[2] = sliding_avg_create(1000, -9.81);
   float alt_rel_last = 0.0;
   int udp_cnt = 0;
   while (1)
   {
      int i;
      float dt = interval_measure(&interval);
      init -= BETA_STEP;
      if (init < FINAL_BETA)
      {
         init = FINAL_BETA;
         init_done = 1;
      }
      madgwick_ahrs.beta = init;
      
      /* sensor data acquisition: */
      itg3200_read_gyro(&itg);
      bma180_read_acc(&bma);
      hmc5883_read(&hmc);
      
      /* state estimates and output: */
      euler_t euler;
      madgwick_ahrs_update(&madgwick_ahrs, itg.gyro.x, itg.gyro.y, itg.gyro.z, bma.raw.x, bma.raw.y, bma.raw.z, hmc.raw.x, hmc.raw.y, hmc.raw.z, 11.0, dt);
      
      quat_t q_body_to_world;
      quat_copy(&q_body_to_world, &madgwick_ahrs.quat);
      quat_rot_vec(&global_acc, &bma.raw, &q_body_to_world);
      for (i = 0; i < 3; i++)
      {
         global_acc.vec[i] -= sliding_avg_calc(avg[i], global_acc.vec[i]);
      }
      if (init_done)
      {
         kalman_in_t kalman_in;
         kalman_in.dt = dt;
         kalman_in.pos = 0;
         kalman_out_t kalman_out;

         kalman_in.acc = global_acc.x;
         kalman_run(&kalman_out, &kalman1, &kalman_in);
         kalman_in.acc = global_acc.y;
         kalman_run(&kalman_out, &kalman2, &kalman_in);
         kalman_in.acc = -global_acc.z;
         pthread_mutex_lock(&mutex);
         kalman_in.pos = alt_rel;
         pthread_mutex_unlock(&mutex);
         kalman_run(&kalman_out, &kalman3, &kalman_in);
         if (!converged)
         {
            if (fabs(kalman_out.pos - alt_rel) < 0.1)
            {
               converged = 1;   
               fprintf(stderr, "init done\n");
            }
         }
         if (converged) // && udp_cnt++ > 10)
         {
            if (udp_cnt++ == 10)
            {
               char buffer[1024];
               udp_cnt = 0;
               int len = sprintf(buffer, "%f %f %f %f %f %f %f", madgwick_ahrs.quat.q0, madgwick_ahrs.quat.q1, madgwick_ahrs.quat.q2, madgwick_ahrs.quat.q3,
                                                                 global_acc.x, global_acc.y, global_acc.z);
               udp_socket_send(socket, buffer, len);
            }
            printf("%f %f %f\n", -global_acc.z, alt_rel, kalman_out.pos);
            fflush(stdout);
         }
      }

      
   }
   return 0;
}


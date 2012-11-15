
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
#include "chips/mpu6050/mpu6050.h"

#include "i2c/i2c.h"
#include <stdio.h>
#include <string.h>
#include <time.h>

#include "ahrs/util.h"
#include "util/interval.h"
#include "util/math.h"
#include "util/sliding_avg.h"

#include <errno.h>
#include <stdlib.h>
#include <math.h>


int main(void)
{
   i2c_bus_t bus;
   int ret = i2c_bus_open(&bus, "/dev/i2c-0");
   if (ret < 0)
   {
      printf("could not open i2c bus", ret);
      return EXIT_FAILURE;
   }

   mpu6050_dev_t mpu;
   mpu6050_init(&mpu, &bus, MPU6050_DLPF_CFG_94_98Hz, MPU6050_FS_SEL_500, MPU6050_AFS_SEL_4G);
   while (1)
   {
      mpu6050_read(&mpu);
   }
   return 0;
}


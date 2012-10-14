

#include "chips/itg3200/itg3200.h"
#include "chips/bma180/bma180.h"
#include "chips/hmc5883/hmc5883.h"

#include "i2c/i2c.h"
#include <stdio.h>
#include <string.h>
#include <time.h>

#include "ahrs/mahony_ahrs.h"

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
   i2c_bus_open(&bus, "/dev/i2c-4");

   itg3200_dev_t itg;
   itg3200_init(&itg, &bus, ITG3200_DLPF_42HZ);

   bma180_dev_t bma;
   bma180_init(&bma, &bus, BMA180_RANGE_4G, BMA180_BW_10HZ);

   hmc5883_dev_t hmc;
   hmc5883_init(&hmc, &bus);

   struct timespec curr, prev;
   clock_gettime(CLOCK_MONOTONIC, &prev);

   mahony_ahrs_t mahony_ahrs;
   mahony_ahrs_init(&mahony_ahrs, 0.5f, 0.0f);
   while (1)
   {
      clock_gettime(CLOCK_MONOTONIC, &curr);
      float dt = (float)ts_diff(&curr, &prev) / 1000000000.0;
      prev = curr; 
      
      itg3200_read_gyro(&itg);
      bma180_read_acc(&bma);
      hmc5883_read(&hmc);
      
      mahony_ahrs_update(&mahony_ahrs, itg.gyro.x, itg.gyro.y, itg.gyro.z, bma.acc.x, bma.acc.y, bma.acc.z, hmc.raw.x, hmc.raw.y, hmc.raw.z, dt);
      euler_angles(mahony_ahrs.q0, mahony_ahrs.q1, mahony_ahrs.q2, mahony_ahrs.q3);

      /*printf("(%.1f, %.1f, %.1f); ", itg.gyro.x, itg.gyro.y, itg.gyro.z);
      printf("(%.1f, %.1f, %.1f); ", bma.acc.x, bma.acc.y, bma.acc.z);
      printf("(%.1f, %.1f, %.1f); ", hmc.raw.x, hmc.raw.y, hmc.raw.z);*/
      printf("y: %.1f, p: %.1f, r: %.1f\n", euler.x, euler.y, euler.z);
   }
   return 0;
}


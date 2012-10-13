

#include "i2c-dev.h"
#include "ms5611.h"
#include <fcntl.h>
#include <stdlib.h>
#include <sys/ioctl.h>
#include <stdio.h>



int main(void)
{
   int addr = 0x77;
   int bus_handle = open("/dev/i2c-4", O_RDWR);
   ioctl(bus_handle, I2C_SLAVE, addr);
   ms5611_dev_t dev;
   ms5611_init(&dev, bus_handle, MS5611_OSR4096, MS5611_OSR4096);
   
   float a = 0.05;
   float ila_s = 0.0;
   float fla_s = 0.0;
   
   while (1)
   {
      ms5611_measure(&dev);
      
      /* float compensation: */
      ms5611_compensate_float(&dev, 0);
      double fc_t = dev.c_t;
      double fc_p = dev.c_p;
      double fc_a = dev.c_a;
      if (fla_s == 0.0)
         fla_s = fc_a;
      fla_s = a * fc_a + (1.0 - a) * fla_s;
      
      /* int compensation: */
      ms5611_compensate_int(&dev);
      double ic_t = dev.c_t;
      double ic_p = dev.c_p;
      double ic_a = dev.c_a;
      if (ila_s == 0.0) 
         ila_s = dev.c_a;
      ila_s = a * dev.c_a + (1.0 - a) * ila_s;

      printf("%d %d %f %f %f %f %f %f %f %f\n",
         /* uncompensated values: */
         dev.raw_t, dev.raw_p,
         /* int compensated: */
         ic_t, ic_p, ic_a, ila_s,
         /* float compensated: */
         fc_t, fc_p, fc_a, fla_s
      );

      fflush(stdout);
   }
   return 0;  
}


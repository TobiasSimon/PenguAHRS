
/*
   HMC5883 I2C Linux Userspace Driver

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


#include <string.h>

#include "hmc5883.h"


/* device address */
#define HMC5883_ADDRESS 0x1E

/* registers */
#define HMC5883_CFG_A   0x00
#define HMC5883_CFG_B   0x01
#define HMC5883_CFG_MR  0x02
#define HMC5883_MAGX_H  0x03
#define HMC5883_MAGX_L  0x04
#define HMC5883_MAGY_H  0x05
#define HMC5883_MAGY_L  0x06
#define HMC5883_MAGZ_H  0x07
#define HMC5883_MAGZ_L  0x08
#define HMC5883_STATUS  0x09
#define HMC5883_ID_A    0x0A
#define HMC5883_ID_B    0x0B
#define HMC5883_ID_C    0x0C

/* A register output data rate: */
#define HMC5883_A_ODR_05 0x00
#define HMC5883_A_ODR_1  0x04
#define HMC5883_A_ODR_2  0x08
#define HMC5883_A_ODR_5  0x0C
#define HMC5883_A_ODR_10 0x10
#define HMC5883_A_ODR_20 0x14
#define HMC5883_A_ODR_50 0x18
/* A register measurement config: */
#define HMC5883_A_NORMAL   0x00
#define HMC5883_A_BIAS_POS 0x01
#define HMC5883_A_BIAS_NEG 0x02

/* B register settings: */
#define HMC5883_B_GAIN_0_7 0x00
#define HMC5883_B_GAIN_1   0x20
#define HMC5883_B_GAIN_1_5 0x40
#define HMC5883_B_GAIN_2   0x60
#define HMC5883_B_GAIN_3_2 0x80
#define HMC5883_B_GAIN_3_8 0xA0
#define HMC5883_B_GAIN_4_5 0xC0
#define HMC5883_B_GAIN_6_5 0xE0

/* MR register: */
#define HMC5883_MODE_CONTINUOUS 0x00
#define HMC5883_MODE_SINGLE     0x01
#define HMC5883_MODE_IDLE       0x02
#define HMC5883_MODE_SLEEP      0x02


/* sensitivity conversion table in LSB/Ga */
static float sens_conv_tab[8] =
{
   1602.0, 1300.0, 970.0, 780.0,
   530.0, 460.0, 390.0, 280.0
};
#define CFG_A_2_SENS(v) (sens_conv_tab[((v) >> 5)])



static int hmc5883_std_config(hmc5883_dev_t *dev)
{
   int ret;
   ret = i2c_write_reg(&dev->i2c_dev, HMC5883_CFG_A, HMC5883_A_ODR_50);
   if (ret < 0)
   {
      return ret;
   }
   ret = i2c_write_reg(&dev->i2c_dev, HMC5883_CFG_B, HMC5883_B_GAIN_1);
   if (ret < 0)
   {
      return ret;
   }
   ret = i2c_write_reg(&dev->i2c_dev, HMC5883_CFG_MR, HMC5883_MODE_CONTINUOUS);
   return ret;
}


static int hmc5883_cal(hmc5883_dev_t *dev)
{
   int ret = hmc5883_std_config(dev);
   
   float raw_min[3];
   float raw_max[3];
   int i;
   for (i = 0; i < 1000; i++)
   {
      int ret = hmc5883_read(dev);
      if (ret < 0)
      {
         return ret;
      }
      int j;
      for (j = 0; j < 3; j++)
      {
         raw_min[j] = raw_min[j] < dev->raw.vec[j] ? raw_min[j] : dev->raw.vec[j];
         raw_max[j] = raw_max[j] > dev->raw.vec[j] ? raw_max[j] : dev->raw.vec[j];
         dev->scale.vec[j] = 2.0 / (raw_max[i] - raw_min[i]);
         dev->shift.vec[j] = 1.0 - raw_max[i] * dev->scale.vec[i];
      }
   }
}


int hmc5883_init(hmc5883_dev_t *dev, i2c_bus_t *bus)
{
   int ret = 0;
   
   int i;
   for (i = 0; i < 3; i++)
   {
      dev->scale.vec[i] = 1.0;
      dev->shift.vec[i] = 0.0;
   }
   
   i2c_dev_init(&dev->i2c_dev, bus, HMC5883_ADDRESS);

   uint8_t id[3];
   ret = i2c_read_block_reg(&dev->i2c_dev, HMC5883_ID_A, id, sizeof(id));
   if (ret < 0)
   {
      goto out;
   }
   if (strncmp("H43", id, 3) != 0)
   {
      ret = -1;
      goto out;
   }

   ret = hmc5883_std_config(dev);
   if (ret < 0)
   {
      goto out;
   }
   //ret = hmc5883_cal(dev);

out:
   return ret;
}


int hmc5883_read(hmc5883_dev_t *dev)
{
   uint8_t data[6];
   int ret = i2c_read_block_reg(&dev->i2c_dev, HMC5883_MAGX_H, data, sizeof(data));
   dev->raw.x = (int16_t)((data[0] << 8) | data[1]);
   dev->raw.z = (int16_t)((data[2] << 8) | data[3]);
   dev->raw.y = (int16_t)((data[4] << 8) | data[5]);

   dev->shift.vec[0] = -61.5;
   dev->shift.vec[1] = 66.0;
   dev->shift.vec[2] = 381.5;

   int i;
   for (i = 0; i < 3; i++)
   {
      dev->mag.vec[i] = dev->scale.vec[i] * dev->raw.vec[i] + dev->shift.vec[i];
   }
}


int hmc5883_avg_mag(hmc5883_dev_t *dev)
{
   int i, j, ret = 0;
   memset(dev->avg.vec, 0, sizeof(dev->avg.vec));

   for (i = 0; i < 200; i++)
   {
      ret = hmc5883_read(dev);
      if(ret < 0)
      {
         goto out;
      }
      for (j = 0; j < 3; j++)
      {
         dev->avg.vec[j] += dev->raw.vec[j];
      }
   }
   for (i = 0; i < 3; i++)
   {
      dev->avg.vec[i] /= 200.0;
   }

out:
   return ret;
}


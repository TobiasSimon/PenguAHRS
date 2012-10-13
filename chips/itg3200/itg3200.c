

/*
   ITG3200 I2C Linux Userspace Driver

   Copyright (C) 2012 Jan Roemisch and Tobias Simon

   This program is free software; you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation; either version 2 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program; if not, write to the Free Software
   Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
   MA 02110-1301 USA.
*/


#include <unistd.h>
#include <stdio.h>
#include <math.h>
#include <time.h>

#include "itg3200.h"


#define ITG3200_ADDRESS 0x69


#define ITG3200_DEBUG			1
#define ITG3200_GYRO_INIT_ITER	300

#define ITG3200_WHO_AM_I						0x00
#define ITG3200_SMPLRT_DIV						0x15

#define ITG3200_DLPF_FS							0x16
#define ITG3200_DLPF_FS_DLPF_CFG(x)				((x) & 0x7)
#define ITG3200_DLPF_FS_FS_SEL(x)				(((x) & 0x3) << 3)

#define ITG3200_INT_CFG							0x17
#define ITG3200_INT_CFG_RAW_RDY_EN				(1 << 0)
#define ITG3200_INT_CFG_ITG_RDY_EN				(1 << 2)
#define ITG3200_INT_CFG_INT_ANYRD_RDY_2CLEAR	(1 << 4)
#define ITG3200_INT_CFG_LATCH_INT_EN			(1 << 5)
#define ITG3200_INT_CFG_OPEN					(1 << 6)
#define ITG3200_INT_CFG_ACTL					(1 << 7)

#define ITG3200_INT_STATUS						0x1A
#define ITG3200_INT_STATUS_RAW_DATA_RDY			(1 << 0)
#define ITG3200_INT_STATUS_ITG_RDY				(1 << 2)

#define ITG3200_TEMP_OUT_H						0x1B
#define ITG3200_TEMP_OUT_L						0x1C

#define ITG3200_GYRO_XOUT_H						0x1D
#define ITG3200_GYRO_XOUT_L						0x1E
#define ITG3200_GYRO_YOUT_H						0x1F
#define ITG3200_GYRO_YOUT_L						0x20
#define ITG3200_GYRO_ZOUT_H						0x21
#define ITG3200_GYRO_ZOUT_L						0x22

#define ITG3200_PWR_MGM							0x3E
#define ITG3200_PWR_MGM_H_RESET					(1 << 7)
#define ITG3200_PWR_MGM_SLEEP					(1 << 6)
#define ITG3200_PWR_MGM_STBY_XG					(1 << 5)
#define ITG3200_PWR_MGM_STBY_YG					(1 << 4)
#define ITG3200_PWR_MGM_STBY_ZG					(1 << 3)
#define ITG3200_PWR_MGM_CLK_SEL(x)				((x) & 0x3)



static int read_gyro_raw(itg3200_dev_t *dev, int16_t *data)
{
   uint8_t raw[6];

   /* read gyro registers */
   int ret = i2c_read_block_reg(&dev->i2c_dev, ITG3200_GYRO_XOUT_H, raw, sizeof(raw));
   if (ret < 0)
   {
      return ret;
   }

   int i;
   for(i = 0; i < 3; i++)
   {
      data[i] = (int16_t)((raw[(i << 1)] << 8) | raw[(i << 1) + 1]);
   }

   return 0;
}


static int zero_gyros(itg3200_dev_t *dev)
{
   int16_t val[3];
   int64_t avg[3] = {0, 0, 0};

   int n;
   for (n = 0; n < ITG3200_GYRO_INIT_ITER; n++) 
   {
      int ret = read_gyro_raw(dev, val);
      if (ret < 0)
      {
         return ret;
      }

      int i;
      for (i = 0; i < 3; i++)
      {
         avg[i] += (int64_t)(val[i]);
      }
   }

   int i;
   for (i = 0; i < 3; i++)
   {
      dev->bias[i] = -(int16_t)(avg[i] / (int64_t)(ITG3200_GYRO_INIT_ITER));
   }

   return 0;
}


int itg3200_init(itg3200_dev_t *dev, i2c_bus_t *bus, itg3200_dlpf_t filter)
{
   int ret = 0;
   i2c_dev_lock_bus(&dev->i2c_dev);

   /* copy values */
   i2c_dev_init(&dev->i2c_dev, bus, ITG3200_ADDRESS);
   dev->lp_filter = filter;

   /* reset */
   ret = i2c_write_reg(&dev->i2c_dev, ITG3200_PWR_MGM, ITG3200_PWR_MGM_H_RESET);
   if (ret < 0)
   {
      goto out;
   }

   /* read back it's address */
   ret = i2c_read_reg(&dev->i2c_dev, ITG3200_WHO_AM_I);
   if (ret < 0)
   {
      goto out;
   }

   /* validate address: */
   uint8_t addr = (uint8_t)(ret);
   if (dev->i2c_dev.addr != addr)
   {
      ret = -1;
      goto out;
   }

#ifdef ITG3200_DEBUG
   printf("ITG3200, i2c_addr: %.2X\n", addr);
#endif

   /* set z-gyro as clock source */
   ret = i2c_write_reg(&dev->i2c_dev, ITG3200_PWR_MGM, ITG3200_PWR_MGM_CLK_SEL(0x3));
   if (ret < 0)
   {
      goto out;
   }

   /* set full scale mode and low-pass filter */
   ret = i2c_write_reg(&dev->i2c_dev, ITG3200_DLPF_FS, ITG3200_DLPF_FS_FS_SEL(0x3) | ITG3200_DLPF_FS_DLPF_CFG(dev->lp_filter));
   if (ret < 0)
   {
      goto out;
   }

   /* 70ms for gyro startup */
   i2c_dev_sleep(&dev->i2c_dev, 70);

#ifdef ITG3200_DEBUG
   printf("ITG3200, calibrating\n");
#endif
   
   /* calibrate: */
   ret = zero_gyros(dev);

out:
   i2c_dev_unlock_bus(&dev->i2c_dev);
   return ret;
}


int itg3200_read_gyro(itg3200_dev_t *dev)
{
   int16_t val[3];
   int ret = read_gyro_raw(dev, val);
   if (ret < 0)
   {
      return ret;
   }

   /* construct, scale and bias-correct values: */
   int i;
   for (i = 0; i < 3; i++)
   {
      dev->gyro.data[i] = ((float)(val[i] + dev->bias[i]) / 14.375) * M_PI / 180.0;
   }
   return 0;
}


int itg3200_read_temp(itg3200_dev_t *dev)
{
   uint8_t raw[2];

   /* read temperature registers: */
   int ret = i2c_read_block_reg(&dev->i2c_dev, ITG3200_TEMP_OUT_H, raw, sizeof(raw));
   if (ret < 0)
   {
      return ret;
   }

   /* construct and scale value: */
   dev->temperature = (3500.0 + (float)((int16_t)(raw[0] << 8 | raw[1]) + 13200) / 2.80) / 100.0;
   return 0;
}


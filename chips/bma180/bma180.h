

/*
   BMA180 I2C Linux Userspace Driver

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



#ifndef __BMA180_H__
#define __BMA180_H__


#include <stdint.h>
#include "../../i2c/i2c.h"


/* accelerometer range: */
typedef enum 
{
   BMA180_RANGE_1G,
   BMA180_RANGE_1_5G,
   BMA180_RANGE_2G,
   BMA180_RANGE_3G,
   BMA180_RANGE_4G,
   BMA180_RANGE_8G,
   BMA180_RANGE_16G
}
bma180_range_t;


/* filter configuration: */
typedef enum
{
   /* low-pass: */
   BMA180_BW_10HZ,
   BMA180_BW_20HZ,
   BMA180_BW_40HZ,
   BMA180_BW_75HZ,
   BMA180_BW_150HZ,
   BMA180_BW_300HZ,
   BMA180_BW_600HZ,
   BMA180_BW_1200HZ,

   /* highpass: 1Hz */
   BMA180_BW_HP,

   /* band-pass: 0.2Hz ... 300Hz */
   BMA180_BW_BP
}
bma180_bw_t;


typedef struct
{
   /* i2c device: */
   i2c_dev_t i2c_dev;

   /* static information */
   uint8_t chip_id;
   uint8_t version;
	
   bma180_range_t range;
   bma180_bw_t bandwidth;

   float temperature;
	
   /* offset: */
   union
   {
      struct
      {
         int8_t t;
         int16_t x;
         int16_t y;
         int16_t z;
      };
      int16_t data[4];
   }
   offset;

   /* gain: */
   union 
   {
      struct
      {
         int8_t t;
         int8_t x;
         int8_t y;
         int8_t z;
      };
      int8_t data[4];
   } 
   gain;

   /* accelerometer readings: */
   union 
   {
      struct
      {
         float x;
         float y;
         float z;
      };
      float data[3];
   } 
   acc;
   
   /* averaged acc values */
   union 
   {
      struct
      {
         float x;
         float y;
         float z;
      };
      float data[3];
   }
   avg;
}
bma180_dev_t;


int bma180_init(bma180_dev_t *dev, i2c_bus_t *bus, bma180_range_t range, bma180_bw_t bandwidth);

int bma180_read_acc(bma180_dev_t *dev);

int bma180_read_temp(bma180_dev_t *dev);

int bma180_avg_acc(bma180_dev_t *dev);

#endif /* __BMA180_H__ */


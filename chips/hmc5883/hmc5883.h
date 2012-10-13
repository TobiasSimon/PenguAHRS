
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

   You should have received a copy of the GNU General Public License
   along with this program; if not, write to the Free Software
   Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
   MA 02110-1301 USA.
*/


#ifndef __HMC5883_H__
#define __HMC5883_H__

#include <stdint.h>
#include "../../i2c/i2c.h"
#include "../../util/math.h"


typedef struct
{
   /* i2c device: */
   i2c_dev_t i2c_dev;

   /* raw measurements: */
   vec3_t raw;

   /* calibration data: */
   vec3_t scale;
   vec3_t shift;

   /* processed measurements: */
   vec3_t processed;
}
hmc5883_dev_t;


int hmc5883_init(hmc5883_dev_t *dev, i2c_bus_t *bus);


int hmc5883_read(hmc5883_dev_t *dev);


#endif /* __HMC5883_H__ */


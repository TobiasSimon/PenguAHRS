
/*
   MS5611 I2C Linux Userspace Driver

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


#ifndef __MS5611_H__
#define __MS5611_H__


#include <stdint.h>


/* over-sampling rates: */
typedef enum
{
   MS5611_OSR256,
   MS5611_OSR512,
   MS5611_OSR1024,
   MS5611_OSR2048,
   MS5611_OSR4096
}
ms5611_osr_t;


typedef struct
{
   /* device access: */
   int bus;
   uint8_t i2c_addr;
   
   /* over-sampling rates: */
   ms5611_osr_t p_osr;
   ms5611_osr_t t_osr;

   /* PROM data: */
   uint16_t prom[8];

   /* raw measurements: */
   int32_t raw_t; /* raw temperature */
   int32_t raw_p; /* raw pressure */

   /* compensated values: */
   double c_t; /* temperature */
   double c_p; /* pressure */
   double c_a; /* altitude */
}
ms5611_dev_t;


int ms5611_init(ms5611_dev_t *dev, int bus, ms5611_osr_t p_osr, ms5611_osr_t t_osr);


void ms5611_measure(ms5611_dev_t *dev);


void ms5611_compensate_float(ms5611_dev_t *dev, int filt_en);


void ms5611_compensate_int(ms5611_dev_t *dev);


#endif /* __MS5611_H__ */


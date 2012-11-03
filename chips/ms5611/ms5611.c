
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


#include <errno.h>
#include <stdint.h>
#include <math.h>

#include "ms5611.h"


#define MS5611_ADDRESS      0x77
#define MS5611_ADC          0x00
#define MS5611_RESET        0x1E
#define MS5611_CONV_D1(osr) (0x40 | ((osr) << 1))
#define MS5611_CONV_D2(osr) (0x50 | ((osr) << 1))
#define MS5611_PROM_READ(x) (0xA0 | ((x) << 1))



static const conv_time_ms[5] =
{
   1 /* 0.60ms */,
   2 /* 1.17 */,
   3 /* 2.28 */,
   5 /* 4.54 */,
   10 /* 9.04 */
};


/* reads prom register into val and returns 0
 * if the read failed, val remains untouched
 * and a negative error code is returned */
static int ms5611_read_prom(ms5611_dev_t *dev, uint8_t reg)
{
   uint8_t raw[2];
   int ret = i2c_read_block_reg(&dev->i2c_dev, MS5611_PROM_READ(reg), raw, sizeof(raw));
   if (ret < 0)
   {
      goto out;
   }
   dev->prom[reg] = raw[1] | (raw[0] << 8);

out:
   return ret;
}


static int ms5611_read_adc(uint32_t *val, ms5611_dev_t *dev)
{
   uint8_t raw[3]; /* 24-bit adc data */
   int ret = i2c_read_block_reg(&dev->i2c_dev, MS5611_ADC, raw, sizeof(raw));
   if (ret < 0)
   {
      goto out;
   }
   *val = raw[2] | (raw[1] << 8) | (raw[0] << 16);

out:
   return ret;
}


/* starts temperature conversion using configurable oversampling rate */
static int ms5611_start_temp_conv(ms5611_dev_t *dev)
{
   return i2c_write(&dev->i2c_dev, MS5611_CONV_D2(dev->t_osr));
}


/* starts pressure conversion using configurable oversampling rate */
static int ms5611_start_pressure_conv(ms5611_dev_t *dev)
{
   return i2c_write(&dev->i2c_dev, MS5611_CONV_D1(dev->p_osr));
}


/* resets the device */
static int ms5611_reset(ms5611_dev_t *dev)
{
   return i2c_write(&dev->i2c_dev, MS5611_RESET);
}


static uint16_t ms5611_crc4(uint16_t *n_prom)
{
   uint16_t n_rem = 0; /* crc remainder */
   uint16_t crc_read; /* original value of the crc */
   crc_read = n_prom[7]; /* save read CRC */
   n_prom[7] = (0xFF00 & (n_prom[7])); /* CRC byte is replaced by 0 */
   int cnt;
   for (cnt = 0; cnt < 16; cnt++)
   {
      if (cnt % 2 == 1)
      {
         n_rem ^= (n_prom[cnt>>1]) & 0x00FF;
      }
      else
      {
         n_rem ^= n_prom[cnt>>1] >> 8;
      }
      int n_bit;
      for (n_bit = 8; n_bit > 0; n_bit--)
      {
         if (n_rem & (0x8000))
         {
            n_rem = (n_rem << 1) ^ 0x3000;
         }
         else 
         {
            n_rem = (n_rem << 1);
         }
      }
   }
   n_rem = (n_rem >> 12) & 0xF; /* final 4-bit reminder is CRC code */
   n_prom[7] = crc_read;
   return n_rem;
}


int ms5611_init(ms5611_dev_t *dev, i2c_bus_t *bus, ms5611_osr_t p_osr, ms5611_osr_t t_osr)
{
   /* copy values */
   i2c_dev_init(&dev->i2c_dev, bus, MS5611_ADDRESS);
   i2c_dev_lock_bus(&dev->i2c_dev);

   /* assign over-sampling settings: */
   dev->p_osr = p_osr;
   dev->t_osr = t_osr;

   /* reset device: */
   int ret = ms5611_reset(dev);
   if (ret < 0)
   {
      goto out;   
   }
   i2c_dev_sleep(&dev->i2c_dev, 3); /* at least 2.8ms */
   
   /* read prom including CRC */
   int i;
   for (i = 0; i < 8; i++)
   {
      ret = ms5611_read_prom(dev, i);
      if (ret < 0) 
      {
         goto out;     
      }
   }
   
   /* validate CRC: */
   uint16_t crc = ms5611_crc4(dev->prom);
   uint16_t crcProm = dev->prom[7] & 0x0F;
   if (crc != crcProm)
   {
      ret = -ENODEV;   
   }

out:
   i2c_dev_unlock_bus(&dev->i2c_dev);
   return ret;
}


static void ms5611_compensate(ms5611_dev_t *dev)
{
   /* aliases for data sheet conformity including casts: */
   int64_t C1 = dev->prom[1];
   int64_t C2 = dev->prom[2];
   int64_t C3 = dev->prom[3];
   int64_t C4 = dev->prom[4];
   int64_t C5 = dev->prom[5];
   int64_t C6 = dev->prom[6];
   int64_t D1 = dev->raw_p;
   int64_t D2 = dev->raw_t;
   
   /* calculate temperature: */
   int64_t dT = D2 - (C5 << 8); /* reference temperature delta */
   int64_t TEMP = 2000 + ((dT * C6) >> 23);
  
   /* compute temperature compensation: */
   int64_t OFF = (C2 << 16) + ((C4 * dT) >> 7);
   int64_t SENS = (C1 << 15) + ((C3 * dT) >> 8);
   
   /* compute 2nd order temperature compensation, if required: */
   if (TEMP < 2000)
   {
      /* low temperature: */
      int64_t T2 = (dT * dT) >> 31;
      int64_t tmp = (TEMP - 2000) * (TEMP - 2000);
      int64_t OFF2 = 5 * tmp >> 1;
      int64_t SENS2 = 5 * tmp >> 2;
      if (TEMP < -1500)
      {
         /* very low temperature: */
         tmp = (TEMP + 1500) * (TEMP + 1500);
         OFF2 = OFF2 + 7 * tmp;
         SENS2 = SENS2 + (11 * tmp >> 1);
      }
      TEMP = TEMP - T2;
      OFF = OFF - OFF2;
      SENS = SENS - SENS2;
   }

   /* compute compensated pressure: */
   dev->c_p = (((D1 * SENS) >> 21) - OFF) >> 15;
   dev->c_a = (44330.0 * (1.0 - pow((float)dev->c_p / 101325.0, 0.190295)));
   dev->c_t = (float)TEMP / 100.0;
}


int ms5611_measure(ms5611_dev_t *dev)
{
   /* read pressure: */
   i2c_dev_lock_bus(&dev->i2c_dev);
   int ret = ms5611_start_pressure_conv(dev);
   if (ret < 0)
   {
      goto out;   
   }
   i2c_dev_sleep(&dev->i2c_dev, conv_time_ms[dev->p_osr]);
   ret = ms5611_read_adc(&dev->raw_p, dev);
   if (ret < 0)
   {
      goto out;   
   }

   /* read temperature: */
   ret = ms5611_start_temp_conv(dev);
   if (ret < 0)
   {
      goto out;   
   }
   i2c_dev_sleep(&dev->i2c_dev, conv_time_ms[dev->t_osr]);
   ret = ms5611_read_adc(&dev->raw_t, dev);
   if (ret < 0)
   {
      goto out;   
   }
   ms5611_compensate(dev);

out:
   i2c_dev_unlock_bus(&dev->i2c_dev);
   return ret;
}




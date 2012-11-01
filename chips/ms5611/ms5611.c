
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


#include <stdint.h>
#include <unistd.h>
#include <stdio.h>
#include <math.h>
#include <time.h>

#include "i2c-dev.h"
#include "ms5611.h"



#define MS5611_RESET        0x1E
#define MS5611_CONV_D1(osr) (0x40 | ((osr) << 1))
#define MS5611_CONV_D2(osr) (0x50 | ((osr) << 1))
#define MS5611_ADC_READ     0x00
#define MS5611_PROM_READ(x) (0xA0 | ((x) << 1))



static const conv_time_ms[5] =
{
   1 /* 0.60ms */,
   2 /* 1.17 */,
   3 /* 2.28 */,
   5 /* 4.54 */,
   10 /* 9.04 */
};



static void msleep(uint32_t ms)
{
   struct timespec tim, tim2;
   tim.tv_sec = 0;
   tim.tv_nsec = ms * 1000 * 1000;
   nanosleep(&tim , &tim2);
}


static uint16_t ms5611_read_prom(ms5611_dev_t *dev, uint8_t reg)
{
   union 
   {
      uint16_t value;
      uint8_t bytes[2];
   } 
   data;
   
   i2c_smbus_read_i2c_block_data(dev->bus, MS5611_PROM_READ(reg), 2, data.bytes);

   data.value = (data.value >> 8) | ((data.value & 0xFF) << 8);
   return data.value;
}


static uint32_t ms5611_read_adc(ms5611_dev_t *dev)
{
   union
   {
      uint32_t value;
      uint8_t bytes[4];
   } 
   data;

   data.value = 0;
   i2c_smbus_read_i2c_block_data(dev->bus, 0x00, 3, data.bytes);
   uint8_t tmp = data.bytes[2];
   data.bytes[2] = data.bytes[0];
   data.bytes[0] = tmp;
   return data.value;
}


static void ms5611_start_temp_conv(ms5611_dev_t *dev)
{
   /* we use the best oversampling ratio here */
   (void)i2c_smbus_write_byte(dev->bus, MS5611_CONV_D2(dev->t_osr));
}


static void ms5611_start_pressure_conv(ms5611_dev_t *dev)
{
   /* we use the best oversampling ratio here */
   (void)i2c_smbus_write_byte(dev->bus, MS5611_CONV_D1(dev->p_osr));
}


static void ms5611_reset(ms5611_dev_t *dev)
{
   (void)i2c_smbus_write_byte(dev->bus, MS5611_RESET);
}


static uint16_t ms5611_crc4(uint16_t *n_prom)
{
   uint16_t n_rem = 0; /* crc remainde */
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


int ms5611_init(ms5611_dev_t *dev, int bus, ms5611_osr_t p_osr, ms5611_osr_t t_osr)
{
   dev->bus = bus;

   /* assign over-sampling settings: */
   dev->p_osr = p_osr;
   dev->t_osr = t_osr;

   /* reset device: */
   ms5611_reset(dev);
   msleep(3); /* at least 2.8ms */
   
   /* read prom including CRC */
   int i;
   for (i = 0; i < 8; i++)
   {
      dev->prom[i] = ms5611_read_prom(dev, i);
   }
   
   /* validate CRC: */
   uint16_t crc = ms5611_crc4(dev->prom);
   uint16_t crcProm = dev->prom[7] & 0x0F;
   return (crc == crcProm) ? 0 : -1;
}


void ms5611_measure(ms5611_dev_t *dev)
{
   /* read pressure: */
   ms5611_start_pressure_conv(dev);
   msleep(conv_time_ms[dev->p_osr]);
   dev->raw_p = ms5611_read_adc(dev);
   
   /* read temperature: */
   ms5611_start_temp_conv(dev);
   msleep(conv_time_ms[dev->t_osr]);
   dev->raw_t = ms5611_read_adc(dev);
}


void ms5611_compensate_float(ms5611_dev_t *dev, int filter_en)
{
   /* aliases for data sheet conformity including casts: */
   double C1 = dev->prom[1];
   double C2 = dev->prom[2];
   double C3 = dev->prom[3];
   double C4 = dev->prom[4];
   double C5 = dev->prom[5];
   double C6 = dev->prom[6];
   double D1 = dev->raw_p;
   double D2 = dev->raw_t;
 
   /* calculate temperature: */
   double dT = D2 - C5 * pow(2.0, 8.0); /* reference temperature delta */
   double TEMP = 2000.0 + (dT * C6) / pow(2.0, 23.0);
  
   /* compute temperature compensation: */
   double OFF = C2 * pow(2.0, 16.0) + (C4 * dT) / pow(2.0, 7.0);
   double SENS = C1 * pow(2.0, 15.0) + (C3 * dT) / pow(2.0, 8.0);
   
   /* compute 2nd order temperature compensation, if required: */
   if (TEMP < 2000.0)
   {
      /* low temperature: */
      double T2 = pow(dT, 2.0) / pow(2.0, 31.0);
      double tmp = pow(TEMP - 2000.0, 2.0);
      double OFF2 = 5.0 * tmp / 2.0;
      double SENS2 = 5.0 * tmp / 4.0;
      if (TEMP < -1500.0)
      {
         /* very low temperature: */
         tmp = pow(TEMP + 1500.0, 2.0);
         OFF2 = OFF2 + 7.0 * tmp;
         SENS2 = SENS2 + (11.0 * tmp / 2.0);
      }
      TEMP = TEMP - T2;
      OFF = OFF - OFF2;
      SENS = SENS - SENS2;
   }

   /* compute and assign outputs: */
   dev->c_p = ((D1 * SENS) / pow (2.0, 21.0) - OFF) / pow(2.0, 15.0);
   dev->c_a = (44330.0 * (1.0 - pow(dev->c_p / 101325.0, 0.190295)));
   dev->c_t = (double)TEMP / 100.0;
}


void ms5611_compensate_int(ms5611_dev_t *dev)
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


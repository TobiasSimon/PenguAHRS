
/*
   I2C Linux Interface

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


#ifndef __I2C_H__
#define __I2C_H__


#include <stdint.h>
#include <pthread.h>


/* bus type definition: */
typedef struct
{
   int handle;
   uint8_t dev_addr;
   pthread_mutex_t mutex;
}
i2c_bus_t;


/* device type definition: */
typedef struct
{
   i2c_bus_t *bus;
   uint8_t addr;
}
i2c_dev_t;


/* management: */
int i2c_bus_open(i2c_bus_t *bus, char *path);
int i2c_bus_close(i2c_bus_t *bus);
void i2c_dev_init(i2c_dev_t *dev, i2c_bus_t *bus, uint8_t addr);

/* writing: */
int i2c_write(i2c_dev_t *dev, uint8_t val);
int i2c_write_reg(i2c_dev_t *dev, uint8_t reg, uint8_t val);

/* reading: */
int i2c_read(i2c_dev_t *dev);
int i2c_read_reg(i2c_dev_t *dev, uint8_t reg);
int i2c_read_block_reg(i2c_dev_t *dev, uint8_t reg, uint8_t *buf, size_t len);

/* locking and sleeping: */
void i2c_dev_lock_bus(i2c_dev_t *dev);
void i2c_dev_unlock_bus(i2c_dev_t *dev);
void i2c_dev_sleep(i2c_dev_t *dev, uint32_t msec);


#endif /* __I2C_H__ */



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


#include <fcntl.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <errno.h>

#include "i2c.h"
#include "i2c-dev.h"


int i2c_bus_open(i2c_bus_t *bus, char *path)
{
   int handle = open(path, O_RDWR);
   if (handle < 0)
   {
      return handle;   
   }
   bus->dev_addr = 0xFF;
   bus->handle = handle;
   return pthread_mutex_init(&bus->mutex, NULL);
}


int i2c_bus_close(i2c_bus_t *bus)
{
   return close(bus->handle);
}


void i2c_dev_init(i2c_dev_t *dev, i2c_bus_t *bus, uint8_t addr)
{
   dev->bus = bus;
   dev->addr = addr;
}


static int set_slave_address_if_needed(i2c_dev_t *dev)
{
   int ret = 0;
   if (dev->bus->dev_addr != dev->addr)
   {
      ret = ioctl(dev->bus->handle, I2C_SLAVE, dev->addr);
      if (ret < 0)
      {
         return ret;
      }
      dev->bus->dev_addr = dev->addr;
   }
   return ret;
}


int i2c_write(i2c_dev_t *dev, uint8_t val)
{
   int ret = set_slave_address_if_needed(dev);
   if (ret < 0)
   {
      return ret;
   }
   return i2c_smbus_write_byte(dev->bus->handle, val);
}


int i2c_write_reg(i2c_dev_t *dev, uint8_t reg, uint8_t val)
{
   int ret = set_slave_address_if_needed(dev);
   if (ret < 0)
   {
      return ret;
   }
   return i2c_smbus_write_byte_data(dev->bus->handle, reg, val);
}


int i2c_read(i2c_dev_t *dev)
{
   int ret = set_slave_address_if_needed(dev);
   if (ret < 0)
   {
      return ret;
   }
   return i2c_smbus_read_byte(dev->bus->handle);
}


int i2c_read_reg(i2c_dev_t *dev, uint8_t reg)
{
   int ret = set_slave_address_if_needed(dev);
   if (ret < 0)
   {
      return ret;
   }
   return i2c_smbus_read_byte_data(dev->bus->handle, reg);
}


int i2c_read_block_reg(i2c_dev_t *dev, uint8_t reg, uint8_t *buf, size_t len)
{
   int ret = set_slave_address_if_needed(dev);
   if (ret < 0)
   {
      return ret;
   }
   return i2c_smbus_read_i2c_block_data(dev->bus->handle, reg, len, buf) == len ? 0 : -EIO;
}


void i2c_dev_lock_bus(i2c_dev_t *dev)
{
   pthread_mutex_lock(&dev->bus->mutex);
}


void i2c_dev_unlock_bus(i2c_dev_t *dev)
{
   pthread_mutex_unlock(&dev->bus->mutex);
}


void i2c_dev_sleep(i2c_dev_t *dev, uint32_t msec)
{
   /* release the bus: */
   i2c_dev_unlock_bus(dev);
   
   /* sleep: */
   struct timespec tim, tim2;
   tim.tv_sec = 0;
   tim.tv_nsec = msec * 1000 * 1000;
   nanosleep(&tim , &tim2);
   
   /* acquire the bus: */
   i2c_dev_lock_bus(dev);
}


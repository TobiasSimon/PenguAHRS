
#include "hmc5883.h"
#include "i2c-dev.h"
#include <stdio.h>


/* sensitivity conversion table in LSB/Ga */
static float sens_conv_tab[8] =
{
   1602.0, 1300.0, 970.0, 780.0,
   530.0, 460.0, 390.0, 280.0
};
#define CFG_A_2_SENS(v) (sens_conv_tab[(v >> 5)])



static void hmc5883_cal(hmc5843_dev_t *dev)
{
   (void)i2c_smbus_write_byte_data(dev->bus, HMC5883_CFG_A, 0x11); /* 10Hz */
   (void)i2c_smbus_write_byte_data(dev->bus, HMC5883_CFG_B, 0x01); /* 10Hz */
   sleep(1);
   hmc5883_read(dev);
   dev->bias_x = dev->raw_x;
   dev->bias_y = dev->raw_y;
   dev->bias_z = dev->raw_x;
   printf("biases: %d %d %d\n", dev->bias_x, dev->bias_y, dev->bias_z);
}


int hmc5883_init(hmc5843_dev_t *dev, int bus)
{
   dev->bus = bus;
   uint8_t id[3];
   (void)i2c_smbus_read_i2c_block_data(dev->bus, HMC5883_ID_A, 3, id);
   if (strncmp("H43\0", id, 3) != 0)
   {
      return -1;
   }

   hmc5883_cal(dev);

   (void)i2c_smbus_write_byte_data(dev->bus, HMC5883_CFG_A, 0x10); /* 10Hz */
   (void)i2c_smbus_write_byte_data(dev->bus, HMC5883_CFG_B, 0x20); /* gain 1 */
   (void)i2c_smbus_write_byte_data(dev->bus, HMC5883_MR, 0x00); /* normal operation */
   return 0;
}


void hmc5883_read(hmc5843_dev_t *dev)
{
   uint8_t data[6];
   uint8_t cfg_a = i2c_smbus_read_byte_data(dev->bus, HMC5883_CFG_A);
   (void)i2c_smbus_read_i2c_block_data(dev->bus, HMC5883_MAGX_H, 6, data);
   dev->raw_y = ((data[4] << 8) | data[5]) * 1000.0 / CFG_A_2_SENS(cfg_a);
   dev->raw_x = ((data[0] << 8) | data[1]) * 1000.0 / CFG_A_2_SENS(cfg_a);
   dev->raw_z = ((data[2] << 8) | data[3]) * 1000.0 / CFG_A_2_SENS(cfg_a);
   //dev->x = (double)dev->bias_x / dev->raw_x;
   //dev->y = (double)dev->bias_y / dev->raw_y;
   //dev->z = (double)dev->bias_z / dev->raw_z;
}



#include <fcntl.h>
#include <stdlib.h>
#include <sys/ioctl.h>
#include <math.h>

int main(void)
{
   int addr = HMC5883_ADDR;
   int bus_handle = open("/dev/i2c-16", O_RDWR);
   ioctl(bus_handle, I2C_SLAVE, addr);
   hmc5883_dev_t dev;
   int status = hmc5883_init(&dev, bus_handle);
   printf("%d\n", status);
   while (1)
   {
      hmc5883_read(&dev);
      printf("%.2f %.2f %.2f %.1f\n", dev.x, dev.y, dev.z,
                              atan2((double)dev.y, (double)dev.x) * (180.0 / M_PI) + 180.0);
   }

   return 0;
}


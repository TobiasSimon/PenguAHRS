
#ifndef __HMC5883_H__
#define __HMC5883_H__

#include <stdint.h>


#define HMC5883_ADDR 0x1E


#define HMC5883_CFG_A   0x00
#define HMC5883_CFG_B   0x01
#define HMC5883_MR      0x02
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

/* Output Data Rate */
#define HMC5883_ODR_05 0x00
#define HMC5883_ODR_1  0x04
#define HMC5883_ODR_2  0x08
#define HMC5883_ODR_5  0x0C
#define HMC5883_ODR_10 0x10
#define HMC5883_ODR_20 0x14
#define HMC5883_ODR_50 0x18

/* Measure configuration */
#define HMC5883_MEASCONF_NORMAL            0x00
#define HMC5883_MEASCONF_BIAS_POS          0x01
#define HMC5883_MEASCONF_BIAS_NEG          0x02

 

/* Gain settings */

#define PIOS_HMC5883_GAIN_0_7                   0x00

#define PIOS_HMC5883_GAIN_1                     0x20

#define PIOS_HMC5883_GAIN_1_5                   0x40

#define PIOS_HMC5883_GAIN_2                     0x60

#define PIOS_HMC5883_GAIN_3_2                   0x80

#define PIOS_HMC5883_GAIN_3_8                   0xA0

#define PIOS_HMC5883_GAIN_4_5                   0xC0

#define PIOS_HMC5883_GAIN_6_5                   0xE0

 

/* Modes */
#define PIOS_HMC5883_MODE_CONTINUOUS            0x00
#define PIOS_HMC5883_MODE_SINGLE                0x01
#define HMC5883_MODE_IDLE                  0x02
#define HMC5883_MODE_SLEEP                 0x02

 


typedef struct
{
   /* device access: */
   int bus;
   uint8_t i2c_addr;

   /* raw measurements: */
   int16_t raw_x; /* raw x value */
   int16_t raw_y; /* raw y value */
   int16_t raw_z; /* raw z value */
   
   /* biases: */
   int16_t bias_x; /* bias x value */
   int16_t bias_y; /* bias y value */
   int16_t bias_z; /* bias z value */
  
   /* scaled / biased measurements: */
   double x; /* x value */
   double y; /* y value */
   double z; /* z value */
}
hmc5883_dev_t;


int hmc5883_init(hmc5843_dev_t *dev, int bus);


void hmc5883_read(hmc5843_dev_t *dev);


#endif /* __HMC5883_H__ */



#include <stdint.h>
#include <unistd.h>
#include <stdio.h>
#include <math.h>
#include <time.h>
#include <errno.h>
#include "i2c-dev.h"
#include "bma180.h"

#define BMA180_DEBUG		1


#define BMA180_CHIP_ID					0x00
#define BMA180_VERSION					0x01
#define BMA180_ACC_X_LSB				0x02
#define BMA180_ACC_X_MSB				0x03
#define BMA180_ACC_Y_LSB				0x04
#define BMA180_ACC_Y_MSB				0x05
#define BMA180_ACC_Z_LSB				0x06
#define BMA180_ACC_Z_MSB				0x07
#define BMA180_TEMP						0x08
#define BMA180_STATUS_REG1				0x09
#define BMA180_STATUS_REG2				0x0A
#define BMA180_STATUS_REG3				0x0B
#define BMA180_STATUS_REG4				0x0C

#define BMA180_CTRL_REG0				0x0D
#define BMA180_CTRL_REG0_DIS_WAKE_UP	(1 << 0)
#define BMA180_CTRL_REG0_SLEEP			(1 << 1)
#define BMA180_CTRL_REG0_ST0			(1 << 2)
#define BMA180_CTRL_REG0_ST1			(1 << 3)
#define BMA180_CTRL_REG0_EE_W			(1 << 4)
#define BMA180_CTRL_REG0_UPDATE_IMAGE	(1 << 5)
#define BMA180_CTRL_REG0_RESET_INT		(1 << 6)

#define BMA180_CTRL_REG1				0x0E
#define BMA180_CTRL_REG1_EN_OFFSET_Z	(1 << 5)
#define BMA180_CTRL_REG1_EN_OFFSET_Y	(1 << 6)
#define BMA180_CTRL_REG1_EN_OFFSET_X	(1 << 7)

#define BMA180_CTRL_REG2				0x0F
#define BMA180_CTRL_REG2_UNLOCK_EE(x)	(((x) & 0x0F) << 4)

#define BMA180_RESET					0x10
#define BMA180_RESET_SOFT_RESET			0xB6

#define BMA180_BW_TCS					0x20
#define BMA180_BW_TCS_BW(x)				(((x) & 0x0F) << 4)
#define BMA180_BW_TCS_TCS(x)			(((x) & 0x0F))

#define BMA180_CTRL_REG3				0x21
#define BMA180_CTRL_REG_3_LAT_INT		(1 << 0)
#define BMA180_CTRL_REG_3_NEW_DATA_INT	(1 << 1)
#define BMA180_CTRL_REG_3_ADV_INT		(1 << 2)
#define BMA180_CTRL_REG_3_TAPSENS_INT	(1 << 3)
#define BMA180_CTRL_REG_3_LOW_INT		(1 << 4)
#define BMA180_CTRL_REG_3_HIGH_INT		(1 << 5)
#define BMA180_CTRL_REG_3_SLOPE_INT		(1 << 6)
#define BMA180_CTRL_REG_3_SLOPE_ALERT	(1 << 7)

#define BMA180_CTRL_REG4				0x22
#define BMA180_HY						0x23
#define BMA180_SLOPE_TAPSENS_INFO		0x24
#define BMA180_HIGH_LOW_INFO			0x25
#define BMA180_LOW_DUR					0x26
#define BMA180_HIGH_DUR					0x27
#define BMA180_TAPSENS_TH				0x28
#define BMA180_LOW_TH					0x29
#define BMA180_HIGH_TH					0x2A
#define BMA180_SLOPE_TH					0x2B
#define BMA180_CD1						0x2C
#define BMA180_CD2						0x2D
#define BMA180_TCO_X					0x2E
#define BMA180_TCO_Y					0x2F
#define BMA180_TCO_Z					0x30
#define BMA180_GAIN_T					0x31
#define BMA180_GAIN_X					0x32
#define BMA180_GAIN_Y					0x33
#define BMA180_GAIN_Z					0x34

#define BMA180_OFFSET_LSB1				0x35
#define BMA180_OFFSET_LSB1_RANGE(x)		(((x) & 0x7) << 1)

#define BMA180_OFFSET_LSB2				0x36
#define BMA180_OFFSET_T					0x37
#define BMA180_OFFSET_X					0x38
#define BMA180_OFFSET_Y					0x39
#define BMA180_OFFSET_Z					0x3A

/* EEPROM versions are just shifted by 0x20 bytes */
#define BMA180_EE(x)					((x) + 0x20)
#define BMA180_EE_CRC					0x5B


static const ACC_RANGE_TABLE[] = {
	1,	/* 1G */
	1.5,	/* 1.5G */
	2,	/* 2G */
	3,	/* 3G */
	4,	/* 4G */
	8,	/* 8G */
	16	/* 16G */
};

static int bma180_read_calibration(bma180_dev_t *dev)
{
	int n, ret;
	uint8_t off[6];
	uint8_t gain[4];

	/* read offset and gain values */
	ret = i2c_smbus_read_i2c_block_data(dev->bus,
		BMA180_OFFSET_LSB1, 6, off);
	if(ret != 6)
		return -1;
	
	ret = i2c_smbus_read_i2c_block_data(dev->bus,
		BMA180_GAIN_T, 4, gain);
	if(ret != 4)
		return -1;

	/* put offset values together
	 *
	 * Offsets and gains are most negative at 0x0 and most positive 
	 * at 2^data_bits, hence the subtraction. */
	dev->offset.t = (int8_t)(off[2] >> 1) - (1 << 6);
	dev->offset.x = (int16_t)((off[3] >> 4) | (off[0] >> 4)) - (1 << 11);
	dev->offset.y = (int16_t)((off[4] >> 4) | (off[1] & 0x0F)) - (1 << 11);
	dev->offset.z = (int16_t)((off[5] >> 4) | (off[1] >> 4)) - (1 << 11);

	/* put gain values together */
	dev->gain.t = (gain[0] >> 1) - (1 << 4);
	for(n = 1; n < 4; n++)
		dev->gain.data[n] = (int8_t)(gain[n] >> 1) - (1 << 6);

	return 0;
}

int bma180_init(bma180_dev_t *dev, int bus, bma180_range_t range, bma180_bw_t bandwidth)
{
	int n, ret;
	
	/* copy values */
	dev->bus = bus;
	dev->range = range;
	dev->bandwidth = bandwidth;

	/* reset unit */
	ret = i2c_smbus_write_byte_data(dev->bus,
		BMA180_RESET,
		BMA180_RESET_SOFT_RESET);
	if(ret < 0)
		return -1;

	usleep(10);

	/* read some information */
	ret = i2c_smbus_read_byte_data(dev->bus,
		BMA180_CHIP_ID);
	if(ret == -1)
		return -1;
	dev->chip_id = (uint8_t)(ret);

	ret = i2c_smbus_read_byte_data(dev->bus,
		BMA180_VERSION);
	if(ret == -1)
		return -1;
	dev->version = (uint8_t)(ret);

	if(dev->chip_id != 0x03)
		return -1;
	else if(dev->version != 0x14)
		return -1;

#ifdef BMA180_DEBUG
	printf("BMA180, chip_id: 0x%.2x, version: 0x%.2x\n",
		dev->chip_id,
		dev->version);
#endif
	
	ret = bma180_read_calibration(dev);
	if(ret < 0)
		return -1;

	/* enable writing */
	ret = i2c_smbus_write_byte_data(dev->bus,
		BMA180_CTRL_REG0,
		BMA180_CTRL_REG0_EE_W);
	if(ret < 0)
		return -1;

	/* set bandwidth */
	ret = i2c_smbus_read_byte_data(dev->bus,
		BMA180_BW_TCS);
	if(ret == -1)
		return -1;
	ret = i2c_smbus_write_byte_data(dev->bus,
		BMA180_BW_TCS,
		((uint8_t)(ret) & 0x0F) | BMA180_BW_TCS_BW(dev->bandwidth));	
	if(ret < 0)
		return -1;

	/* set acceleration range */
	ret = i2c_smbus_read_byte_data(dev->bus,
		BMA180_OFFSET_LSB1);
	if(ret == -1)
		return -1;
	ret = i2c_smbus_write_byte_data(dev->bus,
		BMA180_OFFSET_LSB1,
		((uint8_t)(ret) & 0xF1) | BMA180_OFFSET_LSB1_RANGE(dev->range));
	if(ret < 0)
		return -1;

	/* enable use of offsets */
	for(n = 0; n < 3; n++) {
		ret = i2c_smbus_write_byte_data(dev->bus,
			BMA180_CTRL_REG1,
			BMA180_CTRL_REG1_EN_OFFSET_Z << n);
		if(ret < 0)
			return -1;
	}

	return 0;
}

int bma180_read_temp(bma180_dev_t *dev)
{
	int ret;

	ret = i2c_smbus_read_byte_data(dev->bus, BMA180_TEMP);
	if(ret == -1)
		return -1;

	dev->temperature = (float)((int8_t)(ret)) / 2.0 + 24.0;

	return 0;
}

int bma180_read_acc(bma180_dev_t *dev)
{
	int n, ret;
	uint8_t acc_data[6];

	/* read acc values */
	ret = i2c_smbus_read_i2c_block_data(dev->bus,
		BMA180_ACC_X_LSB, 6, acc_data);
	if(ret != 6)
		return -1;

	for(n = 0; n < 3; n++) {
		int16_t raw;
		
		/* put them together */
		raw = (int16_t)((acc_data[(n << 1) + 1] << 8)
				| (acc_data[(n << 1)] & 0xFC)) / 4;
		
		/* and scale according to range setting */
		dev->acc.data[n] = ((float)(raw)
							* ACC_RANGE_TABLE[dev->range]
							/ (float)(1 << 13))
								* 9.81;
	}

	return ret;
}

#include <stdio.h>
#include <stdint.h>
#include <stdio.h>
#include <math.h>
#include <time.h>
#include <errno.h>

#include "mpu6050.h"


#define MPU6050_ADDRESS						0x69

#define MPU6050_SELF_TEST_X					0x0D
#define MPU6050_SELF_TEST_Y					0x0E
#define MPU6050_SELF_TEST_Z					0x0F
#define MPU6050_SELF_TEST_A					0x10

#define MPU6050_SMPLRT_DIV					0x19
	
#define MPU6050_CONFIG						0x1A
#define MPU6050_CONFIG_DLPF_CFG(x)			((x) & 0x7)
#define MPU6050_CONFIG_EXT_SYNC_SET(x)		(((x) & 0x7) << 3)

#define MPU6050_GYRO_CONFIG					0x1B
#define MPU6050_GYRO_CONFIG_FS_SEL(x)		(((x) & 0x3) << 3)

#define MPU6050_ACCEL_CONFIG				0x1C
#define MPU6050_ACCEL_CONFIG_AFS_SEL(x)		(((x) & 0x3) << 3)

#define MPU6050_MOT_THR						0x1F

#define MPU6050_FIFO_EN						0x23

#define MPU6050_I2C_MST_CTRL				0x24
#define MPU6050_I2C_SLV0_ADDR				0x25
#define MPU6050_I2C_SLV0_REG				0x26
#define MPU6050_I2C_SLV0_CTRL				0x27
#define MPU6050_I2C_SLV1_ADDR				0x28
#define MPU6050_I2C_SLV1_REG				0x29
#define MPU6050_I2C_SLV1_CTRL				0x2A
#define MPU6050_I2C_SLV2_ADDR				0x2B
#define MPU6050_I2C_SLV2_REG				0x2C
#define MPU6050_I2C_SLV2_CTRL				0x2D
#define MPU6050_I2C_SLV3_ADDR				0x2E
#define MPU6050_I2C_SLV3_REG				0x2F
#define MPU6050_I2C_SLV3_CTRL				0x30
#define MPU6050_I2C_SLV4_ADDR				0x31
#define MPU6050_I2C_SLV4_REG				0x32
#define MPU6050_I2C_SLV4_DO					0x33
#define MPU6050_I2C_SLV4_CTRL				0x34
#define MPU6050_I2C_SLV4_DI					0x35
#define MPU6050_I2C_MST_STATUS				0x36

#define MPU6050_INT_PIN_CFG					0x37
#define MPU6050_INT_ENABLE					0x38
#define MPU6050_INT_STATUS					0x3A

#define MPU6050_ACCEL_XOUT_H				0x3B
#define MPU6050_ACCEL_XOUT_L				0x3C
#define MPU6050_ACCEL_YOUT_H				0x3D
#define MPU6050_ACCEL_YOUT_L				0x3E
#define MPU6050_ACCEL_ZOUT_H				0x3F
#define MPU6050_ACCEL_ZOUT_L				0x40

#define MPU6050_TEMP_OUT_H					0x41
#define MPU6050_TEMP_OUT_L					0x42

#define MPU6050_GYRO_XOUT_H					0x43
#define MPU6050_GYRO_XOUT_L					0x44
#define MPU6050_GYRO_YOUT_H					0x45
#define MPU6050_GYRO_YOUT_L					0x46
#define MPU6050_GYRO_ZOUT_H					0x47
#define MPU6050_GYRO_ZOUT_L					0x48

#define MPU6050_EXT_SENS_DATA_00			0x49
#define MPU6050_EXT_SENS_DATA_01			0x4A
#define MPU6050_EXT_SENS_DATA_02			0x4B
#define MPU6050_EXT_SENS_DATA_03			0x4C
#define MPU6050_EXT_SENS_DATA_04			0x4D
#define MPU6050_EXT_SENS_DATA_05			0x4E
#define MPU6050_EXT_SENS_DATA_06			0x4F
#define MPU6050_EXT_SENS_DATA_07			0x50
#define MPU6050_EXT_SENS_DATA_08			0x51
#define MPU6050_EXT_SENS_DATA_09			0x52
#define MPU6050_EXT_SENS_DATA_10			0x53
#define MPU6050_EXT_SENS_DATA_11			0x54
#define MPU6050_EXT_SENS_DATA_12			0x55
#define MPU6050_EXT_SENS_DATA_13			0x56
#define MPU6050_EXT_SENS_DATA_14			0x57
#define MPU6050_EXT_SENS_DATA_15			0x58
#define MPU6050_EXT_SENS_DATA_16			0x59
#define MPU6050_EXT_SENS_DATA_17			0x5A
#define MPU6050_EXT_SENS_DATA_18			0x5B
#define MPU6050_EXT_SENS_DATA_19			0x5C
#define MPU6050_EXT_SENS_DATA_20			0x5D
#define MPU6050_EXT_SENS_DATA_21			0x5E
#define MPU6050_EXT_SENS_DATA_22			0x5F
#define MPU6050_EXT_SENS_DATA_23			0x60

#define MPU6050_I2C_SLV0_DO					0x63
#define MPU6050_I2C_SLV1_DO					0x64
#define MPU6050_I2C_SLV2_DO					0x65
#define MPU6050_I2C_SLV3_DO					0x66
#define MPU6050_I2C_MST_DELAY_CT			0x67

#define MPU6050_SIGNAL_PATH_RES				0x68

#define MPU6050_MOT_DETECT_CTRL				0x69

#define MPU6050_USER_CTRL					0x6A

#define MPU6050_PWR_MGMT_1					0x6B
#define MPU6050_PWR_MGMT_1_CLKSEL(x)		((x) & 0x7)
#define MPU6050_PWR_MGMT_1_TEMP_DIS			(1 << 3)
#define MPU6050_PWR_MGMT_1_CYCLE			(1 << 5)
#define MPU6050_PWR_MGMT_1_SLEEP			(1 << 6)
#define MPU6050_PWR_MGMT_1_DEVICE_RESET		(1 << 7)

#define MPU6050_PWR_MGMT_2					0x6C
#define MPU6050_PWR_MGMT_2_STBY_ZG			(1 << 0)
#define MPU6050_PWR_MGMT_2_STBY_YG			(1 << 1)
#define MPU6050_PWR_MGMT_2_STBY_XG			(1 << 2)
#define MPU6050_PWR_MGMT_2_STBY_ZA			(1 << 3)
#define MPU6050_PWR_MGMT_2_STBY_YA			(1 << 4)
#define MPU6050_PWR_MGMT_2_STBY_XA			(1 << 5)
#define MPU6050_PWR_MGMT_2_LP_WAKE_CTRL(x)	(((x) & 0x3) << 6)


#define MPU6050_FIFO_COUNTH					0x72
#define MPU6050_FIFO_COUNTL					0x73
#define MPU6050_FIFO_R_W					0x74

#define MPU6050_WHO_AM_I					0x75


int mpu6050_init(mpu6050_dev_t *dev, i2c_bus_t *bus, mpu6050_dlpf_cfg_t dlpf, mpu6050_fs_sel_t fs_sel, mpu6050_afs_sel_t afs_sel)
{
	int ret;
	
	dev->dlpf = dlpf;
	dev->gfs = fs_sel;
	dev->afs = afs_sel;

	i2c_dev_init(&dev->i2c_dev, bus, MPU6050_ADDRESS);

	ret = i2c_read_reg(&dev->i2c_dev, MPU6050_WHO_AM_I);
	if (ret < 0)
	{
		goto out;
	}
	
	/* validate address: */
	uint8_t addr = (uint8_t)(ret);
	if (dev->i2c_dev.addr != addr)
	{
		ret = -ENODEV;
		goto out;
	}

	/* Device reset */
	ret = i2c_write_reg(&dev->i2c_dev, MPU6050_PWR_MGMT_1, MPU6050_PWR_MGMT_1_DEVICE_RESET);
	if (ret < 0)
	{
		goto out;
	}

	/* set Z-Axis gyro as clock reference */
	ret = i2c_write_reg(&dev->i2c_dev, MPU6050_PWR_MGMT_1, MPU6050_PWR_MGMT_1_CLKSEL(0x3));
	if (ret < 0)
	{
		goto out;
	}

	/* configure digital low pass filter */
	ret = i2c_write_reg(&dev->i2c_dev, MPU6050_CONFIG, MPU6050_CONFIG_DLPF_CFG(dev->dlpf));
	if (ret < 0)
	{
		goto out;
	}

	/* configure full scale range for gyros */
	ret = i2c_write_reg(&dev->i2c_dev, MPU6050_GYRO_CONFIG, MPU6050_GYRO_CONFIG_FS_SEL(dev->gfs));
	if (ret < 0)
	{
		goto out;
	}
	
	/* configure full scale range for ACCs */
	ret = i2c_write_reg(&dev->i2c_dev, MPU6050_ACCEL_CONFIG, MPU6050_ACCEL_CONFIG_AFS_SEL(dev->afs));
	if (ret < 0)
	{
		goto out;
	}

out:
	return ret;
}

static int read_raw(mpu6050_dev_t *dev, int16_t *data)
{
	int ret, i;
	uint8_t raw[14];	/* 6 bytes ACC, 2 temperature, 6 gyro */

	ret = i2c_read_block_reg(&dev->i2c_dev, MPU6050_ACCEL_XOUT_H, raw, sizeof(raw));
	if(ret < 0)
	{
		goto out;
	}

	for(i = 0; i < (sizeof(raw) >> 1); i++)
	{
		data[i] = (int16_t)((raw[(i << 1)] << 8) | raw[(i << 1) + 1]);
	}

out:
	return ret;
}

int mpu6050_read(mpu6050_dev_t *dev)
{
	int i, off, ret;
	int16_t val[7];

	ret = read_raw(dev, val);
	if(ret < 0)
	{
		goto out;
	}

	for(i = 0; i < 3; i++)
	{
		dev->acc.vec[i] = (float)(val[i]) / (float)((1 << 14) >> dev->afs);
	}
	
	dev->temperature = (float)(val[i++]) / 340.0 + 36.53;
	off = i;
	
	for(; i < 7; i++)
	{
		dev->gyro.vec[i - off] = (float)(val[i]) * (float)(250 << dev->gfs) / (float)(1 << 15);
	}
	
out:
	return ret;
}


#ifndef __MPU6050_H__
#define __MPU6050_H__

#include "../../i2c/i2c.h"
#include "../../util/math.h"


/* Digital low pass filter (dlpf) settings

    Accelerometer            Gyroscope
     (Fs = 1kHz)
#  Bandwidth  Delay    Bandwidth Delay    Fs
   (Hz)       (ms)     (Hz)      (ms)  (kHz)
0        260      0          256  0.98     8 
1        184    2.0          188   1.9     1 
2         94    3.0           98   2.8     1 
3         44    4.9           42   4.8     1 
4         21    8.5           20   8.3     1 
5         10   13.8           10  13.4     1 
6          5   19.0            5  18.6     1 
*/
typedef enum
{
	MPU6050_DLPF_CFG_260_256Hz = 0,
	MPU6050_DLPF_CFG_184_188Hz,
	MPU6050_DLPF_CFG_94_98Hz,
	MPU6050_DLPF_CFG_44_42Hz,
	MPU6050_DLPF_CFG_21_20Hz,
	MPU6050_DLPF_CFG_10_10Hz,
	MPU6050_DLPF_CFG_5_5Hz
}
mpu6050_dlpf_cfg_t;

/* Gyro full scale range in deg/sec */
typedef enum
{
	MPU6050_FS_SEL_250 = 0,		/* 131 LSB/Â°/s */
	MPU6050_FS_SEL_500,			/* 65.5 LSB/Â°/s */
	MPU6050_FS_SEL_1000,		/* 32.8 LSB/Â°/s */
	MPU6050_FS_SEL_2000,		/* 16.4 LSB/Â°/s */
}
mpu6050_fs_sel_t;

/* Accelerometer full scale range in G */
typedef enum
{
	MPU6050_AFS_SEL_2G = 0,		/* 16384 LSB/g */
	MPU6050_AFS_SEL_4G,			/* 8192 LSB/g */
	MPU6050_AFS_SEL_8G,			/* 4096 LSB/g */
	MPU6050_AFS_SEL_16G,		/* 2048 LSB/g */
}
mpu6050_afs_sel_t;

typedef struct
{
	i2c_dev_t i2c_dev;

	mpu6050_dlpf_cfg_t dlpf;
	mpu6050_fs_sel_t   gfs;
	mpu6050_afs_sel_t  afs;

	float temperature;

	vec3_t gyro;
	vec3_t acc;
}
mpu6050_dev_t;


int mpu6050_init(mpu6050_dev_t *dev, i2c_bus_t *bus, mpu6050_dlpf_cfg_t dlpf, mpu6050_fs_sel_t fs_sel, mpu6050_afs_sel_t afs_sel);

int mpu6050_read(mpu6050_dev_t *dev);


#endif /* __MPU6050_H__ */

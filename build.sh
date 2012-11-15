#!/bin/sh

gcc -std=gnu99 kalman.c util/sliding_avg.c util/math.c util/interval.c util/udp4.c mpu_main.c ahrs/madgwick_ahrs.c ahrs/ekf.c ahrs/matrix3x3.c i2c/i2c.c ahrs/util.c chips/itg3200/itg3200.c chips/bma180/bma180.c chips/mpu6050/mpu6050.c chips/hmc5883/hmc5883.c chips/ms5611/ms5611.c ahrs/mahony_ahrs.c -lm -lrt -lmeschach -o pengu_ahrs

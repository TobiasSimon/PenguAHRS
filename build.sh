#!/bin/sh

gcc main.c i2c/i2c.c ahrs/util.c chips/itg3200/itg3200.c chips/bma180/bma180.c chips/hmc5883/hmc5883.c ahrs/mahony_ahrs.c -lm -lrt -o pengu_ahrs

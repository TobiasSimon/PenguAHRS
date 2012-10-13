#!/bin/sh

gcc main.c i2c/i2c.c chips/itg3200/itg3200.c chips/bma180/bma180.c chips/hmc5883/hmc5883.c mahony/MahonyAHRS.c -lm -lrt -o ahrs

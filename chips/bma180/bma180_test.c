#include "i2c-dev.h"
#include "bma180.h"
#include <fcntl.h>
#include <stdlib.h>
#include <sys/ioctl.h>
#include <stdio.h>
#include <unistd.h>



int main(void) {
	bma180_dev_t dev;
	int ret;
	int bus_handle = open("/dev/i2c-4", O_RDWR);
	
	ioctl(bus_handle, I2C_SLAVE, BMA180_ADDRESS);
	
	ret = bma180_init(&dev, bus_handle, BMA180_RANGE_4G, BMA180_BW_10HZ);
	if(ret == -1)
		return EXIT_FAILURE;

	printf("offset_txyz: %i %i %i %i\n" \
		"gain_txyz: %i %i %i %i\n",
		dev.offset.t, dev.offset.x, dev.offset.y, dev.offset.z,
		dev.gain.t, dev.gain.x, dev.gain.y, dev.gain.z);

	while(1) {
		ret = bma180_read_acc(&dev);
		if(ret == -1)
			break;
		ret = bma180_read_temp(&dev);
		if(ret == -1)
			break;

		printf("\racc_xyz: %.4f %.4f %.4f, temperature: %f      ",
			dev.acc.x,
			dev.acc.y,
			dev.acc.z,
			dev.temperature);
		fflush(stdout);
	
		//usleep(2000);
	}

	return EXIT_FAILURE;
}


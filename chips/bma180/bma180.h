


#ifndef __BMA180_H__
#define __BMA180_H__


#include <stdint.h>


#define BMA180_ADDRESS 0x40

/* acceleration range */
typedef enum {
	BMA180_RANGE_1G = 0,
	BMA180_RANGE_1_5G,
	BMA180_RANGE_2G,
	BMA180_RANGE_3G,
	BMA180_RANGE_4G,
	BMA180_RANGE_8G,
	BMA180_RANGE_16G
} bma180_range_t;

typedef enum {
	/* low-pass */
	BMA180_BW_10HZ = 0,
	BMA180_BW_20HZ,
	BMA180_BW_40HZ,
	BMA180_BW_75HZ,
	BMA180_BW_150HZ,
	BMA180_BW_300HZ,
	BMA180_BW_600HZ,
	BMA180_BW_1200HZ,

	/* highpass: 1Hz */
	BMA180_BW_HP,

	/* band-pass: 0.2Hz ... 300Hz */
	BMA180_BW_BP
} bma180_bw_t;

typedef struct bma180_dev_s {
	/* device access */
	int bus;
	uint8_t i2c_addr;
   
	/* static information */
	uint8_t chip_id;
	uint8_t version;
	
	bma180_range_t range;
	bma180_bw_t bandwidth;

	float temperature;
	
	union {
		struct {
			int8_t t;
			int16_t x;
			int16_t y;
			int16_t z;
		};
		int16_t data[4];
	} offset;
	
	union {
		struct {
			int8_t t;
			int8_t x;
			int8_t y;
			int8_t z;
		};
		int8_t data[4];
	} gain;
	
	union {
			struct {
				float x;
				float y;
				float z;
			};
			float data[3];
	} acc;

} bma180_dev_t;

int bma180_init(bma180_dev_t *dev, int bus, bma180_range_t range, bma180_bw_t bandwidth);

int bma180_read_acc(bma180_dev_t *dev);
int bma180_read_temp(bma180_dev_t *dev);

#endif /* __BMA180_H__ */


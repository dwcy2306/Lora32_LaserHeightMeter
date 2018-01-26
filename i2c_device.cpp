#include <iostream>
#include <unistd.h>
#include <errno.h>
#include <stdint.h>
#include <fcntl.h>
#include <stdlib.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>

#include "i2c_device.h"

using namespace std;


int I2CDevice::i2cOpen(uint8_t address) {
	if ((fd = open("/dev/i2c-1", O_RDWR)) < 0) {
		return -1;
	}
	if (ioctl(fd, I2C_SLAVE, address) < 0) {
		return -1;
	}
	return 0;
}

uint8_t I2CDevice::readReg(uint8_t reg) {
	uint8_t buf[1];
	buf[0] = reg;
	if (write(fd, buf, 1) != 1) {
		return -1;
	}
	if (read(fd, buf, 1) != 1) {
		return -1;
	}

	return buf[0];
}

uint8_t I2CDevice::writeReg(uint8_t reg, uint8_t data) {
	uint8_t buf[1];
	buf[0] = reg;
	if (write(fd, buf, 1) != 1) {
		return -1;
	}
	buf[0] = data;
	if (write(fd, buf, 1) != 1) {
		return -1;
	}

	return 0;
}




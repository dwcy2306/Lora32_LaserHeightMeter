#ifndef __I2C_DEVICE__
#define __I2C_DEVICE__

#include <stdint.h>

class I2CDevice {
private:
	int fd;
public:
	int i2cOpen(uint8_t address);
	uint8_t readReg(uint8_t reg);
	uint8_t writeReg(uint8_t reg, uint8_t data);
};
#endif

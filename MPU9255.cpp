#include <math.h>
#include <stdint.h>
#include <sys/time.h>
#include <unistd.h>
#include <iostream>

#include "i2c_device.h"

using namespace std;

#include "MPU9255.h"


uint8_t MPU9255::initialize(uint8_t address) {
	mpu.i2cOpen(address);

	if (fd < 0) {  // If sensor missing
		return 0;
	}

	// 0x19 Sample Rate Divider - standard = 0 (+1)
	mpu9250Reset();
	usleep(20);

	//setEnabled(0);

	setBandWidth(20);
	mpu.writeReg(REG_SMPLRT_DIV, (uint8_t)HZ_TO_DIV(100));  // = setDatarate(100);
	setRegister(I2C_ADDRESS, REG_GYRO_CONFIG, MSK_GYRO_FS_SEL, 0x01 << 3);	// == setGSensitivity(VAL_GYRO_FS_0500);
	setRegister(I2C_ADDRESS, REG_ACCEL_CONFIG, MSK_ACCEL_FS_SEL, 0x01 << 3);  // == setASensitivity(VAL_ACCEL_FS_02);

	// Clock Source
	setRegister(I2C_ADDRESS, REG_PWR_MGMT_1, MSK_CLKSEL, 1);

	// PWR MGMT
	setRegister(I2C_ADDRESS, REG_PWR_MGMT_1, MSK_CYCLE | MSK_GYRO_STANDBY_CYCLE | MSK_PD_PTAT, 0);
	
	// INTs
	setRegister(I2C_ADDRESS, REG_INT_PIN_CFG, MSK_INT_ANYRD_2CLEAR, MSK_INT_ANYRD_2CLEAR);	// Clear INT at read
	setRegister(I2C_ADDRESS, REG_INT_ENABLE, MSK_RAW_RDY_EN, MSK_RAW_RDY_EN);  // INT: Raw data ready to read

	// I2C-Master
	setRegister(I2C_ADDRESS, REG_USER_CTRL, MSK_I2C_MST_EN, 0);  // disable I2C-Master
	setRegister(I2C_ADDRESS, REG_INT_PIN_CFG, MSK_BYPASS_EN, MSK_BYPASS_EN);

	setEnabled(1);

	calibAccelGyro();

	uint8_t who = mpu.readReg(REG_WHO_AM_I);
	if (who == VAL_WHOAMI_6500) {
		cout << "MPU-6500" << endl;
		return 2;
	} else if (who == VAL_WHOAMI_9250) {
		cout << "MPU-9250" << endl;
		return 3;
	} else if (who == VAL_WHOAMI_9255) {
		cout << "MPU-9255" << endl;
		return 4;
	} else {
		cout << "Unknown Sensor" << endl;
		return 1;
	}
}

void MPU9255::mpu9250Reset(void) {
	uint8_t rVector;
	rVector = MSK_GYRO_RST | MSK_ACCEL_RST | MSK_TEMP_RST;
	setRegister(I2C_ADDRESS, REG_SIGNAL_PATH_RESET, rVector, rVector);
	setRegister(I2C_ADDRESS, REG_USER_CTRL, MSK_SIG_COND_RST, MSK_SIG_COND_RST);
	setRegister(I2C_ADDRESS, REG_PWR_MGMT_1, MSK_H_RESET, MSK_H_RESET);
}

void MPU9255::setRegister(const uint8_t address, const uint8_t registeraddress, const uint8_t mask, const uint8_t writevalue) {
	uint8_t _setting;

	_setting = mpu.readReg(registeraddress);
	_setting &= ~mask;
	_setting |= (writevalue & mask);
	mpu.writeReg(registeraddress, _setting);
}

void MPU9255::setBandWidth(const uint16_t hzFreq) {
	uint8_t dlpf;
	if (hzFreq > 184) dlpf = 0;		// 460Hz Acc, FS = 1KHz // 250Hz Gyro, 4000Hz Temp, FS = 8KHz
	else if (hzFreq > 92) dlpf = 1;	// 184Hz Gyro/Acc, 4000Hz Temp, FS = 1KHz
	else if (hzFreq > 41) dlpf = 2;	// 92Hz Gyro/Acc, 4000Hz Temp, FS = 1KHz
	else if (hzFreq > 20) dlpf = 3;	// 41Hz Gyro/Acc, 42Hz Temp, FS = 1KHz
	else if (hzFreq > 10) dlpf = 4;	// 20Hz Gyro/Acc, 20Hz Temp, FS = 1KHz
	else if (hzFreq > 5) dlpf = 5;	// 10Hz Gyro/Acc, 10Hz Temp, FS = 1KHz
	else dlpf = 6;					// 5Hz Gyro/Acc, 5Hz Temp, FS = 1KHz

	setRegister(I2C_ADDRESS, REG_CONFIG, MSK_DLPF_CFG, dlpf);
	setRegister(I2C_ADDRESS, REG_GYRO_CONFIG, MSK_FCHOICE_B, 0);

	setRegister(I2C_ADDRESS, REG_ACCEL_CONFIG2, MSK_A_DLPF_CFG, dlpf);
	setRegister(I2C_ADDRESS, REG_ACCEL_CONFIG2, MSK_ACCEL_FCHOICE_B, 0);
}

void MPU9255::setEnabled(const uint8_t enable) {
	uint8_t _value;
	if (enable) _value = 0;
	else _value = 255;
	setRegister(I2C_ADDRESS, REG_PWR_MGMT_1, MSK_SLEEP, _value);
	setRegister(I2C_ADDRESS, REG_PWR_MGMT_2, 0x3F, _value);  // Enable XYZ of Gyr & Acc
}

void MPU9255::calibAccelGyro(void) {
	float sumAcX = 0, sumAcY = 0, sumAcZ = 0;
	float sumGyX = 0, sumGyY = 0, sumGyZ=  0;

	getMeasurement();

	for (int i = 0; i < 10; i++) {
		getMeasurement();
		sumAcX += xyz_AccTmpGyr[0]; sumAcY += xyz_AccTmpGyr[1]; sumAcZ += xyz_AccTmpGyr[2];
		sumGyX += xyz_AccTmpGyr[4]; sumGyY += xyz_AccTmpGyr[5]; sumGyZ += xyz_AccTmpGyr[6];
		usleep(100000);
	}
	baseAcX = sumAcX / 10; baseAcY = sumAcY / 10; baseAcZ = sumAcZ / 10;
	baseGyX = sumGyX / 10; baseGyY = sumGyY / 10; baseGyZ = sumGyZ / 10;
}

void MPU9255::getMeasurement(void) {
	uint8_t _data[14];

	// i2c.read(I2C_ADDRESS, REG_ACCEL_XOUT_H, _data, 14);
	for (int i = 0; i < 14; i++) {
		_data[i] = mpu.readReg(REG_ACCEL_XOUT_H + i);
	}

	xyz_AccTmpGyr[0] = int16_t(_data[0] << 8 | _data[1]);  // Acc
	xyz_AccTmpGyr[1] = int16_t(_data[2] << 8 | _data[3]); xyz_AccTmpGyr[2] = int16_t(_data[4] << 8 | _data[5]);

	xyz_AccTmpGyr[3] = int16_t(_data[6] << 8 | _data[7]);  // Tmp

	xyz_AccTmpGyr[4] = int16_t(_data[8] << 8 | _data[9]);  // Gyr
	xyz_AccTmpGyr[5] = int16_t(_data[10] << 8 | _data[11]);
	xyz_AccTmpGyr[6] = int16_t(_data[12] << 8 | _data[13]);
	
	xyz_AccTmpGyr[3] /= 333.87 + 21;
}

void MPU9255::initDT() {
	gettimeofday(&systime, NULL);
	t_prev = systime.tv_usec;
	cout << "initDt: " << t_prev << endl;
}

void MPU9255::calcDT() {
	gettimeofday(&systime, NULL);
	t_now = systime.tv_usec;
	dt = t_now - t_prev;
	if (t_prev > t_now) {
		dt += 1000000;
	}
	dt /= 1000000;
	t_prev = t_now;
}

void MPU9255::calcAccelYPR(void) {
	float accel_x, accel_y, accel_z;
	float accel_xz, accel_yz;
	const float RADIANS_TO_DEGREES = 180/3.14159;

	accel_x = xyz_AccTmpGyr[0] - baseAcX;
	accel_y = xyz_AccTmpGyr[1] - baseAcY;
	accel_z = xyz_AccTmpGyr[2] + (16384 - baseAcZ);

	accel_yz = sqrt(pow(accel_y, 2) + pow(accel_z, 2));
	accel_angle_y = atan(-accel_x / accel_yz)*RADIANS_TO_DEGREES;;

	accel_xz = sqrt(pow(accel_x, 2) + pow(accel_z, 2));
	accel_angle_x = atan(accel_y / accel_xz)*RADIANS_TO_DEGREES;

	accel_angle_z = 0;
}

void MPU9255::calcGyroYPR(void) {
	const float GYROXYZ_TO_DEGREES_PER_SED = 131;

	gyro_x = (xyz_AccTmpGyr[4] - baseGyX) / GYROXYZ_TO_DEGREES_PER_SED;
	gyro_y = (xyz_AccTmpGyr[5] - baseGyY) / GYROXYZ_TO_DEGREES_PER_SED;
	gyro_z = (xyz_AccTmpGyr[6] - baseGyZ) / GYROXYZ_TO_DEGREES_PER_SED;

	gyro_angle_x += gyro_x * dt;
	gyro_angle_y += gyro_y * dt;
	gyro_angle_z += gyro_z * dt;
}

void MPU9255::calcFilteredYPR(void) {
	float tmp_angle_x, tmp_angle_y, tmp_angle_z;

	tmp_angle_x = filtered_angle_x + (gyro_x * dt);
	tmp_angle_y = filtered_angle_y + (gyro_y * dt);
	// tmp_angle_z = filtered_angle_z + (gyro_z * dt);

	filtered_angle_x = ALPHA * tmp_angle_x + (1.0 - ALPHA) * accel_angle_x * 2;
	filtered_angle_y = ALPHA * tmp_angle_y + (1.0 - ALPHA) * accel_angle_y * 2;
	// filtered_angle_z = tmp_angle_z;
}

void MPU9255::getResult(void) {  // { x, y }
	getMeasurement();
	calcAccelYPR();
	calcGyroYPR();
	calcFilteredYPR();

	angles[0] = filtered_angle_x;
	angles[1] = filtered_angle_y;

	return;
}

void MPU9255::setAlpha(float val) {
	ALPHA = val;
}


MPU9255 Mpu = MPU9255();





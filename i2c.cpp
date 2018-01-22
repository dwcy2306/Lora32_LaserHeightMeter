#include <math.h>
#include <iostream>
#include <wiringPi.h>
#include <wiringPiI2C.h>
#include <stdint.h>
#include <sys/time.h>
#include <unistd.h>

using namespace std;

static const uint8_t	I2C_ADDRESS						=(0x68);

//MPU9250 Register map
static const uint8_t	REG_SELF_TEST_X_GYRO			=(0x00);
static const uint8_t	REG_SELF_TEST_Y_GYRO			=(0x01);
static const uint8_t	REG_SELF_TEST_Z_GYRO			=(0x02);
static const uint8_t	REG_SELF_TEST_X_ACCEL			=(0x0D);
static const uint8_t	REG_SELF_TEST_Y_ACCEL			=(0x0E);
static const uint8_t	REG_SELF_TEST_Z_ACCEL			=(0x0F);
static const uint8_t	REG_XG_OFFSET_H					=(0x13);
static const uint8_t	REG_XG_OFFSET_L					=(0x14);
static const uint8_t	REG_YG_OFFSET_H					=(0x15);
static const uint8_t	REG_YG_OFFSET_L					=(0x16);
static const uint8_t	REG_ZG_OFFSET_H					=(0x17);
static const uint8_t	REG_ZG_OFFSET_L					=(0x18);
static const uint8_t	REG_SMPLRT_DIV					=(0x19);
inline const uint8_t	HZ_TO_DIV(const uint16_t hz)
{
	return ((1000/hz) - 1);
};

static const uint8_t	REG_CONFIG						=(0x1A);
static const uint8_t	MSK_FIFO_MODE					=(0x40);
static const uint8_t	MSK_EXT_SYNC_SET				=(0x38);
static const uint8_t	MSK_DLPF_CFG					=(0x07);

static const uint8_t	REG_GYRO_CONFIG					=(0x1B);
static const uint8_t	MSK_XGYRO_CTEN					=(0x80);
static const uint8_t	MSK_YGYRO_CTEN					=(0x40);
static const uint8_t	MSK_ZGYRO_CTEN					=(0x20);
static const uint8_t	MSK_GYRO_FS_SEL					=(0x18);
static const uint8_t	VAL_GYRO_FS_0250				=(0x00);
static const uint8_t	VAL_GYRO_FS_0500				=(0x01);
static const uint8_t	VAL_GYRO_FS_1000				=(0x02);
static const uint8_t	VAL_GYRO_FS_2000				=(0x03);
static const uint8_t	MSK_FCHOICE_B					=(0x03);

static const uint8_t	REG_ACCEL_CONFIG				=(0x1C);
static const uint8_t	MSK_AX_ST_EN					=(0x80);
static const uint8_t	MSK_AY_ST_EN					=(0x40);
static const uint8_t	MSK_AZ_ST_EN					=(0x20);
static const uint8_t	MSK_ACCEL_FS_SEL				=(0x18);
static const uint8_t	VAL_ACCEL_FS_02					=(0x00);
static const uint8_t	VAL_ACCEL_FS_04					=(0x01);
static const uint8_t	VAL_ACCEL_FS_08					=(0x02);
static const uint8_t	VAL_ACCEL_FS_16					=(0x03);
static const uint8_t	REG_ACCEL_CONFIG2				=(0x1D);
static const uint8_t	MSK_ACCEL_FCHOICE_B				=(0xC0);
static const uint8_t	MSK_A_DLPF_CFG					=(0x03);

static const uint8_t	REG_LP_ACCEL_ODR				=(0x1E);
static const uint8_t	MSK_LPOSC_CLKSEL				=(0x0F);

static const uint8_t	REG_WOM_THR						=(0x1F);

static const uint8_t	REG_FIFO_EN						=(0x23);
static const uint8_t	MSK_TEMP_FIFO_EN				=(0x80);
static const uint8_t	MSK_GYRO_XOUT					=(0x40);
static const uint8_t	MSK_GYRO_YOUT					=(0x20);
static const uint8_t	MSK_GYRO_ZOUT					=(0x10);
static const uint8_t	MSK_ACCEL						=(0x08);
static const uint8_t	MSK_SLV2						=(0x04);
static const uint8_t	MSK_SLV1						=(0x02);
static const uint8_t	MSK_SLV0						=(0x01);
	
static const uint8_t	REG_I2C_MST_CTRL				=(0x24);
static const uint8_t	MSK_MULT_MST_EN					=(0x80);
static const uint8_t	MSK_WAIT_FOR_ES					=(0x40);
static const uint8_t	MSK_SLV_3_FIFO_EN				=(0x20);
static const uint8_t	MSK_I2C_MST_P_NSR				=(0x10);
static const uint8_t	MSK_I2C_MST_CLK					=(0x0F);

static const uint8_t	REG_I2C_SLV0_ADDR				=(0x25);
static const uint8_t	MSK_I2C_SLV0_RNW				=(0x80);
static const uint8_t	MSK_I2C_ID_0					=(0x7F);

static const uint8_t	REG_I2C_SLV0_REG				=(0x26);
static const uint8_t	REG_I2C_SLV0_CTRL				=(0x27);
static const uint8_t	MSK_I2C_SLV0_EN					=(0x80);
static const uint8_t	MSK_I2C_SLV0_BYTE_SW			=(0x40);
static const uint8_t	MSK_I2C_SLV0_REG_DIS			=(0x20);
static const uint8_t	MSK_I2C_SLV0_GRP				=(0x10);
static const uint8_t	MSK_I2C_SLV0_LENG				=(0x0F);

// [without SLV0 to SLV4]

static const uint8_t	REG_I2C_MST_STATUS				=(0x36);
static const uint8_t	MSK_PASS_THROUGH				=(0x80);
static const uint8_t	MSK_I2C_SLV4_DONE				=(0x40);
static const uint8_t	MSK_I2C_LOST_ARB				=(0x20);
static const uint8_t	MSK_I2C_SLV4_NACK				=(0x10);
static const uint8_t	MSK_I2C_SLV3_NACK				=(0x08);
static const uint8_t	MSK_I2C_SLV2_NACK				=(0x04);
static const uint8_t	MSK_I2C_SLV1_NACK				=(0x02);
static const uint8_t	MSK_I2C_SLV0_NACK				=(0x01);

static const uint8_t	REG_INT_PIN_CFG					=(0x37);
static const uint8_t	MSK_ACTL						=(0x80);
static const uint8_t	MSK_OPEN						=(0x40);
static const uint8_t	MSK_LATCH_INT_EN				=(0x20);
static const uint8_t	MSK_INT_ANYRD_2CLEAR			=(0x10);
static const uint8_t	MSK_ACTL_FSYNC					=(0x08);
static const uint8_t	MSK_FSYNC_INT_MODE_EN			=(0x04);
static const uint8_t	MSK_BYPASS_EN					=(0x02);

static const uint8_t	REG_INT_ENABLE					=(0x38);
static const uint8_t	MSK_WOM_EN						=(0x40);
static const uint8_t	MSK_FIFO_OFLOW_EN				=(0x10);
static const uint8_t	MSK_FSYNC_INT_EN				=(0x08);
static const uint8_t	MSK_RAW_RDY_EN					=(0x01);

static const uint8_t	REG_INT_STATUS					=(0x3A);
static const uint8_t	MSK_WOM_INT						=(0x40);
static const uint8_t	MSK_FIFO_OFLOW_INT				=(0x10);
static const uint8_t	MSK_FSYNC_INT					=(0x08);
static const uint8_t	MSK_RAW_DATA_RDY_INT			=(0x01);

static const uint8_t	REG_ACCEL_XOUT_H				=(0x3B);
static const uint8_t	REG_ACCEL_XOUT_L				=(0x3C);
static const uint8_t	REG_ACCEL_YOUT_H				=(0x3D);
static const uint8_t	REG_ACCEL_YOUT_L				=(0x3E);
static const uint8_t	REG_ACCEL_ZOUT_H				=(0x3F);
static const uint8_t	REG_ACCEL_ZOUT_L				=(0x40);
static const uint8_t	REG_TEMP_OUT_H					=(0x41);
static const uint8_t	REG_TEMP_OUT_L					=(0x42);
static const uint8_t	REG_GYRO_XOUT_H					=(0x43);
static const uint8_t	REG_GYRO_XOUT_L					=(0x44);
static const uint8_t	REG_GYRO_YOUT_H					=(0x45);
static const uint8_t	REG_GYRO_YOUT_L					=(0x46);
static const uint8_t	REG_GYRO_ZOUT_H					=(0x47);
static const uint8_t	REG_GYRO_ZOUT_L					=(0x48);

static const uint8_t	REG_EXT_SENS_DATA_00			=(0x49);
static const uint8_t	REG_EXT_SENS_DATA_01			=(0x4A);
static const uint8_t	REG_EXT_SENS_DATA_02			=(0x4B);
static const uint8_t	REG_EXT_SENS_DATA_03			=(0x4C);
static const uint8_t	REG_EXT_SENS_DATA_04			=(0x4D);
static const uint8_t	REG_EXT_SENS_DATA_05			=(0x4E);
static const uint8_t	REG_EXT_SENS_DATA_06			=(0x4F);
static const uint8_t	REG_EXT_SENS_DATA_07			=(0x50);
static const uint8_t	REG_EXT_SENS_DATA_08			=(0x51);
static const uint8_t	REG_EXT_SENS_DATA_09			=(0x52);
static const uint8_t	REG_EXT_SENS_DATA_10			=(0x53);
static const uint8_t	REG_EXT_SENS_DATA_11			=(0x54);
static const uint8_t	REG_EXT_SENS_DATA_12			=(0x55);
static const uint8_t	REG_EXT_SENS_DATA_13			=(0x56);
static const uint8_t	REG_EXT_SENS_DATA_14			=(0x57);
static const uint8_t	REG_EXT_SENS_DATA_15			=(0x58);
static const uint8_t	REG_EXT_SENS_DATA_16			=(0x59);
static const uint8_t	REG_EXT_SENS_DATA_17			=(0x5A);
static const uint8_t	REG_EXT_SENS_DATA_18			=(0x5B);
static const uint8_t	REG_EXT_SENS_DATA_19			=(0x5C);
static const uint8_t	REG_EXT_SENS_DATA_20			=(0x5D);
static const uint8_t	REG_EXT_SENS_DATA_21			=(0x5E);
static const uint8_t	REG_EXT_SENS_DATA_22			=(0x5F);
static const uint8_t	REG_EXT_SENS_DATA_23			=(0x60);

static const uint8_t	REG_I2C_SLV0_DO					=(0x63);
static const uint8_t	REG_I2C_SLV1_DO					=(0x64);
static const uint8_t	REG_I2C_SLV2_DO					=(0x65);
static const uint8_t	REG_I2C_SLV3_DO					=(0x66);

static const uint8_t	REG_I2C_MST_DELAY_CTRL			=(0x67);
static const uint8_t	MSK_DELAY_ES_SHADOW				=(0x80);
static const uint8_t	MSK_I2C_SLV4_DLY_EN				=(0x10);
static const uint8_t	MSK_I2C_SLV3_DLY_EN				=(0x08);
static const uint8_t	MSK_I2C_SLV2_DLY_EN				=(0x04);
static const uint8_t	MSK_I2C_SLV1_DLY_EN				=(0x02);
static const uint8_t	MSK_I2C_SLV0_DLY_EN				=(0x01);

static const uint8_t	REG_SIGNAL_PATH_RESET			=(0x68);
static const uint8_t	MSK_GYRO_RST					=(0x04);
static const uint8_t	MSK_ACCEL_RST					=(0x02);
static const uint8_t	MSK_TEMP_RST					=(0x01);

static const uint8_t	REG_MOT_DETECT_CTRL				=(0x69);
static const uint8_t	MSK_ACCEL_INTEL_EN				=(0x80);
static const uint8_t	MSK_ACCEL_INTEL_MODE			=(0x40);

static const uint8_t	REG_USER_CTRL					=(0x6A);
static const uint8_t	MSK_FIFO_EN						=(0x40);
static const uint8_t	MSK_I2C_MST_EN					=(0x20);
static const uint8_t	MSK_I2C_IF_DIS					=(0x10);
static const uint8_t	MSK_FIFO_RST					=(0x04);
static const uint8_t	MSK_I2C_MST_RST					=(0x02);
static const uint8_t	MSK_SIG_COND_RST				=(0x01);

static const uint8_t	REG_PWR_MGMT_1					=(0x6B);
static const uint8_t	MSK_H_RESET						=(0x80);
static const uint8_t	MSK_SLEEP						=(0x40);
static const uint8_t	MSK_CYCLE						=(0x20);
static const uint8_t	MSK_GYRO_STANDBY_CYCLE			=(0x10);
static const uint8_t	MSK_PD_PTAT						=(0x08);
static const uint8_t	MSK_CLKSEL						=(0x07);

static const uint8_t	REG_PWR_MGMT_2					=(0x6C);
static const uint8_t	MSK_DISABLE_XA					=(0x20);
static const uint8_t	MSK_DISABLE_YA					=(0x10);
static const uint8_t	MSK_DISABLE_ZA					=(0x08);
static const uint8_t	MSK_DISABLE_XG					=(0x04);
static const uint8_t	MSK_DISABLE_YG					=(0x02);
static const uint8_t	MSK_DISABLE_ZG					=(0x01);
static const uint8_t	MSK_DISABLE_XYZA				=(0x38);
static const uint8_t	MSK_DISABLE_XYZG				=(0x07);

static const uint8_t	REG_FIFO_COUNTH					=(0x72);
static const uint8_t	REG_FIFO_COUNTL					=(0x73);
static const uint8_t	REG_FIFO_R_W					=(0x74);
static const uint8_t	REG_WHO_AM_I					=(0x75);
static const uint8_t	VAL_WHOAMI_6500					=(0x70);
static const uint8_t	VAL_WHOAMI_9250					=(0x71);

static const uint8_t	REG_XA_OFFSET_H					=(0x77);
static const uint8_t	REG_XA_OFFSET_L					=(0x78);
static const uint8_t	REG_YA_OFFSET_H					=(0x7A);
static const uint8_t	REG_YA_OFFSET_L					=(0x7B);
static const uint8_t	REG_ZA_OFFSET_H					=(0x7D);
static const uint8_t	REG_ZA_OFFSET_L					=(0x7E);

//reset values
static const uint8_t	WHOAMI_RESET_VAL				=(0x71);
static const uint8_t	POWER_MANAGMENT_1_RESET_VAL		=(0x01);
static const uint8_t	DEFAULT_RESET_VALUE				=(0x00);
static const uint8_t	WHOAMI_DEFAULT_VAL				=(0x68);

//Magnetometer register maps
static const uint8_t	REG_MAG_WIA						=(0x00);
static const uint8_t	REG_MAG_INFO					=(0x01);
static const uint8_t	REG_MAG_ST1						=(0x02);
static const uint8_t	REG_MAG_XOUT_L					=(0x03);
static const uint8_t	REG_MAG_XOUT_H					=(0x04);
static const uint8_t	REG_MAG_YOUT_L					=(0x05);
static const uint8_t	REG_MAG_YOUT_H					=(0x06);
static const uint8_t	REG_MAG_ZOUT_L					=(0x07);
static const uint8_t	REG_MAG_ZOUT_H					=(0x08);
static const uint8_t	REG_MAG_ST2						=(0x09);
static const uint8_t	REG_MAG_CNTL					=(0x0A);
static const uint8_t	REG_MAG_RSV						=(0x0B); //reserved mystery meat
static const uint8_t	REG_MAG_ASTC					=(0x0C);
static const uint8_t	REG_MAG_TS1						=(0x0D);
static const uint8_t	REG_MAG_TS2						=(0x0E);
static const uint8_t	REG_MAG_I2CDIS					=(0x0F);
static const uint8_t	REG_MAG_ASAX					=(0x10);
static const uint8_t	REG_MAG_ASAY					=(0x11);
static const uint8_t	REG_MAG_ASAZ					=(0x12);
//Magnetometer register masks
static const uint8_t	MPU9250_WIA						=(0x48);
static const uint8_t	MPU9250_						=(0x00);


// Global Variables start
int fd;
float accel_angle_x, accel_angle_y, accel_angle_z;
float gyro_angle_x, gyro_angle_y, gyro_angle_z;
float filtered_angle_x, filtered_angle_y, filtered_angle_z;
float baseAcX, baseAcY, baseAcZ;
float baseGyX, baseGyY, baseGyZ;

static float xyz_AccTmpGyr[9];	// 0, 1, 2 = Acc, 3 = Tmp, 4, 5, 6 = Gyr

struct timeval systime;
double t_prev, t_now;
double dt;

float gyro_x, gyro_y, gyro_z;
// Global Variables end

// Function Prototypes start
uint8_t initialize(void);
void mpu9250Reset(void);
void setRegister(const uint8_t address, const uint8_t registeraddress, const uint8_t mask, const uint8_t writevalue);
void setBandWidth(const uint16_t hzFreq);
void setEnabled(const uint8_t enable);
void calibAccelGyro(void);
void getMeasurement(void);
void initDT(void);
void calcDT(void);
void calcAccelYPR(void);
void calcGyroYPR(void);
void calcFilteredYPR(void);
// Function Prototypes end

/********************************/
/********** Main start **********/
int main(void) {
	int result;

	switch(initialize()) {
		case 0:
			cout << "MPU9250 Missing" << endl;
			return 1;
		case 1:
			cout << "Found unknown sensor" << endl;
			break;
		case 2:
			cout << "Found MPU6500" << endl;
			break;
		case 3:
			cout << "Found MPU9250" << endl;
			break;
	}
	getMeasurement();
	calibAccelGyro();

	initDT();  // calc dt
	dt = 0;
	getMeasurement();
	calcAccelYPR();
	calcGyroYPR();
	calcFilteredYPR();
	printf("");
	printf("");
	calcDT();

	while (1) {
		getMeasurement();
		calcAccelYPR();
		calcGyroYPR();
		calcFilteredYPR();

		printf("%0.2f\t%0.2f\t", filtered_angle_x, filtered_angle_y);
		calcDT();
	}

	return 0;
}
/********** Main end ***********/

// Functions start
uint8_t initialize(void) {
	fd = wiringPiI2CSetup(0x68);
	if (fd == -1) {  // If sensor missing
		return 0;
	}

	// 0x19 Sample Rate Divider - standard = 0 (+1)
	mpu9250Reset();
	delayMicroseconds(20);

	//setEnabled(0);

	setBandWidth(20);
	wiringPiI2CWriteReg8(fd, REG_SMPLRT_DIV, (uint8_t)HZ_TO_DIV(100));	// == setDatarate(100);
	setRegister(I2C_ADDRESS, REG_GYRO_CONFIG, MSK_GYRO_FS_SEL, 0x01 << 3);	// == setGSensitivity(VAL_GYRO_FS_0500);
	setRegister(I2C_ADDRESS, REG_ACCEL_CONFIG, MSK_ACCEL_FS_SEL, 0x00 << 3);  // == setASensitivity(VAL_ACCEL_FS_02);

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

	uint8_t who = wiringPiI2CReadReg8(fd, REG_WHO_AM_I);
	if (who == VAL_WHOAMI_6500) return 2;
	else if (who == VAL_WHOAMI_9250) return 3;
	else return 1;
}

void mpu9250Reset(void) {
	uint8_t rVector;
	rVector = MSK_GYRO_RST | MSK_ACCEL_RST | MSK_TEMP_RST;
	setRegister(I2C_ADDRESS, REG_SIGNAL_PATH_RESET, rVector, rVector);
	setRegister(I2C_ADDRESS, REG_USER_CTRL, MSK_SIG_COND_RST, MSK_SIG_COND_RST);
	setRegister(I2C_ADDRESS, REG_PWR_MGMT_1, MSK_H_RESET, MSK_H_RESET);
}

void setRegister(const uint8_t address, const uint8_t registeraddress, const uint8_t mask, const uint8_t writevalue) {
	uint8_t _setting;

	_setting = wiringPiI2CReadReg8(fd, registeraddress);
	_setting &= ~mask;
	_setting |= (writevalue & mask);
	wiringPiI2CWriteReg8(fd, registeraddress, _setting);
}

void setBandWidth(const uint16_t hzFreq) {
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

void setEnabled(const uint8_t enable) {
	uint8_t _value;
	if (enable) _value = 0;
	else _value = 255;
	setRegister(I2C_ADDRESS, REG_PWR_MGMT_1, MSK_SLEEP, _value);
	setRegister(I2C_ADDRESS, REG_PWR_MGMT_2, 63, _value);  // Enable XYZ of Gyr & Acc
}

void calibAccelGyro(void) {
	float sumAcX = 0, sumAcY = 0, sumAcZ = 0;
	float sumGyX = 0, sumGyY = 0, sumGyZ=  0;

	getMeasurement();

	for (int i = 0; i < 10; i++) {
		getMeasurement();
		sumAcX += xyz_AccTmpGyr[0]; sumAcY += xyz_AccTmpGyr[1]; sumAcZ += xyz_AccTmpGyr[2];
		sumGyX += xyz_AccTmpGyr[4]; sumGyY += xyz_AccTmpGyr[5]; sumGyZ += xyz_AccTmpGyr[6];
		delayMicroseconds(100);
	}
	baseAcX = sumAcX / 10; baseAcY = sumAcY / 10; baseAcZ = sumAcZ / 10;
	baseGyX = sumGyX / 10; baseGyY = sumGyY / 10; baseGyZ = sumGyZ / 10;
}

void getMeasurement(void) {
	uint8_t _data[14];

	// i2c.read(I2C_ADDRESS, REG_ACCEL_XOUT_H, _data, 14);
	for (int i = 0; i < 14; i++) {
		_data[i] = wiringPiI2CReadReg8(I2C_ADDRESS, REG_ACCEL_XOUT_H + i);
	}

	xyz_AccTmpGyr[0] = int16_t(_data[0] << 8 | _data[1]);  // Acc
	xyz_AccTmpGyr[1] = int16_t(_data[2] << 8 | _data[3]);
	xyz_AccTmpGyr[2] = int16_t(_data[4] << 8 | _data[5]);

	xyz_AccTmpGyr[3] = int16_t(_data[6] << 8 | _data[7]);  // Tmp

	xyz_AccTmpGyr[4] = int16_t(_data[8] << 8 | _data[9]);  // Gyr
	xyz_AccTmpGyr[5] = int16_t(_data[10] << 8 | _data[11]);
	xyz_AccTmpGyr[6] = int16_t(_data[12] << 8 | _data[13]);
	
	xyz_AccTmpGyr[3] /= 333.87 + 21;
}

void initDT() {
	gettimeofday(&systime, NULL);
	t_prev = /*(systime.tv_sec % 100000) * 1000000 + */(systime.tv_usec);
	cout << "initDt: " << t_prev << endl;
}

void calcDT() {
	gettimeofday(&systime, NULL);
	t_now = /*(systime.tv_sec % 100000) * 1000000 + */(systime.tv_usec);
	dt = t_now - t_prev;
	cout << "calcDt: t_prev=" << t_prev << ", t_now=" << t_now << ", dt=" << dt << endl;

	t_prev = t_now;
}

void calcAccelYPR(void) {
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

void calcGyroYPR(void) {
	const float GYROXYZ_TO_DEGREES_PER_SED = 131;

	gyro_x = (xyz_AccTmpGyr[4] - baseGyX) / GYROXYZ_TO_DEGREES_PER_SED;
	gyro_y = (xyz_AccTmpGyr[5] - baseGyY) / GYROXYZ_TO_DEGREES_PER_SED;
	gyro_z = (xyz_AccTmpGyr[6] - baseGyZ) / GYROXYZ_TO_DEGREES_PER_SED;

	gyro_angle_x += gyro_x * dt;
	gyro_angle_y += gyro_y * dt;
	gyro_angle_z += gyro_z * dt;
}

void calcFilteredYPR(void) {
	const float ALPHA = 0.92;
	float tmp_angle_x, tmp_angle_y, tmp_angle_z;

	tmp_angle_x = filtered_angle_x + gyro_x * dt;
	tmp_angle_y = filtered_angle_y + gyro_y * dt;
	tmp_angle_z = filtered_angle_z + gyro_z * dt;

	filtered_angle_x = ALPHA * tmp_angle_x + (1.0 - ALPHA) * accel_angle_x * 2;
	filtered_angle_y = ALPHA * tmp_angle_y + (1.0 - ALPHA) * accel_angle_y * 2;
	filtered_angle_z = tmp_angle_z;
}

// Functions end




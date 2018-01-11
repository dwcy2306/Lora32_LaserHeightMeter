#include <U8x8lib.h>
#include <LoRa.h>
#include <Wire.h>
#include "i2c.h"
#include "i2c_MPU9250.h"
//#include <TimerOne.h>

MPU9250 mpu9250;

// WIFI_LoRa_32 ports
// GPIO5  -- SX1278's SCK
// GPIO19 -- SX1278's MISO
// GPIO27 -- SX1278's MOSI
// GPIO18 -- SX1278's CS
// GPIO14 -- SX1278's RESET
// GPIO26 -- SX1278's IRQ(Interrupt Request)

#define SS      18
#define RST     14
#define DI0     26
#define BAND    433E6

#define    AK8963_ADDRESS                 0x0C
#define    AK8963_RA_WHO_AM_I             0x00   // 0x48 return
#define    AK8963_RA_ST1                  0x02
#define    AK8963_RA_HXL                  0x03
#define    AK8963_RA_CNTL1                0x0A
#define    AK8963_RA_ST2                  0x0B
#define    AK8963_RA_ASAX                 0x10
#define    MAX_COUNT                      (0x0f + 1)

// the OLED used
U8X8_SSD1306_128X64_NONAME_SW_I2C u8x8(/* clock=*/ 15, /* data=*/ 4, /* reset=*/ 16);

float dt;
float accel_angle_x, accel_angle_y, accel_angle_z;
float gyro_angle_x, gyro_angle_y, gyro_angle_z;
float filtered_angle_x, filtered_angle_y, filtered_angle_z;
float baseAcX, baseAcY, baseAcZ;
float baseGyX, baseGyY, baseGyZ;
int16_t AcX, AcY, AcZ, Tmp, GyX, GyY, GyZ;

static float xyz_GyrAccMag[9];  // 0, 1, 2 = Acc, 3 = Mag, 4, 5, 6 = Gyr

// Mag
typedef struct MAG {
  int maximum;
  int average;
  int minimum;
} mag_t;
typedef struct MEASUREMENT {
  int x;
  int y;
  int z;
} measurement_t;
uint8_t ASA[3];
mag_t mag[3];
measurement_t m_data[MAX_COUNT];
int m_count = 0;
long count = 0;
volatile bool intFlag = false;

void setup() {
  
  SPI.begin(5, 19, 27, 18);
  LoRa.setPins(SS, RST, DI0);

  Serial.begin(115200);
  while (!Serial); //Wait until the computer connected
  delay(1000);

  u8x8.begin();
  u8x8.setFont(u8x8_font_artossans8_r);
  u8x8.draw2x2String(1, 3, "PowerOn");
  delay(1000);
  u8x8.clear();
  u8x8.setFont(u8x8_font_victoriamedium8_r);
  if (!LoRa.begin(BAND)) {
    Serial.println("Starting LoRa failed!");
    u8x8.drawString(0, 1, "Starting LoRa failed!");
    while (1);
  }

  u8x8.drawString(0, 0, "init...");
  // Initialize Sensor
  Serial.print("Probe MPU9250: ");
  switch (mpu9250.initialize())
  {
    case 0:
      Serial.println("MPU-Sensor missing");
      u8x8.clear();
      u8x8.drawString(0, 0, "MPU-Sensor");
      u8x8.drawString(0, 1, "missing");
      while(1);
    case 1:
      Serial.println("Found unknown Sensor.");
      u8x8.clear();
      u8x8.drawString(0, 0, "Found unknown");
      u8x8.drawString(0, 1, "sensor");
      break;
    case 2:
      Serial.println("MPU6500 found.");
      u8x8.clear();
      u8x8.drawString(0, 0, "Found MPU-6500");
      break;
    case 3:
      Serial.println("MPU9250 found!");
      u8x8.clear();
      u8x8.drawString(0, 0, "Found MPU-9250");
      break;
  }

  Serial.print("Probe AK8963: ");
  if (i2c.probe(0x0C)) {
    Serial.println("AK8963 found!");
    u8x8.drawString(0, 2, "Found AK8963");
  }
  else {
    Serial.println("AK8963 missing");
    u8x8.drawString(0, 2, "AK8963 missing");
  }

  u8x8.clear();
  u8x8.drawString(0, 0, "Calibrating...");
  calibAccelGyro();
  u8x8.drawString(0, 1, "Complete");
  initDT();
  
  // Mag init
  mag[0].maximum = mag[1].maximum = mag[2].maximum = -32768;
  mag[0].minimum = mag[1].minimum = mag[2].minimum = 32767;
  // set Fuse ROM access mode
  I2CwriteByte(AK8963_ADDRESS, AK8963_RA_CNTL1, 0x0F);
  delay(10);
  // read Sensitivity adjust registers
  I2Cread(AK8963_ADDRESS, AK8963_RA_ASAX, 3, (uint8_t *)&ASA);
  // set Power down mode
  I2CwriteByte(AK8963_ADDRESS, AK8963_RA_CNTL1, 0x00);
  delay(10);
  // Request continuous magnetometer measurements mode 2 in 16 bits
  //  7    6    5    4     3     2     1     0
  //  0    0    0   BIT  MODE3 MODE2 MODE1 MODE0
  //
  I2CwriteByte(AK8963_ADDRESS, AK8963_RA_CNTL1, 0x16);
  delay(10);
  //Timer1.initialize(10000);     // magnetometer measures at the 100Hz cycles
  //Timer1.attachInterrupt(tick); // interrupt at 100Hz

  //finish
  u8x8.clear();
  u8x8.draw2x2String(0, 3, "MPU-9250");

}

void loop() {
  mpu9250.getMeasurement(xyz_GyrAccMag);
  readAccelGyro();
  calcDT();
  calcAccelYPR();
  calcGyroYPR();
  calcFilteredYPR();
  SendDataToProcessing();

  // Read register Status 1 and wait for the DRDY: Data Ready
  uint8_t ST1;
  do {
    I2Cread(AK8963_ADDRESS, AK8963_RA_ST1, 1, &ST1);
  } while (!(ST1 & 0x01));

  do {
    // Read magnetometer data
    uint8_t Mag[7];
    I2Cread(AK8963_ADDRESS, AK8963_RA_HXL, 7, Mag);
    // Create 16 bits values from 8 bits data

    // if ST1 overrun or ST2 overflow, then exit
    if (ST1 & 0x02 || Mag[6] & 0x08)
      continue;

    // Display count
    Serial.print (++count, DEC);
    Serial.print ("\t");

    // Magnetometer
    int16_t mx = -((int16_t)(Mag[1] << 8 | Mag[0]));
    int16_t my = -((int16_t)(Mag[3] << 8 | Mag[2]));
    int16_t mz = ((int16_t)(Mag[5] << 8 | Mag[4]));

    // adjust the sensivitiy with the adjustment value
    float x, y, z;
    x = mx * ((ASA[0] - 128) * .5 / 128. + 1);
    y = my * ((ASA[1] - 128) * .5 / 128. + 1);
    z = mz * ((ASA[2] - 128) * .5 / 128. + 1);

    // adjust the measured magnetometer data
    // set max, min, average of x, y, z
    mag[0].maximum = max(mag[0].maximum, (int16_t)x);
    mag[0].minimum = min(mag[0].minimum, (int16_t)x);
    mag[0].average = (mag[0].maximum + mag[0].minimum) / 2;

    mag[1].maximum = max(mag[1].maximum, (int16_t)y);
    mag[1].minimum = min(mag[1].minimum, (int16_t)y);
    mag[1].average = (mag[1].maximum + mag[1].minimum) / 2;

    mag[2].maximum = max(mag[2].maximum, (int16_t)z);
    mag[2].minimum = min(mag[2].minimum, (int16_t)z);
    mag[2].average = (mag[2].maximum + mag[2].minimum) / 2;

    // store the measured magnetometer data into buffer
    m_data[m_count].x = (int16_t)x - mag[0].average;
    m_data[m_count].y = (int16_t)y - mag[1].average;
    m_data[m_count].z = (int16_t)z - mag[2].average;
 
    m_count = (m_count + 1) & (MAX_COUNT - 1);

    // smoothing the measured magnetometer data
    x = y = z = 0;
    for (int i = 0; i < MAX_COUNT; i++) {
      x += m_data[i].x;
      y += m_data[i].y;
      z += m_data[i].z;
    };

    x /= MAX_COUNT;
    y /= MAX_COUNT;
    z /= MAX_COUNT;

    // Magnetometer
    //Serial.print("Heading:");
    Serial.print(getHeading(x, y), DEC);
    Serial.print("\t");
    Serial.print((int16_t)x, DEC);
    Serial.print("\t");
    Serial.print((int16_t)y, DEC);
    Serial.print("\t");
    Serial.print((int16_t)z, DEC);
    //Serial.print("\t");
  } while (0);

  Serial.print("\r\n");
}

void readAccelGyro() {
  AcX = xyz_GyrAccMag[0];
  AcY = xyz_GyrAccMag[1];
  AcZ = xyz_GyrAccMag[2];
  Tmp = xyz_GyrAccMag[3];
  GyX = xyz_GyrAccMag[4];
  GyY = xyz_GyrAccMag[5];
  GyZ = xyz_GyrAccMag[6];
}

void SendDataToProcessing() {
  /*
  Serial.print(accel_angle_x, 2);  // acc
  Serial.print(",");
  Serial.print(accel_angle_y, 2);
  Serial.print(",");
  Serial.print(accel_angle_z, 2);
  Serial.print(",");
  Serial.print(gyro_angle_x, 2);  // gyr
  Serial.print(",");
  Serial.print(gyro_angle_y, 2);
  Serial.print(",");
  Serial.print(gyro_angle_z, 2);
  Serial.print(",");
  */
  Serial.print(filtered_angle_x, 2);  // fil
  Serial.print("\t");
  Serial.print(filtered_angle_y, 2);
  Serial.print("\t");
  Serial.print(filtered_angle_z, 2);
  Serial.print("\t");
}

void calibAccelGyro() {
  float sumAcX = 0, sumAcY = 0, sumAcZ = 0;
  float sumGyX = 0, sumGyY = 0, sumGyZ = 0;

  readAccelGyro();
  
  for(int i=0; i<10; i++){
    readAccelGyro();
    sumAcX += AcX; sumAcY += AcY; sumAcZ += AcZ;
    sumGyX += GyX; sumGyY += GyY; sumGyZ += GyZ;
    delay(100);
  }
  baseAcX = sumAcX / 10; baseAcY = sumAcY / 10; baseAcZ = sumAcZ / 10;
  baseGyX = sumGyX / 10; baseGyY = sumGyY / 10; baseGyZ = sumGyZ / 10;
}

unsigned long t_now; 
unsigned long t_prev;

void initDT() {
  t_prev = millis();
}

void calcDT() {
  t_now = millis();
  dt = (t_now - t_prev) / 1000.0;
  t_prev = t_now;
}

void calcAccelYPR(){
  float accel_x, accel_y, accel_z;
  float accel_xz, accel_yz;
  const float RADIANS_TO_DEGREES = 180/3.14159;

  accel_x = AcX - baseAcX;
  accel_y = AcY - baseAcY;
  accel_z = AcZ + (16384 - baseAcZ);

  accel_yz = sqrt(pow(accel_y, 2) + pow(accel_z, 2));
  accel_angle_y = atan(-accel_x / accel_yz)*RADIANS_TO_DEGREES;;

  accel_xz = sqrt(pow(accel_x, 2) + pow(accel_z, 2));
  accel_angle_x = atan(accel_y / accel_xz)*RADIANS_TO_DEGREES;

  accel_angle_z = 0;
}

float gyro_x, gyro_y, gyro_z;

void calcGyroYPR(){
  const float GYROXYZ_TO_DEGREES_PER_SED = 131;

  gyro_x = (GyX - baseGyX) / GYROXYZ_TO_DEGREES_PER_SED;
  gyro_y = (GyY - baseGyY) / GYROXYZ_TO_DEGREES_PER_SED;
  gyro_z = (GyZ - baseGyZ) / GYROXYZ_TO_DEGREES_PER_SED;

  gyro_angle_x += gyro_x * dt;
  gyro_angle_y += gyro_y * dt;
  gyro_angle_z += gyro_z * dt;
}

void calcFilteredYPR(){
  const float ALPHA = 0.98;
  float tmp_angle_x, tmp_angle_y, tmp_angle_z;

  tmp_angle_x = filtered_angle_x + gyro_x * dt;
  tmp_angle_y = filtered_angle_y + gyro_y * dt;
  tmp_angle_z = filtered_angle_z + gyro_z * dt;

  filtered_angle_x = ALPHA * tmp_angle_x + (1.0 - ALPHA) * accel_angle_x;
  filtered_angle_y = ALPHA * tmp_angle_y + (1.0 - ALPHA) * accel_angle_y;
  filtered_angle_z = tmp_angle_z;
}


// This function read Nbytes bytes from I2C device at address Address.
// Put read bytes starting at register Register in the Data array.
void I2Cread(uint8_t Address, uint8_t Register, uint8_t Nbytes, uint8_t* Data)
{
  // Set register address
  Wire.beginTransmission(Address);
  Wire.write(Register);
  Wire.endTransmission();

  // Read Nbytes
  Wire.requestFrom(Address, Nbytes);
  uint8_t index = 0;
  while (Wire.available())
    Data[index++] = Wire.read();
}

// Write a byte (Data) in device (Address) at register (Register)
void I2CwriteByte(uint8_t Address, uint8_t Register, uint8_t Data)
{
  // Set register address
  Wire.beginTransmission(Address);
  Wire.write(Register);
  Wire.write(Data);
  Wire.endTransmission();
}

int getHeading(int x, int y) {
  float heading;

  // 대한민국 서울: -8º4' W = -8.067º W = -0.1408 radian
  heading = 180 * (atan2(y, x) - 0.1408) / PI;
  
  if (heading < 0) heading += 360;
  return (int)heading;
}









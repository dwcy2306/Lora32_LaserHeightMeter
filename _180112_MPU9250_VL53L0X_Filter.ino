#include <U8x8lib.h>
#include <LoRa.h>
#include <Wire.h>
#include <SPI.h>
#include <VL53L0X.h>
#include "i2c.h"
#include "i2c_MPU9250.h"
#include <Math.h>

MPU9250 mpu9250;
VL53L0X LaserSen;

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

// the OLED used
U8X8_SSD1306_128X64_NONAME_SW_I2C u8x8(/* clock=*/ 15, /* data=*/ 4, /* reset=*/ 16);

float dt;
float accel_angle_x, accel_angle_y, accel_angle_z;
float gyro_angle_x, gyro_angle_y, gyro_angle_z;
float filtered_angle_x, filtered_angle_y, filtered_angle_z;
float baseAcX, baseAcY, baseAcZ;
float baseGyX, baseGyY, baseGyZ;
int16_t AcX, AcY, AcZ, Tmp, GyX, GyY, GyZ;

static float xyz_GyrAccMag[9];  // 0, 1, 2 = Acc, 3 = Tmp, 4, 5, 6 = Gyr

void setup() {
  SPI.begin(5, 19, 27, 18);
  LoRa.setPins(SS, RST, DI0);

  Serial.begin(115200);
  while (!Serial); //Wait until the computer connected

  u8x8.begin();
  u8x8.setFont(u8x8_font_victoriamedium8_r);
  if (!LoRa.begin(BAND)) {
    Serial.println("Starting LoRa failed!");
    u8x8.drawString(0, 1, "Starting LoRa failed!");
    while (1);
  }

  u8x8.drawString(0, 0, "init...");
  // Initialize Sensor
  Serial.print("Probe MPU9250: ");
  switch (mpu9250.initialize()) {
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

  u8x8.clear();
  u8x8.drawString(0, 0, "Calibrating...");  // GY-91 Calibration
  calibAccelGyro();
  u8x8.drawString(0, 1, "Complete");

  initDT();

  //VL53L0X laser sensor initialization
  u8x8.clear();
  Serial.print("VL53L0X init...");
  Wire.begin();
  u8x8.drawString(0, 0, "VL53L0X init...");
  if(LaserSen.init()){
    Serial.println("success");
    u8x8.drawString(0, 1, "success");
  }
  else {
    Serial.println("fail");
    u8x8.drawString(0, 1, "fail");
    while (1);
  }
  
  LaserSen.setTimeout(500);
  // Long range mode
  LaserSen.setSignalRateLimit(0.1);
  LaserSen.setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, 18);
  LaserSen.setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, 14);

  LaserSen.setMeasurementTimingBudget(33000); //get range value per 330ms
                                              //(The longer the time is, the more accurate result)

  //finish
  u8x8.clear();
  u8x8.draw2x2String(0, 2, "MPU-9250");
  u8x8.draw2x2String(0, 4, "LaserSen");
}

void loop() {
  unsigned int laser_Value;

  mpu9250.getMeasurement(xyz_GyrAccMag);
  readAccelGyro();
  calcDT();  // Calculate time per loop  
  calcAccelYPR();
  calcGyroYPR();
  calcFilteredYPR();
  SendDataToProcessing();  // Send the data of GY-91 via Serial port
  
  laser_Value = LaserSen.readRangeSingleMillimeters();

  Serial.print(laser_Value);
  if (LaserSen.timeoutOccurred()) {
    Serial.print("TIMEOUT");
  }
  Serial.print("\t");

  Serial.print(calcDistance(laser_Value), 2);
  
  Serial.print("\r\n");
}

// This function returns actual distance to ground considering the angle
float calcDistance(unsigned int rawDist) {
  float result;
  
  result = sqrt((1 - pow(sin(filtered_angle_x * M_PI / 180), 2)) * pow(rawDist, 2) * pow(cos(filtered_angle_y * M_PI / 180), 2));
  // Multiply M_PI / 180 in order to make degrees to radian
  return result;
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
  /*
  Serial.print(filtered_angle_z, 2);
  Serial.print("\t");
  */
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

void calcGyroYPR() {  // Calculates the actual angle
  const float GYROXYZ_TO_DEGREES_PER_SED = 131;

  gyro_x = (GyX - baseGyX) / GYROXYZ_TO_DEGREES_PER_SED;
  gyro_y = (GyY - baseGyY) / GYROXYZ_TO_DEGREES_PER_SED;
  gyro_z = (GyZ - baseGyZ) / GYROXYZ_TO_DEGREES_PER_SED;

  gyro_angle_x += gyro_x * dt;
  gyro_angle_y += gyro_y * dt;
  gyro_angle_z += gyro_z * dt;
}

void calcFilteredYPR(){
  const float ALPHA = 0.92;
  float tmp_angle_x, tmp_angle_y, tmp_angle_z;

  tmp_angle_x = filtered_angle_x + gyro_x * dt;
  tmp_angle_y = filtered_angle_y + gyro_y * dt;
  tmp_angle_z = filtered_angle_z + gyro_z * dt;

  filtered_angle_x = ALPHA * tmp_angle_x + (1.0 - ALPHA) * accel_angle_x * 2;
  filtered_angle_y = ALPHA * tmp_angle_y + (1.0 - ALPHA) * accel_angle_y * 2;
  filtered_angle_z = tmp_angle_z;
}



















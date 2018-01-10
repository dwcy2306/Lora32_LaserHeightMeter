#include <U8x8lib.h>
#include <LoRa.h>
#include <Wire.h>
#include "i2c.h"
#include "i2c_MPU9250.h"

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

// the OLED used
U8X8_SSD1306_128X64_NONAME_SW_I2C u8x8(/* clock=*/ 15, /* data=*/ 4, /* reset=*/ 16);

float dt;
float accel_angle_x, accel_angle_y, accel_angle_z;
float gyro_angle_x, gyro_angle_y, gyro_angle_z;
float filtered_angle_x, filtered_angle_y, filtered_angle_z;
float baseAcX, baseAcY, baseAcZ;  //가속도 평균값 저장 변수
float baseGyX, baseGyY, baseGyZ;  //자이로 평균값 저장 변수
int16_t AcX, AcY, AcZ, Tmp, GyX, GyY, GyZ;

static float xyz_GyrAccMag[9];  // 0, 1, 2 = Acc, 3 = Mag, 4, 5, 6 = Gyr

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
  
  delay(100);
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
  Serial.print(filtered_angle_x, 2);  // fil
  Serial.print(",");
  Serial.print(filtered_angle_y, 2);
  Serial.print(",");
  Serial.print(filtered_angle_z, 2);
  Serial.print("\r\n");
}

void calibAccelGyro() {
  float sumAcX = 0, sumAcY = 0, sumAcZ = 0;
  float sumGyX = 0, sumGyY = 0, sumGyZ = 0;

  readAccelGyro();

  //평균값 구하기
  for(int i=0; i<10; i++){
    readAccelGyro();
    sumAcX += AcX; sumAcY += AcY; sumAcZ += AcZ;
    sumGyX += GyX; sumGyY += GyY; sumGyZ += GyZ;
    delay(100);
  }
  baseAcX = sumAcX / 10; baseAcY = sumAcY / 10; baseAcZ = sumAcZ / 10;
  baseGyX = sumGyX / 10; baseGyY = sumGyY / 10; baseGyZ = sumGyZ / 10;
}

unsigned long t_now;  //현재 측정 주기 시간
unsigned long t_prev; //이전 측정 주기 시간

void initDT() {
  t_prev = millis();
}

void calcDT() {
  t_now = millis();
  dt = (t_now - t_prev) / 1000.0; //millis()로 얻은 값은 밀리초 단위
  t_prev = t_now;
}

void calcAccelYPR(){
  float accel_x, accel_y, accel_z; //가속도 센서의 최종적인 보정값
  float accel_xz, accel_yz;
  const float RADIANS_TO_DEGREES = 180/3.14159;

  accel_x = AcX - baseAcX; // 가속도(직선) X축에 대한 현재 값 - 가속도 센서의 평균값
  accel_y = AcY - baseAcY;
  accel_z = AcZ + (16384 - baseAcZ);

  //직석 +X축이 기울어진 각도 구함
  accel_yz = sqrt(pow(accel_y, 2) + pow(accel_z, 2));
  accel_angle_y = atan(-accel_x / accel_yz)*RADIANS_TO_DEGREES;

  accel_xz = sqrt(pow(accel_x, 2) + pow(accel_z, 2));
  accel_angle_x = atan(accel_y / accel_xz)*RADIANS_TO_DEGREES;

  accel_angle_z = 0;
}

float gyro_x, gyro_y, gyro_z; //각속도 저장 전역변수 //각속도 : 단위시간당 회전한 각도

void calcGyroYPR(){
  const float GYROXYZ_TO_DEGREES_PER_SED = 131;
                                  //131 값은 1초동안 1도 돌때 자이로 값이 131이다.
                                  
  gyro_x = (GyX - baseGyX) / GYROXYZ_TO_DEGREES_PER_SED;
  gyro_y = (GyY - baseGyY) / GYROXYZ_TO_DEGREES_PER_SED;
  gyro_z = (GyZ - baseGyZ) / GYROXYZ_TO_DEGREES_PER_SED;

  gyro_angle_x += gyro_x * dt; //변화된 각 : 각속도 x 측정 주기 시간
  gyro_angle_y += gyro_y * dt;
  gyro_angle_z += gyro_z * dt;
}

void calcFilteredYPR(){
  const float ALPHA = 0.96;
  float tmp_angle_x, tmp_angle_y, tmp_angle_z;  //이전 필터 각도(prev)

  tmp_angle_x = filtered_angle_x + gyro_x * dt; //자이로 각도 = 각속도 x 센서 입력 주기
                                                //각속도 = 단위시간당 회전한 각도 -> 회전한 각도 / 단위시간
  tmp_angle_y = filtered_angle_y + gyro_y * dt;
  tmp_angle_z = filtered_angle_z + gyro_z * dt;

  filtered_angle_x = ALPHA * tmp_angle_x + (1.0 - ALPHA) * accel_angle_x;
  filtered_angle_y = ALPHA * tmp_angle_y + (1.0 - ALPHA) * accel_angle_y;
  filtered_angle_z = tmp_angle_z; //곡선 +Z 축은 자이로 센서만 이용해서 나타내고 있음(가속도 센서는 안함)
  //그래서 위에(calcAccelYPR) 보게 되면 accel_angle_z 는 0이다.
}


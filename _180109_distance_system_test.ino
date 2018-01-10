// Doesn't have filter

#include <U8x8lib.h>
#include <LoRa.h>
#include <VL53L0X.h>
#include <Wire.h>
#include "i2c.h"
#include "i2c_MPU9250.h"

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
      u8x8.drawString(0, 1, "MPU-Sensor");
      u8x8.drawString(0, 2, "missing");
      while(1);
    case 1:
      Serial.println("Found unknown Sensor.");
      u8x8.clear();
      u8x8.drawString(0, 1, "Found unknown");
      u8x8.drawString(0, 2, "sensor");
      break;
    case 2:
      Serial.println("MPU6500 found.");
      u8x8.clear();
      u8x8.drawString(0, 1, "Found MPU-6500");
      break;
    case 3:
      Serial.println("MPU9250 found!");
      u8x8.clear();
      u8x8.drawString(0, 1, "Found MPU-9250");
      break;
  }
  
  //VL53L0X laser sensor
  u8x8.clear();
  Serial.print("VL53L0X init...");
  u8x8.drawString(0, 0, "VL53L0X init...");
  if(LaserSen.init()){
    Serial.println("success");
    u8x8.drawString(0, 1, "success");
  }
  else {
    Serial.println("fail");
    u8x8.drawString(0, 1, "fail");
  }
  
  LaserSen.setTimeout(500);
  LaserSen.setMeasurementTimingBudget(200000); //get range value per 200ms

  //finish
  u8x8.clear();
  
  u8x8.drawString(0, 0, "View sensors\'");
  u8x8.drawString(0, 1, "results on serial");
  u8x8.drawString(0, 2, "monitor");
}

void loop() {
  static float xyz_GyrAccMag[9];
  char laser_Value[6];
  
  mpu9250.getMeasurement(xyz_GyrAccMag);

  Serial.print("XYZ ACC g[");
  Serial.print(xyz_GyrAccMag[0],2);
  Serial.print(";");
  Serial.print(xyz_GyrAccMag[1],2);
  Serial.print(";");
  Serial.print(xyz_GyrAccMag[2],2);
  Serial.print("]");

  Serial.print(" \t GYR dps[");
  Serial.print(xyz_GyrAccMag[4],2);
  Serial.print(";");
  Serial.print(xyz_GyrAccMag[5],2);
  Serial.print(";");
  Serial.print(xyz_GyrAccMag[6],2);
  Serial.print("]");

  Serial.print(" \t T: ");
  Serial.print(xyz_GyrAccMag[3],2);
  Serial.print(" C");

  Serial.println("");

  sprintf(laser_Value, "%d", LaserSen.readRangeSingleMillimeters());
  u8x8.clearLine(0);
  u8x8.setFont(u8x8_font_artossans8_r);
  u8x8.drawString(0,0,"Range: ");
  u8x8.drawString(7,0,laser_Value);
  u8x8.drawString(13,0," mm");
  
  Serial.print("Range: ");
  Serial.print(laser_Value);
  Serial.print(" mm");
  if (LaserSen.timeoutOccurred()) { Serial.print(" TIMEOUT"); }
  Serial.println();
   
  delay(20);
}

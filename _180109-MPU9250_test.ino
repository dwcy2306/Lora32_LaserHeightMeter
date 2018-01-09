

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

void setup() {
  SPI.begin(5, 19, 27, 18);
  LoRa.setPins(SS, RST, DI0);

  Serial.begin(115200);
  while (!Serial); //Wait until the computer connected
  delay(1000);

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
  switch (mpu9250.initialize())
  {
    case 0: Serial.println("MPU-Sensor missing"); while(1) {};
    case 1: Serial.println("Found unknown Sensor."); break;
    case 2: Serial.println("MPU6500 found."); break;
    case 3: Serial.println("MPU9250 found!"); break;
  }

  Serial.print("Probe AK8963: ");
  if (i2c.probe(0x0C)) Serial.println("AK8963 found!");
  else                 Serial.println("AK8963 missing");

  //finish
  u8x8.clear();
  u8x8.drawString(4, 0, "MPU-9250");
}

void loop() {
  static float xyz_GyrAccMag[9];

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
    delay(20);
}


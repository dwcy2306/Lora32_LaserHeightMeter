#include <U8x8lib.h>
#include <LoRa.h>
#include <Wire.h>
#include <VL53L0X.h>
#include "I2Cdev.h"
#include "MPU9250.h"

String receivedText;
String receivedRssi;

char sensorVal[6];

VL53L0X sensor;
MPU9250 accelgyro;

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

int16_t ax, ay, az;
int16_t gx, gy, gz;
int16_t mx, my, mz;

char lines[3][16];

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

  // Initialize Sensor
  Wire.begin();
  Serial.println("Initializing I2C devices...");
  u8x8.drawString(0, 0, "init...");
  accelgyro.initialize();

  Serial.println("Testing device connections...");
  Serial.println(accelgyro.testConnection() ? "MPU9250 connection successful" : "MPU9250 connection failed");

  //finish
  u8x8.clear();
  u8x8.drawString(4, 0, "MPU-9250");
}

void loop() {
  accelgyro.getMotion9(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz);
  // display tab-separated accel/gyro x/y/z values
  Serial.print("a/g/m:\t");
  Serial.print(ax); Serial.print("\t");
  Serial.print(ay); Serial.print("\t");
  Serial.print(az); Serial.print("\t");
  Serial.print(gx); Serial.print("\t");
  Serial.print(gy); Serial.print("\t");
  Serial.print(gz); Serial.print("\t");
  Serial.print(mx); Serial.print("\t");
  Serial.print(my); Serial.print("\t");
  Serial.println(mz);
  
  sprintf(lines[0], "  %5d  %5d  ", ax, gx);
  sprintf(lines[1], "  %5d  %5d  ", ay, gy);
  sprintf(lines[2], "  %5d  %5d  ", az, gz);
  
  u8x8.drawString(2, 2, "Accel  Gyro");
  for (int i = 0; i < 3; i++) {
    u8x8.drawString(0, i + 3, lines[i]);
  }
}


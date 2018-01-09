#include <U8x8lib.h>
#include <LoRa.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>

char buf[11];


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

Adafruit_BMP280 bme; // I2C

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
  u8x8.drawString(5, 0, "BMP280");
  u8x8.drawString(0, 2, "Temp:");
  u8x8.drawString(0, 3, "Pres:");
  u8x8.drawString(0, 4, "Alt:");

  if (!bme.begin()) {  
    Serial.println("Could not find a valid BMP280 sensor, check wiring!");
    u8x8.clear();
    u8x8.drawString(0, 0, "ERROR");
    while (1);
  }
}

void loop() {
  sprintf(buf, "%0.2f", bme.readTemperature());
  Serial.print("Temperature = ");
  Serial.print(buf);
  Serial.println(" *C");
  u8x8.drawString(5, 2, buf);

  sprintf(buf, "%f", bme.readPressure());
  Serial.print("Pressure = ");
  Serial.print(buf);
  Serial.println(" Pa");
  u8x8.drawString(5, 3, buf);

  sprintf(buf, "%f", bme.readAltitude(1013.25));
  Serial.print("Approx altitude = ");
  Serial.print(buf); // this should be adjusted to your local forcase
  Serial.println(" m");
  u8x8.drawString(5, 4, buf);
  
  Serial.println();
  delay(2000);
}


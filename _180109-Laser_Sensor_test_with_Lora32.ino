#include <U8x8lib.h>
#include <LoRa.h>
#include <Wire.h>
#include <VL53L0X.h>

String receivedText;
String receivedRssi;

char sensorVal[6];

VL53L0X sensor;

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
  u8x8.drawString(2, 0, "Laser Sensor");
  u8x8.drawString(0, 2, "Value:");

  // Initialize Sensor
  Wire.begin();
  sensor.init();
  sensor.setTimeout(500);
  sensor.setMeasurementTimingBudget(200000);  // High accuracy mode
}

void loop() {
  itoa(sensor.readRangeSingleMillimeters(), sensorVal, 10);  // change sensor value into string
  
  Serial.print(sensorVal);
  u8x8.drawString(6, 2, sensorVal);
  
  if (sensor.timeoutOccurred()) {
    Serial.print(" TIMEOUT");
    u8x8.drawString(0, 3, "TIMEOUT");
  } else {
    u8x8.clearLine(3);  // delete the text "TIMEOUT" on OLED display if timeout not occured
  }

  Serial.println();
}


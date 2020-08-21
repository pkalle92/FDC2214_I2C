/*
 * ESP32 - FDC2214EVM
 * V3.3 - V3.3
 * GND - GND
 * SD - GND
 * 22 - SCL
 * 21 - SDA
 */


#include <Wire.h>
#include "FDC2214.h"

#define I2C_SDA 21
#define I2C_SCL 22

FDC2214 capsense(FDC2214_I2C_ADDR_0); // Use FDC2214_I2C_ADDR_1

byte FDC = 0x2A;// FDC address either 0x2A or 0x2B;
int altzeit;
int delayzeit = 1000;
unsigned long capa; // Variable als Zwischenspeicher
int g = 0;
int i = 0;

void setup() {
  Wire.begin(I2C_SDA, I2C_SCL);
  Serial.begin(115200);
  Configure();
  Wire.beginTransmission(FDC);
}
void loop() {
  // Bereitet die Datenübertragung vor
  capa = capsense.getReading28(g);// Befehl zum Lesen der Kapa.
  if (millis() - altzeit > delayzeit) {
    altzeit = millis();
    if (capa > 0 ) {
      Serial.println(capa);
    }
  }
  delay(50);
}

//Configuring the FDC2214
void writeConfig(int FDC, byte reg, byte MSB, byte LSB) {
  Wire.beginTransmission(FDC);
  Wire.write(reg);
  Wire.write(MSB);
  Wire.write(LSB);
  Wire.endTransmission();
}

void Configure() {
  // based on best results with the TI demo program
  writeConfig(FDC, 0x14, 0x10, 0x01);//CLOCK_DIVIDERS_CH0
  writeConfig(FDC, 0x1E, 0x88, 0x00);//DRIVE_CURRENT_CH0
  writeConfig(FDC, 0x10, 0x04, 0x00);//SETTLECOUNT_CH0
  writeConfig(FDC, 0x08, 0xFF, 0xFF);//RCOUNT_CH0
  writeConfig(FDC, 0x15, 0x10, 0x01);//CLOCK_DIVIDERS_CH1
  writeConfig(FDC, 0x1F, 0x88, 0x00);//DRIVE_CURRENT_CH1
  writeConfig(FDC, 0x11, 0x04, 0x00);//SETTLECOUNT_CH1
  writeConfig(FDC, 0x09, 0xFF, 0xFF);//RCOUNT_CH1
  writeConfig(FDC, 0x16, 0x10, 0x01);//CLOCK_DIVIDERS_CH2
  writeConfig(FDC, 0x20, 0x88, 0x00);//DRIVE_CURRENT_CH2
  writeConfig(FDC, 0x12, 0x04, 0x00);//SETTLECOUNT_CH2
  writeConfig(FDC, 0x0A, 0xFF, 0xFF);//RCOUNT_CH2
  writeConfig(FDC, 0x17, 0x10, 0x01);//CLOCK_DIVIDERS_CH3
  writeConfig(FDC, 0x21, 0x88, 0x00);//DRIVE_CURRENT_CH3
  writeConfig(FDC, 0x13, 0x04, 0x00);//SETTLECOUNT_CH3
  writeConfig(FDC, 0x0B, 0xFF, 0xFF);//RCOUNT_CH3
  writeConfig(FDC, 0x19, 0x00, 0x01);//ERROR_CONFIG
  writeConfig(FDC, 0x1B, 0xC2, 0x0C);//MUX_CONFIG
  writeConfig(FDC, 0x1A, 0x1E, 0x01);//CONFIG
}

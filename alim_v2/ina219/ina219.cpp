
/* INA219 class for amp and volt measure */

#include "Arduino.h"
#include <Wire.h>
#include "ina219.h"

uint8_t slave;
uint8_t inaShunt;

Ina219::Ina219()
{
}

void Ina219::config(uint8_t pslave,uint8_t prshunt)
{
  slave=pslave;
  inaShunt=prshunt;

  write(CALIBREG, 4096);         // init cal reg
  write(CONFREG, DEFCONF);       // init config (moyenne 8 mesures)
}


void Ina219::ampVolt(float* amp, int* volt)
{
  int v;
  v = read(2); *volt = (v >> 3) * 4;
  v = read(4); *amp = (float)v / (100 * inaShunt);
}

void Ina219::write(uint8_t reg, uint16_t data)
{
  Wire.beginTransmission(slave);
  Wire.write(reg);
  Wire.write(data / 256);
  Wire.write(data % 256);
  Wire.endTransmission();
}

uint16_t Ina219::read(uint8_t reg)
{
  Wire.beginTransmission(slave);
  Wire.write(reg);
  Wire.endTransmission();

  Wire.requestFrom(slave, 2);
  uint16_t data = 0;
  while (Wire.available())     // slave may send less (or more) than requested
  {
    data *= 256;
    data += Wire.read();
  }
  return data;
}

void Ina219::show()        // INA219
{
  /*
  for (int i = 0; i < 6; i++) {

    Serial.print("    S"); Serial.print(slave); Serial.print(" R"); Serial.print(i); Serial.print(" : ");
    int v = ina219_read(slave, i);

    for (int j = 12; j > 0; j -= 4) {
      if (v >> j == 0) {
        Serial.print("0");
      } else {
        break;
      }
    }
    Serial.print(v, HEX); Serial.print(" ");

    if (i == 1) {
      Serial.print((float)v / 100);
      Serial.print("mV");
    }
    if (i == 2) {
      v = (v >> 3) * 4;
      if (v < 10) {
        oldInaV = 0;
      }
      Serial.print(v); Serial.print("mV"); Serial.print(" "); Serial.print(v - oldInaV); oldInaV = v;
    }
    if (i == 3) {
      Serial.print(v * 2 / (10 * RSHUNT));
      Serial.print("mW");
    }
    if (i == 4) {
      f = (float)v / (100 * RSHUNT);
      if (f < 100 * RSHUNT) {
        oldInaA = 0;
      }
      Serial.print(f); Serial.print("mA"); Serial.print(" "); Serial.print(f - oldInaA); oldInaA = f;
    }
    Serial.println();
  }
  */
}


/*

MCP4921 class for voltage and current setting

*/

#include "Arduino.h"
#include <Spi.h>
#include "mcp4921.h"

uint8_t ss[2]; // ss[0] tension ss[1] courant

Mcp4921::Mcp4921()
{
}

void Mcp4921::config(uint8_t ss1,uint8_t ss2)
{
    ss[0]=ss1;
    ss[1]=ss2;
}

void Mcp4921::write(uint8_t numunit, uint16_t data)
{
  digitalWrite(ss[numunit], LOW);

  data &= 0x0fff;
  data |= 0x3000;
  SPI.transfer(data / 256); // MSB
  SPI.transfer(data % 256); // LSB

  digitalWrite(ss[numunit], HIGH);
}

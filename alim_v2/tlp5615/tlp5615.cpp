
/*

TLP5615 class for voltage and current setting

*/

#include "Arduino.h"
#include <Spi.h>
#include "tlp5615.h"

uint8_t ss[2]; // ss[0] tension ss[1] courant

Tlp5615::Tlp5615()
{
}

void Tlp5615::config(uint8_t ss1,uint8_t ss2)
{
    ss[1]=ss1;
    ss[2]=ss2;
}

void Tlp5615::write(uint8_t numunit, uint16_t data)
{
  digitalWrite(ss[numunit], LOW);

  data *= 4; // format 0000xxxxxxxxxx00
  SPI.transfer(data / 256); // MSB
  SPI.transfer(data % 256); // LSB

  digitalWrite(ss[numunit], HIGH);
}




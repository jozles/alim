
#ifndef TLP5615_H_INCLUDED
#define TLP5615_H_INCLUDED

#include "tlp5615.h"

class Tlp5615
{
  public:
    Tlp5615();
    void config(uint8_t ss1,uint8_t ss2);
    void write(uint8_t numunit, uint16_t data);
};

#endif // INA219_H_INCLUDED

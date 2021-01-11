
#ifndef MCP4921_H_INCLUDED
#define MCP4921_H_INCLUDED

#include "mcp4921.h"

class Mcp4921
{
  public:
    Mcp4921();
    void config(uint8_t ss1,uint8_t ss2);
    void write(uint8_t numunit, uint16_t data);
};

#endif // MCP4921_INCLUDED

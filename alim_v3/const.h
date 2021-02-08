#ifndef CONST_H_INCLUDED
#define CONST_H_INCLUDED

#define VERSION "3.0_"

//#define INA219
#define CS5550

//#define TLP5615
#define MCP4921

//#define THERMO
//#define DS18X20

#define LED 4
#define PORT_LED PORTD
#define DDR_LED  DDRD
#define BIT_LED  4
#define ONLED  HIGH
#define OFFLED LOW

#define SSADC   8      // cs5550 CS pin
#define PORT_SSADC PORTB
#define DDR_SSADC  DDRB
#define BIT_SSADC  0
#define INTADC  7      // cs5550 INT pin

#define SSDAC2 9       // spi select courant
#define PORT_SSDAC2 PORTB
#define BIT_SSDAC2  1
#define SSDAC1 10      // spi select tension
#define PORT_SSDAC1 PORTB
#define BIT_SSDAC1  2

#define RSHUNT 1       // valeur resistance shunt

enum {FAUX, VRAI};


#endif // CONST_H_INCLUDED

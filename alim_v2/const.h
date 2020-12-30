#ifndef CONST_H_INCLUDED
#define CONST_H_INCLUDED

#define VERSION "2.0_"

//#define INA219
#define CS5550

#define TLP5615

//#define THERMO
//#define DS18X20

#define LED 4
#define ONLED HIGH
#define OFFLED LOW

#define SSADC   8      // cs5550 CS pin
#define INTADC  7      // cs5550 INT pin

#define SSDAC2 9       // spi select courant
#define SSDAC1 10      // spi select tension

#define RSHUNT 1       // valeur resistance shunt

enum {FAUX, VRAI};


#endif // CONST_H_INCLUDED

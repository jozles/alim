#ifndef CS5550_H_INCLUDED
#define CS5550_H_INCLUDED

#include <SPI.h>

/* CS5550 commands */

#define STARTS 0xE0     // single conv
#define STARTC 0xE8     // continuous conv
#define SYNC0  0xFE
#define SYNC1  0xFF
#define PWRUP  0xA0
#define RESET  0x80
#define STDBY  0x88
#define SLEEP  0x90
#define CAL1   0xC8     // AIN1 Normal
#define CAL1G  0xCA     // AIN1 GAIN
#define CAL1O  0xC9     // AIN1 Offset
#define CAL2   0xD0     // AIN2 Normal
#define CAL2G  0xD2     // AIN2 GAIN
#define CAL2O  0xD1     // AIN2 Offset
#define REGRD  0x00
#define REGWR  0x40

/* CS5550 registers */

#define CONFIG    0x00
#define AIN1OFF   0x01
#define AIN1GAIN  0x02
#define AIN2OFF   0x03
#define AIN2GAIN  0x04
#define CYCCOUNT  0x05
#define AIN1OUT   0x07
#define AIN2OUT   0x08
#define AIN1FIL   0x0B
#define AIN2FIL   0x0C
#define STATUS    0x0F
#define MASK      0x1A
#define CONTROL   0x1C

/* valeurs pour cycles */

#define T250MS    0x0003B6
#define T500MS    0x00078B

/* CONFIGURATION BITS */

#define GAIN  0x010000  // AIN1 gain ; 0 gain 10 ; 1 gain 50
#define IMODE 0x001800  // 00 active LOW ; 01 active HIGH ; 10 falling edge ; 11 rising edge
#define HPF1  0x000020  // AIN1 High PassFilter enable if 1
#define HPF2  0x000040  // AIN2 High PassFilter enable if 1
#define ICPU  0x000010  // noise reduction if 1
#define KCLK  0x00000F  // CLK divider (1 to 16 for 0x0 to 0xF) (2 for 8MHz quartz)

/* STATUS bits */

#define DRDY 0x800000   // data ready (end of calibration or end of computation)
#define OR1  0x020000   // ain out of range
#define OR2  0X010000
#define CRDY 0x100000   // conversion ready
#define FOR1 0x004000   // filter out of range
#define FOR2 0x002000
#define OD1  0x000008   // modulator oscillation
#define OD2  0x000010
#define IC   0x000001   // invalid command if 0 

/* CONROL bits */

#define INTOD 0x000010  // put INT pin open drain 
#define NOCPU 0x000004  // disable CPUCLK pin
#define NOOSC 0x000002  // disable crystal oscillator

#define DATALEN 3

#define REGNLEN 8
#define NBREG   13

#define VAL1F 0xFFFFFF  // valeur pour "1" mode filtre
#define VAL1O 0x7FFFFF  // valeur pour "1" mode out

#define ON HIGH
#define OFF LOW

#define CS5550_SS_LOW  bitClear(portCs,bitCs);
#define CS5550_SS_HIGH bitSet(portCs,bitCs);

class Cs5550
{
  public:
    Cs5550();
    void config(uint8_t pportCs,uint8_t pddrCs,uint8_t pbitCs,uint8_t pintPin,uint8_t prshunt);
    void ampVolt(float* amp, int* volt);
    void show();

  private:
    void regWrite(uint8_t reg,uint32_t data);
    void regRead(uint8_t reg, byte* data);
    void command(uint8_t cd);
    void calibrateOffset();
    void calibrateGain();
};

#endif // CS5550_H INCLUDED

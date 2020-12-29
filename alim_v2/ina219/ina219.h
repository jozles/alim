
#ifndef INA219_H_INCLUDED
#define INA219_H_INCLUDED

#define SLAVE 64 // (A0,A1=LOW)
#define CONFREG 0
#define CALIBREG 5
#define DEFCONF 0x3DDF      /* 32V, /8, moyenne sur 8 conv, mode continu */

class Ina219
{
  public:
    Ina219();
    void config(uint8_t pslave,uint8_t prshunt);
    void ampVolt(float* amp, int* volt);
    void write(uint8_t reg, uint16_t data);
    uint16_t read(uint8_t reg);
    void show();
};

#endif // INA219_H_INCLUDED

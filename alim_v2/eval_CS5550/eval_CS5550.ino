 #define VERSION "1.0 "

#include <SPI.h>
#include <Wire.h>

enum {FAUX, VRAI};

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

char regNames[]={"CONFIG  AIN1OFF AIN1GAINAIN2OFF AIN2GAINCYCCOUNTAIN1OUT AIN2OUT AIN1FIL AIN2FIL STATUS  MASK    CONTROL "};
uint8_t regCodes[]={CONFIG,AIN1OFF,AIN1GAIN,AIN2OFF,AIN2GAIN,CYCCOUNT,AIN1OUT,AIN2OUT,AIN1FIL,AIN2FIL,STATUS,MASK,CONTROL};


/* SPI select pins */

#define SS  10      // spi CSi
#define INT 9       // cs5550 INT pin

/* LED */

#define LED 6
#define ONLED HIGH
#define OFFLED LOW

long tmpBlink = millis();         // temps dernier blink
int  perBlink = 100;
uint8_t savled;
#define TBLINKON 5
#define TBLINKOFF 2000-TBLINKON

#define ON HIGH
#define OFF LOW

int      i,j;
byte     data[5];  // buffer registres maxi

byte     csStatus[5];  // buffer status
byte     tfb[5];  // buffer read/write reg
int      len;
uint16_t reg;
char     a=' ';

uint32_t cal1Off=0xFE7000,cal1Gain=0x3FF000; //400000;   // default values for offset and gain calibration 4.5V
uint32_t cal2Off=0x052CA8,cal2Gain=0x3FF000; //400000;   // default values for offset and gain calibration 4.5V
char     buff[16];

bool     conv=false;  // true running 
bool     filt=false;  // true mode filtré
uint32_t val1=VAL1O;  // valeur pour 1 selon filt
uint32_t val2=VAL1O;  // valeur pour 2 selon filt
uint32_t valueNb=0;   // compteur affichage
byte     hexval[DATALEN];
bool     hv=false;    // if true hvv value valid (from hexval)
uint32_t hvv;
bool     disp=false;
bool     contsingle=false;
unsigned long durat0;
unsigned long durat1;

#define MAXREFV 25000000   // max shunt voltage mV
//float    maxref=(float)MAXREFV/(float)0x01000000;
//uint32_t  maxref=596046;  // E-10

uint64_t ain1;
uint64_t ain1f;
uint64_t ain2;
uint64_t ain2f;
uint64_t offset1=0;
uint64_t offset2=0;
byte     filtd1[3];
byte     filtd2[3];
int      sgn=0;
float    voltage;
float    current;
float    cumul;       // pour calcul moyenne
float    mini,maxi;
uint32_t kdly=0x0000FF;   // delay entre lectures (commande 'k')
bool     gain5=false;
uint8_t  shunt=1;    // shunt res


/* prototypes */

void cs5550RegWrite(uint8_t ss, uint8_t reg, uint32_t data);
void cs5550RegRead(uint8_t ss, uint8_t reg, byte* data);
void cs5550Command(uint8_t ss, uint8_t cd);
void cs5550ShowReg();
void calibrateOffset();
void calibrateGain();

bool bitTst(byte* v1,uint32_t v2);
char inch(char* data);
void getData(byte* data);
void hexPrint8(byte b);
void hexPrint24(byte* data);
void hexPrint32(byte* data);
void printFloat(float val);
uint32_t uint32conv(byte* data);

void setup() {

  Serial.begin(115200);delay(1000);
  Serial.println("\n\nready");  

  pinMode(INT,INPUT_PULLUP);
  pinMode(LED+1,OUTPUT);digitalWrite(LED+1,LOW);
  pinMode(LED, OUTPUT); digitalWrite(LED, LOW);

  /* SPI */

    pinMode(SS, OUTPUT); digitalWrite(SS,HIGH);
    SPI.begin();
    SPI.beginTransaction(SPISettings(2000000, MSBFIRST, SPI_MODE0));

  /* CS5550 INITS */

  //cs5550ShowReg();

  cs5550RegWrite(SS,CONFIG,0x000002);     // gain 10 ; clk divide/2
  cs5550RegWrite(SS,CYCCOUNT,T250MS);
  cs5550RegWrite(SS,CONTROL,NOCPU);
  
  cs5550ShowReg();

  /* CALIBRATION */

  a=inch("input g when shunt shortened ; d default ");
  if(a=='g'){calibrateOffset();}
  if(a=='d'){
    cs5550RegWrite(SS, AIN1OFF, cal1Off);
    cs5550RegWrite(SS, AIN2OFF, cal2Off);
    }
  a=inch("input g when max current settled ; d default ");
  if(a=='g'){calibrateGain();}
  if(a=='d'){cs5550RegWrite(SS, AIN1GAIN, cal1Gain);}

  cs5550RegWrite(SS, AIN2GAIN, cal2Gain);
  
  cs5550RegWrite(SS,STATUS,0xffffff);
  cs5550RegWrite(SS,MASK,0x000000);

  cs5550ShowReg();menu();
  
}

void menu()
{
  Serial.println("d display ; k cont single ; c conversion ; s stop ; f filter ; o instant ; r registers ; m clear mini/maxi");Serial.print("H shunt ; C config ; S status ; M mask ; G Gain1 ; F Gain2 ; O Offset1 ; P Offset2 ; Y Cycles ; v value ");
}

uint32_t cal32(byte* data)
{
  //hexPrint24(data);Serial.print(" ");
  return ((uint32_t)data[0]*0x010000+(uint32_t)data[1]*0x0100+(uint32_t)data[2]);

}

void loop() {


  /* LED */

  if (millis() > (tmpBlink + perBlink)) {
    tmpBlink = millis();
    if (digitalRead(LED) == ONLED) {
      digitalWrite(LED, OFFLED);
      perBlink = TBLINKOFF;
    }
    else {
      digitalWrite(LED, ONLED);
      perBlink = TBLINKON;
    } 
  }

  /* pooling INT */

    if(contsingle && digitalRead(INT)==0){
      
      cs5550RegRead(SS,STATUS,csStatus);      
//      cs5550RegRead(SS,AIN1OUT,data1);
//      cs5550RegRead(SS,AIN2OUT,data2);      
      if((csStatus[0]&0x80)==0){
        hexPrint24(csStatus);Serial.println();
        cs5550RegWrite(SS, STATUS,0x7fffff);
      }
      else {
        durat1=micros();

        cs5550RegRead(SS,AIN1FIL,filtd1);
        cs5550RegRead(SS,AIN2FIL,filtd2);
        cs5550RegWrite(SS, STATUS,0xffffff);
        cs5550Command(SS,STARTS);

//        Serial.print(durat1-durat0);Serial.print(" ");
        durat0=durat1;
        valueNb++;
        Serial.print(valueNb);Serial.print(" ");
        
                
        if(valueNb>10){
          
          hexPrint24(filtd1);
          ain1f=(uint64_t)cal32(filtd1);                        // 0->2^24
          Serial.print((float)ain1f);Serial.print(" ");
          ain1f=ain1f*(uint64_t)250000; //MAXREFV;              // *250000 ref uV
          //current=(float)ain1f;
          //printFloat(current);Serial.print(" ");
          ain1f=ain1f>>24;                                      // /2^24  = uV 
          if(ain1f>=offset1){ain1f-=offset1;}else ain1f=0;
          current=(float)(ain1f)/1000;
          printFloat(current);

/*        
        ain2f=cal32(filtd2);
        //current=(float)(ain1f*maxref)/10000000000;  //mV
        if(gain5){current/=5;}
        //voltage=(float)ain2f*maxref/5;  // AIN2 gain 5 constant

          
          current/=shunt;
          cumul+=current;
          if(current<mini && current>0.1){mini=current;}
          if(current>maxi){maxi=current;}
        
          

          hexPrint24(filtd1);printFloat(current);
          Serial.print(" ");
          hexPrint24(filtd2);printFloat(voltage);
//          printFloat(mini);
//          printFloat(maxi);
//          printFloat(cumul/(valueNb-10));
*/
        }        
        Serial.println(); 
      }
    }

/*

    if(conv){
        cs5550RegRead(SS,STATUS,csStatus);    
        if((csStatus[0]&0x10)!=0){
          valueNb++;
          Serial.print(valueNb);Serial.print(" ");hexPrint24(csStatus);Serial.println();
          cs5550RegWrite(SS, STATUS,0xffffff);  
        }
    }
*/    
/*    
    if(digitalRead(INT)==0 || disp){      

          cs5550RegWrite(SS, STATUS, CRDY | DRDY);
          
          if(filt==false){
            cs5550RegRead(SS,AIN1OUT,data); 
            cs5550Command(SS,STARTS);       
          }
          if(filt==true){
            cs5550RegRead(SS,AIN1FIL,data);
            cs5550Command(SS,PWRUP);          // halt
            cs5550Command(SS,STARTC);
          }
                    
          valueNb++;
          Serial.print(valueNb);Serial.print(" ");
          //hexPrint24(data);
          ain1=(uint32_t)data[0]*0x010000+(uint32_t)data[1]*0x0100+(uint32_t)data[2];
          //Serial.print("/");Serial.print(ain1);
          current=MAXREFV*ain1/val1;
          dtostrf(current, 3, 6,buff);buff[8]='0';
          //Serial.print("/ ");
          Serial.println(buff);
          //Serial.println("mA");
    }
*/

  /* commandes Serial */
  
  if (Serial.available()) {
    char a=Serial.read();Serial.print("\n");Serial.println((char)a);
    switch (a) {
      case 'd': if(disp==true){disp=false;}else {disp=true;}break;
      case 'k': cs5550Command(SS,PWRUP);                // halt
                cs5550RegWrite(SS,MASK,DRDY);
                cs5550Command(SS,STARTS);
                valueNb=0;cumul=0;mini=MAXREFV;maxi=0;
                if(contsingle==true){contsingle=false;}else contsingle=true;delay(10);break;
      case 'c': if(conv==true){conv=false;} 
                else {
                  conv=true;
                  valueNb=0;
                  //cs5550Command(SS,PWRUP);                // halt
                  cs5550RegWrite(SS,MASK,DRDY|CRDY);
                  cs5550Command(SS,STARTC);
                  cs5550RegWrite(SS, STATUS, 0xffffff);
                }
                break;

      case 's': conv=false;                             // stop conversions                     
                cs5550Command(SS,PWRUP);                // halt
                cs5550RegWrite(SS, STATUS, CRDY | DRDY);
                break;       

      case 'f': filt=true;val1=VAL1F;break;   // filter
      
      case 'o': filt=false;val1=VAL1O;break;  // out

      case 'r': cs5550ShowReg();break;

      case 'm': valueNb=0;cumul=0;mini=MAXREFV;maxi=0;break;

      case 'S': if(hv){cs5550RegWrite(SS, STATUS,hvv);cs5550ShowReg();menu();hv=false;}break;

      case 'C': if(hv){cs5550RegWrite(SS, CONFIG,hvv);hv=false;
                  if(((hvv>>16)&0x000000FF)!=0){gain5=true;}
                  else gain5=false;
                  cs5550ShowReg();menu();
                }break;

      case 'M': if(hv){cs5550RegWrite(SS, MASK,hvv);cs5550ShowReg();menu();hv=false;}break;

      case 'Y': if(hv){cs5550RegWrite(SS, CYCCOUNT,hvv);cs5550ShowReg();menu();hv=false;}break;

      case 'H': if(hv){shunt=(uint8_t)hvv;cs5550ShowReg();menu();hv=false;}break;

      case 'G': if(hv){cs5550RegWrite(SS, AIN1GAIN,hvv);cs5550ShowReg();menu();hv=false;}break;

      case 'F': if(hv){cs5550RegWrite(SS, AIN2GAIN,hvv);cs5550ShowReg();menu();hv=false;}break;

      case 'O': if(hv){cs5550RegWrite(SS, AIN1OFF,hvv);cs5550ShowReg();menu();hv=false;}break;

      case 'P': if(hv){cs5550RegWrite(SS, AIN2OFF,hvv);cs5550ShowReg();menu();hv=false;}break;
      
      case 'v': getData(hexval);hexPrint24(hexval);hvv=uint32conv(hexval);hv=true;
                Serial.print(" ");
                Serial.println();break;
      
      case 'p': if(hv){kdly=hvv;}hv=false;break;
      
      default: break;
    }
  }

}
/* --------------- fin loop ---------------------- */


void cs5550RegWrite(uint8_t ss, uint8_t reg,uint32_t data)  
{ 
  tfb[0]=reg*2 | REGWR;
  
  tfb[1]=(data>>16)&0x000000FF;  
  tfb[2]=(data>>8)&0x000000FF;   
  tfb[3]=data&0x000000FF;        

//  hexPrint32(tfb);Serial.println();

  digitalWrite(ss, LOW);
  SPI.transfer(tfb,DATALEN+1);
  digitalWrite(ss, HIGH);
}

void cs5550RegRead(uint8_t ss, uint8_t reg, byte* data)
{
  reg *= 2;  
  reg |= REGRD ;
  tfb[0]=reg;
  memset(tfb+1,SYNC0,DATALEN);

  digitalWrite(ss, LOW);
  SPI.transfer(tfb,DATALEN+1);
  digitalWrite(ss, HIGH);

  memcpy(data,tfb+1,DATALEN);
}

void cs5550Command(uint8_t ss, uint8_t cd)  
{
  digitalWrite(ss, LOW);

  SPI.transfer(cd);  

  digitalWrite(ss, HIGH);
}


void cs5550ShowReg()
{
  Serial.print("disp/conv/filt/gain5;shunt;kdly ");Serial.print(disp);Serial.print(conv);Serial.print(filt);Serial.print(gain5);Serial.print(";");Serial.print(shunt);Serial.print(";");Serial.println(kdly);
  for(int r=0;r<NBREG;r++){
    Serial.print(r);Serial.print(" ");
    if(r<10){Serial.print(' ');}
    hexPrint8(regCodes[r]);Serial.print(" ");
    for(int l=0;l<REGNLEN;l++){Serial.print(regNames[r*REGNLEN+l]);}Serial.print(" ");
    cs5550RegRead(SS,regCodes[r],data);
    hexPrint24(data);
    Serial.println();
  }
  Serial.println();
}

void calibrateOffset()
{
  Serial.print("Offset calibration running... ");
  
  cs5550RegWrite(SS,STATUS,0xffffff);
  cs5550Command(SS,CAL1O);
  /*while(bitTst(csStatus,DRDY)==0){
    cs5550RegRead(SS, STATUS,csStatus);
  }*/
  delay(2000);
  cs5550Command(SS,CAL1);

  cs5550Command(SS,CAL2O);
  delay(2000);
  cs5550Command(SS,CAL2);

  cs5550RegWrite(SS,STATUS,0xffffff);

        cs5550RegWrite(SS, STATUS,0xffffff);
        cs5550Command(SS,STARTS);
        delay(1200);
        cs5550RegRead(SS,AIN1FIL,filtd1);
        ain1f=(uint64_t)cal32(filtd1);                        // 0->2^24
        ain1f=ain1f*(uint64_t)250000; //MAXREFV;              // *250000 ref uV
        ain1f=ain1f>>24;
        offset1=ain1f;
        
  Serial.print((float)offset1/1000);Serial.println(" offset calibration complete ");
}

void calibrateGain()
{
  Serial.print("Gain calibration running...");
  byte csStatus[DATALEN+1];memset(csStatus,0x00,DATALEN);
  
  cs5550Command(SS,CAL1G);
  while(bitTst(csStatus,DRDY)==0){
    cs5550RegRead(SS, STATUS,csStatus);
  }
  cs5550Command(SS,CAL1);
  Serial.println(" gain calibration complete");
}

bool bitTst(byte* v1,uint32_t v2)
{
//Serial.print(" ");Serial.print(v2,HEX); 
 
  byte b2t[DATALEN];
  b2t[0]=(v2>>16)&0x000000FF;
  b2t[1]=(v2>>8)&0x000000FF;
  b2t[2]=v2&0x000000FF;

//Serial.print(" ; v1=");hexPrint24(v1);Serial.print(" b2t=");hexPrint24(b2t);
  
  for(uint8_t i=0;i<DATALEN;i++){
    for(uint8_t j=0;j<8;j++){
      if( ((v1[i]>>j)&0x01 !=0) && ((b2t[i]>>j)&0x01 !=0) ) {return 1;}
    }
  }
  return 0;
}

/*bool bitTst(byte* v1,uint32_t v2)
{
  Serial.print("v1=");hexPrint24(v1);Serial.print(" v2=");hexPrint24(v2);
  if(bitTst0(v1,v2)){Serial.print(" =1 ; ");}
  else Serial.print(" =0");
}
*/

char inch(char* data)
{
  Serial.print(data);
  char a=' ';
  while(a==' '){
    if (Serial.available()) {a=Serial.read();}
  }
  Serial.print(a);Serial.println();
  return a;
}

uint32_t uint32conv(byte* data)
{
  uint32_t val=(uint32_t)data[0]*0x010000+(uint32_t)data[1]*0x0100+(uint32_t)data[2];  
  return val;
}

byte cvhex(byte c)
{
  c-='0';if(c>9){c&=0x07;c+=9;}
  return c;
}

void getData(byte* data)
{
  Serial.print("getData");Serial.print(Serial.available());Serial.print(" ");
  char a;
  uint8_t c=0,d=0;
  byte dd[16];
  while(Serial.available()){
    a=Serial.read();
    dd[c]=a;
    Serial.print(c);Serial.print(":");Serial.print(a);Serial.print(" ");
    c++;if(c>6){break;}
  }
  Serial.print(" ");
  hexPrint24(dd);hexPrint24(dd+3);Serial.print(" ");Serial.print(c);Serial.print(" ");
  for(d=0;d<c;d+=2){
    data[d/2]=cvhex(dd[d])*16+cvhex(dd[d+1]);
  }
}

void hexPrint8(byte b)
{
  if((b&0xF0)==0){Serial.print("0");}
  Serial.print(b,HEX);
}

void hexPrint32(byte* data)
{
  hexPrint24(data);
  hexPrint8(data[3]);
}

void hexPrint24(byte* data)
{
  hexPrint8(data[0]);
  hexPrint8(data[1]);
  hexPrint8(data[2]);
  Serial.print(" ");
}

void printFloat(float val)
{
      memset(buff,'0',9);
      dtostrf(val, 3, 4,buff);
      Serial.print(buff);
      Serial.print(" ");
}

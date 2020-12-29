
/*   
    Interrupt-free, small memory wasting ds1820 interface (about 1200 bytes)
    this is working with any pin who allow digitalRead and digitalWrite.
    Interrupt suspend should not be longer than 85 uSec for each written bit and
    less than 10uSec for the other occurencys.

    2 steps : first converting (could be as long as 750 mSec) then get value.

    convertDs(uint8_t pin) returns 1 or error codes (see below)
    readDs(uint8_t pin) returns float value between -55 and +125 or error codes (see below)
    
    getDs(uint8_t pin,uint8_t* frameout,uint8_t nbbyteout,uint8_t* framein,uint8_t nbbytein)
    return a status (codes below) and fill framein with data provided by Ds in accordance
    with the frameout data transmitted to Ds.
    (see datasheet)
    
    pin is the Arduino pin number used to connect Ds unit
*/

#include "Arduino.h"
#include "ds1820.h"

// possible return status for getDs() function
// values between -55 to +125 are valid data

#define TOPRES -100
#define CRC_ERR -101 

#define T_WAIT_PRESENCE 240
#define FAUX 0
#define VRAI 1
bool bitmessage=FAUX; // VRAI Serial.print les bits du meesage

Ds1820::Ds1820()  // constructeur de la classe 
{
}

byte calcBitCrc(byte shiftReg, byte data_bit);
static uint8_t maskbit[]={1,2,4,8,16,32,64,128};  //  128,64,32,16,8,4,2,1


void writeDs(uint8_t pin,uint8_t nbbyteout,uint8_t* frameout)
//write command nbbyteout @ nbbit/byte
{  
  boolean pinstat=HIGH;
  uint8_t i=0,j=0,delaylow,delayhigh;
    
  digitalWrite(pin,HIGH);
  pinMode(pin,OUTPUT);
  for(i=0;i<nbbyteout;i++){//Serial.print(frameout[i],HEX);
    for(j=0;j<8;j++){
      delaylow=80;delayhigh=0;pinstat=LOW;if((frameout[i] & maskbit[j])!=0){pinstat=HIGH;delaylow=0;delayhigh=80;}
      noInterrupts();
      digitalWrite(pin,LOW);
      // if bit =1 low state about 2,7 uSec long (UNO)
      delayMicroseconds(delaylow);digitalWrite(pin,pinstat);delayMicroseconds(delayhigh);
      digitalWrite(pin,HIGH);
      interrupts();
    }
  }
  pinMode(pin,INPUT);
  //Serial.println();
}

int Ds1820::convertDs(uint8_t pin)
{
  uint8_t h[2]={0xCC,0x44},r[2];
  return getDs(pin,h,2,r,0,VRAI); 
}

float Ds1820::readDs(uint8_t pin)
{
  uint8_t cmd[2]={0x33,0xBE},rep[9];
  uint8_t r0;
  int v;
  float th=0,th0=0;
  uint8_t ds18x=0;
  char cm;

  cm=cmd[0];
  v=getDs(pin,&cm,1,rep,8,VRAI);
  if(v<TOPRES){return v;}
  else if(cmd[0]!=0xCC){ // read Rom cmd (0x33) 0x28=18B20 0x10=18S20 ou 1820
    ds18x=rep[0];//Serial.print("DS18x20 ");Serial.println(ds18x,HEX);
    }

  cm=cmd[1];
  v=getDs(pin,&cm,1,rep,9,FAUX);
  //Serial.print((int)rep[0]/2);Serial.print(" ");Serial.print(v);Serial.print(" ");Serial.print((int)rep[7]);Serial.print(" ");Serial.print((int)rep[6]);Serial.print(" ");Serial.println((float)(rep[7]-rep[6])/rep[7]);
  if(v<TOPRES){return v;}
  else{
//   complément à 2 : si rep[1] != 0 c'est négatif : th = -(complt rep[0] + 1)
//   le bit 0 vaut 0,5° le traiter à part
/*
rep[1]=0xFF;rep[0]=0xF6; 
Serial.println("test conversion ");
for (int w=0;w<40;w++){rep[0]++;if(rep[0]==0){rep[1]=0;}
Serial.print(rep[1],HEX);Serial.print(rep[0],HEX);Serial.print(" ");
*/
    rep[0]&=0x7F; // pour DS18S20 buggé !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    r0=rep[0]>>1;
    //Serial.print(r0,HEX);Serial.print(" ");
    th0=(uint8_t)r0;
/*    traiter ici le 18B20 (ds18x=0x28) */
/*    if(rep[1]!=0){       // pour alim pas de valeurs négatives (assume le 18S20 buggé)
      r0=r0|0x80; // r0 valeur entière complt à 2 de la température ; si bit 0 de rep[0] =1 ajouter 0,5° au résult float
      th0=-((uint8_t)(r0^0xFF)+1);
      }
*/      
    th=(float)(th0)+0.5*((uint8_t)(rep[0]&0x01));       // th précision 0.5°
  //Serial.print(th);Serial.print("  ");
  //th=(float)(th0)-0.25+(float)(rep[7]-rep[6])/rep[7]; // th précision améliorée
  //Serial.println(th);
//}  
  //th0=th*10-(int)(th*10);if(th0>0.5){th+=0.1;}th=(int)(th*10);th=(float)(th/10);
  //Serial.print(" ");Serial.print(th0);Serial.print(" ");Serial.println(th);
  return th;
  }
}

int Ds1820::getDs(uint8_t pin,uint8_t* frameout,uint8_t nbbyteout,uint8_t* framein,uint8_t nbbytein,bool start)
{

  uint8_t crc=0;
  uint8_t i=0,j=0,v=0;
  long micr=0;
  uint8_t delaylow=0,delayhigh=0;
  boolean pinstat=HIGH;
  uint8_t buff[128];for(i=0;i<128;i++){buff[i]=0;}
  
  //reset pulse
  if(start){
    noInterrupts();
    digitalWrite(pin,HIGH);
    pinMode(pin,OUTPUT);
    interrupts();
    delay(1);
    digitalWrite(pin,LOW);
    delayMicroseconds(500);
    pinMode(pin,INPUT);
    delayMicroseconds(65);
  
    //presence detect
    micr=micros();
    while(1){
      if(micros()>(micr+T_WAIT_PRESENCE)){return TOPRES;} // pas de réponse
      if(digitalRead(pin)==LOW){delayMicroseconds(400);break;}
    }
  } // start
  // write command
  writeDs(pin,nbbyteout,frameout);
  
  //read response nbbytein @ 8 bits/byte
  
  if(nbbytein!=0){ 
    memset(framein,0,nbbytein);
    for (i=0;i<nbbytein;i++){
      for (j=0;j<8;j++){
        v=i*8+j;
        noInterrupts();
        digitalWrite(pin,LOW);
        pinMode(pin,OUTPUT);
        // delay between low and read about 5,2 uSec (UNO) + delayMicroseconds
        // delayMicroseconds is n-1 micros ; pinMode about 3,1 uSec
        pinMode(pin,INPUT_PULLUP);delayMicroseconds(5);buff[v]=digitalRead(pin);
        interrupts();
        if(buff[v]!=0){framein[i]+= maskbit[j];}
        delayMicroseconds(50);//Serial.print(buff[v]); /* serial.dump.bits */
      }//Serial.print(" ");                            /* serial.dump.bits */
    }//Serial.println();                               /* serial.dump.bits */
  }
  // check CRC
  byte shift_reg=0;
  for(i=0;i<nbbytein;i++){
    for(j=0;j<8;j++){shift_reg=calcBitCrc(shift_reg,buff[i*8+j]);
    if(bitmessage){Serial.print(buff[i*8+j]);}
      }
    if(bitmessage){Serial.print(" ");}
    } 
    if(bitmessage){Serial.print(" CRC ");Serial.println(shift_reg);}
  if(shift_reg==0){return 1;}else{return CRC_ERR;} 
}

byte Ds1820::calcBitCrc (byte shiftReg, byte data_bit)
{
  byte fb;
  
  fb = (shiftReg & 0x01) ^ data_bit;
   /* exclusive or least sig bit of current shift reg with the data bit */
   shiftReg = shiftReg >> 1;                  /* shift one place to the right */
   if (fb==1){shiftReg = shiftReg ^ 0x8C;}    /* CRC ^ binary 1000 1100 */
   return(shiftReg); 
}


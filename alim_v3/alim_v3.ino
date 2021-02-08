
#include <SPI.h>
#include <TFT.h>

#include "const.h"

/* ----------------- thermomètre ------------------- */

#ifdef DS18X20

#include <ds18x20.h>
Ds1820 thermo;

#endif // DS18X20

/* ---------------------- ADC ----------------------- */

#ifdef INA219
#include "ina219.h"
#include <Wire.h>
int oldInaV = 0;
float oldInaA = 0, f = 0;
Ina219 measure;
#endif // INA219

#ifdef CS5550
#include "cs5550.h"
Cs5550 measure;
#endif
/* ---------------------- DAC's ---------------------- */

#ifdef TLP5615
#include "tlp5615.h"
Tlp5615 setVoltsAmps;
#define MAXDAC 1023
#endif
#ifdef MCP4921
#include "mcp4921.h"
Mcp4921 setVoltsAmps;
#define MAXDAC 4095
#define BINV 0.002    // mV/point
#define BINA 5/4.096/24
#endif

/* ----------------------- TFT ----------------------- */

/* afficheur SPI TFT 128*160 */

#define TFTH 128
#define TFTW 160
#define TFTCH 10      // points hauteur caractères
#define TFTCW 6       // points largeur caractères

#define SSTFT   6
#define RSTTFT -1 
#define DCTFT   5
#define BLTFT  A2

TFT tft = TFT(SSTFT, DCTFT, RSTTFT);

/* ------------- Encodeur rotatif -------------------- */
/*  Utilise 2 entrées d'interruption donc pour UNO digital 2 et 3  */

#define COD_INT1 2
//#define PORT_COD_INT1 PORTD                    // bitRead() ne lit pas les ports ; utilisez digitalRead
//#define BIT_COD_INT1 2
#define COD_INT2 3
//#define PORT_COD_INT2 PORTD
//#define BIT_COD_INT2 3

int codeur = 0;     // la variable mouvementée lors des interruptions
                    // codeur est préchargée à la valeur courante de la valeur lorsqu'une saisie est initiée ;
                    // à tout moment codeur peut être lu pour valoriser la valeur
                   
int oldValeur;      // lorsque la saisie d'une valeur est initiée avec startsaisie(), la valeur 
                    // est archivée pour permettre sa restauration si la saisie n'est pas validée.
                  
byte etat = 0, test = 0;
long timeIsrCod = 0; /* timeIsrcod millis() de la précédente Interruption     */
long t;              /* local de l'Isr pour calcul durée entre interruptions  */
uint16_t cntIsr=0;   /* cnt isr pour debug */

#define TINCR4 125
#define TINCR3 50
#define TINCR2 20

#define INCR4 5
#define INCR3 50
#define INCR2 500
#define INCR1 1

/* --------------- poussoir (actif haut) ---------------- */

#define POUSSOIR LED  // même pin que la led
#define PORT_POUSSOIR PORT_LED
#define DDR_POUSSOIR DDR_LED                             // bitRead() ne lit pas les ports ; utiliser digitalRead
#define BIT_POUSSOIR BIT_LED
#define TPOUSS  25
long tmpPouss = millis();
int  perPouss = TPOUSS;

/* -------------------  température --------------------- */

#define PINTEMP 17 /* (A3) */
#define TTEMP 2000  
long tmpTemp = millis();          // temps dernière température
int  perTemp = TTEMP;
float temp=0,oldtemp=0;

/* ------------------------ LED -------------------------- */

long tmpBlink = millis();         // temps dernier blink
int  perBlink = 0;
uint8_t savled;
#define TBLINKON 10
#define TBLINKOFF 1990

/* valeurs (volts, amps, periode rampe, step rampe, voltmin rampe, voltmax rampe */

#ifdef  MCP4921
#define NBRAMP 64     // pointeur maxi rampe
uint16_t volt[NBRAMP+1] = {   0,   64,  128,  192,  256,  320,  384,  448,  512,  576,  640,  704,  768,  832,  896,
                          960, 1024, 1088, 1152, 1216, 1280, 1344, 1408, 1472, 1536, 1600, 1664, 1728, 1792, 1856,
                         1920, 1984, 2048, 2112, 2176, 2240, 2304, 2368, 2432, 2496, 2560, 2624, 2688, 2752, 2816,
                         2880, 2944, 3008, 3072, 3136, 3200, 3264, 3328, 3392, 3456, 3520, 3584, 3648, 3712, 3776,
                         3840, 3904, 3968, 4032, 4095};
#endif
#ifndef MCP4921
#define NBRAMP 16     // pointeur rampe
uint16_t volt[NBRAMP+1] = {0, 64, 128, 192, 256, 320, 384, 448, 512, 576, 640, 704, 768, 832, 896, 960, 1023};
#endif                         

#define MAXPERRAMP  10000
#define MAXSTEPRAMP NBRAMP

int ptrdac[2] = {0, NBRAMP};       // ptrdac[0] n° du step courant de rampe tension ; ptrdac[1] courant

#define NBVALEURS 6         // 0=dac volts ; 1=dac amps ; 2=perRamp ; 3=nbStepRamp ; 4=voltsMinRamp ; 5=voltsMaxRamp
#define NVALEURVOLTS  0     // n° valeur volts
#define NVALEURAMPS   1     // n° valeur amps
#define NVALEURPERRAMP 2    // n° valeur période rampe
#define FASTPERRAMP 500     // mS mini pour fastTft FAUX (sinon les affichages ralentissent la rampe)
int     valeurs[NBVALEURS]={0,MAXDAC,5000,NBRAMP,0,9};                                  // valeurs initiales
// 0=volts ; 1=amp ; 2=période rampe ; 3=nbre steps ; 4 vmin ; 5 vmax
int   backValeurs[NBVALEURS];                                                           // save valeurs avant validation,
bool  statusValeurs[NBVALEURS]={FAUX,FAUX,FAUX,FAUX,FAUX,FAUX};                         // si VRAI saisie en cours
int   minValeurs[NBVALEURS]={0,0,10,2,0,0};                                             // valeur mini pour saisie
int   maxValeurs[NBVALEURS]={MAXDAC,MAXDAC,MAXPERRAMP,MAXSTEPRAMP,9,9};// valeur maxi pour saisie
#define LENUNITVALEUR 3
char* unitValeurs="mV\0mA\0mS\0  \0mV\0mV\0";                                           // unités des valeurs

int p0,p1;

/* analog inputs */

#define NBDACS  2
#define DACVOLT 0   // DAC VOLTS
#define DACAMPS 1   // DAC AMPS

//uint8_t dac[NBDACS] = {UOUT, DACI};                  // numunit pour mcp4921

/* ---------------------- menus ------------------------------ 
 *  mécanisme 
 *  
 *  dimensions                                                                                                   */
 
#define LENENTVMENU 8  // longueur texte entrées menu vertical
#define LENENTHMENU 8  // longueur texte entrées menu horizontal
#define MAXLEVMENU  3  // nombre de niveaux maxi
#define MAXENTMENU  5  // nombre d'entrée maxi d'un niveau

/* navigation                                                                                                    */

int     levelMenu=-1;                        // niveau du menu courant (0 à n) ; pointe dans les piles
int     choixMenu[MAXLEVMENU]={0,0,0};       // pile des choix (n° de l'entrée choisie dans chaque menu)
int     numerMenu[MAXLEVMENU]={0,0,0};       // pile des menus choisis 

/* chaque niveau de menu est décrit par les tables générales du système de menu
 * et les tables des entrées du niveau (accédées par les tables de pointeurs)
 * 
 * tables générales
 * 
 * table du nombre d'entrées pour chaque menu                                                                          */
int     nbentMenu[]={4,2,5};
/* table du modèle de menu (0 vertical, 1 horizontal)                                                                  */
int     modelMenu[]={0,1,0};
/* table de la position horizontale pour chaque menu                                                                   */
int     hposMenu[] ={7,10,65+7};   // texte
/* table de la position verticale pour chaque menu                                                                     */
int     vposMenu[] ={75,110,75};      
/* table de la position verticale du curseur pour chaque menu                                                          */
int     hpcMenu[]  ={0,0,65};    

/* tables des entrées de chaque menu

/* menu 0 */
char*  lev0Menu  = "tension courant rampe   st/stop \0";
int    typ0Menu[]= {7,7,1,8};
int    par0Menu[]= {0,1,2,0};
bool   cmp0Menu[]= {VRAI,VRAI,FAUX,FAUX};
int    foloMenu[]= {1,1,0,0};
/* menu 1 */
char*  validMenu = "Valider Abandon \0";
int    typ1Menu[]= {5,6};
int    par1Menu[]= {0,0};
bool   cmp1Menu[]= {FAUX,FAUX};
int    fol1Menu[]= {0,0};
/* menu 2 */
char*  rampeMenu = "periode nb stepsvoltmin voltmax retour\0";
int    typ2Menu[]= {7,7,7,7,3};
int    par2Menu[]= {2,3,4,5,1};
bool   cmp2Menu[]= {VRAI,VRAI,VRAI,VRAI,FAUX};
int    fol2Menu[]= {0,0,0,0,0};

/* tables de pointeurs sur les tabmles d'entrées                                                                         */

/* 1 table de pointeurs sur l'état "sélectionnable" des entrées du menu                                                  */
bool*   compMenu[] ={cmp0Menu,cmp1Menu,cmp2Menu}; 
/* 1 table de pointeurs sur les libellés des entrées de menus :                                                          */
 char*  entryMenu[]={lev0Menu,validMenu,rampeMenu};
/* 1 table des types (un type/entrée qui détermine ce qui est à faire lorsque l'entrée est sélectionnée)                 */
 int*   typeMenu[] ={typ0Menu,typ1Menu,typ2Menu};               
/*                     0 rien, 
 *                     1 accès niveau menu suivant, 
 *                     2 retour niveau précédent sans effet
 *                     3 retour via menu validation
 *                     4 retour niveau précédent 
 *                     5 retour 2 niveaux (menu validation validé)
 *                     6 retour 2 niveaux sans effet
 *                     7 saisie numérique sur ligne menu (par ex volts, ampères, nbre steps etc)
 *                     8 start/stop rampe
 * 1 table des paramètres :                                                                                                      
 * si type 7 le n° du param à valoriser ; si type 1 le n° du menu suivant ; si type 3 le n° du menu auquel revenir          */
int*    paramMenu[]={par0Menu,par1Menu,par2Menu};


#define LENCOMP  6     // longueur chaine complémentaire
char    comptMenu[MAXENTMENU*LENCOMP+1]; // 4*6
uint8_t statusMenu = 0;
  /* statusMenu!=0 -> menu en cours, scrutation codeur
     de 1 à 3 Menu 1 vertical
     de 4 à 8 Menu 2 horizontal
      "choix1 ou 2" variable de l'option courante   
      "prechoix1 ou 2" choix précédent (pour effacer le curseur)
  */
int choix1 = 0, prechoix1 = 0;  /* position dans le menu/pos précédente pour effacer le curseur */

int m0,m1;      

/* affichage */

#define VPOSVA 35     // pos vertivale pour Volts et Amps
#define TAFF 200
uint8_t rfrAff=0;
long tmpAff   = millis();
int  perAff   = TAFF;
bool unefois  = FAUX;
int vcollector = 0, vcollectorM = 0;
float iout = 0, ioutM = 0, vout = 0, voutM = 0;
#define LTEXT0 32
char text0[LTEXT0], text1[12];

/* divers  */

uint8_t bid = 0;

#define ON HIGH
#define OFF LOW

float a, a0, b, b0, c, c0;
bool fastTft  = FAUX;       // si VRAI, rampe rapide en cours, pas de maj de l'affichage 
long tmpRamp=millis();      // temps dernier step rampe 
bool arretRampe = VRAI;     // si VRAI pas de rampe en cours 
bool show     = FAUX;
bool testDaci = FAUX;
bool poussoir = OFF;
bool tempChg  = VRAI;       // indique que la temp a changé -> maj de l'affichage 
bool perRampChg  = VRAI;    // indique que la période de la rampe a changé -> maj de l'affichage 
bool stepRampChg = VRAI;    // indique que le nbre de step de la rampe a changé -> maj de l'affichage 
bool menuChg  = VRAI;       // indique que le menu a changé -> maj de l'affichage 

/* prototypes */

void serialShow(uint8_t numunit, int consigne);
void isrCod();
void showTft();
void saisieValCodeur(int* valcodeur, int numspi);
void printfix(int value);
void printAtTft(uint8_t posx,uint8_t posy,char* text);
void menu10(char* complt);                                              // menu vertical
void menu1(uint8_t numitem, char* arrow,char* complt);                  // 1 ligne
void menu20();                                                          // menu horizontal
void menu2(uint8_t numitem, char* arrow);                               // 1 item
void effaceMenu2();
void choixCodeur(int* choix, int* prechoix);      // saisie du codeur pour le choix dans les menus
void saisiePerRamp();
void startSaisie(bool fast,int numunit,int menu);
char* paramItem(int value,char* unit);

void setup() {

  Serial.begin(115200);
  Serial.print("alim "); Serial.print(VERSION);Serial.print(" led "); Serial.print(LED);

  pinMode(LED, OUTPUT); digitalWrite(LED, ONLED);

  /* SPI */
  pinMode(SSDAC1, OUTPUT);
  digitalWrite(SSDAC1, HIGH);
  pinMode(SSDAC2, OUTPUT);
  digitalWrite(SSDAC2, HIGH);

  SPI.begin();
  SPI.beginTransaction(SPISettings(8000000, MSBFIRST, SPI_MODE0));


#ifdef INA219
  measure.config(SLAVE,RSHUNT);
  Wire.begin();
  delay(9);                                    // 8 mesures 4,26mS
#endif // INA219

#ifdef CS5550
  //measure.config(PORT_SSADC,DDR_SSADC,BIT_SSADC,INTADC,RSHUNT);
#endif // CS5550

  /* TFT */

//pinMode(RSTTFT,OUTPUT);digitalWrite(RSTTFT,HIGH);delay(1000);digitalWrite(RSTTFT,LOW);  
pinMode(BLTFT,OUTPUT);digitalWrite(BLTFT,HIGH);

  tft.begin();
  tft.background(0, 0, 0);
  
  strcpy(text0,"alim \0"); strncat(text0, VERSION, 3);
  tft.setRotation(3);
  tft.setTextColor(0xFFFF, 0x0000);
  tft.setTextSize(1);  
  tft.setCursor(10, 10);
  tft.print(text0);

  /* DACs TENSION & COURANT */

  ptrdac[0]=0;
  setVoltsAmps.config(SSDAC1,SSDAC2);
  setVoltsAmps.write(0,0);          // init tension 0
  ptrdac[1]=NBRAMP-1;
  setVoltsAmps.write(1,MAXDAC);     // init courant max
  
  delay(1000);                      // conversions
  showTft();

//while(1){digitalWrite(LED,HIGH);delay(50);digitalWrite(LED,LOW);delay(950);}


  /* encoder */

  pinMode(COD_INT1, INPUT_PULLUP);
  pinMode(COD_INT2, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(COD_INT1), isrCod, CHANGE);
  attachInterrupt(digitalPinToInterrupt(COD_INT2), isrCod, CHANGE);

  /* Menu */
  
  memset(comptMenu,0x00,sizeof(comptMenu));

  Serial.println(" ready");
  
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

  /* Température */
#ifdef THERMO   
  if(millis() > (tmpTemp + perTemp)) {
    tmpTemp = millis();
    //pinMode(PINTEMP,OUTPUT);digitalWrite(PINTEMP,HIGH);delay(1);digitalWrite(PINTEMP,LOW);
    temp=thermo.readDs(PINTEMP);
    if(temp!=oldtemp){oldtemp=temp;tempChg=VRAI;}
    //Serial.print("T ");Serial.println(temp);
    thermo.convertDs(PINTEMP);
    pinMode(POUSSOIR, INPUT);
  }
#endif

  /* poussoir (positionne la variable "poussoir" ON ou OFF) */

  if (millis() > (tmpPouss + perPouss)) {

    savled = bitRead(PORT_POUSSOIR,BIT_POUSSOIR); //digitalRead(POUSSOIR);
    bitClear(DDR_POUSSOIR,BIT_POUSSOIR);  //pinMode(POUSSOIR,INPUT);
    if (digitalRead(POUSSOIR) == HIGH) {poussoir = ON;} 
    else {poussoir = OFF;}
    
    bitSet(DDR_POUSSOIR,BIT_POUSSOIR);if(savled!=0){bitSet(PORT_POUSSOIR,BIT_POUSSOIR);}  // else bitClear(PORT_POUSSOIR,BIT_POUSSOIR);
    tmpPouss = millis();
  }
  
  /* rampe & test dac i */
  
  if ( !arretRampe && millis() > (tmpRamp+valeurs[NVALEURPERRAMP]) ) {                 // valeurs[NVALEURPERRAMP] est la période de la rampe
    tmpRamp = millis();
      if(!fastTft){
        serialShow(0, volt[ptrdac[0]]); //Serial.println();
        serialShow(1, volt[ptrdac[1]]); //Serial.println();
      }
      ptrdac[0]++;                                                  // avance le step courant de la rampe
      if (ptrdac[0] >= NBRAMP) {
        ptrdac[0] = 0;
      }
      valeurs[NVALEURVOLTS] = volt[ptrdac[0]];                      // valeurs[NVALEURVOLTS] est la consigne de tension de sortie ; 
                                                                    // ptrdac[0] est le n° de step courant de la rampe
                                                                    // volt[ptrdac[0]] est la tension du step courant de la rampe
      setVoltsAmps.write(0, volt[ptrdac[0]]);                       // charge la tension de sortie dans le dac tension
    
/*
    if (testDaci) {
      serialShow(1, volt[ptrdac[1]]); 
      ptrdac[1]++;
      if (ptrdac[1] >= MAXDAC) {
        ptrdac[1] = 0;
      }
      setVoltsAmps.write(1, volt[ptrdac[1]]);
    }
*/  
  }

  /* affichage volts/amp actuels */

  if ((millis() > tmpAff + perAff) && digitalRead(LED) == OFFLED)
  {
    tmpAff = millis();
    showTft();
  }

  if (show == VRAI && !fastTft) { // Serial
    show = FAUX;
    measure.ampVolt(&iout, &vcollector);
  }

  /* codeur */

  for(int i=0;i<NBVALEURS;i++){
    if(statusValeurs[i] && (valeurs[i] != codeur)) {saisieValCodeur(&valeurs[i],i);      // saisie en cours sur la valeur i et la valeur a changé dans codeur
                                                                                         // donc maj codeur->valeurs[i]
      //Serial.print(" codeur ");Serial.print(codeur);Serial.print(" param");Serial.print(i);Serial.print(" ");Serial.println(valeurs[i]);  
      if(i==NVALEURPERRAMP && valeurs[i]<FASTPERRAMP){fastTft=VRAI;}
      else {fastTft=FAUX;}
      if(i!=NVALEURVOLTS && i!=NVALEURAMPS){                                                         // saisie en cours sur param rampe
        strncpy(comptMenu+LENCOMP*choix1,paramItem(valeurs[i],unitValeurs+i*LENUNITVALEUR),LENCOMP);
        menu1(choix1,"*",comptMenu);
      }
    }
  }
  
  /* statusMenu!=0 -> menu en cours, scrutation codeur
     états 1 à 3 Menu 1 vertical
        1 le menu vient d'être affiché (passer à 2 si le poussoir est off)
        2 scutation codeur pour changement de ligne si poussoir ON passer à 3 pour valider la positipon au relachement
        3 attente relachement du poussoir ; 
              si poussoir relaché afficher * et passer à 4 ; faire le traitement selon type de l'entrée
     états 4 à 8 Menu 2 horizontal
      "choix1 ou 2" variable de l'option courante   
      "prechoix1 ou 2" choix précédent (pour effacer le curseur)
  */
  if (statusMenu > 0 && statusMenu<90){choixCodeur(&choix1,&prechoix1);}

  /* menu */
  
  if (poussoir == ON  && statusMenu == 2) {       // poussoir appuyé -> attente relachement pour valider "choix1" 
    statusMenu = 3;  menuChg=VRAI;
    Serial.println("M3");
  }

  if (poussoir == OFF && statusMenu == 1) {       // poussoir relaché -> scrutation codeur 
    statusMenu = 2;  menuChg=VRAI;
    Serial.println("M2");
    //delay(100);
  }

  if ((poussoir == ON  && statusMenu == 0) || statusMenu==4) {       // appui du poussoir -> affichage menu 
    statusMenu = 1; menuChg=VRAI;
    Serial.println("M1");
    testDaci = FAUX;
    memset(statusValeurs,FAUX,NBVALEURS);
    //arretRampe = VRAI;
    showTft();    
    codeur = 0; choix1 = 0; prechoix1 = 0;

    if(levelMenu<0){
      levelMenu=0;
      memset(comptMenu,0x20,LENCOMP*MAXENTMENU);
      comptMenu[LENCOMP*MAXENTMENU]='\0';
    } 
    affichMenu(numerMenu[levelMenu]);
  }

  if (poussoir == OFF && statusMenu == 3) {       // poussoir relaché -> choix valide 
    switch(modelMenu[numerMenu[levelMenu]]){      // maj de l'indicateur de l'item choisi    
      case 0: menu1(choix1, "*",comptMenu); break;
      case 1: menu2(choix1, "*"); break;
      default: break;
    }
    statusMenu = 4;  menuChg=VRAI;
    m1=numerMenu[levelMenu]; // numéro du menu courant
    Serial.print("M4 choix1=");Serial.print(choix1);Serial.print(" menu=");Serial.print(m1);Serial.print(" type=");Serial.print(*(typeMenu[m1]+choix1));
    //Serial.print(" paramMenu=");Serial.print(*(paramMenu[m1]+choix1));
    switch(*(typeMenu[m1]+choix1)){
      case 1: choixMenu[levelMenu]=choix1;
              entreMenu();initMenu(m1);
              break; // entrée menu vertical (init complt et save valeurs)
      case 2: restoreMenu();   // restore param
              break; // back sans effet
      case 3: choixMenu[levelMenu]=0;
              entreMenu();effaceLigneFin(TFTH-TFTCH-4);
              break; // entrée menu horizontal (ni complt ni save valeurs)
      case 4: levelMenu--;choix1=choixMenu[levelMenu];initMenu(numerMenu[levelMenu]);
              break; // back validé
      case 5: levelMenu-=2;choix1=choixMenu[levelMenu];initMenu(numerMenu[levelMenu]);
              effaceLigneFin(TFTH-TFTCH+2);
              break; // back 2 niveaux 
      case 6: levelMenu--;m1=numerMenu[levelMenu];
              restoreMenu();affichMenu(m1);effaceLigneFin(TFTH-TFTCH-4);effaceLigneFin(TFTH-TFTCH+2);
              break; // back 2 niveaux sans effet
      case 7: statusMenu = 90;                                  // saisie numérique en cours
              startSaisie(FAUX,*(paramMenu[m1]+choix1),0);      // (paramMenu[m1] pointe la table des paramètres du menu courant
                                                                // (paramMenu[m1]+choix1 pointe le paramètre de l'entrée courante du menu courant
                                                                //    donc le numéro de valeur (volt/amp/periode etc)
                                                                // codeur est préchargé puis mis à jour par la rotation
              Serial.print(" -- init saisie ; valeur=");Serial.print(valeurs[*(paramMenu[m1]+choix1)]);Serial.print(" codeur=");Serial.print(codeur);
              delay(3);
              break; // saisie num
      case 8: arretRampe = !arretRampe;statusMenu=1;
              //Serial.print(" Rampe ");Serial.println(arretRampe);
              if(!arretRampe && valeurs[NVALEURPERRAMP]<FASTPERRAMP){fastTft=VRAI;}
              break;  // start/stop
      default:break;
    }
    Serial.println();
  }
 
  if (poussoir == ON && statusMenu == 90) {          // poussoir relaché -> saiise numérique valide 
                                                      // ajouter menu de validation horizontal (">Valid  Annul")
    Serial.print("M90 -- transfert ; valeur=");Serial.print(valeurs[*(paramMenu[m1]+choix1)]);Serial.print(" codeur=");Serial.println(codeur);
    valeurs[*(paramMenu[m1]+choix1)]=codeur;          // si "Valid" il faut transférer codeur dans la valeur 
                                                      // et la valeur dans le DAC
    switch(*(paramMenu[m1]+choix1)){                  // 0=dac volts ; 1=dac amps ; 2=perRamp ; 3=nbStepRamp ; 4=voltsMinRamp ; 5=voltsMaxRamp
      case 0:setVoltsAmps.write(DACVOLT,valeurs[*(paramMenu[m1]+choix1)]);break;
      case 1:setVoltsAmps.write(DACAMPS,valeurs[*(paramMenu[m1]+choix1)]);break;
      default:break;          
    }
    statusMenu = 0;  menuChg=VRAI;
  }


  /* commandes Serial */

/*  if (Serial.available()&&!fastTft) {
    switch (Serial.read()) {
      case 'A': if (arretRampe == VRAI) {
          arretRampe = FAUX;
        } else {
          arretRampe = VRAI;
        }; break;
      case 'B': if (statusValeurs[0] == VRAI) {
          statusValeurs[0] = FAUX;
        }
        else {
          startSaisie(FAUX,0,0);
        }
        break;
      case 'C': if (statusValeurs[1] == VRAI) {
          statusValeurs[1] = FAUX;
        }
        else {
          startSaisie(FAUX,1,0);
        }
        break;
      case 'D': if (testDaci == FAUX) {
          testDaci = VRAI;
          statusValeurs[0]=FAUX;
          statusValeurs[1]=FAUX;
          arretRampe = VRAI;
          statusMenu = 0; menuChg=VRAI;
        }
        else {
          testDaci = FAUX;
        }
        break;
      default: break;
    }
  }
  */
}

/* --------------- fin loop ---------------------- */

void affichMenu(int menu)
{
  switch(modelMenu[menu]){          
      case 0: menu10(comptMenu); break;
      case 1: menu20(); break;
      default: break;
  }
}

void menu10(char* complt)     // menu vertical
{
  tft.setTextSize(1);
  for (int i = 0; i < nbentMenu[numerMenu[levelMenu]]; i++) {
    menu1( i, ">",complt);
  }
}

void menu1(int numitem, char* arrow,char* complt)                 // affiche une ligne de menu vertical + complt et curseur éventuel
                                                                  // numitem n° de l'entrée
{
  m0=numerMenu[levelMenu]; // num menu courant
  char item[LENENTHMENU+LENCOMP+1];
  for (int j = 0; j < LENENTHMENU; j++) {
    item[j] = *(entryMenu[m0]+numitem*LENENTHMENU+j);             // libellé de l'entrée -> item[]
  } 
    for (int j = 0; j < LENCOMP; j++) {
    item[j+LENENTHMENU] = complt[numitem * LENCOMP +j];           // chaine complémentaire -> item[]
  } item[LENENTHMENU+LENCOMP] = '\0';
  
  //Serial.print(" menu1(");Serial.print(item);Serial.print(",");
  //Serial.print(arrow[0]);Serial.println(") ");
/*  Serial.print(levelMenu,HEX);Serial.print("-");Serial.print(numerMenu[levelMenu]);Serial.print("-");
  for(int j=0;j<12;j++){Serial.print(*(entryMenu[numerMenu[levelMenu]]+j));};Serial.println(); */
  tft.setTextSize(1);
  tft.setCursor(hposMenu[m0],vposMenu[m0] + (numitem * TFTCH));   // curseur début ligne de l'entrée +1 caractère
  tft.print(item);
  tft.setCursor(hpcMenu[m0],vposMenu[m0] + (numitem * TFTCH));
  if (numitem == choix1) {tft.print(arrow[0]);}
  else {tft.print(" ");}
}

void menu20()     // menu horizontal
{
  char item[8];
  tft.setTextSize(1);
  for (int i = 0; i < nbentMenu[numerMenu[levelMenu]]; i++) {
    menu2( i, ">");
  }
}

void menu2(uint8_t numitem, char* arrow)
{
  m0=numerMenu[levelMenu]; // num menu courant
  char item[LENENTVMENU+1]; item[LENENTVMENU] = '\0';

  for (int j = 0; j < LENENTVMENU; j++) {
    item[j] = *(entryMenu[m0]+numitem*LENENTVMENU+j);
  }

  int k0=(TFTW-nbentMenu[m0]*LENENTVMENU*TFTCW)/(nbentMenu[m0]+1);   // intervalle horizontal
  int k=LENENTVMENU*TFTCW+k0;                                        // pas horizontal
  //Serial.print("menu2(");Serial.print(item);Serial.print(", ");Serial.print(numitem); Serial.print(", ");
  //Serial.print(k0);Serial.print(", ");Serial.print(k);Serial.print(", ");Serial.print(arrow[0]);Serial.println(") ");

  tft.setTextSize(1);
  tft.setCursor(k0+numitem*k,TFTH-TFTCH+2);tft.print(item);
  tft.setCursor(k0+numitem*k-TFTCW,TFTH-TFTCH+2);
  if (numitem == choix1) {tft.print(arrow[0]);}
  else {tft.print(" ");}
}

void effaceLigneFin(int pos)
{
  tft.setCursor(0,pos);tft.print("                          ");
}

void restoreMenu()
{
  for(m0=0;m0<MAXENTMENU;m0++){p0=*(paramMenu[m1]+m0);valeurs[p0]=backValeurs[p0];}   // restore valeurs
    levelMenu--;choix1=choixMenu[levelMenu];initMenu(numerMenu[levelMenu]);
}

void entreMenu()
{
  numerMenu[levelMenu+1]=*(paramMenu[m1]+choix1);
  levelMenu ++;m1=numerMenu[levelMenu];choix1=0;prechoix1=0;choixMenu[levelMenu]=0;
  Serial.print(" model=");Serial.print(modelMenu[m1]);
}

void initMenu(int menu)
{
  memset(comptMenu,0x00,sizeof(comptMenu));
  for(m0=0;m0<MAXENTMENU;m0++){
    p0=*paramMenu[menu]+m0; // n° du paramètre de l'entrée de menu
    p1=valeurs[p0];          // valeur
    backValeurs[p0]=p1;      // save valeur
    if(*(compMenu[menu]+m0)){
      strncpy(comptMenu+LENCOMP*m0,paramItem(p1,unitValeurs+p0*LENUNITVALEUR),LENCOMP);}
//   Serial.print(" param");Serial.print(valeurs[2+m0]);Serial.print("=");Serial.print(p0);delay(100);
  }
  Serial.print("\nmenu N°");Serial.print(m1);Serial.print(" compt=");Serial.println(comptMenu);
}

void saisieValCodeur(int* valcodeur, int numunit)    /* valcodeur sortie modifiée ; numunit entrée numéro paramètre saisi */
{
  *valcodeur = codeur;
  if (*valcodeur <= minValeurs[numunit]) {
    *valcodeur = minValeurs[numunit];  
    codeur = *valcodeur;
  }
  if (*valcodeur >= maxValeurs[numunit]) {
    *valcodeur = maxValeurs[numunit];
    codeur = *valcodeur;
  }

  switch(numunit){
    case 0: break;                                //setVoltsAmps.write(numunit,*valcodeur);break;   // volts
    case 1: break;                                //setVoltsAmps.write(numunit,*valcodeur);break;   // amps
    case 2: perRampChg=VRAI;break;                // periode rampe
    case 3: stepRampChg=VRAI;break;               // nb step rampe
    default:break;
  }

  show = VRAI;
}

void startSaisie(bool fast,int numunit,int menu)  // débute la saisie d'un paramètre
                                                  // elle se termine avec la dernière étape du menu (OFF et 7 ou OFF et 3)
                                                  // numunit est la valeur du paramètre de l'entrée courante du menu courant
                                                  // donc le numéro d'entrée dans valeurs[]
{
  fastTft    = fast;
  memset(statusValeurs,FAUX,NBVALEURS);             // désactive la saisie sur tous les valeurs[]
  statusValeurs[numunit]=VRAI;                      // active la saisie sur valeurs[numunit]
  //statusMenu = menu;  menuChg=VRAI;
  oldValeur   = valeurs[numunit];                   // sauve la valeur
  codeur     = valeurs[numunit];                    // charge le codeur
  timeIsrCod = 0;

  /* maintenant les mouvements du codeur modifient la variable codeur depuis la valeur initiale valeurs[numunit] 
   * c'est le poussoir qui stoppe la saisie 
  */
}

char* paramItem(int value,char* unit)
{
    unit[2]='\0';
    dtostrf(value, 4, 0, text0); strcat(text0, unit);
//    Serial.print(" value=");Serial.print(value);Serial.print(" text0=");Serial.print(text0);
    delay(10);
    return text0;
}

void showTft()  // récupération valeurs et affichage Tft
{
/*  
  if(!fastTft){
    vout = (float)(analogRead(UOUT) * 5 * 3) / 1024 + 0.05;
    if(voutM==0){voutM=vout;}
    voutM = (voutM * 4 + vout) / 5; 
    if (abs(vout - voutM) > voutM / 50) {
      voutM = vout;}      
    measure.ampVolt(&iout, &vcollector);                        // ina219 shunt on NPN collector
    if(ioutM==0){ioutM=iout;}
    ioutM = (ioutM * 4 + iout) / 5; 
    if (abs(iout - ioutM) > ioutM / 50) {
      ioutM = iout;}
    if(vcollectorM==0){vcollectorM=vcollector;}
    vcollectorM = (vcollectorM * 4 + vcollector) / 5; 
    if (abs(vcollector - vcollectorM) > vcollectorM / 50) {
      vcollectorM = vcollector;
    }
  }
  affTftVoltAmp(voutM, ioutM, vcollectorM, (float)(analogRead(dac[1]) * 5) / 1024, (float)(analogRead(ILIMIT) * 5) / 1024);
*/  
  affTftCurrData(0, 0, 0,(float)valeurs[0]*BINV, (float)valeurs[1]*BINA);

}

void printAtTft(uint8_t posx,uint8_t posy,char* text)
{
  tft.setCursor(posx, posy);
  tft.print(text);
  memset(text, '\0', 32);
}

void affTftCurrData(float outVolts, float outAmps, int vcollector, float reqVolts, float reqAmps)
/* affichage Tft hors menu */
{
  unsigned long cdt=micros();
  rfrAff++;rfrAff&=0x03;                                         // a chaque passage une partie est affichée

  
  tft.setTextColor(0xFFFF, 0x0000);
  tft.setRotation(3);

/*  outtVolts */

  if(!fastTft && rfrAff==1){
    memset(text0, '\0', LTEXT0);
    dtostrf(outVolts, 5, 2, text0);                              // output voltage
    tft.setTextSize(2);
    printAtTft(0,VPOSVA,text0);

/* outAmps */

    dtostrf(outAmps, 7, 2, text0);                               // output current
    printAtTft(70,VPOSVA,text0);
  }

  tft.setTextSize(1);
    //Serial.print(" ");Serial.print(micros()-cdt);
/* puissance */

  if(!fastTft && rfrAff==2){
    memset(text0,'\0',LTEXT0);
    memcpy(text0,"load ",5);
    dtostrf(outVolts*outAmps/1000, 2, 3, text0+5);                    // output power
    strcat(text0,"W   trans ");
    dtostrf(((vcollector/1000)-outVolts)*outAmps/1000, 2, 2, text0+strlen(text0));
    strcat(text0,"W");
    printAtTft(0,VPOSVA+20,text0);
  }
  //Serial.print(" ");Serial.print(micros()-cdt);
  uint8_t pos=10;
  if (!unefois) {
    unefois = VRAI;
    strcpy(text0, "collec  dac_V  I_limit");
    printAtTft(0,pos,text0);
  }

/* collector/reqVolts/reqAmps */

  if(!fastTft && rfrAff==3){
    dtostrf(vcollector, 4, 0, text0); strcat(text0, "mV ");    
    dtostrf(reqVolts, 5, 3, text1); strcat(text0, text1); strcat(text0, "V  ");        // voltage
    dtostrf(reqAmps, 5, 3, text1); strcat(text1, "mA"); strcat(text0, text1);          // current
    printAtTft(0,pos+10,text0);
  }
  //Serial.print(" ");Serial.print(micros()-cdt);
/* menu */

  if(menuChg){
    menuChg=FAUX;
    memset(text0,0x00,LTEXT0);
    strcat(text0, "(");
    dtostrf(statusMenu, 1, 0, text0+1); strcat(text0, ")");
    printAtTft(141,pos,text0);
  }

/* température trans */

  if(tempChg){
    tempChg=FAUX;
    dtostrf(temp, 2, 1, text0); strcat(text0, ".C");
    printAtTft(0,pos-10,text0);
  }
  //Serial.print(" aTCD=");Serial.println(micros()-cdt);
}


void choixCodeur(int* choix, int* prechoix)      // transfère codeur dans choix et choix dans prechoix
{
    int loc_codeur=codeur;
    
    if(*choix!=loc_codeur){
      *choix = loc_codeur;
      if (*choix < 0) {*choix = 0;loc_codeur = *choix;}
      if (*choix >= nbentMenu[numerMenu[levelMenu]]) {*choix = nbentMenu[numerMenu[levelMenu]]-1;loc_codeur = *choix;}
      
      switch(modelMenu[numerMenu[levelMenu]]){          // maj du curseur
        case 0: menu1(*prechoix, " ",comptMenu);menu1(*choix, ">",comptMenu);
                break;
        case 1: menu2(*prechoix, " ");menu2(*choix, ">"); 
                break;
        default: break;
      }
      *prechoix = *choix;codeur=loc_codeur;
      }
}

void serialShow(uint8_t numunit, int consigne)
{
  /* VOLTS affiche 
    consigne,
    valeur (0-1024) tension de sortie du dac,
    écart avec valeur précédente,
    tension de sortie (V) brute,
    tension de sortie (V) mise à l'échelle 

    AMPS  affiche 
    consigne,
    valeur (0-1024) tension de sortie du dac,
    écart avec valeur précédente,
    tension de sortie (V) brute,
    tension sur mesure courant (V) 
  */
/*
  if (consigne == 0) {
    oldVoltlu[numunit] = 0;
    oldVoltout[numunit] = 0;
  }
  if (consigne < 1000) {Serial.print(" ");}
  printfix(consigne);

  voltlu[numunit] = analogRead(dac[numunit]);          // 0-1023 sortie dac
  printfix(voltlu[numunit]);

  Serial.print("/");
  int vv = (voltlu[numunit] - oldVoltlu[numunit]);
  printfix(abs(vv));
  oldVoltlu[numunit] = voltlu[numunit];

  Serial.print(" dac"); Serial.print(numunit); Serial.print(":"); Serial.print((float)(voltlu[numunit] * 5) / 1024);

  int vx ; 
  if(numunit==0){vx=analogRead(UOUT);
    Serial.print(" out:");
    Serial.print((float)(vx * 5 * 3 * 1.04) / 1024); // tension sortie alim
    Serial.println("V");
  }
  else{vx=analogRead(ILIMIT);
    Serial.print(" ilimit:");
    Serial.print((float)(vx * 5) / 1024);            // tension sortie DACI
    Serial.println("V");  
  }
*/ 
}


void printfix(int value)
{
/*  if (value < 100) {
    Serial.print(" ");
  }
  if (value < 10) {
    Serial.print(" ");
  }
  Serial.print(" ");
  Serial.print(value );
*/}

/* gestion interruptions */

void codIntIncr(int mul)
{
  t = millis() - timeIsrCod;

  if (t<TINCR4 && t>=TINCR3){        // && codeur>INCR4*2) {
    codeur += INCR4*mul;
  }
  else if (t<TINCR3 && t>=TINCR2){   // && codeur>INCR3*3) {
    codeur += INCR3*mul;
  }
  else if (t<TINCR2 ){   // && codeur>INCR2*4) {
    codeur += INCR2*mul;
  }
  else {
    codeur += INCR1*mul;
  }

  timeIsrCod = millis();
}

void isrCod()
{
  etat = etat << 1 | digitalRead(COD_INT1);
  etat = etat << 1 | digitalRead(COD_INT2);

  //test = (etat | B11110000) - B11110000; //test des 4 bits de poids faible
  test=etat&0x0f;

  if (test == B0111 ) {
    codIntIncr(1);
  } //codeur++;}
  else if (test == B1011 ) {
    codIntIncr(-1);
  } //codeur--;}
}

/*---------------------------------------------*\
 * CARkar22                                    *
 * 06-10-2022                                  *
 * 21-1--2024                                  * 
 * 20-03-2025 VINYL default 0                  *
 * 06-07-2025 check: parse                     *
\*--------------------------------------------*/
#include <Wire.h>
#include "carkar.h"

byte sq_d;              //dispatch
byte leds;              // redled & yellowled status

void setup() {
int n;
 Serial.begin(9600);          // start serial
  pinMode(LED, OUTPUT);       // led pin op output
  pinMode(DIRL, OUTPUT);       
  pinMode(DIRR, OUTPUT);   
  pinMode(PWML, OUTPUT);     
  pinMode(PWMR, OUTPUT);
  pinMode(ENC_L,INPUT);
  pinMode(ENC_R,INPUT);
  pinMode(BUTT,INPUT_PULLUP);  
  TCCR2B=TCCR2B & B11111000 | B00000001;  // 31,3 kHz 270mA @wmove(0x18,0x18)
 //TCCR2B=TCCR2B & B11111000 | B00000010; // 4 kHz  300mA @wmove(0x18,0x18)
  msref=millis()+10;
  wmove(0x0,0x0);
  enco0_L=digitalRead(ENC_L);
  enco0_R=digitalRead(ENC_R);
  redled(0); // redled OFF
  ylwled(0); // yellow led OFF
  Wire.begin();              // start I2C 
  n= Wire.requestFrom(PCF8574_I2C_ADDRESS,1); // aantal uint8_ts
 if(n==1){
  sq_d=(byte)Wire.read()&0x7;   // read 3bit switchregister
  Wire.endTransmission();
 }
}

/*=========================== LOOP =========================*/
void loop(){
int i;
int n;
  switch(sq_d){           // dipswitch 0:down  1:up
    case(0): shuttle();   //  000  
    case(1): wall_1();    //  001
    case(2): shuffle();   //  010
    case(3): LineFol(0);  //  011
    case(4): LineFol(1);  //  100
    case(5): demo();      //  101
    case(6): scanraw();   //  110
    case(7): sharptest(); //  111
    default: break;
  }  
}

//==== check RC5req for pending rc5_msg ->  Get rc5_msg ===
bool getRC5(void){
int sts, req;
   sts= Wire.requestFrom(PCF8574_I2C_ADDRESS,1); // aantal uint8_ts
   if(sts==1){        // check remote message flag
     req=Wire.read();
     Wire.endTransmission();
     if(req&0x8){                 // ? new remote mmessage
       sts= Wire.requestFrom(RC5_I2C_ADDRESS,1); // aantal uint8_ts
       if(sts==1){
         rc5_msg=Wire.read();   // acquire
         Wire.endTransmission();
         return(1);
       }else Serial.println("RC5read fail");  
    }                           // nothing
  }
  return(0);
}

//======= read IR SHARP sensors ==========
void acq_sensors(void){
int raw;
static uint8_t derate=30;
  raw=analogRead(IRS_L);
  raw= (40000)/raw;
  if(raw > 255) raw=255;          
  range[Lft]=(uint8_t)raw;    // left sensor
  raw=analogRead(IRS_C);
  raw= (40000)/raw;
  if(raw > 255) raw=255;          
  range[Ctr]=(uint8_t)raw;    // center sensor
  raw=analogRead(IRS_R);
  raw= (40000)/raw;
  if(raw > 255) raw=255;          
  range[Rgt]=(uint8_t)raw;    // right sensor
}
    

/* === cancel drifting left/right ===
 straight ?pos->drifting right   ?neg->drifting left
 v=8 -> about straight ahead   v<8 -> turn slightly right   v>8  -> turn slightly left
*/

void ahead(uint8_t cnfg){
static uint8_t v, sqa;
  if(cnfg){
    if(cnfg==0xFF) sqa=0;  // disable 'ahead'
    else{
      v=cnfg; straight=0; sqa=1;
      v &= 0x3F;
      v=v<<2;
      }   // v+8 about straight
  }  
  switch(sqa){
      case(0): break;
      case(1): if(straight > 0){digitalWrite(LED,1); analogWrite(PWMR, v+12); sqa=2;} //+12
               else if(straight < 0){digitalWrite(LED,1); analogWrite(PWMR, v+3); sqa=3;} break;

      case(2): if(straight <= 0){digitalWrite(LED,0); analogWrite(PWMR, v+8); sqa=1;} break;
      case(3): if(straight >= 0){digitalWrite(LED,0); analogWrite(PWMR, v+8); sqa=1;} break;
    }  
}


void wmove(uint8_t wlft, uint8_t wrgt){
uint8_t tmpl, tmpr;
bool gos;
  wlft&=0xBF; wrgt&=0xBF;
  gos=((wlft)==(wrgt))?1:0;   // go straight
  ahead(0xFF);                // disable ahead()
  if(wlft&0x80) digitalWrite(DIRL,1); else digitalWrite(DIRL,0);
  tmpl=wlft&0x3F;
  tmpl=tmpl<<2;
  analogWrite(PWML, tmpl);
  if(wrgt&0x80) digitalWrite(DIRR,1); else digitalWrite(DIRR,0);
  tmpr=wrgt&0x3F;
  tmpr=tmpr<<2;
  if(tmpr < 248) tmpr+=8;     // straight compensation
  analogWrite(PWMR, tmpr);
  if(gos) ahead(wlft&0x3F); else ahead(0xFF);
}

void encoders(void){
uint8_t enco_L, enco_R;
  enco_L=digitalRead(ENC_L);      //PIND kan ook
  if(enco_L!=enco0_L){
    if(enco_L) digitalWrite(PROBE,1); else digitalWrite(PROBE,0);  // trace
    c_odo++;
    enco0_L=enco_L;
    straight++;
    rung=10;        // keep running preset value
  }
  enco_R=digitalRead(ENC_R);
  if(enco_R!=enco0_R){   //Serial.println(odo_R,DEC);
    c_odo++;
    enco0_R=enco_R;
    straight--;
    rung=10;      // keep running preset value
  }  
}

// on board LED
void ledBlink(void){ 
static uint8_t derate=0;
static bool onof;
  derate++;
  if(derate > 25){
    onof=onof^1;
    if(onof) derate=18; else derate=0;
    digitalWrite(LED,onof);
  }  
}

bool button(void){
static uint8_t sq=0;
  switch(sq){
    case(0): if(!digitalRead(BUTT)) sq=1;      // ?pushed
              break;
    case(1): if(digitalRead(BUTT)) {sq=0; return(1);} // ?released
              break;
  }
  return(0);
}

void redled(bool Nf){
uint8_t tmp;
  switchLed((uint8_t) Nf);  
}

void ylwled(bool Nf){
uint8_t tmp;
  tmp=Nf?0x3:0x2;
  switchLed(tmp);  
}


void switchLed(uint8_t sel){
static uint8_t ledstat=0x0F;
static uint8_t ledstat0=0xFF;
  switch(sel){
    case(0x0): ledstat|=0x80; break;
    case(0x1): ledstat&=0x7F; break;
    case(0x2): ledstat|=0x40; break;
    case(0x3): ledstat&=0xBF; break;
  }
  if(ledstat != ledstat0){    // handle when changed
    Wire.beginTransmission(PCF8574_I2C_ADDRESS);
    Wire.write(ledstat);
    Wire.endTransmission();
    ledstat0=ledstat;
  }
}

// acquire proximity sensors
uint8_t prox(void){
uint8_t sts;
static uint8_t p;
    p=0;
    sts= Wire.requestFrom(PROX_I2C_ADDRESS,1);
    if(sts==1) p=Wire.read();
    Wire.endTransmission();
    return(p);
}

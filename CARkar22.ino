/*--------------------------------------------*\
 * CARkar22                                    *
 * 06-10-2022                                  *
\*--------------------------------------------*/
#include <Wire.h>
#include "carkar.h"

static int sq_d;      //dispatch

void setup() {
int n;
 Serial.begin(9600);          // start serial
 // Serial.begin(115200);     // start serial
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
  Wire.begin();              // start I2C 
  n= Wire.requestFrom(PCF8574_I2C_ADDRESS,1); // aantal bytes
 if(n==1){
  sq_d=Wire.read()&0x7;   // read 3bit switchregister
  Wire.endTransmission();
 }
}

/*=========================== LOOP =========================*/
void loop(){
int i;
int n;
  switch(sq_d){           // dipswitch 0:down  1:up
    case(0): mission_1(); //  000  
    case(1): wall_1();    //  001
    case(4): wall_eval(); //  100
    case(5): demo();      //  101
    default: break;
  }  
}

//==== check RC5req for pending rc5_msg ->  Get rc5_msg ===
bool getRC5(void){
static byte derate=10;
int n, m;
   n= Wire.requestFrom(PCF8574_I2C_ADDRESS,1); // aantal bytes
   if(n==1){        // check remote message flag
     m=Wire.read();
     Wire.endTransmission();
     if(m&0x8){                 // ? new remote mmessage
       n= Wire.requestFrom(RC5_I2C_ADDRESS,1); // aantal bytes
       if(n==1){
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
static byte derate=30;
  raw=analogRead(IRS_L);
  raw= (40000)/raw;
  if(raw > 255) raw=255;          
  range[0]=(byte)raw;
  raw=analogRead(IRS_C);
  raw= (40000)/raw;
  if(raw > 255) raw=255;          
  range[1]=(byte)raw;
  raw=analogRead(IRS_R);
  raw= (40000)/raw;
  if(raw > 255) raw=255;          
  range[2]=(byte)raw;
}
    

/* === cancel drifting left/right ===
 straight ?pos->drifting right   ?neg->drifting left
 v=8 -> about straight ahead   v<8 -> turn slightly right   v>8  -> turn slightly left
*/

void ahead(byte cnfg){
static byte v, sqa;
uint16_t itmp;
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


void wmove(byte wlft, byte wrgt){
//static byte tmpl, tmpr;
//static bool gos;
byte tmpl, tmpr;
bool gos;
uint16_t itmp;
  wlft&=0xBF; wrgt&=0xBF;
  gos=((wlft)==(wrgt))?1:0;   // go straight
  ahead(0xFF);    // disable ahead()
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
byte enco_L, enco_R;
  enco_L=digitalRead(ENC_L);      //PIND kan ook
  if(enco_L!=enco0_L){      // Serial.println(odo_L,DEC);
    if(enco_L) digitalWrite(PROBE,1); else digitalWrite(PROBE,0);
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
//    Serial.println(straight);
  }  
}

void ledBlink(void){ 
static byte derate=0;
static bool onof;
  derate++;
  if(derate > 25){
    onof=onof^1;
    if(onof) derate=18; else derate=0;
    digitalWrite(LED,onof);
  }  
}

bool button(void){
static byte sq=0;
  switch(sq){
    case(0): if(!digitalRead(BUTT)) sq=1; break;
    case(1): if(digitalRead(BUTT)) {sq=0; return(1);}
  }
  return(0);
}

// offboard LED
void redled(bool Nf){
  Wire.beginTransmission(PCF8574_I2C_ADDRESS);
  if(Nf) Wire.write(0x0F);  // redled on
  else Wire.write(0x8F);    // redled off
  Wire.endTransmission();
}

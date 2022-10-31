/*--------------------------------------------*\
 * CARkar22                                    *
 * 06-10-2022                                  *
\*--------------------------------------------*/
#include <Wire.h>
#include "carkar.h"

static int sq_d;      //dispatch

void setup() {
int n;
  Serial.begin(115200);          // start serial
  pinMode(LED, OUTPUT);          // led pin op output
  pinMode(DIRL, OUTPUT);       
  pinMode(DIRR, OUTPUT);   
  pinMode(PWML, OUTPUT);     
  pinMode(PWMR, OUTPUT);
  pinMode(ENC_L,INPUT);
  pinMode(ENC_R,INPUT);
  pinMode(BUTT,INPUT_PULLUP);  
  //TCCR2B=TCCR2B & B11111000 | B00000001;   // 31,3 kHz   05-05-2020
  TCCR2B=TCCR2B & B11111000 | B00000010;   // 4 kHz   04-10-2022
  msref=millis()+10;
  wmove(0x0,0x0);
  enco0_L=digitalRead(ENC_L);
  enco0_R=digitalRead(ENC_R);
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
 // for(i=2;i<64;i++){
 //     n= Wire.requestFrom(i,1); // aantal bytes
 //     if(n==1)Serial.println(i);
 //     else{Serial.print(". "); Serial.println(i);}
 // }  
  switch(sq_d){
    case(0): mission_1(); break;
    case(1): wall_1(); break;
    default: break;
  }  
}

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

//======= read IR sensors here ==========
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
 #if 0
  derate--;
  if(derate==0){
    derate=30;
    Serial.print(range[0]);   // left
    Serial.print(" ");
    Serial.print(range[1]);   // center
    Serial.print(" ");
    Serial.println(range[2]); // right
   } 
#endif  
}
    

void ahead(byte cnfg){
static byte s, sqa;
  if(cnfg){
    if(cnfg==0xFF) sqa=0;  // disable 'ahead'
    else{s=cnfg; s=s<<2; straight=0; sqa=1;}   // s+8 about straight
  }  
  switch(sqa){
      case(0): break;
      case(1): if(straight > 0){digitalWrite(LED,1); analogWrite(PWMR, s+10); sqa=2;} //+12
               else if(straight < 0){digitalWrite(LED,0); analogWrite(PWMR, s+3); sqa=3;} break;

      case(2): if(straight <= 0){digitalWrite(LED,0); analogWrite(PWMR, s+8); sqa=1;} break;
      case(3): if(straight >= 0){analogWrite(PWMR, s+8); sqa=1;} break;
    }  
}
 
void wmove(byte wlft, byte wrgt){
static byte tmpl, tmpr;
static bool gos;
  wlft&=0xBF; wrgt&=0xBF;
//  if((wlft&0x80)==(wrgt&0x80)) gos=1; else gos=0;
  if((wlft)==(wrgt)) gos=1; else gos=0;
  ahead(0xFF);    // disable ahead()
  if(wlft&0x80) digitalWrite(DIRL,1); else digitalWrite(DIRL,0);
  tmpl=wlft&0x3F;
  tmpl=tmpl<<2;
//  Serial.print(tmpl);
//  Serial.print(" ");
  analogWrite(PWML, tmpl);  
  if(wrgt&0x80) digitalWrite(DIRR,1); else digitalWrite(DIRR,0);
  tmpr=wrgt&0x3F;
  tmpr=tmpr<<2;
  tmpr+=8;
  analogWrite(PWMR, tmpr);     // straight compensation
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
    rung=10;
  }
  enco_R=digitalRead(ENC_R);
  if(enco_R!=enco0_R){   //Serial.println(odo_R,DEC);
    c_odo++;
    enco0_R=enco_R;
    straight--;
    rung=10;
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

#if 0
   // 10 ms lus
  int ms = millis();
  if ((ms - MainTakt) > 0) {
    MainTakt = ms + 10;  // zet tijd voor volgende interval 
//    printf("%d\n",SharpAfstand(A2));
    if(SharpAfstand(A2)>100) edge=true; else edge=false;
    RijdenTakt();  
  }                

int SharpAfstand(int Pin){
  int SensorValue = analogRead(Pin);
  int Afstand = (40000) / SensorValue;
  return Afstand;
}
#endif

#include "carkar.h"

/* priority     vect
 * 0000.0000    0
 * 0000.0001    10
 * 0000.0010    20
 * 0000.0100    30
 * 0000.1000    40
 * 0001.0000    50
 */

//==== dipswitch setting 010 ====
void shuffle(){
const uint16_t esc=0b1100010111001001; //escape pattern
uint16_t escm;          //escape mask
byte vect[6]={0,10,20,30,40,50};
uint8_t spd;      
uint8_t sq_m, sq_b;
uint8_t sq_m0;    // probe
uint8_t prity, prity0;
uint8_t i, mask, mask0, X, susp;   // X manage task_end
bool s;    // style
  spd=0x18;
  prity=0;
  prity0=0xFF;
  sq_m=sq_b=0;
  sq_m0=0xFF;     // trace
  escm=0x1;
  mask0=0x80;     // force handling
  while(1){
    encoders();
    ms=millis();
    while(c_odo){c_odo--; odosum++; if(odosum==odoref) b_odo=1;}
    if(ms > msref){
      msref=ms+30;
      if(getRC5()){
//        Serial.println(rc5_msg);
      }
      if(rung) rung--; // running
      if(!rung)  ledBlink();
      ahead(0);
      if(prity != prity0){
        mask=0x10;              // b0001.0000
        for(i=5; i>0; i--){
          if(prity & mask){
            if(mask==mask0) break;
            else if(!(mask==mask0)){sq_m=vect[i];  mask0=mask; break;}
          }
          mask=mask >> 1;
        }
        prity0=prity;
//        Serial.println(prity);
      }
//      if(sq_m != sq_m0){Serial.print("   "); Serial.println(sq_m); sq_m0=sq_m;}
      switch(sq_b){
        case(0): if(button()){prity=1; sq_b=1;} break;
        case(1): if(button()){
                    wmove(0x0,0x0); prity=0; X=0; mask0=0x80; sq_m=sq_b=0;} break;              
      }
       switch(sq_m){
        case(0): break;                                               // idle
        case(10): X=0; wmove(0x12,0x12); sq_m=0; break;    // pr level 1
        case(20): break;    // priority level 2
        case(30): break;    // priority level 3
        case(40): if(s) wmove(0xA,0x14); else wmove(0x14,0xA);
                  X|=0x8; O_Set(60); sq_m=41; break;                      // exec pr level 4
        case(41): if(MoveRdy) {X &= ~0x8; sq_m=0;} break; 
        case(50): X|=0x10; wmove(0x92,0x92); O_Set(50); sq_m=51; break;   // exec pr level 5
        case(51): if(MoveRdy){
                    if(esc & escm) wmove(0x0,0x92); else wmove(0x92,0x0);
                    if(escm==0x8000) escm=0x1; else escm=escm << 1;
                    O_Set(30); sq_m=52;} break;  // restore pr level 1
          case(52): if(MoveRdy) {b_odo=0; prity &= 0x1; X &= ~0x10; sq_m=0;} break;

        default: break;                 
     } // end switch
/*=========== sensor events ===========*/
      if(sq_b){   // ? /standby
        acq_sensors();
        if(prox() && !(prity & 0x10)){prity |= 0x10;} else if(!(X&0x10)) prity &= ~0x10;  // activate level 5
        if(!(prity&0x8)){
          if(range[Lft] < 130) {prity |= 0x8; s=0;}                        // activate level 4
          else if(range[Rgt] < 130) {prity |= 0x8; s=1;}                   // activate level 4
        }
        else if (!(X&0x8)) prity &= ~0x8;
      }
    } // end ms
  } // end while
}


//==== dipswitch setting 000 ====
void shuttle(){
uint8_t spd;      
uint8_t sq_m;
  spd=0x18;
  while(1){
    encoders();
    ms=millis();
    if(ms > msref){
      msref=ms+30;
      if(getRC5()){
        Serial.println(rc5_msg);
        if(rc5_msg < 0x3F) spd=rc5_msg;
      }
      if(rung) rung--; // running
      if(!rung)  ledBlink();
      acq_sensors();
      ahead(0);
      while(c_odo){c_odo--; odosum++; if(odosum==odoref) b_odo=1;}      
      if(sq_m && button()){wmove(0x0,0x0); sq_m=0;}
      if(sq_m) Serial.println(straight); 
      switch(sq_m){
        case(0): if(button()){
                  wmove(spd,spd);
                     O_Set(900); 
                    sq_m=1;
                  }break;
        case(1): if(MoveRdy){
                  wmove(0x0,0x0);
                  sq_m=2;            
                }break;
        case(2): if(Halted){Serial.println("X"); sq_m=3;} break;
        case(3): wmove(0x11,0x91);
                  O_Set(99);   // 98 95 100
                  sq_m=4;
                  break;
        case(4): if(MoveRdy){
                    wmove(0x0,0x0);
                    sq_m=5;               
                 }break;
        case(5): if(Halted){Serial.println("X"); sq_m=6;} break;
        case(6): wmove(spd,spd);
                  O_Set(900);
                 sq_m=1;
                 break;
          
     } // end switch
    } // end ms
  } // end while
}

//==== dipswitch setting 101 ====
void demo(){
bool togg=0;
uint8_t sq;
uint8_t range0;
   while(1){
    ms=millis();
    if(ms > msref){
      msref=ms+30;
      ledBlink();
      acq_sensors();
      prox();             // bumpers
      if(getRC5()){
        if(rc5_msg==0xFF){
           togg=!togg;
          redled(togg); 
          sq=0;          
        }else if(rc5_msg > 0xFB) sq=rc5_msg;
        else{
          sq=0;
          Serial.println(rc5_msg);
        }
      }
      switch(sq){
        case(0): break;
        case(0xFE): acq_sensors();                  // show left sensor
                      if(range0 != range[Lft]) Serial.println(range[0]);
                      range0=range[Lft]; break;
        case(0xFD): acq_sensors();                  // show centre sensor
                      if(range0 != range[Ctr]) Serial.println(range[1]);
                      range0=range[Ctr]; break;
        case(0xFC): acq_sensors();                  // show right sensor
                      if(range0 != range[Rgt]) Serial.println(range[2]);
                      range0=range[Rgt]; break;
        default: break;              
      }
    } // end ms
  } // end while  
}

/* sharp sensor saturationtest
  redled ON when range[x] < 80
  x=Lft|Ctr|Rgt  */
  
//==== dipswitch setting 100 ====
void sharptest(void){
    while(1){
      ms=millis();
      if(ms > msref){
        msref=ms+30;
        ledBlink();
        acq_sensors();
        if((range[Lft] < 80) || (range[Ctr] <80) || (range[Rgt] <80)) redled(1); else redled(0);
      }
   }  
}

//==== dipswitch setting 011 ===
void linetest(void){
uint8_t sts, i;
bool acq;  
uint8_t linedat[10], *p_linedat;
      while(1){
      ms=millis();
      if(ms > msref){
        msref=ms+100;
        if(button()) acq = !acq; 
        ledBlink();
        if(acq){
          p_linedat=linedat;
          sts= Wire.requestFrom(LINE_I2C_ADDRESS,9);
          while(Wire.available()) {
            *p_linedat++ = Wire.read();
          }
            for(i=1 ; i < 8 ; i++){
            Serial.print(linedat[i]); Serial.print(" ");
          }
          Serial.println(linedat[8]);
        }   
      }
   } 
}  

 

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
byte vect[6]={0,10,20,30,40,50};
uint8_t spd;      
uint8_t sq_m, sq_b;
uint8_t prity, prity0;
uint8_t i, mask, X, susp;   // X manage task_end
bool acquire, s;    // style
  spd=0x18;
  prity=0;
  acquire=0;
  sq_m=sq_b=0;
  prity0=0xFF;
  while(1){
    encoders();
    ms=millis();
    if(ms > msref){
      msref=ms+30;
      if(getRC5()){
        Serial.println(rc5_msg);
      }
      if(rung) rung--; // running
      if(!rung)  ledBlink();
      while(c_odo){c_odo--; odosum++; if(odosum==odoref) b_odo=1;}
      ahead(0);
      if(prity != prity0){
        Serial.println(prity);
        mask=0x10;              // b0001.0000
        for(i=5; i>0; i--){
          if(mask & prity){
            sq_m=vect[i];  prity0=prity; Serial.println(sq_m); break;
          }
          mask=mask >> 1;
          if(mask==0)sq_m=0;
        }
        prity0=prity;  
      }
      switch(sq_b){
        case(0): if(button()){prity=1; sq_b=1;} break;
        case(1): if(button()){wmove(0x0,0x0); acquire=0; prity=0; X=0; sq_m=sq_b=0;} break;              
      }
       switch(sq_m){
        case(0): break;                                               // idle
        case(10): X=0; wmove(0x12,0x12); acquire=1; sq_m=0; break;    // pr level 1
        case(20): break;    // priority level 2
        case(30): break;    // priority level 3
        case(40): if(s) wmove(0xA,0x14); else wmove(0x14,0xA);
                  X|=0x8; O_Set(60); sq_m=41; break;                              // exec pr level 4
        case(41): if(MoveRdy) {X &= ~0x8; wmove(0x12,0x12); sq_m=0;} break;        
        case(50): X|=0x10; wmove(0x92,0x92); O_Set(50); sq_m=51; break;             // exec pr level 5
        case(51): if(MoveRdy){wmove(0x0,0x92); O_Set(30); sq_m=52;} break;  // restore pr level 1
        case(52): if(MoveRdy){prity &= 0x1; sq_m=0;} break;               // restore pr level 1
        default: break;                 
     } // end switch
/*=========== sensor events ===========*/
      if(acquire){
        acq_sensors();
        if(prox()) prity |= 0x10; else if(!(X&0x10)) prity &= ~0x10;        // activate level 5
        if(range[Lft] < 130) {prity |= 0x8; s=0;}                           // activate level 4
        else if(range[Rgt] < 130) {prity |= 0x8; s=1;}                      // activate level 4
        else if (!(X&0x8)) prity &= ~0x8;
      }
    } // end ms
  } // end while
  ;
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
uint8_t p;
    while(1){
      ms=millis();
      if(ms > msref){
        msref=ms+30;
        ledBlink();
        acq_sensors();
        if((range[Lft] < 80) || (range[Ctr] <80) || (range[Rgt] <80)) redled(1); else redled(0);
        p=prox();
//        if(p&0x3) redled(1); else redled(0);
       Serial.println(p);
      }
    }  
}
 

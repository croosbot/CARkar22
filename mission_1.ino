#include "carkar.h"

//==== dipswitch setting 000 ====
void mission_1(){
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
      while(c_odo){c_odo--; odosum++; if(odosum==odoref) b_odo=1;}
      ahead(0);
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
 

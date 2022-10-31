#include "carkar.h"

void mission_1() {     
static byte sq_m;
  while(1){
    encoders();
    ms=millis();
    if(ms > msref){
      msref=ms+30;
      if(getRC5()) Serial.println(rc5_msg);
      if(rung) rung--; // running
      if(!rung)  ledBlink();
      while(c_odo){c_odo--; odosum++; if(odosum==odoref) b_odo=1;}
      ahead(0);
      if(sq_m && button()){wmove(0x0,0x0); sq_m=0;} 
      switch(sq_m){
        case(0): if(button()){
                  wmove(0x18,0x18);
                  //wmove(0x18,0x1A);
                    O_Set(900); 
                    sq_m=1;
                  }break;
        case(1): if(MoveRdy){
                  wmove(0x0,0x0);
                  sq_m=2;            
                }break;
        case(2): if(Halted) sq_m=3; break;
        case(3): wmove(0x11,0x91);
                  O_Set(99);   // 98 95 100
                  sq_m=4;
                  break;
        case(4): if(MoveRdy){
                    wmove(0x0,0x0);
                    sq_m=5;               
                 }break;
        case(5): if(Halted) sq_m=6; break;
        case(6): wmove(0x18,0x18);
                 //wmove(0x18,0x1A);                
                 O_Set(900);
                 sq_m=1;
                 break;
          
     } // end switch
    } // end ms
  } // end while
}

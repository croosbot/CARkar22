#include "carkar.h"


// STANDARD DO NOT MODIFY
void wall_0(){
byte sq_m; //, sq_m0;
  sq_m=0;
  while(1){
    encoders();
    ms=millis();
    while(c_odo){c_odo--; odosum++; if(odosum==odoref) b_odo=1;}
    if(ms > msref){
      msref=ms+30;
      if(rung) rung--; // running
      acq_sensors();
      ledBlink();
      if(sq_m && button()){wmove(0x0,0x0); sq_m=0;} 
      switch(sq_m){
       case(0): if(button()){
                    wmove(0x18,0x18);
                    O_Set(100); 
                    sq_m=1;
                  }break;
       case(1): if(MoveRdy)sq_m=2; break;
       case(2): if(range[1] < 80){wmove(0x0,0x0); sq_m=3; break;}
                if(range[0] > 120) wmove(0x16,0x18);
                else if(range[0] < 110) wmove(0x19,0x16);
                else wmove(0x18,0x18);                               
                break;
       case(3): if(Halted) sq_m=4; break;
       case(4): wmove(0x11,0x91);
                O_Set(99);   // 98 95 100
                sq_m=5;
                break;
        case(5): if(MoveRdy){
                    wmove(0x0,0x0);
                    sq_m=6;               
                 }break;
        case(6): if(Halted){
                  wmove(0x18,0x18); sq_m=7;
                  }break;
        case(7): if(range[1] < 80){wmove(0x0,0x0); sq_m=8; break;}
                 if(range[2] > 120) wmove(0x18,0x16);
                 else if(range[2] < 110) wmove(0x16,0x19);
                 else wmove(0x18,0x18);                               
                 break;
        case(8): if(Halted) sq_m=9; break;
        case(9): wmove(0x11,0x91);
                  O_Set(99);   // 98 95 100
                  sq_m=10;
                  break;
        case(10): if(MoveRdy){
                    wmove(0x0,0x0);
                    sq_m=11;               
                 }break;
        case(11): if(Halted){
                  wmove(0x18,0x18); sq_m=2;
                  }break;
       } // end switch  
    }  // end ms
  }  // end while
}

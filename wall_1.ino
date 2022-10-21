#include "carkar.h"

void wall_1(){
byte spdL, spdR;
int8_t rangeL0, rangeR0, tmp, sq_m,dervL, dervR;
int avgsumL, avgsumR;
int errL, errR;
  sq_m=0;
  rangeL0=0;
  avgsumL=dervL=0;
  while(1){
    encoders();
    ms=millis();
    while(c_odo){c_odo--; odosum++; if(odosum==odoref) b_odo=1;}
    if(ms > msref){
      msref=ms+30;      
      if(rung) rung--; // running
      acq_sensors();
      errL=range[0]-120;   // far:pos near:neg   -78..135;
      errL/=12;     // 10 12 16
      errR=range[2]-120;
      errR/=12;     // 10 12 16
      tmp=rangeL0-range[0];
      rangeL0=range[0];
      avgsumL*=10;
      avgsumL/=20;
      avgsumL+=tmp;      
      dervL=avgsumL/3;      // 3 2
      Serial.println(dervL);
      tmp=rangeR0-range[2];
      rangeR0=range[2];
      avgsumR*=10;
      avgsumR/=20;      
      avgsumR+=tmp;
      dervR=avgsumR/3;      // 3 2      
 //     Serial.println(errL);
      ledBlink();
      if(sq_m && button()){wmove(0x0,0x0); sq_m=0;} 
      switch(sq_m){
       case(0): if(button()){
                    wmove(0x18,0x18);
                    O_Set(100); 
                    sq_m=1;
                  }break;
       case(1): if(MoveRdy)sq_m=2; break;
       case(2): if(range[1] < 80){wmove(0x0,0x0); sq_m=3; break;} // stop by front
                  if(errL){
                    spdL=0x18-errL;
                    spdL+=dervL;
                    if(spdL < 0xA) spdL=0xA;
                    spdR=0x18+errL;
                    spdR-=dervL;
                    if(spdR > 0x30) spdR=0x30;
                    wmove(spdL,spdR);    // 0x18
                  }
                  else wmove(0x18,0x18);                               
                break;
       case(3): if(Halted) sq_m=4; break;
//       case(3): if(Halted) sq_m=0; break;
       
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
       case(7): if(range[1] < 80){wmove(0x0,0x0); sq_m=8; break;}   // stop by front       
                 if(errR){
                    spdL=0x18+errR;
                    spdL-=dervR;    // +??                    
                    if(spdL > 0x30) spdL=0x30;
                    spdR=0x18-errR;
                    spdR+=dervR;    // - ??
                    if(spdR < 0xA) spdR=0xA;
                    wmove(spdL,spdR);    // 0x18                  
                 }
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

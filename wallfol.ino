#include "carkar.h"

//#define limHi 0x30
//#define limLo 0xA
// range[x]  [40..255]
//==== dipswitch setting 001 ====
void wall_1(){
uint8_t spdL, spdR;
uint8_t tmp;
int8_t rangeL0, rangeR0, sq_m, dervL, dervR;
int errLS, errRS;   // error LeftSide,RightSide
int kP, kD;
bool ONoff, sel_Pd; // select 1:kP 0:kD
  sq_m=0;
  rangeL0=0;
  dervL=dervR=0;
  kP=6;   //12
  kD=5;  //10;  //10;
  ONoff=sel_Pd=0;
  redled(0); // RedLed Off
  while(1){
    encoders();
    while(c_odo){c_odo--; odosum++; if(odosum==odoref) b_odo=1;}
    ms=millis(); 
    if(ms > msref){
      msref=ms+30;
      if(getRC5()){
        Serial.println(rc5_msg);
        if(rc5_msg==254) ONoff=!ONoff;
        if(rc5_msg==255){
          sel_Pd=!sel_Pd;
          redled(sel_Pd);     // on:kP  off:kD         
        }else if(sel_Pd){
          kP=rc5_msg;
          Serial.print("kP"); Serial.println(kP);
        }else{
          kD=rc5_msg;
          Serial.print("kD"); Serial.println(kD);
        }
      }      
      if(rung) rung--; // running
      acq_sensors();
      errLS=range[Lft]-120;   // far:pos near:neg   -78..135;
      errLS/=kP;    //   12;     // kP    
      errRS=range[Rgt]-120;
      errRS/=kP;    //12;     // kP
      dervL=rangeL0-range[Lft];   // derivative left sensor
      rangeL0=range[Lft];
      dervL/=kD;
      dervR=rangeR0-range[Rgt];   // derivative right sensor
      rangeR0=range[Rgt];
      dervR/=kD;
      if(!sel_Pd){dervR=0; dervL=0;}
       ledBlink();
      if(sq_m && button()){wmove(0x0,0x0); sq_m=0;} 
      switch(sq_m){
       case(0): if(button()){
                    wmove(0x18,0x18);
                    O_Set(100);
                    if(range[Lft] < range[Rgt]) sq_m=1; else sq_m=2;
                  }break;
       case(1): if(MoveRdy)sq_m=3; break;
       case(2): if(MoveRdy)sq_m=8; break;       
//===== track on the leftside wall
       case(3): if(range[Ctr] < 80){wmove(0x0,0x0); sq_m=4; break;} // stop by front
                if(errLS){    /* error LeftSide */
                  spdL=0x18-errLS;
                  if(dervL > 0) spdL += dervL;
                  else if(dervL < 0){
                    tmp=-dervL;       // tmp-> pos
                    if(spdL > tmp) spdL-=tmp;
                    else spdL=2;
                  }
                  spdR=0x18+errLS;
                  if(dervL > 0){
                    if(dervL >= spdR) spdR=2;
                    else spdR-=dervL;
                  }else if(dervL <0) spdR-=dervL;
                  wmove(spdL,spdR);
                } else wmove(0x18,0x18);
 //               if(!sel_Pd){
 //                 Serial.print(spdL); Serial.print("  "); Serial.print(spdR);
 //                 Serial.print("  "); Serial.println(dervL);}
                break;
       case(4): if(Halted) sq_m=5; break;
       case(5): wmove(0x11,0x91);
                O_Set(99);   // 98 95 100
                sq_m=6;
                break;
       case(6): if(MoveRdy){
                   wmove(0x0,0x0);
                   sq_m=7;               
                }break;
       case(7): if(Halted){
                  wmove(0x18,0x18); sq_m=8;
                }break;
//===== track on the rightside wall             
       case(8): if(range[Ctr] < 80){wmove(0x0,0x0); sq_m=9; break;}   // stop by front
                if(errRS){    /* error RightSide  */
                    spdL=0x18+errRS;      // proportional error
                    if(dervR > 0){
                      if(dervR >= spdL) spdL=2;
                      else spdL-=dervR;
                    }else if(dervR < 0) spdL-=dervR;
                    spdR=0x18-errRS;
                    if(dervR >0 ){
                      spdR+=dervR;                    
                    }else if(dervR < 0){
                      tmp=-dervR;
                      if(spdR>tmp) spdR-=tmp;
                      else spdR=2;
                    }
                    wmove(spdL,spdR);
                 } else wmove(0x18,0x18);
 //                if(!sel_Pd){
 //                 Serial.print(spdL); Serial.print("  "); Serial.print(spdR);
 //                 Serial.print("  "); Serial.println(dervL);}
                 break;
        case(9): if(Halted) sq_m=10; break;
        case(10): wmove(0x11,0x91);
                  O_Set(99);   // 98 95 100
                  sq_m=11;
                  break;
        case(11): if(MoveRdy){
                    wmove(0x0,0x0);
                    sq_m=12;               
                 }break;
        case(12): if(Halted){
                  wmove(0x18,0x18); sq_m=3;
                  }break;
        case(20): if(MoveRdy) sq_m=8; break;         
       } // end switch  
    }  // end ms
  }  // end while
}

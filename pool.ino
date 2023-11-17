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
void LineFol(void){
uint8_t sts, i, sq0;   // wheelRightExtra
uint16_t tmp;
bool go, mov;  
uint8_t ld[10], *p_ld, ipol, kk;
int8_t nn, nn0;
int16_t errr, errr0, derv;
    errr=go=sq0=0;
    errr0=0x80;
    while(1){
       ms=millis();
       if(ms > msref){
        msref=ms+40;
        if(getRC5()){
           Serial.println(rc5_msg);
        }
        if(button()) go = !go;
        ledBlink();
        if(go){
          mov=1;
          p_ld=ld;
          sts= Wire.requestFrom(LINE_I2C_ADDRESS,9);
            while(Wire.available()) {
              *p_ld++ = Wire.read();
            }
            p_ld=&ld[1];
            for(i=1;i<9;i++){       // ld[0]..ld[7] sensorArray
              *(p_ld-1)=*p_ld;
              *p_ld++;
            }
            for(i=0 ; i < 8 ; i++){   //compliant HCC
              tmp=ld[i];
              if(tmp > 29) tmp -= 30; else tmp= 0;
              tmp *= 170;
              tmp /= 220;
              tmp += 80;
              ld[i] = (uint8_t) tmp;   
            }
            for(i=0; i<8; i++){       // black line -> high output 
              ld[i]=256-ld[i];
            }
//            for(i=0; i<7; i++){
//              Serial.print(ld[i]);
//              Serial.print("   ");
//            }
//            Serial.println(ld[7]);
            sq0=1;
            for(i=0;i<8;i++){
              if(ld[i] > 12) break;
              sq0++;              
            }
 //           Serial.println(sq0);
            ipol=0;
            kk=0;
            switch(sq0){
              case(1):  if((ld[2] <15) && (ld[3] <15)) kk=1;
                        if((ld[0] >170) && (ld[1] <80) && kk) break;
                        else if ((ld[1] > 160) && kk) ipol=1;
                        else if ((ld[0] > 160) && (ld[1] > 160) && (ld[3] < 15)) ipol=2;
                        else if ((ld[3] < 60) && (ld[1] > 60) && !kk) ipol=3;
                        break;    
              case(2):  if(ld[1] < 100) ipol=1; break;
              case(3):  if(ld[2] < 120) ipol=1; break;
              case(4):  if(ld[3] < 130) ipol=1; break;
              case(5):  if(ld[4] < 130) ipol=1; break;
              case(6):  if(ld[5] < 130) ipol=1; break;
              case(7):  if(ld[6] < 160) ipol=1; break;
              default:  break;        // out of range ????????
          }
          nn=(int8_t)sq0;   // 1...9
          if(nn==1){
              nn += ipol;
          }else if(nn < 10){    // (2-1)*2 =2 || (2-1)*2 + ipol =3 || (3-1)*2 =4  etc.
              nn-=1;
              nn*=2;            
              nn += ipol;
              nn += 3;          // scale range 1..17
          }
          errr=(int16_t)nn-9;      //   scale range -8..0..+8
//          Serial.print(nn); Serial.print("  "); Serial.println(errr);
          derv=errr-nn0;
          derv*=8;
//          Serial.print(derv);Serial.print("  "); Serial.print(errr);Serial.print("  "); Serial.println(nn0);
          nn0=errr;
  
//          Serial.print(errr); Serial.print("  "); Serial.println(errr0);
//            if(errr0 != errr){
                errr0=errr;
                errr *= 15;
                errr /= 5;
//                Serial.print(80+errr); Serial.print("  "); Serial.println(80+errr+derv);
                analogWrite(PWML, 80-(int8_t)errr -(int8_t) derv );
                analogWrite(PWMR, 88+(int8_t)errr + (int8_t)derv);
//          }
       }else{
          if(mov){analogWrite(PWML, 0); analogWrite(PWMR, 0); mov=0;} // end go         
       }
    }     // end ms 
  }       // end while
}  

 


//==== dipswitch setting 011 ===
void LineFol(void){             // LEFT/RIGHT TRACKING   Lr 1:Left   0:Right
uint8_t getErr(uint8_t *, bool);
uint8_t nodes(uint8_t *, bool);
uint8_t xdet;           // node_statusbyte  [0] finish  [1] linecrossdetect
uint8_t sqm, sqm0;      // master sequencer        
const uint8_t offs[8]={175,200,200,200,235,218,210,185};  
uint8_t sts, i;
bool togg, Lr; 
uint8_t ld[10], *p_ld, ipol;
int8_t motPwr;
int16_t hdg, hdg0, errr, errr0, derv, kP, kD;
    togg=0;
    xdet=0;
    sqm0=1;
    sqm=0x80;
    Lr=1;         //  default tracking on Left edge
    hdg0=0x80;
    kP=14;
    kD=16;        //  mod 09-12-23  (15)
    motPwr=105;   //  ~50cm/sec      mod 09-12-23 (95)
    Serial.println("START");
    while(1){
      encoders();
      ms=millis();
      if(ms > msref){
        msref=ms+40;
        if(getRC5()){
          if(rc5_msg==0xFF){
            togg=!togg;
            redled(togg);
            Lr=togg?0:1;
          }   
        }
        ledBlink();   //  onboard yellow
        while(c_odo){c_odo--; odosum++; if(odosum==odoref) b_odo=1;} 
        p_ld=ld;
        sts= Wire.requestFrom(LINE_I2C_ADDRESS,9);
        while(Wire.available()){
          *p_ld++ = Wire.read();
        }
        p_ld=&ld[1];
        for(i=1;i<9;i++){       // ld[0]..ld[7] sensorArray
          *(p_ld-1)=*p_ld;
          p_ld++;
        }
        for(i=0; i<8; i++){       // black line -> high output
          if(offs[i] >= ld[i])  
            ld[i]=offs[i]-ld[i];
          else ld[i]=0;              
        }
        p_ld=&ld[0]; 
        errr=(int16_t)getErr(p_ld,Lr);
        errr-=14;          
        derv=errr-errr0;
        errr0=errr;
        derv*=kD;
        if(derv < -80) derv = -80;       // limit derivative
        else if(derv > 80) derv = 80;    // limit derivative
        errr *= kP;
        errr /= 5;
        hdg=errr+derv;
//        if(sqm){
//          for(i=0;i<7;i++){
//            Serial.print(ld[i]); Serial.print("  ");
//          }
//          Serial.println(ld[7]);
//        }
//        if(sqm != sqm0) {Serial.println(sqm); sqm0=sqm;}
        switch(sqm){
          case(0): if(button()) sqm=1; break;
          case(1): if(xdet & 0x1) {        //==== FIRST CROSSING ===
                      wmove(0x0,0x90);
                      O_Set(0x7); sqm=2;    // some backtrack leftwheel
                      break; 
                   }
                   if(hdg != hdg0){
                      hdg0=hdg;
                      analogWrite(PWML, motPwr - (int8_t)hdg);
                      analogWrite(PWMR, motPwr + (int8_t) hdg + 8);
                   } break;
          case(2): if(MoveRdy){           // start CW rotation
                        wmove(0x10,0x90);
                        O_Set(0x12);
                        sqm=3;
                     } break;
          case(3): if(MoveRdy) sqm=4;
                    break;
          case(4): if(hdg > 0){            // CW while heading negative
                      wmove(0x14,0x15);   // (0x14,0x15) -> extra to top rotation
                      O_Set(0x3);
                      sqm=5;
                   } break;
          case(5): if(MoveRdy){sqm=6; break;}
                      
/* first linecrossing done, proceed   */

          case(6): if(xdet & 0x1) {        // === SECOND CROSSING ===
                      wmove(0x0,0x90);
                      O_Set(0x7); sqm=7;  // wheelbase on x centre   
                      break; 
                    }
                    if(hdg != hdg0){
                      hdg0=hdg;
                      analogWrite(PWML, motPwr - (int8_t)hdg);
                      analogWrite(PWMR, motPwr + (int8_t) hdg + 8);
                    } break;
          case(7): if(MoveRdy){             // 2
                      wmove(0x10,0x90);
                      O_Set(0x12);          // rotate 56 dgr CW
                      sqm=8;
                     } break;
          case(8): if(MoveRdy) sqm=9;
                    break;                    
          case(9): if(hdg > 0){            // CW while heading negative
                      wmove(0x14,0x15);  // 14,14
                      O_Set(0x3);
                      sqm=10;
                   }                    
                   break; 
/* second linecrossing done, proceed to finish   */                    
          case(10):  if(xdet & 0x2){sqm=0x80; break;}
                    if(hdg != hdg0){
                      hdg0=hdg;
                      analogWrite(PWML, motPwr - (int8_t)hdg);
                      analogWrite(PWMR, motPwr + (int8_t) hdg + 8);
                    } break;
          case(0x80): wmove(0x0,0x0);
                      sqm=0;
                      break;
       }
       if(sqm && button()) sqm=0x80;      // button stop
       if(sqm) xdet=nodes(p_ld, Lr); else xdet=0;
    }     // end ms 
  }       // end while
}         // end LineFol
  
// xdet[n]        evaluate nodes  
// [7]
// [6]
// [5]
// [4]
// [3]
// [2]
// [1]    end line
// [0]    crossing detect

// holdoff: suppress successive xdet events for 1 second when triggered

uint8_t nodes(uint8_t *p_px, bool Lr){
static uint8_t holdoff=0, strght=0, more=0;
static uint8_t sqn=0;
static uint8_t px0_0, px7_0;
uint8_t xdet;
  xdet=0;
  px0_0=px7_0=0;
  if(holdoff==0){
    if(((*(p_px+4) + *(p_px+3)) > 300) && ((*p_px + *(p_px+7))) <  90) strght=20; // straight line
    if(strght && (*(p_px+1) < 100) && (*(p_px+3) < 100) && (*(p_px+4) < 100) && (*(p_px+6) < 100)) xdet|=0x2;
    switch(sqn){
        case(0): if(strght) strght--; 
                 if(((*(p_px+7)-px7_0) > 90) && strght){more=10; sqn=1;}  // linecrossing may be in sight
                 break;
        case(1): more--; if(more==0){sqn=2; break;}   // ? linecrossing detect is false event
                  if((*p_px-px0_0) > 90){
                      xdet|=0x1; sqn=0;               // linecrossing is true
                  } break;        
        case(2): holdoff=20; sqn=0; break;
     }
      px7_0 = *(p_px+7);  // derivative
      px0_0 = *p_px;      // derivative
      if(xdet) holdoff=25;
      if(holdoff) redled(1); else redled(0);
      
  } //end holdoff
//  Serial.print(strght); Serial.print("  ");/* Serial.print(holdoff); Serial.print("  "); */
//  Serial.print(sqn); Serial.print("  "); Serial.println(xdet);
//  Serial.print(*(p_px+4) + *(p_px+3)); Serial.print("  "); Serial.println(*p_px + *(p_px+7));
  if(holdoff) holdoff--;  
  return(xdet);  
}

/*
 lineError range  -13..0..13
 linesensor provides a linear 8 bytes array  pointed by *p_px    *(p_px+0) ..*(p_px+7)
 array to be evaluated from low to high *(p_px+0) -> *(p_px+7) focussed on right edge of line
 or evaluated from high to low  *(p_px+7) -> *(p_px+0) focussed on left edge of line
 left/right evaluation results in the option to swing off an abatement as desired
 */

uint8_t getErr(uint8_t *p_px, bool Lr){      // pointer naar pixel
uint8_t sq0, i, ipol, nn=0;
int16_t prfl;
  sq0=1;  
  if(Lr){      // TRACK LEFT LINESIDE VERSION    
      for(i=7; i>0 ;i--){
        if(*(p_px+i) > 100) break;
        sq0++;              
      }
      if(sq0 > 7) sq0=7;
      ipol=4;
      switch(sq0){
        case(1):  prfl=210 + *(p_px+7) - *(p_px +4)- *(p_px+5);
                  if(prfl > 336) ipol=1;
                  else if(prfl > 290) ipol=2;
                  else if(prfl > 215) ipol=3;
                  else if(prfl > 113) ipol=4;              
                  break;    
        case(2):  prfl=160 + *(p_px+6)+ *(p_px+7) - *(p_px+3) -*(p_px+4);
                  if(prfl > 250) ipol=1;
                  else if(prfl > 180) ipol=2;
                  else if(prfl > 100) ipol=3;
                  break;
        case(3):  prfl=150 + *(p_px+5) + *(p_px+6) -*(p_px+2) - *(p_px+3);
                  if(prfl > 280) ipol=1;
                  else if(prfl > 170) ipol=2;
                  else if(prfl > 80) ipol=3;
                  break;
        case(4):  prfl=80 + *(p_px+4) + *(p_px+5) -*(p_px+2);
                  if(prfl > 245) ipol=1;
                  else if(prfl > 175) ipol=2;
                  else if(prfl > 100) ipol=3;
                  break;
        case(5): prfl=80 + *(p_px+3) + *(p_px+4) -*(p_px+1);
                  if(prfl > 270) ipol=1;
                  else if(prfl > 180) ipol=2;
                  else if(prfl > 100) ipol=3;
                  break;
        case(6):  prfl=60 + *(p_px+2) + *(p_px+3)- *(p_px+0);
                  if(prfl > 212) ipol=1;
                  else if(prfl > 124) ipol=2;
                  else if(prfl > 73) ipol=3;
                  break;
        case(7):  prfl=10 + *(p_px+2);
                  if(prfl > 80) ipol=1;
                  else if(prfl > 60) ipol=2;
                  else ipol=3;                        
                  break;
        default:  break;
    }
    nn=(int8_t)sq0;   // 1...7
    if(nn==1){
        nn += ipol;
        nn += 1;
        nn=30-nn;
    }else if (nn < 9){
        nn-=1;
        nn*=4;            
        nn += ipol;
        nn += 2;
        nn=30-nn;
    }
  }else{    // TRACK RIGHT LINESIDE VERSION        
    for(i=0;i<7;i++){
      if(*(p_px+i) > 100) break;
      sq0++;              
    }
    if(sq0 > 7) sq0=7;
    ipol=0;
    switch(sq0){
      case(1):  prfl=150 + *(p_px+3) + *(p_px+2) - *p_px;
                if(prfl > 215) ipol=4;
                else if(prfl > 150) ipol=3;
                else if(prfl > 90) ipol=2;
                else if(prfl > 50) ipol=1;              
                break;    
      case(2):  prfl=250 + *(p_px+3) - *(p_px+1) - *p_px;
                if(prfl > 290) ipol=3;
                else if(prfl > 210) ipol=2;
                else if(prfl > 120) ipol=1;
                break;
      case(3):  prfl=50 + *(p_px+4) + *(p_px+5) - *p_px;
                if(prfl > 260) ipol=3;
                else if(prfl > 150) ipol=2;
                else if(prfl > 80) ipol=1;
                break;
      case(4):  prfl=50 + *(p_px+5) + *(p_px+6) - *(p_px+2);
                if(prfl > 285) ipol=3;
                else if(prfl > 250) ipol=2;
                else if(prfl > 150) ipol=1;
                break;
      case(5):  prfl=50 + *(p_px+6) + *(p_px+7) - *(p_px+3);
                if(prfl > 290) ipol=3;
                else if(prfl > 230) ipol=2;
                else if(prfl > 160) ipol=1;
                break;
      case(6):  prfl=200 + *(p_px+7) - *(p_px+5) - *(p_px+4);
                if(prfl > 220) ipol=2;
                else if(prfl > 150) ipol=1;
                break;
      case(7):  prfl=*(p_px+5);
                if(prfl < 20) ipol=2;
                else if(prfl < 50) ipol=1;                        
                break;
      default:  break;
    }
    nn=(int8_t)sq0;   // 1...7
    if(nn==1){
        nn += ipol;
    }else{
        nn-=1;
        nn*=4;            
        nn += ipol;
        nn += 2;
        if(nn > 25) nn--;
    }      
  }
    return(nn);
}

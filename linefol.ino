
/*
      refer carkar.h and set
      DISSEL  0 for standard
      DISSEL  1 for dissel compliant

*/

//==== dipswitch setting 011 ===
void LineFol(void){             // LEFT/RIGHT TRACKING   Lr 1:Left   0:Right
uint8_t getErr(uint8_t *, bool);
uint8_t ntype(uint8_t *, bool);   // node type
uint8_t node(uint8_t, uint16_t);   // node handle
uint8_t dissel;
uint8_t xdet;           // node_statusbyte  [0] finish  [1] linecrossdetect
uint8_t sqm, sqm0;      // master sequencer        
const uint8_t offs[8]={175,200,200,200,235,218,210,185};  
uint8_t sts, i;
bool togg, Lr; 
uint8_t ld[10], *p_ld, ipol;
int8_t motPwr;
int16_t tmp, hdg, hdg0, errr, errr0, derv, kP, kD;
    togg=0;
    xdet=0;
    sqm0=1;
    sqm=0x80;
    Lr=1;         //  default tracking on Left edge
    hdg0=0x80;
    kP=14;
    kD=16;        //  mod 09-12-23  (15)
    motPwr=105;   //  ~50cm/sec      mod 09-12-23 (95)
    dissel=DISSEL;
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
          if(dissel){             // comply 'dissel' with 'standard'
            tmp=(int16_t)ld[i];
            tmp*=12;
            tmp/=10;
            if(tmp > 255) tmp=255;
            ld[i]=uint8_t(tmp);            
          }                        
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
//                    node(xrght, hdg);
                      node(xlft, hdg);
                      sqm=2;  
                      break; 
                   }
                   if(hdg != hdg0){
                      hdg0=hdg;
                      analogWrite(PWML, motPwr - (int8_t)hdg);
                      analogWrite(PWMR, motPwr + (int8_t) hdg + 8);
                   } break;
          case(2): if(node(nop, hdg)) sqm=6; break;
          
          case(6): if(xdet & 0x1) {        // === SECOND CROSSING ===
//                    node(xrght, hdg);
                      node(xlft, hdg);
                      sqm=7;  
                      break; 
                    }
                    if(hdg != hdg0){
                      hdg0=hdg;
                      analogWrite(PWML, motPwr - (int8_t)hdg);
                      analogWrite(PWMR, motPwr + (int8_t) hdg + 8);
                    } break;
          case(7):  if(node(nop, hdg)) sqm=10; break;          
                 
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
       if(sqm) xdet=ntype(p_ld, Lr); else xdet=0;
    }     // end ms 
  }       // end while
}         // end LineFol
  
// xdet[n]        evaluate node type  
// [7]
// [6]
// [5]
// [4]
// [3]
// [2]
// [1]    end line
// [0]    crossing detect

// holdoff: suppress successive xdet events for 1 second when triggered

uint8_t ntype(uint8_t *p_px, bool Lr){
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

uint8_t node(uint8_t type, uint16_t hdg){
static uint8_t sqn=0;
static uint8_t sqn0=0x80;
  if(type) sqn=type;
  switch(sqn){
    case(nop): return(0); break;
    case(xrght): wmove(0x0, 0x90); O_Set(0x7); sqn=10; break;
    case(xlft): wmove(0x90, 0x0); O_Set(0x7); sqn=20; break;
    case(trgt150): break;
    case(tlft150): break;
    case(trgt160): break;
    case(tlft160): break;
    
    case(10): if(MoveRdy){                    // X turn right
                wmove(0x10,0x90); O_Set(0x12); sqn=11;
               } break;
    case(11): if(MoveRdy) sqn=12; break;
    case(12): if(hdg > 0){
               wmove(0x14,0x15); O_Set(0x3); sqn=13;
              } break;
    case(13): if(MoveRdy){sqn=nop; return(1);} break;

    case(20): if(MoveRdy) {wmove(0x90,0x10); O_Set(0x48); sqn=21;} break;
    case(21): if(MoveRdy){
                wmove(0x14,0x14); O_Set(8); sqn=22;}
                break;
    case(22): if(MoveRdy){wmove(0x0,0x0); return(1);}
                break;                
    default: break;
  }
  return(0);  
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
                  if(prfl > 325) ipol=1;
                  else if(prfl > 250) ipol=2;
                  else if(prfl > 145) ipol=3;
                  break;    
        case(2):  prfl=160 + *(p_px+6)+ *(p_px+7) - *(p_px+3) -*(p_px+4);
                  if(prfl > 240) ipol=1;
                  else if(prfl > 160) ipol=2;
                  else if(prfl > 80) ipol=3;
                  break;
        case(3):  prfl=150 + *(p_px+5) + *(p_px+6) -*(p_px+2) - *(p_px+3);
                  if(prfl > 285) ipol=1;
                  else if(prfl > 200) ipol=2;
                  else if(prfl > 105) ipol=3;
                  break;
        case(4):  prfl=80 + *(p_px+4) + *(p_px+5) -*(p_px+2);
                  if(prfl > 240) ipol=1;
                  else if(prfl > 170) ipol=2;
                  else if(prfl > 100) ipol=3;
                  break;
        case(5): prfl=80 + *(p_px+3) + *(p_px+4) -*(p_px+1);
                  if(prfl > 260) ipol=1;
                  else if(prfl > 180) ipol=2;
                  else if(prfl > 100) ipol=3;
                  break;
        case(6):  prfl=60 + *(p_px+2) + *(p_px+3)- *(p_px+0);
                  if(prfl > 205) ipol=1;
                  else if(prfl > 135) ipol=2;
                  else if(prfl > 60) ipol=3;
                  break;
        case(7):  prfl=10 + *(p_px+2);
                  if(prfl > 60) ipol=1;
                  else if(prfl > 40) ipol=2;
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
    ipol=1;
    switch(sq0){
      case(1):  prfl=150 + *(p_px+3) + *(p_px+2) - *p_px;
                if(prfl > 215) ipol=4;
                else if(prfl > 150) ipol=3;
                else if(prfl > 90) ipol=2;
                 break;    
      case(2):  prfl=250 + *(p_px+3) - *(p_px+1) - *p_px;
                if(prfl > 260) ipol=4;
                else if(prfl > 190) ipol=3;
                else if(prfl > 100) ipol=2;
                break;
      case(3):  prfl=65 + *(p_px+4) + *(p_px+5) - *(p_px+1);
                if(prfl > 260) ipol=4;
                else if(prfl > 150) ipol=3;
                else if(prfl > 80) ipol=2;
                break;
      case(4):  prfl= 20 + *(p_px+5) + *(p_px+6) - *(p_px+2);
                if(prfl > 270) ipol=4;
                else if(prfl > 255) ipol=3;
                else if(prfl > 160) ipol=2;
                break;
      case(5):  prfl=30 + *(p_px+6) + *(p_px+7) - *(p_px+3);
                if(prfl > 250) ipol=4;
                else if(prfl > 200) ipol=3;
                else if(prfl > 115) ipol=2;
                break;
      case(6):  prfl=200 + *(p_px+7) - *(p_px+5) - *(p_px+4);
                if(prfl > 200) ipol=4;
                else if(prfl > 140) ipol=3;
                else if(prfl > 80) ipol=2;
                break;
      case(7):  prfl=*(p_px+5);
                if(prfl < 35) ipol=3;
                else if(prfl < 65) ipol=2;                        
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
        nn += 1;
     }      
  }
    return(nn);
}


/*
      refer carkar.h and set
      VINYL   0 for standard
      VINYL   1 HCC robotica compliant
      10-04-24
      20-04-24 include de Nayer revisions
*/


//==== dipswitch setting 011 ===
void LineFol(bool sel){             // 
uint8_t getErr(uint8_t *, bool);
uint8_t ntype(uint8_t *, bool, bool);   // node type
uint8_t node(uint8_t, int8_t, int8_t);   // node handle
void wheels(uint8_t , int8_t);
uint8_t vinyl;
uint8_t sql, sqm, sqm0;      // master sequencer        
const uint8_t offs[8]={175,200,200,200,235,218,210,185};  
uint8_t sts, i, susp;
bool togg, togy, Lr, Lr0, enb_xy;
uint8_t ld[10], *p_ld, ipol;
uint8_t motPwr;
int16_t tmp, errr, errr0, derv, kP, kD;
int8_t err8;
int8_t hdg;
int16_t ag_sum, ag_tmp, ag_res, ag_drv;  
    sql=0;
    togg=1;
    togy=0;
    sqm0=1;
    sqm=0;
    ag_tmp=0;
    Lr=1;         //  default tracking on left edge
    kP=13;        // 14
    kD=16;        //  mod 09-12-23  (15)
    motPwr=105;   //  ~50cm/sec      mod 09-12-23 (95)
    vinyl=VINYL;
    redled(!togg);
    Serial.println("START");
    while(1){
      encoders();
      ms=millis();
      if(ms > msref){
        msref=ms+40;
        if(sqm != sqm0){Serial.println(sqm); sqm0=sqm;}
        if(getRC5()){
          if(rc5_msg==0xFF){
            togg=!togg;
            redled(!togg);    // redON track right edge
            Lr=togg;
          }
          if(rc5_msg==0xFD){
            togy=!togy;
          }
        }
        ledBlink();   //  onboard yellow
        while(c_odo){c_odo--; odosum++; if(odosum==odoref) b_odo=1;} 
        p_ld=ld;    // initz pointer
        sts= Wire.requestFrom(LINE_I2C_ADDRESS,9);
        while(Wire.available()){
          *p_ld++ = Wire.read();
        }
        p_ld=&ld[1];
        for(i=1;i<9;i++){       // ld[0]..ld[7] sensorArray
          *(p_ld-1)=*p_ld;
          p_ld++;
        }
        for(i=0; i < 8; i++){       // black line -> high output
          if(offs[i] >= ld[i])  
          ld[i]=offs[i]-ld[i];
          else ld[i]=0;
          if(vinyl){             // comply 'vinyl' with 'standard'
            tmp=(int16_t)ld[i];
            tmp *= 12;
            tmp /= 10;
            if(tmp > 255) tmp=255;
            ld[i]=uint8_t(tmp);            
          }                        
        }
        p_ld=&ld[0]; 
        errr=(int16_t)getErr(p_ld,Lr);
        errr-=14;     // -13..0..+13
        err8=(int8_t)errr;     // linesensor range -13..0..13
//=== average
        ag_sum=0;
        for(i=0;i<8;i++)ag_sum += ld[i];
        ag_tmp=(ag_tmp-ag_tmp/10) + ag_sum;
        ag_res=ag_tmp/10;
        ag_drv=ag_sum - ag_res;
        enb_xy= (ag_drv > 100)? 1:0;
        if(enb_xy) ylwled(1); else ylwled(0);
//===
        kP=((err8 > 9) || (err8 < -9)) ? 18:13;   // 03-03-2024
        derv=errr-errr0;
        errr0=errr;
        derv*=kD;
        if(derv < -80) derv = -80;       // limit derivative
        else if(derv > 80) derv = 80;    // limit derivative
        errr *= kP;
        errr /= 5;        // -36..0..+36
        tmp=errr+derv;    // int16                        // pwr+tmp
        if(((int)motPwr + tmp) > 250) tmp = 250-motPwr;   // 105+160 -> tmp=250-105=145 (105+145=250)
        if(((int)motPwr + tmp) < 5)   tmp = motPwr-5;     // 105-110 -> tmp=105-5=100 (105-100=5)
        hdg = (int8_t) tmp;

        
//        hdg=(int8_t)tmp;    
//        if(hdg > (245- motPwr)) hdg = 245-motPwr;    // clip overrange
//        if(hdg > (motPwr-5)) hdg=motPwr-5;           // clip negative motPwr

        
        if(sel){        //=== EVALUATION SECTION
          switch(sqm){
            case(0): if(button()) sqm=1;
                       break;
//            case(1): Serial.print(ag_res); Serial.print("    "); Serial.println(ag_drv);
//                     if(enb_xy) ylwled(1); else ylwled(0);
//                     wheels(motPwr, hdg);                      
//                     if(button()) sqm=0;
//                     break;
#if 1                      
            case(1): if(ntype(p_ld, enb_xy, Lr) & 0x1){
                        wmove(0x84,0x84);   // active brake
                        Lr=1;               // track left edge
                        sqm=2;
                        break;
                      }
                      wheels(motPwr, hdg);
                     break;
            case(2): wmove(0x0,0x0);
                        susp=10;
                        sqm=3;
                        break;
            case(3):  susp--; if(susp==0){
                      O_Set(90);
                      wmove(0x12,0x92);
                      sqm=4;
                      break;
             case(4): if(MoveRdy){
                       wmove(0x13,0x13); 
                       O_Set(50);
                       sqm=5;
                       }
                       break; 
             case(5): if(MoveRdy) sqm=1;
                       break;         
             }
#endif  
          }
        }else{    // REAL WORLD SECTION
          switch(sqm){
          case(0): if(button()){Lr=1;  sqm=1;} break;
          case(1): if(ntype(p_ld, enb_xy, Lr) & 0x40) {  //==== swing Y4 left off ===
                      sqm=2;  
                      break; 
                   }
                   wheels(motPwr, hdg);
                   break;
         case(2): if(!(ntype(p_ld, enb_xy, Lr) & 0x40)){
                     O_Set(0x20); sqm=3;
                     break;                  
                   }
                   wheels(motPwr, hdg);
                   break;
         case(3):  if(MoveRdy){             // switch to rightedge tracking
                      Lr=0; sqm=4; break;   // swing Y4 left off  done ===
                    }
                    wheels(motPwr,hdg);
                    break;                    
         case(4): if(ntype(p_ld, enb_xy, Lr)& 0x10) {        //==== X CW
                      node(xrght55, err8, hdg);
                      sqm=5;  
                      break; 
                   }
                    wheels(motPwr,hdg);
                    break;
          case(5): if(node(nop, err8, hdg)) sqm=6;
                    break;
          case(6):  O_Set(0xA); sqm=7;
                    break;
          case(7): if(MoveRdy){
                      Lr=1;
                      sqm=8;
                      break;
                    }
                    wheels(motPwr, hdg);
                    break;                 // end X CW
          case(8): if(ntype(p_ld, enb_xy, Lr) & 0x40) {        //  === Y3 CW
                      node(CW_Y3, err8, hdg);
                      sqm=9;  
                      break; 
                   }
                    wheels(motPwr,hdg);
                    break;
          case(9):  if(node(nop, err8, hdg)) sqm=10;
                    break;
          case(10): O_Set(0x60); sqm=11;            // 15 cm suspend
                    break;                          // checking node Y4
          case(11): if(MoveRdy) {sqm=12; break;}
                    wheels(motPwr, hdg);
                    break;
          case(12): if(ntype(p_ld, enb_xy, Lr) & 0x40){  //   Y4 CW
                      node(CW_Y4, err8, hdg);
                      sqm=13;
                      break;
                     }     
                    wheels(motPwr, hdg);
                    break;                    
          case(13): if(node(nop, err8, hdg))
                    sqm=14;
                    break;
          case(14): if(!(ntype(p_ld, enb_xy, Lr) &0x40)){
                       O_Set(0x20); sqm=15;           //   5cm
                     break;                           // end swing Y4 left off ===
                   }
                   wheels(motPwr, hdg);
                   break;
          case(15): if(MoveRdy){Lr=0; sqm=16; break;}  //switch to rightedge tracking
                    wheels(motPwr, hdg);
                    break;                   
          case(16): if(ntype(p_ld, enb_xy, Lr)& 0x10) {     //==== X straight ahead
                      O_Set(0x42);
                      ylwled(1);
                      wmove(0x18,0x18);  // 0x12 20.04
                      sqm=17;
                      break; 
                   }
                    wheels(motPwr,hdg);
                    break;          
          case(17): if(MoveRdy) {
                      ylwled(0);
                      sqm=18;
                      }
                      break;
          case(18): if(ntype(p_ld, enb_xy, Lr) & 0x80){             // === Y2 CCW 
                        node(CCW_Y2, err8, hdg);
                        sqm=19;
                        break;  
                     }
                     wheels(motPwr, hdg);
                     break;
           case(19): if(node(nop, err8, hdg)) sqm=20;
                      break;
           case(20): O_Set(0x60); sqm=21; break;         
           case(21): if(MoveRdy) {sqm=22; break;}
                     wheels(motPwr, hdg);
                     break;
           case(22): if(ntype(p_ld, enb_xy, Lr) & 0x80){      // ===Y1 CCW
                      node(CCW_Y1, err8, hdg);
                      sqm=23;
                      break;
                    }
                     wheels(motPwr, hdg);
                     break;
            case(23): if(node(nop, err8, hdg)) sqm=30; break;
          
            case(30): if(ntype(p_ld, enb_xy, Lr) & 0x20){    //==== X straight ahead
                      O_Set(0x42);
                      ylwled(1);
                      Lr=1;
                      wmove(0x18,0x18);   // 0x12 20.04
                      sqm=32;
                      break;
                    }
                    wheels(motPwr,hdg);
                    break;
           case(32): if(MoveRdy){
                       ylwled(0);
                       O_Set(0xF0);
                        sqm=33;
                      }
                      break;
          case(33): if(MoveRdy){
                      Lr=0; 
                      sqm=34;
                      break;
                    }
                    wheels(motPwr, hdg);
                    break;
//----------------------------------------- end line FINISH                   
          case(34): if(ntype(p_ld, enb_xy, Lr) & 0x1){
                      wmove(0x0,0x0);
                      susp=10;
                      sqm=35;
                      break;
                    }
                    wheels(motPwr, hdg);
                    break;
          case(35):  susp--;
                     if(susp==0){
                        O_Set(90);
                        wmove(0x12,0x92);
                        sqm=36;
                     }   
                     break;
           case(36): if(MoveRdy){
                       wmove(0x13,0x13);
                         O_Set(50);
                       sqm=37;
                       }
                       break; 
           case(37): if(MoveRdy){
                        sqm=38;
                        Lr=1;              // track on left edge 
                     }  
                     break; 
//----------------------------------------- proceed to  START                     
           case(38): if(ntype(p_ld, enb_xy, Lr) & 0x1){
                      wmove(0x0,0x0);
                      susp=10;
                      sqm=39;
                      break;
                    }
                    wheels(motPwr, hdg);
                    break;          
          case(39): susp--;
                    if(susp==0){
                       O_Set(90);
                       wmove(0x12,0x92);
                       sqm=40;
                    }   
                    break;
          case(40): if(MoveRdy){
                       wmove(0x13,0x13);
                         O_Set(50);
                       sqm=41;
                       }
                       break; 
          case(41): if(MoveRdy){
                        sqm=1;
                        Lr=1;              
                     }  
                     break;     // ==== START
          
          case(0x80): wmove(0x0,0x0);
                      sqm=0;
                      break;
                    
          }
      }   // end sel
    if(sqm && button()) sqm=0x80;      // button stop
    if(Lr != Lr0) {redled(!Lr); Lr0=Lr;}
    }     // end ms 
  }       // end while
}         // end LineFol


//======== identify node type ==============
//
// [7]  Y junction detect on left side 
// [6]  Y junction detect on right side
// [5]  X crossing
// [4]  X crossing
// [3]
// [2]  
// [1]  enter X or Y and ready for turn
// [0]  end line detect


uint8_t ntype(uint8_t *p_px, bool enb_xy , bool Lr){
uint16_t sum;
uint8_t xdet;
uint8_t ii, endln;
  xdet=0;
  if(enb_xy){
      if(*p_px > 120) xdet|=0x40;         //   Y junction detect on right sensor
      if(*(p_px+7) > 120) xdet|=0x80;     //   Y junction detect on left sensor    
      sum=*p_px + *(p_px+1);
      if(sum > 200 ) xdet |= 0x20;        //   X cross detect on right side 
      sum=*(p_px+6) + *(p_px+7);        
      if(sum > 200) xdet|= 0x10;          //   X cross detect on left side
  }
// end line detect
// endln : blackline > 150   :fullwhite < 50   threshold 90
  endln=0;
  for(ii=0; ii<8; ii++) if(*(p_px+ii) > endln) endln= *(p_px+ii);
  if(endln < 90) xdet|=0x1;
  return(xdet);  
}

// perform rotation on junction Y1..Y4 CW resp. CCW
// perform rotation on linecrossing X CW resp. CCW
uint8_t node(uint8_t type, int8_t err8, int8_t hdg){
static uint8_t susp;  
static uint8_t sqn=0;
  if(type) sqn=type;
  switch(sqn){
    case(nop): return(0); break;
    case(xrght55): wmove(0x16, 0x16); O_Set(0x18); sqn=15; break; // Lr=0
    case(xrght125): break;    // nog
    case(xlft55):  break;     // nog
    case(xlft125): wmove(0x16, 0x16); O_Set(0x18); sqn=20; break; // Lr=0
    case(CW_Y1):  break;
    case(CCW_Y1): wmove(0x16, 0x16); O_Set(0x46); sqn=70; break; // Lr=0
    case(CW_Y2):  break;
    case(CCW_Y2): wmove(0x16, 0x16); O_Set(0x46); sqn=60; break; // Lr=0
    case(CW_Y3):  wmove(0x16, 0x16); O_Set(0x46); sqn=50; break; //Lr=1     // 10 cm
    case(CCW_Y3): break;
    case(CW_Y4): wmove(0x18,0x12); O_Set(0x60); sqn=55; break; //Lr=1     // 10 cm
    case(CCW_Y4): break;
    
    
    
/* X turn CW  */
    case(15): if(MoveRdy){wmove(0x0,0x0); susp=4; sqn=16;} break;
    case(16): susp--; if(susp==0){wmove(0x10, 0x90); O_Set(0x12); sqn=17;} break; 
    case(17): if(MoveRdy) sqn=18; break;
    case(18): if(err8 > -2){wmove(0x0,0x0); sqn=0; return(1);} break;
   
 
/* X turn CCW  */
    case(20): if(MoveRdy){wmove(0x0,0x0); susp=4; sqn=21;} break;
    case(21): susp--; if(susp==0){wmove(0x90,0x10); O_Set(0x32); sqn=22;} break;
    case(22): if(MoveRdy) sqn=23; break;
    case(23): if(err8 < 0) {wmove(0x0,0x0); sqn=0; return(1);} break;

/* 150dgr CW */   
    case(30): if(MoveRdy){wmove(0x0,0x0); susp=4; sqn=31;} break;  // 31
    case(31): susp--; if(susp==0){wmove(0x10,0x90); O_Set(0x4B); sqn=32;} break;
    case(32): if(MoveRdy) {wmove(0x0,0x0); sqn=0; return(1);} break;

/* 150dgr CCW */   
    case(40): if(MoveRdy){wmove(0x0,0x0); susp=4; sqn=41;} break;
              wheels(105,hdg); break;
    case(41): susp--; if(susp==0){wmove(0x90,0x10); O_Set(0x4B); sqn=42;} break;
    case(42): if(MoveRdy) sqn=43; break;
    case(43): if(err8 < 4){wmove(0x0,0x0); sqn=0; return(1);} break;
 
//=== CW_Y3     
    case(50): if(MoveRdy){wmove(0x0,0x0); susp=4; sqn=51;}  //51
               break;  // 31
    case(51): susp--; if(susp==0){wmove(0x10,0x90); O_Set(0x4B); sqn=52;} break;  //3c
    case(52): if(MoveRdy) sqn=53; break;
    case(53): if(err8 > -2){wmove(0x0,0x0); sqn=0; return(1);} break;
    
//=== CW_Y4
    case(55): if(MoveRdy){wmove(0x0,0x0); susp=4; sqn=56; break;}  //51
    case(56): susp--; if(susp==0) {wmove(0x10,0x90); O_Set(0x4B); sqn=57;} break;  //3c
    case(57): if(MoveRdy)sqn=58; break;
    case(58): if(err8 < 0){wmove(0x0,0x0); sqn=0; return(1);} break;

//===CCW Y2
    case(60): if(MoveRdy){wmove(0x0,0x0); susp=4; sqn=61;} break;
    case(61): susp--; if(susp==0){wmove(0x92,0x10); O_Set(0x4B); sqn=62;} break;
    case(62): if(MoveRdy) sqn=63; break;
    case(63): if(err8 < 4) {wmove(0x0,0x0); sqn=0; return(1);} break;

//===CCW Y1
    case(70): if(MoveRdy){wmove(0x0,0x0); susp=4; sqn=71;} break;
    case(71): susp--; if(susp==0){wmove(0x92,0x10); O_Set(0x4B); sqn=72;} break;
    case(72): if(MoveRdy) sqn=73; break;
    case(73): if(err8 < 4) {wmove(0x0,0x0); sqn=0; return(1);} break;
    default: break;
  } 
  return(0);  
}

void wheels(uint8_t motPwr, int8_t hdg){
static uint8_t hdg0=0xFF;
static uint8_t timeout=0;
  if((hdg != hdg0) || (timeout==0)){
    hdg0=hdg;
    analogWrite(PWML, motPwr - hdg);
    analogWrite(PWMR, motPwr + hdg + 12 );  //+8
    timeout=10;
  }
  if(timeout) timeout--;  
}
/*
 lineError range  -13..0..13
 linesensor provides a linear 8 bytes array  pointed by *p_px    *(p_px+0) ..*(p_px+7)
 This array may be evaluated from low to high *(p_px+0) -> *(p_px+7) focussed on right edge of line
 or evaluated from high to low  *(p_px+7) -> *(p_px+0) focussed on left edge of line
 left/right evaluation results in the option to swing off an abatement left or right as desired
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
  return(nn);   // 1..27
}

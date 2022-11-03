#include "carkar.h"

//==== dipswitch setting 001 ====
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
                  O_Set(900);
                 sq_m=1;
                 break;
          
     } // end switch
    } // end ms
  } // end while
}

//==== dipswitch setting 010 ====
void demo(){
bool togg=0;
uint8_t sq;
uint8_t range_p;
   while(1){
    ms=millis();
    if(ms > msref){
      msref=ms+30;
      ledBlink();
      acq_sensors();
      if(getRC5()){
        if(rc5_msg==0xFF){
            Wire.beginTransmission(PCF8574_I2C_ADDRESS);
            if(togg) Wire.write(0x8F);   // redled off
            else Wire.write(0x0F);   // redled on
            Wire.endTransmission();
            togg=!togg;
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
                      if(range_p != range[0]) Serial.println(range[0]);
                      range_p=range[0]; break;
        case(0xFD): acq_sensors();                  // show centre sensor
                      if(range_p != range[1]) Serial.println(range[1]);
                      range_p=range[1]; break;
        case(0xFC): acq_sensors();                  // show right sensor
                      if(range_p != range[2]) Serial.println(range[2]);
                      range_p=range[2]; break;
      }
    } // end ms
  } // end while  
}

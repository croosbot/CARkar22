//------------------------------------------------------------------------------
// EncoderTakt - Lees encoders uit; zo vaak mogelijk aanroepen.
//------------------------------------------------------------------------------
// Tel pulsen van linkse en rechtse encoder.
// CARkar encoder  geeft 32 pulsen per omwenteling. 
// Wieldiameter 65 mm => 200 mm omtrek => 6,3 mm/puls/wiel
// 
//------------------------------------------------------------------------------
void EncoderTakt(){
  static bool VorigeEncoderL;
  static bool VorigeEncoderR;

  // lees linkse encoder    
  bool ActPin = digitalRead(ENCODER_L_PIN);
  if (ActPin != VorigeEncoderL) {
    VorigeEncoderL = ActPin;  
    b_EncoS=b_EncoL=1;
    Odo_L++;
    if(Odo_L==Odo_LL) b_OdoL=1; 
    Odo_S++;
    if(Odo_S==Odo_SS) b_OdoS=1;
    drift++;    
  }

  // lees rechtse encoder    
  ActPin = digitalRead(ENCODER_R_PIN);
  if (ActPin != VorigeEncoderR) {
    VorigeEncoderR = ActPin;
    b_EncoS=b_EncoR=1;  
    Odo_R++;
    if(Odo_R==Odo_RR) b_OdoR=1;
    Odo_S++;
    if(Odo_S==Odo_SS) b_OdoS=1;
    drift--;
  }
}

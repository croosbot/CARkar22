//-----------------------------------------------------------------------------
// Motors.ino  - aansturing van de CARkar motors via enable (standaard methode)
//-----------------------------------------------------------------------------

// Constantes
const int PWML    = 11;
const int PWMR    =  3;

const int DIRL    = 10;
const int DIRR    = 12;

//-----------------------------------------------------------------------------
// SetupMotors - Stel IO in voor aansturing motoren
//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
void SetupMotors()
{
  // Rijrichting
  pinMode(DIRL, OUTPUT);       
  pinMode(DIRR, OUTPUT);        

  // Pwm pins
  pinMode( PWML, OUTPUT);     
  pinMode( PWMR, OUTPUT);     

  // zet output op 0 via Motors()
  Motors(0, 0);
}


//-----------------------------------------------------------------------------
// Motors - Stel PWM duty cycle in voor beide motoren
//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
void Motors(int PwmL, int PwmR) {
  //--------
  // Motor L 
  //--------

  // begrens waarden
  if (PwmL >  255) PwmL =  255; 
  if (PwmL < -255) PwmL = -255;
 
  // Set PWM en richting  
  if (PwmL >= 0) {
    digitalWrite(DIRL, 0);
    analogWrite (PWML, PwmL);   
  }else{
    digitalWrite(DIRL, 1);
    analogWrite (PWML, -PwmL);   
   } 


  //--------
  // Motor R
  //--------

  // begrens waarden
  if (PwmR >  255) PwmR =  255;
  if (PwmR < -255) PwmR = -255; 
  //Set PWM en richting  
  if (PwmR >= 0){ 
    digitalWrite(DIRR, 0);  // vooruit
    analogWrite(PWMR, PwmR);   
    }else{
      digitalWrite(DIRR, 1); 
      analogWrite(PWMR, -PwmR);   
    }
}



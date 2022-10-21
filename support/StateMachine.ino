// StateMachine.ino - 

//-----------------------------------------------------------------------------
// RijdenTakt - State machine voor aansturing motor.
//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------

// Odo_x  current 
// Odo_xx target
// b_Odox  done           handled: encoder()
// b_EncoX  encoderevent  handled: encoder()


//char moves[11]={20,60,-20,20,-20,40,-40,20,20,40,0};
char moves[11]={-20,60,-20,20,-20,40,-40,20,-60,40,0};

void RijdenTakt(){ 
static int State, PrevState = -1;
static char ang_L, ang_R;
char tmp;
static byte susp;
int Afwijking, straight;
const int ScSpd=80;
static char *pmov=moves;
static byte run_cnt;
    if(State && KnopFlank()) State=30;
    printf("%d\n",State);
    switch(State){
      case (0) : if (KnopFlank()){ 
                  Motors(LOW_PWM-5,LOW_PWM);
                  run_cnt=b_OdoS=Odo_S=0; Odo_SS=10; //   10*3mm
                  State =1;
                }break;
      case(1): if(b_OdoS){drift=b_OdoS=Odo_S=0; Odo_SS=150; State=2;} break;
      case(2): if(b_OdoS){Motors(0,0); susp=20; State=3; break;}
                if(b_EncoS){
                  b_EncoS=0;
                  if(drift>0) {digitalWrite(LED,1); Motors(60-10,60);}
                  else {digitalWrite(LED,0); Motors(60,60);}
                } break;
      case(3):  susp--;
                if(susp==0) {b_EncoS=0; State=4;} break;
      case(4):  if(b_EncoS){                    // go when encoderhit
                   Motors(LOW_PWM-5,LOW_PWM);
                   b_OdoS=Odo_S=0; Odo_SS=10; //   10*3mm  startup
                    State=5;
                  } break;
      case(5):  if(b_EncoS){        // wait for edge
                  if(edge){Motors(-ScSpd,-ScSpd); State=6; break;}
                  b_EncoS=0;
                  if(drift>0) {digitalWrite(LED,1); Motors(LOW_PWM-10,LOW_PWM);}
                  else {digitalWrite(LED,0); Motors(LOW_PWM,LOW_PWM);}
                } break;

      case(6): if(!edge){   // left 2cm FWD
                    Motors(ScSpd,0); b_EncoL=ang_L=b_OdoL=Odo_L=0; Odo_LL=5;  State=7;
               } break;
      case(7): if(edge && b_EncoL) {ang_L++; b_EncoL=0;}
               if(b_OdoL){    // 2 cm done
                    b_OdoL=Odo_L=0; Odo_LL=5;
                    Motors(-ScSpd,0);
                    State=8;
                } break;
      case(8): if(edge && b_EncoL) {ang_L++; b_EncoL=0;}
                if(b_OdoL) {Motors(0,ScSpd); b_EncoR=ang_R=b_OdoR=Odo_R=0; Odo_RR=5; State=9;} break;   // left 2cm REV
      case(9): if(edge && b_EncoR) {ang_R++; b_EncoR=0;}
                if(b_OdoR){
                    b_OdoR=Odo_R=0; Odo_RR=5;
                    Motors(0,-ScSpd);
                    State=10;                
                }break;
      case(10): if(edge && b_EncoR) {ang_R++; b_EncoR=0;}
                if(b_OdoR){
                  State=11;   
                } break;
      case(11): tmp=ang_L-ang_R;
                if(tmp>0){
                    b_EncoR=b_OdoR=Odo_R=0; Odo_RR=tmp;
                    Motors(0,-ScSpd);
                    State=12;
                }else if(tmp<0){
                  tmp=abs(tmp);
                  b_EncoL=b_OdoL=Odo_L=0; Odo_LL=tmp;
                    Motors(-ScSpd,0);
                    State=13;
                }else State=14; break;
      case(12): if(b_OdoR) State=14; break;
      case(13): if(b_OdoL)State=14; break;
      case(14): Motors(-100,-100); Odo_S=b_OdoS=0;  Odo_SS=20; State=15; break; 
      case(15): if(b_OdoS){               // backtrack
                  run_cnt++;
                        if(run_cnt==15){susp=20; Motors(0,0); State=20; break;} 
                        else if(run_cnt==30){susp=20; State=25; break;}
                  tmp=abs(*pmov);
                  if(*pmov<0) Motors(-80,80); else Motors(80,-80);       
                    Odo_S=b_OdoS=0; Odo_SS=tmp; pmov++; if(pmov==(&moves[10])) pmov=moves; State=16;} break;
      case(16): if(b_OdoS) {Motors(0,0); State=17;} break;     // turn complete
      case(17): Motors(LOW_PWM-5,LOW_PWM);  b_EncoS=0; State=5; break;

// need help      
      case(20): susp--;
                if(susp==0) {b_EncoS=0; State=21;} break;
      case(21): if(b_EncoS){                    // go when encoderhit
                   Motors(-60,-55);
                   b_OdoS=Odo_S=0; Odo_SS=30; //   10 cm backtrack
                    State=22;
                  } break;
      case(22): if(b_OdoS){susp=100; Motors(0,0); State=23;} break;
      case(23): susp--;
                if(susp==0) State=17; break;

// final
       case(25): Motors(-70,70); Odo_S=b_OdoS=0;  Odo_SS=20; State=26; break;
       case(26): if(b_OdoS){Motors(70,-70); Odo_S=b_OdoS=0;  Odo_SS=20; State=27;} break;
       case(27): if(b_OdoS) State=25; break;




                
//#endif     
                
      case(30): Motors(0,0);
                 digitalWrite(LED,0);
                  State=0; break;


      
                



  default : 
            printf("RijdenTakt: ongeldige state %d\n", State);
            Motors(0,0);
            State = 0;
            break;
    }
}

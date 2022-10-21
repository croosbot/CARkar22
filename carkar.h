
// carkar.h
// 06-10-2022
// O_Set(200) :: 360 dgr rotate



#ifndef __carkar_h
#define __carkar_h

typedef unsigned int uint;


#define PCF8574_I2C_ADDRESS 0x20
#define TCCR2B _SFR_MEM8(0xB1)
#define PROBE   A0
#define IRS_L   A1
#define IRS_C   A2
#define IRS_R   A3
#define MoveRdy b_odo & 0x1
#define O_Set(x) odoref=odosum +(x); b_odo=0
#define Halted rung==0

// board pins
enum{
  PWMR  = 3,
  ENC_L = 4,
  ENC_R = 5,
  DIRR  = 8,
  DIRL  = 9,
  PWML  = 11,
  BUTT  = 12,
  LED   = 13
};


unsigned long ms, msref;
bool edge;
uint16_t odosum, odoref;
byte enco0_L, enco0_R;
byte c_odo, b_odo;
int8_t straight;
int8_t rung;    // running
byte range[3];

#endif

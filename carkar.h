
// carkar.h
//  VERSIE 1.0 29/01/24
// 06-10-2022
// O_Set(200) :: 360 dgr rotate
// O_Set(s) :: 1.5 mm/lsb straight

#ifndef __carkar_h
#define __carkar_h

#define VINYL 1

void LineFol(bool);
bool button(void);

typedef unsigned int uint;

#define PCF8574_I2C_ADDRESS 0x20
#define RC5_I2C_ADDRESS 0x0B
#define PROX_I2C_ADDRESS 0xD
#define LINE_I2C_ADDRESS 0xA
#define TCCR2B _SFR_MEM8(0xB1)
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
  LED   = 13,
  PROBE = 14,   //A0
  IRS_L = 15,   //A1
  IRS_C = 16,   //A2
  IRS_R = 17    //A3
};

enum nodes{
  nop, 
  xrght55,
  xrght125,
  xlft55,
  xlft125,
  CW_Y1,
  CCW_Y1,
  CW_Y2,
  CCW_Y2,
  CW_Y3,
  CCW_Y3,
  CW_Y4,
  CCW_Y4
};

enum{Lft,Ctr,Rgt};

void redled(bool);
void ylwled(bool);

unsigned long ms, msref;
bool edge;
uint16_t odosum, odoref;
uint8_t enco0_L, enco0_R;
uint8_t c_odo, b_odo;
int8_t straight;
int8_t rung;    // running
uint8_t range[3];
uint8_t rc5_msg;


#endif

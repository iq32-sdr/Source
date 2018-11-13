/*
 * RS-HFIQ subroutines 8/11/2016
 * Written by Chris Scholefield VE7QCS
 */

#include "Init_I2C.h"
#include "RS-HFIQ.h"
#include "si5351.h"
#include "Si570.h"
#include "FrequencyManager.h"
#include "xprintf.h"
#include "yprintf.h"
#include <stdlib.h>
#include <stdint.h>

void pinMode (int, uint8_t);
void digitalWrite (int, uint8_t);
uint8_t digitalRead (int);
void changeBit (int, int, uint8_t);
void RS_HFIQ_loop(void);
void Set_Band (void);
void T_adjust(long);

// The Silicon Labs SI-5351 chip is used to generate RF signals
// for the RS-HFIQ. It generates the LO signal for the up/down converter,
// the Built-in Test (BIT) signal and a third signal which is either
// sent to the 'EXT RF' jack or used to generate CW
//Si5351 freq_gen;

// This sets up a send only serial port on EX_GP to send
// frequency data to the Hardrock-50 power amplifier
const uint8_t txPin = 16;  //pin A2
//SendOnlySoftwareSerial HR50 (txPin);

float F_Offset = 29.0;

long TEMP_C, T_TOT = 0, OTEMP, LF_Offset;
long T_ARY[32] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
unsigned long T_freq;
unsigned char T_NDX = 0;
unsigned char Ser_Flag = 0, Ser_NDX = 0;
//char S_Input[16], O_DIT = 1, O_DAH = 1;
unsigned long LO_freq, BIT_freq, EXT_freq, CW_Freq;
float T_Offset;
uint64_t F_Long;
uint8_t acount = 0, band = 0;
uint16_t clip;
uint16_t cl[] = {284, 337, 246, 250, 325, 286, 296, 300, 308};
uint8_t HR50band, oldHRB = '0';
uint64_t T_count = 0;
uint8_t Active_T, AT_addr = 0;
//static uint8_t inTxMode =0;

// Setup code runs once before the loop starts to initialize
// the board components
void RS_HFIQ_setup(void) {
	debug (RS_HFIQ, "RS_HFIQ_setup:start\n");

//  Serial.begin(57600); // Communicate via USB serial port at 57600 Baud
//  HR50.begin(19200); // Communicate with the HR50 at 19200 Baud

// Set up the SI5351 for a 25MHz crystal with a 6PF load, set PLLA for
// 900 MHZ and turn off all outputs
  Si5351_init(SI5351_CRYSTAL_LOAD_6PF, 0);
  Si5351_set_correction(0);
  Si5351_set_pll(SI5351_PLL_FIXED, SI5351_PLLA);
  Si5351_set_pll(SI5351_PLL_FIXED, SI5351_PLLB);
  Si5351_set_ms_source(SI5351_CLK0, SI5351_PLLA);
  Si5351_set_ms_source(SI5351_CLK1, SI5351_PLLB);
  Si5351_set_ms_source(SI5351_CLK2, SI5351_PLLB);
  Si5351_output_enable(SI5351_CLK0, 0);
  Si5351_output_enable(SI5351_CLK1, 0);
  Si5351_output_enable(SI5351_CLK2, 0);

// Set up the band select and keying lines
// and set them all low
  pinMode(B80M, OUTPUT); digitalWrite (B80M, LOW);
  pinMode(B60_40M, OUTPUT); digitalWrite (B60_40M, LOW);
  pinMode(B30_20M, OUTPUT); digitalWrite (B30_20M, LOW);
  pinMode(B17_15M, OUTPUT); digitalWrite (B17_15M, LOW);
  pinMode(B12_10M, OUTPUT); digitalWrite (B12_10M, LOW);
  pinMode(RX_HI, OUTPUT); digitalWrite (RX_HI, HIGH);
  pinMode(TX_HI, OUTPUT); digitalWrite (TX_HI, LOW);
//  pinMode(DC_PTT, OUTPUT); digitalWrite (DC_PTT, LOW);
  pinMode(EX_PTT, OUTPUT); digitalWrite (EX_PTT, LOW);
  pinMode(PTT, OUTPUT); digitalWrite (PTT, LOW);
  pinMode(EX_GP, INPUT);
  pinMode(R_LED, OUTPUT); digitalWrite (R_LED, LOW);
  pinMode(Y_LED, OUTPUT); digitalWrite (Y_LED, LOW);
  pinMode(KEY1, INPUT);
  pinMode(PIN2, INPUT);

  T_Offset = +750.0;

  F_Offset = (float) (LF_Offset/10);

  debug (RS_HFIQ, "RS_HFIQ_setup:done\n");
//  Si5351_dump();
}

int analogRead (void){
	I2C_WriteRegister16(ADS1015_Addr, 1, 0x5483);
	int value = I2C_ReadSlave16(ADS1015_Addr, 0)>>5; // take the 11 MSBs (10 bits and sign)
//	debug (RS_HFIQ, "analogRead, %d\n", value);
	return value; // take the 10 MSBs
}

void RS_HFIQ_loop(void) {
//	debug (RS_HFIQ, "RS_HFIQ_loop:\n");
	if (T_count++%1000 == 0) {// runs every 1000 times through the loop, roughly every few ms
		FrequencyManager_ControlCurrentFrequency();
//	if ((inTxMode = 1) & (analogRead() > clip))
//		digitalWrite(R_LED, HIGH);
//	else
//		digitalWrite(R_LED, LOW);
	}
//	xprintf ("0x%x\n", RS_HFIQ_isKeyed (KEY1));
}

void T_adjust(long FREQ){
	debug (RS_HFIQ, "T_adjust:\n");
  float corr = (float) ((FREQ)/1000000.0 * T_Offset);
  F_Long = (uint64_t) (FREQ) * 100 - (uint64_t) (corr);
  debug (RS_HFIQ, "T_adjust:done\n");
}

#define ENABLE 1
#define DISABLE 0

void RS_HFIQ_Command_TX(int isSet) {
  debug (RS_HFIQ, "RS_HFIQ_Command_TX enable = %x\n",isSet);
  if (isSet == DISABLE){
      digitalWrite (TX_HI, LOW);
      digitalWrite (PTT, LOW);
      digitalWrite (Y_LED, LOW);
      digitalWrite (RX_HI, HIGH);
      digitalWrite (EX_PTT, LOW);
 } else if (isSet == ENABLE){
	  digitalWrite (EX_PTT, HIGH);
      digitalWrite (PTT, HIGH);
      digitalWrite (RX_HI, LOW);
      digitalWrite (TX_HI, HIGH);
      digitalWrite (Y_LED, HIGH);
 } else
	 debug (RS_HFIQ, "RS_HFIQ_Command_TX Error\n");
}

void RS_HFIQ_Command_ClockDrive(uint8_t clock, uint8_t level) {
	  debug (RS_HFIQ, "RS_HFIQ_Command_ClockDrive\n");

	  si5351_clock clock_sel=0;
	  si5351_drive drive=0;

	  switch (clock) {
	  case 2:
		  clock_sel = SI5351_CLK2;
		  break;
	  case 1:
		  clock_sel = SI5351_CLK1;
		  break;
	  case 0:
		  clock_sel = SI5351_CLK0;
		  break;
	  default:
		  debug (RS_HFIQ, "RS_HFIQ_Command_ClockDrive Error\n");
		  break;
	  }

  Si5351_output_enable(clock_sel, level > 0 ? 1 : 0);

  switch (level){
    case 2: drive = SI5351_DRIVE_4MA; break;
    case 3: drive = SI5351_DRIVE_6MA; break;
    case 4: drive = SI5351_DRIVE_8MA; break;
    default: drive = SI5351_DRIVE_2MA; break;
  }
  Si5351_drive_strength(clock_sel, drive);
}

void RS_HFIQ_Command_Freq(uint32_t freq){
	debug (RS_HFIQ, "RS_HFIQ_Command_Freq %d\n", freq);

      LO_freq = (unsigned long) freq * 4;
      if (LO_freq > 11999999 && LO_freq < 120000001){
    	Set_Band();
        T_adjust(LO_freq);
        Si5351_set_freq(F_Long, SI5351_PLL_FIXED, SI5351_CLK0);
      } else
    	  debug (RS_HFIQ, "Freq out of range.");
}

void Set_Band (void){
	debug (RS_HFIQ, "Set_Band:\n");
	static uint8_t lastBand = 0xFF;
  uint8_t B_freq = LO_freq / 4000000 ;
  HR50band = 0;
  switch (B_freq){
    case 3: case 4: band = 0; HR50band = 9;break;
    case 5: band = 1; HR50band = 8; break;
    case 7: band = 2; HR50band = 7; break;
    case 10: band = 3; HR50band = 6; break;
    case 14: band = 4; HR50band = 5; break;
    case 18: band = 5; HR50band = 4; break;
    case 21: band = 6; HR50band = 3; break;
    case 24: case 25: band = 7; HR50band = 2; break;
    case 28: case 29: band = 8; HR50band = 1; break;

  }

  debug (RS_HFIQ, "Set_Band:band=%d,HR50band=%d\n", band, HR50band);

  if (band != lastBand){
	  lastBand = band;
	  if (HR50band == 0){
	        yprintf ("HRBN10;");
	      }
	      else{
	    	  yprintf("HRBN0");
	    	  yprintf("%d", HR50band);
	    	  yprintf (";");
	      }
  clip = cl[band];
  digitalWrite (B80M, LOW);
  digitalWrite (B60_40M, LOW);
  digitalWrite (B30_20M, LOW);
  digitalWrite (B17_15M, LOW);
  digitalWrite (B12_10M, LOW);
  if (LO_freq < 17200000) {digitalWrite (B80M, HIGH); debug (RS_HFIQ, "Set_Band:B80M\n"); return;}
  if (LO_freq < 36000000) {digitalWrite (B60_40M, HIGH); debug (RS_HFIQ, "Set_Band:B60_40M\n"); return;}
  if (LO_freq < 60000000) {digitalWrite (B30_20M, HIGH); debug (RS_HFIQ, "Set_Band:B30_20M\n"); return;}
  if (LO_freq < 96000000) {digitalWrite (B17_15M, HIGH); debug (RS_HFIQ, "Set_Band:B17_15M\n"); return;}
  digitalWrite (B12_10M, HIGH); debug (RS_HFIQ, "Set_Band:B12_10M\n");
  }
}

//------------------------------------------

void pinMode (int pin, uint8_t direction) {
	debug (RS_HFIQ, "pinMode:pin=%d,direction=%d\n", pin, direction);

	if (pin < 8)
		changeBit (IODIR_A, pin, direction);
	else
		changeBit (IODIR_B, pin-8, direction);
}

void digitalWrite (int pin, uint8_t level) {
//	debug (RS_HFIQ, "digitalWrite:pin=%d,level=%d\n", pin, level);
//	watchdog (534);
	if (pin < 8)
		changeBit (GPIO_A, pin, level);
	else
		changeBit (GPIO_B, pin-8, level);
//	watchdog (535);
}

unsigned char RS_HFIQ_read[6] = {0,0,0,0,0,0};

void Read_RS_HFIQ_registers(unsigned char read[])
	{
	debug (RS_HFIQ, "Read_RS_HFIQ_registers:\n");
	unsigned char i;
	for (i = 0; i < 6; i++)
		read[i] = I2C_ReadSlave(MCP23017_Addr, i + 7);
	}

unsigned char setBit( unsigned char bit_fld, int n ) {
//	debug (RS_HFIQ, "setBit:\n");
  return bit_fld |= (1 << n);
}

unsigned char clearBit( unsigned char bit_fld, int n ) {
//	debug (RS_HFIQ, "clearBit:\n");
  return bit_fld &= ~(1 << n);
}

void changeBit (int regAddress, int bitPosition, uint8_t level){
//	debug (RS_HFIQ, "changeBit:\n");
	unsigned char value;

		value = I2C_ReadSlave(MCP23017_Addr, regAddress);
		if (level == 1)
			value = setBit(value, bitPosition);
		else
			value = clearBit(value, bitPosition);
		I2C_WriteRegister( MCP23017_Addr, regAddress, value);

}


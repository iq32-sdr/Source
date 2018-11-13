/*
 * RS-HFIQ header 8/11/2016
 * Written by Chris Scholefield VE7QCS
 */

#define MCP23017_Addr 0x40
#define ADS1015_Addr 0x48<<1

#define IODIR_A 0
#define IODIR_B 1
#define GPIO_A 0x12
#define GPIO_B 0x13

#define B80M 0 //GPA0
#define TX_HI 1 //GPA1
#define R_LED 2 //GPA2
#define Y_LED 3 //GPA3
#define EX_GP 4 //GPA4
#define EX_PTT 5 //GPA5
#define PTT 6 //GPA6

#define KEY1 9 //GPB1
#define PIN2 10 //GPB2
#define B12_10M 11 //GPB3
#define RX_HI 12 //GPB4
#define B17_15M 13 //GPB5
#define B30_20M 14 //GPB6
#define B60_40M 15 //GPB7

#define OUTPUT 0
#define INPUT 1

#define LOW 0
#define HIGH 1

void Init_RS_HFIQ(void);
void RS_HFIQ_loop(void);
void RS_HFIQ_setup(void);
void RS_HFIQ_Command_P(char *);
void RS_HFIQ_Command_TX(int);
void RS_HFIQ_Command_ClockDrive(uint8_t, uint8_t);
void RS_HFIQ_Command_Freq(uint32_t);

/*
 * Code to handle the 2 encoder knobs
 *
 * STM32-SDR: A software defined HAM radio embedded system.
 * Copyright (C) 2013, STM32-SDR Group
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */


#include	"stm32f4xx_rcc.h"
#include	"stm32f4xx_exti.h"
#include	"stm32f4xx_syscfg.h"
#include 	"stm32f4xx_gpio.h"
#include	"stm32f4xx_tim.h"
#include	"arm_math.h"
#include	"SI570.h"
#include	"Init_I2C.h"
#include	"Init_Codec.h"
#include	"Encoders.h"
#include	"FrequencyManager.h"
#include	"options.h"
#include	"LcdHal.h"
#include	"ChangeOver.h"
#include	"xprintf.h"
#include	"screen_All.h"
#include	"main.h"
#include 	"RS-HFIQ.h"
#include	"build_options.h"

extern int 	NCOTUNE;

typedef struct
{
	uint16_t old;
	int8_t tracking;
	int16_t deltaChange;
	int16_t change;
	GPIO_TypeDef *pPort;
	uint8_t lowPinNumber;
} EncoderStruct_t;

#ifdef RSHFIQ
static EncoderStruct_t s_encoderStruct1 = {0, 0, 0, 0, GPIOD, 12};
#else
static EncoderStruct_t s_encoderStruct1 = {0, 0, 0, 0, GPIOC, 5};
#endif

static EncoderStruct_t s_encoderStruct2 = {0, 0, 0, 0, GPIOB, 4};
//static EncoderStruct_t s_encoderStruct1 = {0, 0, GPIOC, 5};
//static EncoderStruct_t s_encoderStruct2 = {0, 0, GPIOB, 4};

// Prototypes
static void initEncoderTimer(void);
static void configureGPIOEncoder1(void);
static void configureGPIOEncoder2(void);
//static void configureEncoderInterrupt(void);
static void init_encoder1(void);
static void init_encoder2(void);
//static int8_t calculateEncoderChange(EncoderStruct_t *pEncoder);
static void calculateEncoderChange(EncoderStruct_t *pEncoder);
static void applyEncoderChange1(int8_t changeDirection);
static void applyEncoderChange2(int8_t changeDirection);

/* ----------------------
 * Initialization
 * ---------------------- */
void Encoders_Init(void)
{
	configureGPIOEncoder1();
	configureGPIOEncoder2();
	init_encoder1();
	init_encoder2();  //this may be required to introduce a delay for start up
	FrequencyManager_StepFrequencyDown();
	FrequencyManager_StepFrequencyUp();
	initEncoderTimer();
}

static void initEncoderTimer(void)
{
	debug(GUI, "initEncoderTimer:\n");

	NVIC_InitTypeDef NVIC_InitStructure;

	/* Enable the TIM8 global Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = TIM7_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 7;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	/* TIM7 clock enable */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM7, ENABLE);

	/* Time base configuration */

	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_TimeBaseStructure.TIM_Period = 500; // 1 MHz down to 2 KHz (0.5 ms)
	TIM_TimeBaseStructure.TIM_Prescaler = 84 - 1; // 24 MHz Clock down to 1 MHz (adjust per your clock)
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM7, &TIM_TimeBaseStructure);

	/* TIM IT enable */
	TIM_ITConfig(TIM7, TIM_IT_Update, ENABLE);

	/* TIM4 enable counter */
	TIM_Cmd(TIM7, ENABLE);
}

// Filter parameter beta controls the percentage of raw versus IIR filter
// 100 = no filter, all raw data. 0 = all filter output, no raw data input
// filter beta value varies between minimum of BASE and maximum of BASE + SCALE

#define BASE 30 // percentage filter input of the raw encoder counts
#define SCALE 50 // range to adjust filter beta value

void TIM7_IRQHandler(void)
{
		calculateEncoderChange(&s_encoderStruct1);
		calculateEncoderChange(&s_encoderStruct2);
		Options_UnMuteAudio();
		TIM_ClearITPendingBit(TIM7, TIM_IT_Update );
}

static void configureGPIOEncoder1(void)
{

	GPIO_InitTypeDef GPIO_InitStructure;

	/* Enable GPIOC clock */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	/* Enable SYSCFG clock */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

#ifdef RSHFIQ
	/* Configure PC58 pin as input with Pull Up */
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Pin = (GPIO_Pin_8 );
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	/* Configure PD12&13 pin as input with Pull Up */
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Pin = (GPIO_Pin_12 | GPIO_Pin_13 );
	GPIO_Init(GPIOD, &GPIO_InitStructure);
#else
	/* Configure PC58 pin as input with Pull Up */
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Pin = (GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_8 );
	GPIO_Init(GPIOC, &GPIO_InitStructure);
#endif

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
	/* Configure PE3,4,5&6 pin as input with Pull Up */
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Pin = (GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5
			| GPIO_Pin_6 );
	GPIO_Init(GPIOE, &GPIO_InitStructure);

}
static void configureGPIOEncoder2(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	/* Enable GPIOC clock */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	/* Enable SYSCFG clock */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

	/* Configure PC1,2,3&4 pin as input with Pull Up */
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Pin = (GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3	| GPIO_Pin_4 );
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	/* Configure PB3,4,5&7 pin as input with Pull Up */
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Pin = (GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_7 );
	GPIO_Init(GPIOB, &GPIO_InitStructure);
}

void init_encoder1(void)
{
}

void init_encoder2(void)
{
	// TODO:- Is there a need to do this here?
	RxTx_SetReceive();
}


// Return if one (or both) encoders pressed.
_Bool Encoders_IsOptionsEncoderPressed(void)
{
	return !GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_7);
}
_Bool Encoders_IsFrequencyEncoderPressed(void)
{
	return !GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_8);
}
_Bool Encoders_AreBothEncodersPressed(void)
{
	return Encoders_IsOptionsEncoderPressed()
			&& Encoders_IsFrequencyEncoderPressed();
}

/*
 * Process Encoder Changes
 */
void Encoders_CalculateAndProcessChanges(void){

//	TIM_Cmd(TIM7, DISABLE);

	if (s_encoderStruct1.change > 0){
//		xprintf("1:%3d %3d %4x\n",s_encoderStruct1.change, s_encoderStruct1.tracking, s_encoderStruct1.old);
//		xprintf("old = %x change = %d tracking = %d\n", s_encoderStruct1.old, s_encoderStruct1.change, s_encoderStruct1.tracking);
//		while (s_encoderStruct1.change > 0) {
			applyEncoderChange1(1);
			s_encoderStruct1.change--;
//		}
	}
	if (s_encoderStruct1.change < 0){
//		xprintf("1:%3d %3d %4x\n",s_encoderStruct1.change, s_encoderStruct1.tracking, s_encoderStruct1.old);
//		xprintf("old = %x change = %d tracking = %d\n", s_encoderStruct1.old, s_encoderStruct1.change, s_encoderStruct1.tracking);
//		while (s_encoderStruct1.change < 0) {
			applyEncoderChange1(-1);
			s_encoderStruct1.change++;
//		}
	}
	if (s_encoderStruct2.change > 0){
//		xprintf("2:%3d %4x\n",s_encoderStruct2.change, s_encoderStruct2.old);
		applyEncoderChange2(1);
		s_encoderStruct2.change--;
	}
	if (s_encoderStruct2.change < 0){
//		xprintf("2:%3d %4x\n",s_encoderStruct2.change, s_encoderStruct2.old);
		applyEncoderChange2(-1);
		s_encoderStruct2.change++;
	}
//	TIM_Cmd(TIM7, ENABLE);
}

// encoder sequence is 0, 1, 3, 2 anti-clockwise
// or 2, 3, 1, 0 clockwise
// uses a 8 state machine to detect rotation to eliminate noise
// anti-clockwise states are 0x20, 0x21, 0x23, 0x22
// clockwise states are 0x12, 0x13, 0x11, 0x10
// bits 0/1 are the stored state of the encoder code
// bits 2/3 are not used
// bits 4/5 determine the possible direction of clockwise or anti-clockwise
// bits 6/7 determine the qualified direction of clockwise or anti-clockwise
// which requires the previous state to be consistent
// for coding efficiency the upper nibble is shifted right 2 bits so that
// state lookup is a contiguous array
// 0 input LSB
// 1 input MSB
// 2 not used
// 3 not used
// 4 possible CW
// 5 possible CCW
// 6 qualified CW
// 7 qualified CCW

// Encoder used for options =0 or filter code selection = 1

static uint8_t nextState[12][4] =
//  code   0     1     2     3    OldState
{ // from unknown states:
		{0x00, 0x11, 0x22, 0x03}, // 0
		{0x20, 0x01, 0x02, 0x13}, // 1
		{0x10, 0x01, 0x02, 0x23}, // 2
		{0x00, 0x01, 0x02, 0x03}, // 3
//   from clockwise states:
		{0x10, 0x51, 0x22, 0x03}, // 10
		{0x20, 0x11, 0x02, 0x53}, // 11
		{0x50, 0x01, 0x12, 0x23}, // 12
		{0x00, 0x21, 0x52, 0x13}, // 13
//   from anti-clockwise states:
		{0x20, 0x11, 0xA2, 0x03}, // 20
		{0xA0, 0x21, 0x02, 0x13}, // 21
		{0x10, 0x01, 0x22, 0xA3}, // 22
		{0x00, 0xA1, 0x12, 0x23}, // 23
};

static void calculateEncoderChange(EncoderStruct_t *pEncoder) {

	static uint8_t code = 0;
	static uint8_t temp = 0;

	code = (GPIO_ReadInputData(pEncoder->pPort) >> pEncoder->lowPinNumber) & 0x03;

	if (code != (pEncoder->old & 0x03)) {
		// make index a contiguous array lookup by eliminating unused bits
		temp = ((pEncoder->old & 0x30) >> 2) + (pEncoder->old & 0x03);

		code = nextState[temp][code];

		// detect bounce and disregard state transitions at high spin rates
		if (pEncoder->tracking < 5) { // only apply when tracking is failing
			temp = (pEncoder->old >> 8); // find the old state
			// if there is a qualified state and new = old
			if (((temp & 0xC0) > 0) & ((code & 0x0F) == (temp & 0x0F)))
				code = temp; // keep the old state
		}

		pEncoder->old = ((pEncoder->old) << 8) + code;

		if ((pEncoder->old & 0x40) == 0x40) { // qualified Anti-clock
			pEncoder->change++;
			pEncoder->tracking++;
		} else if ((pEncoder->old & 0x80) == 0x80) { // qualified clock
			pEncoder->change--;
			pEncoder->tracking++;
		}
		else
			pEncoder->tracking--;

		if (pEncoder->tracking > 10)
			pEncoder->tracking = 10;
		else if (pEncoder->tracking < 0)
			pEncoder->tracking = 0;

		debug(ENCODER, "        old = %x change = %d code = %5x tracking = %d\n", pEncoder->old, pEncoder->change, code, pEncoder->tracking);
	}
}

/*
#define RIGHT -1
#define LEFT +1
#define BOUNCE 0
#define MAXTRACK 10
#define MINTRACK 0

static int8_t calculateEncoderChange(EncoderStruct_t *pEncoder) {

	// encoder sequence is 0, 1, 3, 2 anti-clockwise
	// or 2, 3, 1, 0 clockwise
	// uses a 8 state machine to detect rotation to eliminate noise
	// anti-clockwise states are 0x20, 0x21, 0x23, 0x22
	// clockwise states are 0x12, 0x13, 0x11, 0x10


	uint8_t code = (GPIO_ReadInputData(pEncoder->pPort) >> pEncoder->lowPinNumber & 0x03);

	if (code != (pEncoder->old & 0x03)) {
//		xprintf("        old = %x change = %d code = %x tracking = %d\n", pEncoder->old, pEncoder->change, code, pEncoder->tracking);

		if (pEncoder->tracking > MAXTRACK)
			pEncoder->tracking = MAXTRACK-1;
		else if (pEncoder->tracking < MINTRACK)
					pEncoder->tracking = MINTRACK+1;

	switch (pEncoder->old) {
		case 0x00:
			switch (code) {
				case 0x01:
					pEncoder->old = 0x11;
					return BOUNCE;
					break;
				case 0x02:
					pEncoder->old = 0x22;
					return BOUNCE;
					break;
				default:
					pEncoder->old = 0x00 | code;
					pEncoder->tracking--;
					return BOUNCE;
					break;
			}
			break;
		case 0x01:
			switch (code) {
				case 0x00:
					pEncoder->old = 0x20;
					return BOUNCE;
					break;
				case 0x03:
					pEncoder->old = 0x13;
					return BOUNCE;
					break;
				default:
					pEncoder->old = 0x00 | code;
					pEncoder->tracking--;
					return BOUNCE;
					break;
			}
			break;
		case 0x03:
			switch (code) {
				case 0x01:
					pEncoder->old = 0x21;
					return BOUNCE;
					break;
				case 0x02:
					pEncoder->old = 0x12;
					return BOUNCE;
					break;
				default:
					pEncoder->old = 0x00 | code;
					pEncoder->tracking--;
					return BOUNCE;
					break;
			}
			break;
		case 0x02:
			switch (code) {
				case 0x00:
					pEncoder->old = 0x10;
					return BOUNCE;
					break;
				case 0x03:
					pEncoder->old = 0x23;
					return BOUNCE;
					break;
				default:
					pEncoder->old = 0x00 | code;
					pEncoder->tracking--;
					return BOUNCE;
					break;
			}
			break;
		case 0x20:
			switch (code) {
				case 0x01:
					pEncoder->old = 0x11;
					return BOUNCE;
					break;
				case 0x02:
					pEncoder->old = 0x22;
					pEncoder->tracking++;
					pEncoder->change--;
					return RIGHT;
					break;
				default:
					pEncoder->old = 0x00 | code;
					pEncoder->tracking--;
					return BOUNCE;
					break;
			}
			break;
			case 0x21:
				switch (code) {
					case 0x00:
						pEncoder->old = 0x20;
						pEncoder->tracking++;
						pEncoder->change--;
						return RIGHT;
						break;
					case 0x03:
						pEncoder->old = 0x13;
						return BOUNCE;
						break;
					default:
						pEncoder->old = 0x00 | code;
						pEncoder->tracking--;
						return BOUNCE;
						break;
				}
				break;
			case 0x23:
				switch (code) {
					case 0x01:
						pEncoder->old = 0x21;
						pEncoder->tracking++;
						pEncoder->change--;
						return RIGHT;
						break;
					case 0x02:
						pEncoder->old = 0x12;
						return BOUNCE;
						break;
					default:
						pEncoder->old = 0x00 | code;
						pEncoder->tracking--;
						return BOUNCE;
						break;
				}
				break;
			case 0x22:
				switch (code) {
					case 0x00:
						pEncoder->old = 0x10;
						return BOUNCE;
						break;
					case 0x03:
						pEncoder->old = 0x23;
						pEncoder->tracking++;
						pEncoder->change--;
						return RIGHT;
						break;
					default:
						pEncoder->old = 0x00 | code;
						pEncoder->tracking--;
						return BOUNCE;
						break;
				}
				break;
			case 0x12:
				switch (code) {
					case 0x00:
						pEncoder->old = 0x10;
						pEncoder->change++;
						pEncoder->tracking++;
						return LEFT;
						break;
					case 0x03:
						pEncoder->old = 0x23;
						return BOUNCE;
						break;
					default:
						pEncoder->old = 0x00 | code;
						pEncoder->tracking--;
						return BOUNCE;
						break;
				}
				break;
			case 0x13:
				switch (code) {
					case 0x01:
						pEncoder->old = 0x21;
						return BOUNCE;
						break;
					case 0x02:
						pEncoder->old = 0x12;
						pEncoder->tracking++;
						pEncoder->change++;
						return LEFT;
						break;
					default:
						pEncoder->old = 0x00 | code;
						pEncoder->tracking--;
						return BOUNCE;
						break;
				}
				break;
			case 0x11:
				switch (code) {
					case 0x00:
						pEncoder->old = 0x20;
						return BOUNCE;
						break;
					case 0x03:
						pEncoder->old = 0x13;
						pEncoder->tracking++;
						pEncoder->change++;
						return LEFT;
						break;
					default:
						pEncoder->old = 0x00 | code;
						pEncoder->tracking--;
						return BOUNCE;
						break;
				}
				break;
			case 0x10:
				switch (code) {
					case 0x01:
						pEncoder->old = 0x11;
						pEncoder->tracking++;
						pEncoder->change++;
						return LEFT;
						break;
					case 0x02:
						pEncoder->old = 0x22;
						return BOUNCE;
						break;
					default:
						pEncoder->old = 0x00 | code;
						pEncoder->tracking--;
						return BOUNCE;
						break;
				}
				break;
		default:
			pEncoder->tracking--;
			break;

	}
	}
	return 0;
}

#define THRESHOLD 2

static int8_t calculateEncoderChange(EncoderStruct_t *pEncoder)
{
	// Determine encoder motion direction

	static uint8_t trackingCount = 0;

	if (trackingCount > 2 * THRESHOLD)
		trackingCount = 2 * THRESHOLD;

	uint32_t code = (GPIO_ReadInputData(pEncoder->pPort) >> pEncoder->lowPinNumber & 0x03);

	// encoder sequence is 0, 1, 3, 2 anti-clockwise
	// or 2, 3, 1, 0 clockwise
	// uses a 8 state machine to detect rotation to eliminate noise
	// requires 3 sequences before determining direction

	// Encoder samples are stored sequentially in nibble for clarity

	//new encoder bits are prepended to the MSB's
	code = (code << 12) | (pEncoder->old >> 4);

	// when a change is detected and not a bounce
	if (      (((code >> 12) & 0x000F) != ((code >> 8) & 0x000F))
			& (((code >> 12) & 0x000F) != ((code >> 4) & 0x000F))
		){

		// store the sequence for use next cycle
		if (trackingCount == 0)
			pEncoder->old = code; //save the new encoder bits

	switch (code){

		// clockwise
		case 0x3102:
		case 0x2310:
		case 0x0231:
		case 0x1023:
			trackingCount++;
			if (trackingCount > THRESHOLD) {
				pEncoder->direction = +1; // qualified clockwise
				pEncoder->old = code; //save the new encoder bits
				return pEncoder->direction;
			}
			break;

		// anti-clockwise
		case 0x1320:
		case 0x0132:
		case 0x2013:
		case 0x3201:
			trackingCount++;
			if (trackingCount > THRESHOLD) {
				pEncoder->direction = -1; // qualified anti-clockwise
				pEncoder->old = code; //save the new encoder bits
				return pEncoder->direction;
			}
			break;

// remover remainder...
 *
		// clockwise change to anti-clockwise
		case 0x0102:
		case 0x1310:
		case 0x3231:
		case 0x2023:
		case 0x2010:
		case 0x0131:
		case 0x1323:
		case 0x3202:
			if (trackingCount > THRESHOLD){
				pEncoder->direction = -1; // qualified anti-clockwise
				pEncoder->old = code; //save the new encoder bits
				return pEncoder->direction;
			}
			break;

			// anti-clockwise change to clockwise
		case 0x2320:
		case 0x3132:
		case 0x1013:
		case 0x0201:
		case 0x0232:
		case 0x2313:
		case 0x3101:
		case 0x1020:
			if (trackingCount > THRESHOLD) {
				pEncoder->direction = +1; // qualified clockwise
				pEncoder->old = code; //save the new encoder bits
				return pEncoder->direction;
			}
			break;

	// ignore any other sequences and disqualify, assume clicks are in same direction
	default:
		if (trackingCount >0)
			trackingCount--;
		return 0;
		break;
	}

	}
	return 0;
}
 */

static void applyEncoderChange1(int8_t changeDirection)
{
	// Check for no change
	if (changeDirection == 0) {
		return;
	}

	// Are we pressed in?
	if (Encoders_IsFrequencyEncoderPressed()) {
		if (changeDirection > 0) {
			FrequencyManager_DecreaseFreqStepSize();
		} else {
			FrequencyManager_IncreaseFreqStepSize();
		}
	}
	else {

		if (NCOTUNE){
			if (changeDirection > 0) {
				Tune_NCO_Up( );
			} else {
				Tune_NCO_Down();
			}
		}
		else {
			if (changeDirection > 0) {
				FrequencyManager_StepFrequencyUp();
			} else {
				FrequencyManager_StepFrequencyDown();
			}
		}
	}

	// Filter screen requires an update band frequency
	if (Screen_GetScreenMode() == FILTER){
		int band = Screen_GetFilterFreqID();
		uint32_t currentFrequency = FrequencyManager_GetCurrentFrequency();
		FrequencyManager_SetBandFreqFilter (band, currentFrequency);
	}

}
static void applyEncoderChange2(int8_t changeDirection)
{
	int curOptIdx;
	int16_t currentValue;
	int16_t newValue=0;
	int16_t minValue;
	int16_t maxValue;
	static int loopCount = 0;

	// Check for no change
	if (changeDirection == 0) {
		return;
	}
	// Are we setting general options or the band filter code values
	// Options uses an array of min/max values
	// Filter sets a fixed 3-bit binary code
	if (Screen_GetScreenMode() != FILTER){
		/*
		 * Check the limits
		 */
		curOptIdx = Options_GetSelectedOption();
		currentValue = Options_GetValue(curOptIdx);
		int changeRate = Options_GetChangeRate(curOptIdx);
		if (changeRate == 0) {
			loopCount++;
			if (loopCount%4 == 0){
				newValue = currentValue + changeDirection;
			} else {
				newValue = currentValue;
			}
		} else
			newValue = currentValue + Options_GetChangeRate(curOptIdx) * changeDirection;
		minValue = Options_GetMinimum(curOptIdx);
		maxValue = Options_GetMaximum(curOptIdx);
		if (newValue < minValue) {
			newValue = minValue;
		}
		if (newValue > maxValue) {
			newValue = maxValue;
		}
		// Set the value & Display it

		Options_SetValue(curOptIdx, newValue);
	} else {
		int band = Screen_GetFilterCodeID();
		currentValue = FrequencyManager_GetFilterCode (band);
		int changeRate = BAND_FILTER_CHANGE_RATE;
		newValue = currentValue + changeRate * changeDirection;
		minValue = BAND_FILTER_MINIMUM;
		maxValue = BAND_FILTER_MAXIMUM;
		if (newValue < minValue) {
			newValue = minValue;
		}
		if (newValue > maxValue) {
			newValue = maxValue;
		}

		// Set the value & Display it
		FrequencyManager_SetBandCodeFilter (band, newValue);
	}
}



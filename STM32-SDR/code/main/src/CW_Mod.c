/*
 * Code that handles the CW transmit routines
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

#include "stm32f4xx_rcc.h"
#include "stm32f4xx_gpio.h"
#include "xprintf.h"
#include <assert.h>
#include "stm32f4xx_tim.h"
#include "Codec_Gains.h"
#include "ChangeOver.h"
#include "Band_Filter.h"
#include "ModeSelect.h"
#include "RS-HFIQ.h"

// Create a circular buffer used for reading/writing key samples.
// Need not match our sampling rate or the DMA rate. Just our data-storage area

static _Bool s_wantToTransmit = 0;
static char lastsample = 0;
static _Bool debouncedIsKeyPressed = 0;
static uint32_t tick=0, check=0;

// Number samples required on key to accept a change in state.
#define DEBOUNCE_THRESHOLD 10

// CW Wave form generation constants
#define SLEW_RATE 0.010          // Desired slew rate for CW (in ms)
#define SAMPLE_PERIOD 0.000125   // DMA sampling period for each single sample.
#define CW_AMPLITUDE_RISE_SLOPE (SAMPLE_PERIOD / SLEW_RATE)
#define CW_AMPLITUDE_FALL_SLOPE (-1 * CW_AMPLITUDE_RISE_SLOPE)

// Handle return to RX timeout
#define RETURN_TO_RX_TIME_MS       800
#define RETURN_TO_RX_COUNTER_RESET (RETURN_TO_RX_TIME_MS * 55 / 65)
	// 55 = approximate number of ISRs per DMA ISR
	// 65 = approximate number of ms per DMA ISR


// Prototypes:
static void initCwTimerInterrupt(void);
static void initCwGPIO(void);
//void CW_TestRoutine(void);

void watchdog (int reference){
	if (tick - check > 100)
		xprintf ("Watchdog: delay at %d\n", reference);
	check = tick;
}

void CW_Init(void)
{
	//CW_TestRoutine();

	initCwGPIO();
	initCwTimerInterrupt();
}

void initCwGPIO(void)
{
	debug(INIT, "initCwGPIO:\n");
	GPIO_InitTypeDef GPIO_InitStructure;

	/* Enable GPIOC clock */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	/* Enable SYSCFG clock */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

	/* Configure PC9 pin as input floating */
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
	GPIO_Init(GPIOE, &GPIO_InitStructure);

}

// Setup the timer interrupt
static void initCwTimerInterrupt(void)
{
	debug(INIT, "initCwGPIO:NVIC\n");
	NVIC_InitTypeDef NVIC_InitStructure;

	/* Enable the TIM3 global Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 5;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	/* TIM3 clock enable */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

	debug(INIT, "initCwGPIO:TIM_TimeBase\n");
	/* Time base configuration */
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_TimeBaseStructure.TIM_Period = 1000 - 1; // 1 MHz down to 1 KHz (1 ms)
	TIM_TimeBaseStructure.TIM_Prescaler = 84 - 1; // 24 MHz Clock down to 1 MHz (adjust per your clock)
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

	debug(INIT, "initCwGPIO:TIM_ITConfig\n");
	/* TIM IT enable */
	TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);
	debug(INIT, "initCwGPIO:Enable\n");
	/* TIM3 enable counter */
	TIM_Cmd(TIM3, ENABLE);
}


// Sampling
void writeKeySampleToKeyBuffer(char key)
{
	if(key != lastsample)  // only change sidetone if state changed.
		{
		if(key) Sidetone_Key_Down();
			else Sidetone_Key_Up();
		lastsample = key;
		}

}
// ISR which samples the key pin.
void CW_KeyPollTimerIRQ(void) {
	if (TIM_GetITStatus(TIM3, TIM_IT_Update ) != RESET) {
		tick++; // count mS
		GPIO_ToggleBits(GPIOE, GPIO_Pin_2); // Side tone oscillator
	if (Mode_GetCurrentMode() == MODE_CW) {

		// Read pin state

		_Bool isKeyPressed =0;
		if (Mode_GetCurrentMode() == MODE_CW)
			isKeyPressed = GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_5) == 0;

		// Debounce on

		static int debounceOnCounter = 0;
		static int debounceOffCounter = 0;
		static int returnToRxCounter = 0;

		if (isKeyPressed & !debouncedIsKeyPressed) {
			debounceOnCounter++;
			debug (CONTROL, "Debounce On %d %d %d\n", debounceOnCounter, isKeyPressed, debouncedIsKeyPressed);
			if (debounceOnCounter >= DEBOUNCE_THRESHOLD) {
				debouncedIsKeyPressed = 1;
				debounceOnCounter = 0;
			}
		} else {
			debounceOnCounter = 0;
		}

		if (!isKeyPressed & debouncedIsKeyPressed) {
			debounceOffCounter++;
			debug (CONTROL, "Debounce Off %d %d %d\n", debounceOffCounter, isKeyPressed, debouncedIsKeyPressed);
			if (debounceOffCounter >= DEBOUNCE_THRESHOLD) {
				debouncedIsKeyPressed = 0;
				debounceOffCounter = 0;
			}
		} else {
			debounceOffCounter = 0;
		}

		// Handle if we "want" to transmit:
		if (debouncedIsKeyPressed) {
			s_wantToTransmit = 1;
			returnToRxCounter = RETURN_TO_RX_COUNTER_RESET;
			GPIO_SetFilter(1);
//			xprintf("Tx\n");
		} else {
			returnToRxCounter --;
//			xprintf("counter %d\n", returnToRxCounter);
			if (returnToRxCounter <= 0) {
				s_wantToTransmit = 0;
				GPIO_SetFilter(3);
				returnToRxCounter = 0;
//				xprintf("Rx\n");
			}
		}

		writeKeySampleToKeyBuffer(debouncedIsKeyPressed);

	}
//	xprintf("c");
	TIM_ClearITPendingBit(TIM3, TIM_IT_Update );
	}
}

_Bool CW_DesiresTransmitMode(void)
{
	return s_wantToTransmit;
}

// Called in the DMA IRQ to get the desired amplitudes for each sample.
// Buffer to be filled with values between 0 and 1; I/Q encoding done
// by calling code.

void CW_FillTxAmplitudeBuffer(float amplitudeBuffer[], int bufferSize)
{
	// Record algorithm state between executions
	// ..amplitude percentage
	static float curAmplitude = 0.0;

	// Use integer math to interpolate between key-sample points.

	for (int ampBuffIdx = 0; ampBuffIdx < bufferSize; ampBuffIdx++) {

		// Handle ramping up/down:
		float amplitudeChange = debouncedIsKeyPressed ? CW_AMPLITUDE_RISE_SLOPE : CW_AMPLITUDE_FALL_SLOPE;
		curAmplitude += amplitudeChange;
		if (curAmplitude > 1.0) {
			curAmplitude = 1.0;

		}
		if (curAmplitude < 0) {
			curAmplitude = 0;
		}

		// Store the value:

		amplitudeBuffer[ampBuffIdx] = curAmplitude;

	}

	// Ensure the above interpolation algorithm applied the expected number of steps:
//	assert(s_keySampleReadIdx == (latestSampleIdx+ 1) % KEY_SAMPLE_ARRAY_SIZE);
}




/*
 * CW TESTING CODE
 */
#if 0
#define DMA_BUFFER_SIZE             512
#define KEY_SAMPLES_PER_DMA_BUFFER  55	// What hardware currently achieves.

#define SAMPLES_PER_DOT    10
#define SAMPLES_PER_DASH   (SAMPLES_PER_DOT * 3)
#define SAMPLES_BETWEEN_DOTDASH    SAMPLES_PER_DOT
#define SAMPLES_PER_SPACE  (SAMPLES_PER_DOT * 3)
#define SAMPLES_PER_SLASH  SAMPLES_PER_DOT // There are spaces on either size of a /, should be worth 7 dots, but minus 3*2 = 1
#define SAMPLE_AT_END      (KEY_SAMPLES_PER_DMA_BUFFER) // There are spaces on either size of a /, should be worth 7, but minus 3*2 = 1



void cwTestTx(_Bool isKeyDown, int sampleCount)
{
	static int numKeySamplesWritten = 0;

	for (int i = 0; i < sampleCount; i++) {
		// Insert this key data
		xprintf("%d,", isKeyDown);
		writeKeySampleToKeyBuffer(isKeyDown);
		numKeySamplesWritten++;

		// Simulate DMA buffer swap:
		if (numKeySamplesWritten  > 0 &&
			numKeySamplesWritten % KEY_SAMPLES_PER_DMA_BUFFER == 0)
		{
			float cwAmplitudes[DMA_BUFFER_SIZE];
			CW_FillTxAmplitudeBuffer(cwAmplitudes, DMA_BUFFER_SIZE);

			xprintf("\nDMA Buffer Amplitudes:\n");
			for (int i = 0; i < DMA_BUFFER_SIZE; i++) {
				int amp = cwAmplitudes[i] * 100;
				xprintf("%d,", amp);
			}

			xprintf("\n\nKey Samples:\n");

		}
	}
}

void CW_TestRoutine(void)
{
	xprintf("*********************************\n");
	xprintf("Beginning CW Testing....\n");
	xprintf("*********************************\n");
//	const char* message = "   --- ...";
	//const char* message = "   --- ... --- ... --- ... --- ... --- ... --- ... --- ... --- ... --- ... --- ...";
	const char* message = "   .... . .-.. .-.. --- / .-- --- .-. .-.. -.. / - .... .. ... / .. ... / .- / - . ... - / -- . ... ... .- --. . / .-- .... .. -.-. .... / .-- .. .-.. .-.. / -... . / -.-. --- -. ...- . .-. - . -.. / - --- / -- --- .-. ... . / -.-. --- -.. . .-.-.-";


	// Generate the message:
	_Bool wasDotOrDash = 0;
	for (int charIdx = 0; message[charIdx] != 0; charIdx++) {
		char nextChar = message[charIdx];

		_Bool isDotOrDash = (nextChar == '.' || nextChar == '-');
		if (wasDotOrDash && isDotOrDash) {
			cwTestTx(0, SAMPLES_BETWEEN_DOTDASH);
		}
		wasDotOrDash = isDotOrDash;

		switch (nextChar) {
		case '.':
			cwTestTx(1, SAMPLES_PER_DOT);
			break;
		case '-':
			cwTestTx(1, SAMPLES_PER_DASH);
			break;
		case ' ':
			cwTestTx(0, SAMPLES_PER_SPACE);
			break;
		case '/':
			cwTestTx(0, SAMPLES_PER_SLASH);
			break;
		default:
			assert(0);
		}
	}
	cwTestTx(0, SAMPLE_AT_END);

	xprintf("*********************************\n");
	xprintf("Done CW Testing.\n");
	xprintf("*********************************\n");

	// Don't operate otherwise we'll likely transmit part of the sample message!
	assert(0);

}
#endif


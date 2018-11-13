/*
 * Code that handles switching the various mode from TX to RX
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

#include "ChangeOver.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "Codec_Gains.h"
#include "Init_I2C.h"  //Referenced for Delay(n);
#include "PSKMod.h"
#include "CW_Mod.h"
#include "DMA_IRQ_Handler.h"
#include "options.h"
#include "ScrollingTextBox.h"
#include "AGC_Processing.h"
#include "ModeSelect.h"
#include "Init_Codec.h"
#include "FrequencyManager.h"
#include <assert.h>
#include "main.h"
#include <xprintf.h>
#include <yprintf.h>
#include "Si570.h"
#include "RS-HFIQ.h"
#include "Band_Filter.h"

// Constants
//const int DEBOUNCE_COUNT_REQUIRED = 3;	// Called from IRQ
const int DEBOUNCE_COUNT_REQUIRED = 10;	// Called from main()

// Private state variables:
static _Bool s_isPttPressed = 0;
static _Bool s_inTxMode = 0;
//static uint16_t s_inTransition = 0;

// Private functions:
static void Receive_Sequence(void);
static void Xmit_SSB_Sequence(void);
static void Xmit_CW_Sequence(void);
static void Xmit_PSK_Sequence(void);
static void Init_PTT_IO(void);
void handlePttStateChange(void);

/****************************************
 * Public interface
 ****************************************/
// Initialize radio at startup.
void RxTx_Init(void)
{
	debug (CONTROL, "RxTx_Init\n");
	Init_PTT_IO();
}

// Control switching between receive and transmit mode.
void RxTx_SetReceive(void)
{
	debug (CONTROL, "RxTx_SetReceive\n");
	if (FrequencyManager_isSplit())
		FrequencyManager_SetRxFrequency();
	Receive_Sequence();
	if (!Si570_isEnabled()) {
		RS_HFIQ_Command_TX(DISABLE);
	}
}

void RxTx_SetTransmit(void)
{
	debug (CONTROL, "RxTx_SetTransmit\n");
	switch(Mode_GetCurrentMode()) {
	case MODE_SSB:
		Xmit_SSB_Sequence();
		break;
	case MODE_CW:
		if (FrequencyManager_isSplit())
			FrequencyManager_SetTxFrequency();
		Xmit_CW_Sequence();
		break;
	case MODE_PSK:
		Xmit_PSK_Sequence();
		break;
	default:
		assert(0);
	}
	if (!Si570_isEnabled()) {
		RS_HFIQ_Command_TX(ENABLE); //??

	}
}

// Query current mode (transmit or receive).
_Bool RxTx_InRxMode(void)
{
	return !s_inTxMode;
}
_Bool RxTx_InTxMode(void)
{
	return s_inTxMode;
}

// Handle Push To Talk
void RxTx_CheckAndHandlePTT(void)
{
	// Debounce PTT signal changes
	static int debounceCount = 0;

	/* Note on PTT pins:
	 * Connector "Key" has:
	 *   Pin 1: PTT_ --> Port D, Pin 2
	 *   Pin 5: Key1 --> Port C, Pin 9
	 *
	 * Pin C9 is the "normal" PTT pin, D2 allows room for future expansion
	 * of the PTT approach and support unique input pins for dot vs dash generation.
	 */
//
	_Bool isKeyDown=0;
//	if (Mode_GetCurrentMode() == MODE_CW)
//		isKeyDown = GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_5) == 0;
//	else
		isKeyDown = GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_9) == 0;

	// Are we trying to change?
	if (isKeyDown != s_isPttPressed) {
		// Debounce:
		if (debounceCount < DEBOUNCE_COUNT_REQUIRED) {
			// Count up:
			debounceCount++;
//			debug (CONTROL, "debounce %d\n", debounceCount);

			// Debounce done?
			if (debounceCount >= DEBOUNCE_COUNT_REQUIRED) {
				s_isPttPressed = isKeyDown;
				debounceCount = 0;
//				debug (CONTROL, "PTT Changed to %d\n", s_isPttPressed);
				handlePttStateChange();
			}
		}
	} else {
		// Not currently trying to change:
		debounceCount = 0;
	}


	// Special case to handle CW:
	if (Mode_GetCurrentMode() == MODE_CW) {
		if (CW_DesiresTransmitMode() && !RxTx_InTxMode()) {
			debug (CONTROL, "CW_DesiresTransmitMode() && !RxTx_InTxMode()\n");
//		if (s_isPttPressed && !RxTx_InTxMode()) {
			Connect_Sidetone_Input();  //  Route the CW Sidetone to Headphones
			RxTx_SetTransmit();
			debug (CONTROL, "To CW Tx\n");
		}
		if (!CW_DesiresTransmitMode() && RxTx_InTxMode()) {
			debug (CONTROL, "!CW_DesiresTransmitMode() && RxTx_InTxMode() && !isKeyDown\n");
			Disconnect_Sidetone_Input();
			GPIO_SetFilter(0);
			RxTx_SetReceive();
			debug (CONTROL, "To CW Rx\n");

		}
	}
}

_Bool RxTx_IsPttPressed(void)
{
	return s_isPttPressed;
}


/****************************************
 * Private Functions
 ****************************************/
void Receive_Sequence(void)
{
	debug (CONTROL, "Receive_Sequence\n");
	//Codec_Init();

	Mute_HP();
	Mute_LO();

	// Added by CS to fix changeover bug 20170907
	Disconnect_Sidetone_Input();  //  Disconnect the CW Sidetone to Headphones

	s_inTxMode = 0;
	if (AGC_Mode != 3)
	{AGC_On =1;
	Init_AGC();
	}
	else
	{
	AGC_On =0;  //This forces the manual AGC mode
	Init_AGC();  //added JDG
	}
	GPIO_WriteBit(GPIOD, GPIO_Pin_3, Bit_SET);	//Make PTT_Out High, Remember FET Inversion
	Disconnect_PGA();

	Connect_IQ_Inputs();
	Set_DAC_DVC(Options_GetValue(OPTION_RX_AUDIO));
	Set_ADC_DVC(-10);  //was -20 using Milt's AGC scheme
	Set_HP_Gain(6);

}

void Xmit_SSB_Sequence(void)
{
	debug (CONTROL, "Xmit_SSB_Sequence\n");

	Mute_HP();
	Mute_LO();
	s_inTxMode = 1;
	AGC_On = 0;  //Turn off AGC so that DAC is held constant during transmit
	//	Init_AGC();  //added JDG ???

	Disconnect_Sidetone_Input();  //added JDG
	//	Disconnect_PGA(); //???
	Set_DAC_DVC(15); //SSB Xmit DAC Gain
	Set_ADC_DVC(0);
	Connect_Microphone_Input();

	Set_PGA_Gain(Options_GetValue(OPTION_Mic_Gain));
	GPIO_WriteBit(GPIOD, GPIO_Pin_3, Bit_RESET);  //Make PTT_Out Low,Remember FET Inversion
	//Set_LO_Gain(24);
	Set_LO_Gain(Options_GetValue(OPTION_Tx_LEVEL));
}

void Xmit_CW_Sequence(void)
{
	debug (CONTROL, "Xmit_CW_Sequence\n");
	//Mute_HP();
	Mute_LO();
	s_inTxMode = 1;
	AGC_On = 0;
	//Init_AGC();  //added JDG
	Set_DAC_DVC(-33); //CW Xmit DAC Gain
	GPIO_WriteBit(GPIOD, GPIO_Pin_3, Bit_RESET);  //Make PTT_Out Low,Remember FET Inversion
//	Delay(1000);
	//Set_LO_Gain(20);
	Set_LO_Gain(Options_GetValue(OPTION_Tx_LEVEL));

}

void Xmit_PSK_Sequence(void)
{
	debug (CONTROL, "Xmit_PSK_Sequence\n");
	Mute_HP();
	Mute_LO();
	s_inTxMode = 1;
	AGC_On = 0;  //Turn off AGC so that DAC is held constant during transmit
	Disconnect_Sidetone_Input();  //added JDG
	//Disconnect_PGA();
	Set_DAC_DVC(12); //PSK Xmit DAC Gain
	GPIO_WriteBit(GPIOD, GPIO_Pin_3, Bit_RESET);  //Make PTT_Out Low,Remember FET Inversion
	Delay(1000);
	//Set_LO_Gain(20);
	Set_LO_Gain(Options_GetValue(OPTION_Tx_LEVEL));
	Set_HP_Gain(Options_GetValue(OPTION_ST_LEVEL)); // scaled back due to CW side tone modification
}

void Init_PTT_IO(void)
{
	debug (CONTROL, "Init_PTT_IO\n");
	GPIO_InitTypeDef GPIO_InitStruct;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);

	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_2; // Pin 2 is PTT_In
	GPIO_Init(GPIOD, &GPIO_InitStruct);

	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_3; //  Pin 3 controls the FET-Gate for PTT_Out
	GPIO_Init(GPIOD, &GPIO_InitStruct);

	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStruct.GPIO_Pin = (GPIO_Pin_5 | GPIO_Pin_6); // PE5/6 (SS1 B2/3) are the CW paddle
	GPIO_Init(GPIOE, &GPIO_InitStruct);

}

// Called when Push To Talk state has changed (after debounce)
void handlePttStateChange(void)
{
	debug (CONTROL, "handlePttStateChange\n");
	switch (Mode_GetCurrentMode()) {
	case MODE_SSB:
		if (s_isPttPressed) {
			RxTx_SetTransmit();
		} else {
			RxTx_SetReceive();
		}
		break;
	case MODE_CW:
		break;
	case MODE_PSK:
		break;
	default:
		assert(0);
	}
}

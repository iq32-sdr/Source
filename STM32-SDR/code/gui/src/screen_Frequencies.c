/*
 * Code for frequency manager and selection screen
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
#include "screen_All.h"
#include "widgets.h"
#include <assert.h>
#include "FrequencyManager.h"
#include "Text_Enter.h"
#include "ModeSelect.h"
#include "xprintf.h"

// Used in this file to refer to the correct screen (helps to keep code copy-paste friendly.
static GL_Page_TypeDef *s_pThisScreen = &g_screenFrequencies;

// Store reference to lables to change them.
GL_PageControls_TypeDef* lbl1;
GL_PageControls_TypeDef* lbl2;
GL_PageControls_TypeDef* lbl3;
GL_PageControls_TypeDef* lbl4;
GL_PageControls_TypeDef* btnMode;

#define ID_FREQBTN_START     200
#define FIRST_BUTTON_Y       30
#define SPACE_PER_BUTTON_Y   30
#define LEFT_COL_X           0
#define COLUMN_WIDTH         110
#define BUTTONS_PER_COLUMN   5

#define TRUE 1
#define FALSE 0

static int editMode = FALSE;
static BandPreset saveBand;
//static uint8_t toggleOffsetButton=0;

extern void set_kybd_mode (int mode);

void displayPresetInstructions(void);
void displayEditInstructions(void);

/**
 * Call-back prototypes
 */
static void freqButton_Click(GL_PageControls_TypeDef* pThis);
static void edit_Click(GL_PageControls_TypeDef* pThis);
static void mode_Click(GL_PageControls_TypeDef* pThis);

/**
 * Create the screen
 */
void ScreenFrequencies_Create(void)
{
	Create_PageObj(s_pThisScreen);

	/*
	 * Create the UI widgets
	 */
	GL_PageControls_TypeDef* lblTitle = Widget_NewLabel("Frequencies", LCD_COLOR_WHITE, LCD_COLOR_BLACK, 0, GL_FONTOPTION_12x12, 0);
	lbl1 = Widget_NewLabel("Tap a band to select.", LCD_COLOR_WHITE, LCD_COLOR_BLACK, 0, GL_FONTOPTION_8x12Bold, 0);
	lbl2 = Widget_NewLabel("", LCD_COLOR_WHITE, LCD_COLOR_BLACK, 0, GL_FONTOPTION_8x12Bold, 0);
	lbl3 = Widget_NewLabel("Press & turn frequency knob", LCD_COLOR_WHITE, LCD_COLOR_BLACK, 0, GL_FONTOPTION_8x12Bold, 0);
	lbl4 = Widget_NewLabel("to set frequency step size.", LCD_COLOR_WHITE, LCD_COLOR_BLACK, 0, GL_FONTOPTION_8x12Bold, 0);


	GL_PageControls_TypeDef* btnFreqBigButton = Widget_NewBigButtonFrequency();
	GL_PageControls_TypeDef* btnEdit = NewButton(0, " Edit ", edit_Click);

	/*
	 * Place the widgets onto the screen
	 */
	// Title
	AddPageControlObj(  0,   0, lblTitle,        s_pThisScreen);

	// Populate the frequency buttons:
	for (int i = 0; i < FREQBAND_NUMBER_OF_BANDS-1; i++) {
		GL_PageControls_TypeDef* btnFreq = NewButton(ID_FREQBTN_START + i, FrequencyManager_DisplayBandName(i), freqButton_Click);

		int x = (i / BUTTONS_PER_COLUMN) * COLUMN_WIDTH;
		int y = (i % BUTTONS_PER_COLUMN) * SPACE_PER_BUTTON_Y + FIRST_BUTTON_Y;
		AddPageControlObj(x, y, btnFreq, s_pThisScreen);
	}

	// Button Row
	AddPageControlObj(150, 0, lbl1,  s_pThisScreen);
	AddPageControlObj(150, 10, lbl2,  s_pThisScreen);
	AddPageControlObj(0,   LCD_HEIGHT - 62, lbl3,  s_pThisScreen);
	AddPageControlObj(0,   LCD_HEIGHT - 46, lbl4,  s_pThisScreen);
	AddPageControlObj(0,  LCD_HEIGHT - 30, btnEdit, s_pThisScreen);
	AddPageControlObj(
			LCD_WIDTH - ((GL_Custom_TypeDef*)(btnFreqBigButton->objPTR))->GetWidth(btnFreqBigButton),
			LCD_HEIGHT - ((GL_Custom_TypeDef*)(btnFreqBigButton->objPTR))->GetHeight(btnFreqBigButton),
			btnFreqBigButton, s_pThisScreen);

//create mode button and make it invisible for now
	btnMode = NewButton(1, " Mode ", mode_Click);
	GL_Button_TypeDef* pThat = (GL_Button_TypeDef*) (btnMode->objPTR);
	AddPageControlObj(160,   LCD_HEIGHT - 30, btnMode, s_pThisScreen);
	pThat->Control_Visible = GL_FALSE;
	RefreshPageControl (s_pThisScreen, 1);


}

void displayPresetInstructions(void){
	debug(GUI, "displayPresetInstructions: editMode = %x\n", editMode);
	Widget_ChangeLabelText (lbl1, "Tap a band to select.");
	Widget_ChangeLabelText (lbl2, "");
	Widget_ChangeLabelText (lbl3, "Press & turn frequency knob to set");
	Widget_ChangeLabelText (lbl4, "the frequency step size.");
}

void displayEditInstructions(void){
	debug(GUI, "displayEditInstructions: editMode = %x\n", editMode);
	Widget_ChangeLabelText (lbl1, "Tap band to edit.    ");
	Widget_ChangeLabelText (lbl2, "");
	Widget_ChangeLabelText (lbl3, "                                  ");
	Widget_ChangeLabelText (lbl4, "                            ");
}

static void freqEditButton_Click(GL_PageControls_TypeDef* pThis)
{
	debug(GUI, "freqEditButton_Click: editMode = %x\n", editMode);
	uint16_t id = pThis->ID - ID_FREQBTN_START;
	editMode=TRUE;
	Freq_SetSelectedText(id);
	displayEditInstructions ();
	if (id != FREQBAND_OFFSET) {
	FreqName_Display();
	set_kybd_mode(3);
	} else {
//		toggleOffsetButton++;
//		toggleOffsetButton = toggleOffsetButton%2;
//		if (toggleOffsetButton){
			Widget_ChangeLabelText (lbl3, "Set transverter LO frequency.");
			Widget_ChangeLabelText (lbl4, "Adjust frequency knob");
//		} else {
//			FrequencyManager_SetSelectedBand(FREQBAND_SI570_F0);
//			Widget_ChangeLabelText (lbl3, "Set Si570 frequency");
//			Widget_ChangeLabelText (lbl4, "Adjust frequency knob");
//		}
	}
}

void Screen_ChangeButtonFreqLabel(int i){
	debug(GUI, "Screen_ChangeButtonFreqLabel: editMode = %x\n", editMode);
	ChangeButtonText(s_pThisScreen, ID_FREQBTN_START + i, FrequencyManager_DisplayBandName(i));
//	FrequencyManager_SaveCurrentFrequency();
}

/**
 * Callbacks
 */
static void freqButton_Click(GL_PageControls_TypeDef* pThis)
{
	debug(GUI, "freqButton_Click: editMode = %x\n", editMode);

	uint16_t userBand = pThis->ID - ID_FREQBTN_START;
	assert(userBand >= 0 && userBand <= FREQBAND_NUMBER_OF_BANDS);
	if (editMode==TRUE){
		FrequencyManager_SaveCurrentFrequency();
		FrequencyManager_SetSelectedBand(userBand);
		freqEditButton_Click(pThis);

	} else {
		if (userBand != FREQBAND_OFFSET){
			FrequencyManager_SetSelectedBand(userBand);
		}
		FrequencyManager_ReadBandsFromEeprom();
		Screen_SetScreenMode(MAIN);
		displayPresetInstructions ();
		Screen_PSK_SetTune();
		Screen_ShowMainScreen();

	}
}

static void edit_Click(GL_PageControls_TypeDef* pThis)
{
	debug(GUI, "edit_Click: editMode = %x\n", editMode);
	if (Screen_GetScreenMode()==FREQUENCY){
		if (FrequencyManager_GetSelectedBand() != FREQBAND_OFFSET)
			saveBand = FrequencyManager_GetSelectedBand();
		GL_Button_TypeDef* pThat = (GL_Button_TypeDef*) (btnMode->objPTR);
		pThat->Control_Visible = GL_TRUE; // make the mode button visible
		Screen_SetScreenMode(FREQEDIT);
		displayEditInstructions ();
		RefreshPageControl (s_pThisScreen, 1);
//		toggleOffsetButton=0;
		editMode=TRUE;
	} else {
		FrequencyManager_SaveCurrentFrequency();
		Screen_SetScreenMode(FREQUENCY);
		editMode=FALSE;
		set_kybd_mode(0);
		GL_Button_TypeDef* pThat = (GL_Button_TypeDef*) (btnMode->objPTR);
		pThat->Control_Visible = GL_FALSE; // make the mode button invisible
		RefreshPageControl (s_pThisScreen, 1);
		displayPresetInstructions ();
	}
}

static void mode_Click(GL_PageControls_TypeDef* pThis) {
	// Reset to defaults in preparation for next display.
//	displayNormalFeedback();
	debug(GUI, "mode_Click: editMode = %x\n", editMode);
	GL_Button_TypeDef* pThat = (GL_Button_TypeDef*) (btnMode->objPTR);
	if (pThat->Control_Visible == GL_TRUE){

	UserModeType userMode = Mode_GetCurrentUserMode();
	debug(GUI, "mode_Click:userMode = before %d\n", userMode);
	if (userMode<USERMODE_NUM_MODES-2)
		userMode++;
	else
		userMode = USERMODE_USB;
	debug(GUI, "mode_Click:userMode after = %d\n", userMode);
	Mode_SetCurrentMode(userMode);
	FreqName_Display();
	}
}

void Screen_FreqDone(void) {
	// Reset to defaults in preparation for next display.
	debug(GUI, "screen_FreqDone: editMode = %x\n", editMode);
	if (editMode==TRUE)
		FrequencyManager_SaveCurrentFrequency();

	BandPreset userBand = FrequencyManager_GetSelectedBand();
	if (userBand >= FREQBAND_OFFSET)
		FrequencyManager_SetSelectedBand(saveBand);

	Screen_ChangeButtonFreqLabel(FrequencyManager_GetSelectedBand());
	editMode=FALSE;
	set_kybd_mode(0);
	GL_Button_TypeDef* pThat = (GL_Button_TypeDef*) (btnMode->objPTR);
	pThat->Control_Visible = GL_FALSE; // make the mode button invisible
	RefreshPageControl (s_pThisScreen, 1);
	Screen_SetScreenMode(MAIN);
	Screen_PSK_SetTune();
	displayPresetInstructions ();
	Screen_ShowMainScreen();
}

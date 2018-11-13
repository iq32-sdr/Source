/*
 * si5351.cpp - Si5351 library for Arduino
 *
 * Copyright (C) 2015 Jason Milldrum <milldrum@gmail.com>
 *                    Dana H. Myers <k6jq@comcast.net>
 *
 * Some tuning algorithms derived from clk-si5351.c in the Linux kernel.
 * Sebastian Hesselbarth <sebastian.hesselbarth@gmail.com>
 * Rabeeh Khoury <rabeeh@solid-run.com>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdint.h>

//#include "Arduino.h"
//#include "Wire.h"
#include "si5351.h"
#include <stdio.h>		// For sprintf()
#include "Init_I2C.h"
#include "xprintf.h"


/********************/
/* Public functions */
/********************/

//Si5351::Si5351(void)
//{
	struct Si5351Status dev_status;
	struct Si5351IntStatus dev_int_status;
	int32_t ref_correction;

	uint8_t lock_plla = SI5351_CLKNONE;
	uint8_t lock_pllb = SI5351_CLKNONE;
	uint8_t clk0_int_mode = 0;
	uint8_t clk1_int_mode = 0;
	uint8_t clk2_int_mode = 0;
	uint64_t plla_freq = (unsigned long long) 0;
	uint64_t pllb_freq = (unsigned long long) 0;
	uint64_t clk0_freq = (unsigned long long) 0;
	uint64_t clk1_freq = (unsigned long long) 0;
	uint64_t clk2_freq = (unsigned long long) 0;
	uint32_t xtal_freq = SI5351_XTAL_FREQ;
//}
//private:
	uint64_t Si5351_pll_calc(uint64_t, struct Si5351RegSet *, int32_t);
	uint64_t Si5351_multisynth_calc(uint64_t, uint64_t, struct Si5351RegSet *);
	void Si5351_update_sys_status(struct Si5351Status *);
	void Si5351_update_int_status(struct Si5351IntStatus *);
	void Si5351_ms_div(si5351_clock, uint8_t, uint8_t);
	uint8_t Si5351_select_r_div(uint64_t *);
	void Si5351_dump (void);

/*
 * init(uint8_t xtal_load_c, uint32_t ref_osc_freq)
 *
 * Setup communications to the Si5351 and set the crystal
 * load capacitance.
 *
 * xtal_load_c - Crystal load capacitance. Use the SI5351_CRYSTAL_LOAD_*PF
 * defines in the header file
 * ref_osc_freq - Crystal/reference oscillator frequency in 1 Hz increments.
 * Defaults to 25000000 if a 0 is used here.
 *
 */
void Si5351_init(uint8_t xtal_load_c, uint32_t ref_osc_freq)
{
	debug (RS_HFIQ, "Si5351_init:start\n");
	// Start I2C comms
//	Wire.begin();

	// Set crystal load capacitance
	Si5351_write(SI5351_CRYSTAL_LOAD, xtal_load_c);

	// Change the ref osc freq if different from default
	if (ref_osc_freq != 0)
	{
		xtal_freq = ref_osc_freq;
	}

	// Initialize the CLK outputs according to flowchart in datasheet
	// First, turn them off
	Si5351_write(16, 0x80);
	Si5351_write(17, 0x80);
	Si5351_write(18, 0x80);

	// Turn the clocks back on...
	Si5351_write(16, 0x0c);
	Si5351_write(17, 0x0c);
	Si5351_write(18, 0x0c);

	// Then reset the PLLs
	Si5351_pll_reset(SI5351_PLLA);
	Si5351_pll_reset(SI5351_PLLB);
	debug (RS_HFIQ, "Si5351_init:done\n");
}

/*
 * set_freq(uint64_t freq, uint64_t pll_freq, enum si5351_clock output)
 *
 * Sets the clock frequency of the specified CLK output
 *
 * freq - Output frequency in Hz
 * pll_freq - Frequency of the PLL driving the Multisynth
 *   Use a 0 to have the function choose a PLL frequency
 * clk - Clock output
 *   (use the si5351_clock enum)
 */
uint8_t Si5351_set_freq(uint64_t freq, uint64_t pll_freq, si5351_clock clk)
{
	debug (RS_HFIQ, "Si5351_set_freq:start freq=%d,pll_freq=%d,clk=%d\n", freq, pll_freq, clk);
	struct Si5351RegSet ms_reg; //, pll_reg;
	si5351_pll target_pll;
	uint8_t write_pll = 0;
//	uint8_t reg_val;
	uint8_t r_div = SI5351_OUTPUT_CLK_DIV_1;
	uint8_t int_mode = 0;
	uint8_t div_by_4 = 0;

	// PLL bounds checking
	if(pll_freq != 0)
	{
		if ((pll_freq < SI5351_PLL_VCO_MIN * SI5351_FREQ_MULT) || (pll_freq > SI5351_PLL_VCO_MAX * SI5351_FREQ_MULT))
		{
			debug (RS_HFIQ, "Si5351_set_freq: Error 1\n");
			return 1;
		}
	}

	// Lower bounds check
	if(freq < SI5351_CLKOUT_MIN_FREQ * SI5351_FREQ_MULT)
	{
		freq = SI5351_CLKOUT_MIN_FREQ * SI5351_FREQ_MULT;
	}

	// Upper bounds check
	if(freq > SI5351_MULTISYNTH_MAX_FREQ * SI5351_FREQ_MULT)
	{
		freq = SI5351_MULTISYNTH_MAX_FREQ * SI5351_FREQ_MULT;
	}

	// Select the proper R div value
	r_div = Si5351_select_r_div(&freq);

	// Calculate the synth parameters
	// If pll_freq is 0 and freq < 150 MHz, let the algorithm pick a PLL frequency
	if((pll_freq) && (freq < SI5351_MULTISYNTH_DIVBY4_FREQ * SI5351_FREQ_MULT))
	{
		Si5351_multisynth_calc(freq, pll_freq, &ms_reg);
		write_pll = 0;
		div_by_4 = 0;
		int_mode = 0;

		switch(clk)
		{
		case SI5351_CLK0:
			clk0_freq = freq;
			break;
		case SI5351_CLK1:
			clk1_freq = freq;
			break;
		case SI5351_CLK2:
			clk2_freq = freq;
			break;
		default:
			break;
		}
	}
	else
	{
		// The PLL must be calculated and set by firmware when 150 MHz <= freq <= 160 MHz
		if(freq >= SI5351_MULTISYNTH_DIVBY4_FREQ * SI5351_FREQ_MULT)
		{
			pll_freq = Si5351_multisynth_calc(freq, 0, &ms_reg);
			write_pll = 1;
			div_by_4 = 1;
			int_mode = 1;
		}

		// Determine which PLL to use
		// CLK0 gets PLLA, CLK1 gets PLLB
		// CLK2 gets PLLB if necessary
		// Only good for Si5351A3 variant at the moment
		switch(clk)
		{
		case SI5351_CLK0:
			pll_freq = Si5351_multisynth_calc(freq, 0, &ms_reg);
			target_pll = SI5351_PLLA;
			write_pll = 1;
			Si5351_set_ms_source(SI5351_CLK0, SI5351_PLLA);

			plla_freq = pll_freq;
			clk0_freq = freq;
			break;
		case SI5351_CLK1:
			// Check to see if PLLB is locked due to other output being < 1.024 MHz or >= 112.5 MHz
			if(lock_pllb == SI5351_CLK2)
			{
				// We can't have a 2nd output < 1.024 MHz or >= 112.5 MHz on the same PLL unless exact same freq, so exit
				if((freq >= SI5351_MULTISYNTH_SHARE_MAX * SI5351_FREQ_MULT
					|| freq < SI5351_CLKOUT_MIN_FREQ * SI5351_FREQ_MULT * 128) && freq != clk2_freq)
				{
					clk1_freq = 0;
					debug (RS_HFIQ, "Si5351_set_freq: Error 2\n");
					return 1;
				}
				// Else, set multisynth to same PLL freq as CLK2
				else
				{
					pll_freq = pllb_freq;
					Si5351_multisynth_calc(freq, pll_freq, &ms_reg);
					write_pll = 0;
					Si5351_set_ms_source(SI5351_CLK1, SI5351_PLLB);
				}
			}
			else
			{
				pllb_freq = pll_freq;
				pll_freq = Si5351_multisynth_calc(freq, 0, &ms_reg);
				write_pll = 1;
				Si5351_set_ms_source(SI5351_CLK1, SI5351_PLLB);
			}

			if(freq >= SI5351_MULTISYNTH_SHARE_MAX * SI5351_FREQ_MULT || freq < SI5351_CLKOUT_MIN_FREQ * SI5351_FREQ_MULT * 128)
			{
				lock_pllb = SI5351_CLK1;

				// Recalc and rewrite the multisynth parameters on CLK2
				if(clk2_freq != 0)
				{
					struct Si5351RegSet ms_temp_reg;
					r_div = Si5351_select_r_div(&clk2_freq);
					Si5351_multisynth_calc(clk2_freq, pllb_freq, &ms_temp_reg);
					Si5351_set_ms(SI5351_CLK2, ms_temp_reg, 0, r_div, 0);
				}
			}
			else
			{
				lock_pllb = SI5351_CLKNONE;
			}

			target_pll = SI5351_PLLB;
			clk1_freq = freq;
			break;
		case SI5351_CLK2:
			// Check to see if PLLB is locked due to other output being < 1.024 MHz or >= 112.5 MHz
			if(lock_pllb == SI5351_CLK1)
			{
				// We can't have a 2nd output < 1.024 MHz  or >= 112.5 MHz on the same PLL unless exact same freq, so exit
				if((freq >= SI5351_MULTISYNTH_SHARE_MAX * SI5351_FREQ_MULT
					|| freq < SI5351_CLKOUT_MIN_FREQ * SI5351_FREQ_MULT * 128) && freq != clk2_freq)
				{
					clk2_freq = 0;
					debug (RS_HFIQ, "Si5351_set_freq: Error 3\n");
					return 1;
				}
				// Else, set multisynth to same PLL freq as CLK1
				else
				{
					pll_freq = pllb_freq;
					Si5351_multisynth_calc(freq, pll_freq, &ms_reg);
					write_pll = 0;
					Si5351_set_ms_source(SI5351_CLK2, SI5351_PLLB);
				}
			}
			// need to account for CLK2 set before CLK1
			else
			{
				pllb_freq = pll_freq;
				pll_freq = Si5351_multisynth_calc(freq, 0, &ms_reg);
				write_pll = 1;
				Si5351_set_ms_source(SI5351_CLK2, SI5351_PLLB);
			}

			if(freq >= SI5351_MULTISYNTH_SHARE_MAX * SI5351_FREQ_MULT || freq < SI5351_CLKOUT_MIN_FREQ * SI5351_FREQ_MULT * 128)
			{
				lock_pllb = SI5351_CLK2;

				if(clk1_freq != 0)
				{
					// Recalc and rewrite the multisynth parameters on CLK1
					struct Si5351RegSet ms_temp_reg;
					r_div = Si5351_select_r_div(&clk1_freq);
					Si5351_multisynth_calc(clk1_freq, pllb_freq, &ms_temp_reg);
					Si5351_set_ms(SI5351_CLK1, ms_temp_reg, 0, r_div, 0);
				}
			}
			else
			{
				lock_pllb = SI5351_CLKNONE;
			}

			target_pll = SI5351_PLLB;
			clk2_freq = freq;
			break;
		default:
			debug (RS_HFIQ, "Si5351_set_freq: Error 4\n");
			return 1;
		}
	}

	// Set multisynth registers (MS must be set before PLL)
	Si5351_set_ms(clk, ms_reg, int_mode, r_div, div_by_4);

	// Set PLL if necessary
	if(write_pll == 1)
	{
		Si5351_set_pll(pll_freq, target_pll);
	}

//	Si5351_write(3, 0);

	debug (RS_HFIQ, "Si5351_set_freq:done\n");
	return 0;
}

/*
 * set_pll(uint64_t pll_freq, enum si5351_pll target_pll)
 *
 * Set the specified PLL to a specific oscillation frequency
 *
 * pll_freq - Desired PLL frequency
 * target_pll - Which PLL to set
 *     (use the si5351_pll enum)
 */
void Si5351_set_pll(uint64_t pll_freq, si5351_pll target_pll)
{
	debug (RS_HFIQ, "Si5351_set_pll:start pll_freq=%x,target_pll=%x\n", pll_freq, target_pll);
  struct Si5351RegSet pll_reg;

  Si5351_pll_calc(pll_freq, &pll_reg, ref_correction);

  // Derive the register values to write

  // Prepare an array for parameters to be written to
//  uint8_t *params = new uint8_t[20];
  uint8_t params[20];
  uint8_t i = 0;
  uint8_t temp;

  // Registers 26-27
  temp = ((pll_reg.p3 >> 8) & 0xFF);
  params[i++] = temp;

  temp = (uint8_t)(pll_reg.p3  & 0xFF);
  params[i++] = temp;

  // Register 28
  temp = (uint8_t)((pll_reg.p1 >> 16) & 0x03);
  params[i++] = temp;

  // Registers 29-30
  temp = (uint8_t)((pll_reg.p1 >> 8) & 0xFF);
  params[i++] = temp;

  temp = (uint8_t)(pll_reg.p1  & 0xFF);
  params[i++] = temp;

  // Register 31
  temp = (uint8_t)((pll_reg.p3 >> 12) & 0xF0);
  temp += (uint8_t)((pll_reg.p2 >> 16) & 0x0F);
  params[i++] = temp;

  // Registers 32-33
  temp = (uint8_t)((pll_reg.p2 >> 8) & 0xFF);
  params[i++] = temp;

  temp = (uint8_t)(pll_reg.p2  & 0xFF);
  params[i++] = temp;

  // Write the parameters
  if(target_pll == SI5351_PLLA)
  {
	  Si5351_write_bulk(SI5351_PLLA_PARAMETERS, i, params);
  }
  else if(target_pll == SI5351_PLLB)
  {
	  Si5351_write_bulk(SI5351_PLLB_PARAMETERS, i, params);
  }

//  delete params;
  debug (RS_HFIQ, "Si5351_set_pll:done\n");
}

/*
 * set_ms(enum si5351_clock clk, struct Si5351RegSet ms_reg, uint8_t int_mode, uint8_t r_div, uint8_t div_by_4)
 *
 * Set the specified multisynth parameters. Not normally needed, but public for advanced users.
 *
 * clk - Clock output
 *   (use the si5351_clock enum)
 * int_mode - Set integer mode
 *  Set to 1 to enable, 0 to disable
 * r_div - Desired r_div ratio
 * div_by_4 - Set Divide By 4 mode
 *   Set to 1 to enable, 0 to disable
 */
void Si5351_set_ms(si5351_clock clk, struct Si5351RegSet ms_reg, uint8_t int_mode, uint8_t r_div, uint8_t div_by_4)
{
	debug (RS_HFIQ, "Si5351_set_ms:start clk=%x,ms_reg=%x,int_mode=%x,r_div=%x,div_by_4=%x\n", clk, ms_reg, int_mode, r_div, div_by_4);
//	uint8_t *params = new uint8_t[20];
	uint8_t params[20];
	uint8_t i = 0;
 	uint8_t temp;
 	uint8_t reg_val;

	// Registers 42-43 for CLK0
	temp = (uint8_t)((ms_reg.p3 >> 8) & 0xFF);
	params[i++] = temp;

	temp = (uint8_t)(ms_reg.p3  & 0xFF);
	params[i++] = temp;

	// Register 44 for CLK0
	reg_val = Si5351_read((SI5351_CLK0_PARAMETERS + 2) + (clk * 8));
	reg_val &= ~(0x03);
	temp = reg_val | ((uint8_t)((ms_reg.p1 >> 16) & 0x03));
	params[i++] = temp;

	// Registers 45-46 for CLK0
	temp = (uint8_t)((ms_reg.p1 >> 8) & 0xFF);
	params[i++] = temp;

	temp = (uint8_t)(ms_reg.p1  & 0xFF);
	params[i++] = temp;

	// Register 47 for CLK0
	temp = (uint8_t)((ms_reg.p3 >> 12) & 0xF0);
	temp += (uint8_t)((ms_reg.p2 >> 16) & 0x0F);
	params[i++] = temp;

	// Registers 48-49 for CLK0
	temp = (uint8_t)((ms_reg.p2 >> 8) & 0xFF);
	params[i++] = temp;

	temp = (uint8_t)(ms_reg.p2  & 0xFF);
	params[i++] = temp;

	// Write the parameters
	switch(clk)
	{
		case SI5351_CLK0:
			Si5351_write_bulk(SI5351_CLK0_PARAMETERS, i, params);
			break;
		case SI5351_CLK1:
			Si5351_write_bulk(SI5351_CLK1_PARAMETERS, i, params);
			break;
		case SI5351_CLK2:
			Si5351_write_bulk(SI5351_CLK2_PARAMETERS, i, params);
			break;
		case SI5351_CLK3:
			Si5351_write_bulk(SI5351_CLK3_PARAMETERS, i, params);
			break;
		case SI5351_CLK4:
			Si5351_write_bulk(SI5351_CLK4_PARAMETERS, i, params);
			break;
		case SI5351_CLK5:
			Si5351_write_bulk(SI5351_CLK5_PARAMETERS, i, params);
			break;
		case SI5351_CLK6:
			Si5351_write_bulk(SI5351_CLK6_PARAMETERS, i, params);
			break;
		case SI5351_CLK7:
			Si5351_write_bulk(SI5351_CLK7_PARAMETERS, i, params);
			break;
		default:
			debug (RS_HFIQ, "Si5351_set_ms:Error\n");
	}

	Si5351_set_int(clk, int_mode);
	Si5351_ms_div(clk, r_div, div_by_4);

//	delete params;
	debug (RS_HFIQ, "Si5351_set_ms:done\n");

}

/*
 * output_enable(enum si5351_clock clk, uint8_t enable)
 *
 * Enable or disable a chosen output
 * clk - Clock output
 *   (use the si5351_clock enum)
 * enable - Set to 1 to enable, 0 to disable
 */
void Si5351_output_enable(si5351_clock clk, uint8_t enable)
{
	debug (RS_HFIQ, "Si5351_output_enable:start clk=%d,enable=%x\n",clk,enable);
  uint8_t reg_val;

  reg_val = Si5351_read(SI5351_OUTPUT_ENABLE_CTRL);

  if(enable == 1)
  {
    reg_val &= ~(1<<(uint8_t)clk);
  }
  else
  {
    reg_val |= (1<<(uint8_t)clk);
  }

  Si5351_write(SI5351_OUTPUT_ENABLE_CTRL, reg_val);
  debug (RS_HFIQ, "Si5351_output_enable:done\n");
}

/*
 * drive_strength(enum si5351_clock clk, enum si5351_drive drive)
 *
 * Sets the drive strength of the specified clock output
 *
 * clk - Clock output
 *   (use the si5351_clock enum)
 * drive - Desired drive level
 *   (use the si5351_drive enum)
 */
void Si5351_drive_strength(si5351_clock clk, si5351_drive drive)
{
	debug (RS_HFIQ, "Si5351_drive_strength:start clk=%d,drive=%x\n",clk,drive);
  uint8_t reg_val;
  const uint8_t mask = 0x03;

  reg_val = Si5351_read(SI5351_CLK0_CTRL + (uint8_t)clk);
  reg_val &= ~(mask);

  switch(drive)
  {
  case SI5351_DRIVE_2MA:
    reg_val |= 0x00;
    break;
  case SI5351_DRIVE_4MA:
   reg_val |= 0x01;
    break;
  case SI5351_DRIVE_6MA:
    reg_val |= 0x02;
    break;
  case SI5351_DRIVE_8MA:
    reg_val |= 0x03;
    break;
  default:
    break;
  }

  Si5351_write(SI5351_CLK0_CTRL + (uint8_t)clk, reg_val);
  debug (RS_HFIQ, "Si5351_drive_strength:done\n");
}

/*
 * update_status(void)
 *
 * Call this to update the status structs, then access them
 * via the dev_status and dev_int_status global variables.
 *
 * See the header file for the struct definitions. These
 * correspond to the flag names for registers 0 and 1 in
 * the Si5351 datasheet.
 */
void Si5351_update_status(void)
{
	debug (RS_HFIQ, "Si5351_update_status:start\n");
//	update_sys_status(&dev_status);
//	update_int_status(&dev_int_status);
	debug (RS_HFIQ, "Si5351_update_status:done\n");
}

/*
 * set_correction(int32_t corr)
 *
 * Use this to set the oscillator correction factor to
 * EEPROM. This value is a signed 32-bit integer of the
 * parts-per-billion value that the actual oscillation
 * frequency deviates from the specified frequency.
 *
 * The frequency calibration is done as a one-time procedure.
 * Any desired test frequency within the normal range of the
 * Si5351 should be set, then the actual output frequency
 * should be measured as accurately as possible. The
 * difference between the measured and specified frequencies
 * should be calculated in Hertz, then multiplied by 10 in
 * order to get the parts-per-billion value.
 *
 * Since the Si5351 itself has an intrinsic 0 PPM error, this
 * correction factor is good across the entire tuning range of
 * the Si5351. Once this calibration is done accurately, it
 * should not have to be done again for the same Si5351 and
 * crystal. The library will read the correction factor from
 * EEPROM during initialization for use by the tuning
 * algorithms.
 */
void Si5351_set_correction(int32_t corr)
{
	debug (RS_HFIQ, "Si5351_set_correction:start corr=%x\n",corr);
	ref_correction = corr;
	debug (RS_HFIQ, "Si5351_set_correction:done\n");
}

/*
 * set_phase(enum si5351_clock clk, uint8_t phase)
 *
 * clk - Clock output
 *   (use the si5351_clock enum)
 * phase - 7-bit phase word
 *   (in units of VCO/4 period)
 *
 * Write the 7-bit phase register. This must be used
 * with a user-set PLL frequency so that the user can
 * calculate the proper tuning word based on the PLL period.
 */
void Si5351_set_phase(si5351_clock clk, uint8_t phase)
{
	debug (RS_HFIQ, "Si5351_set_phase:start clk=%d,phase=%x\n",clk,phase);
	// Mask off the upper bit since it is reserved
	phase = phase & 0b01111111;

	Si5351_write(SI5351_CLK0_PHASE_OFFSET + (uint8_t)clk, phase);
	debug (RS_HFIQ, "Si5351_set_phase:done\n");
}

/*
 * get_correction(void)
 *
 * Returns the oscillator correction factor stored
 * in RAM.
 */
int32_t Si5351_get_correction(void)
{
	debug (RS_HFIQ, "Si5351_get_correction:start/done\n");
	return ref_correction;
}

/*
 * pll_reset(enum si5351_pll target_pll)
 *
 * target_pll - Which PLL to reset
 *     (use the si5351_pll enum)
 *
 * Apply a reset to the indicated PLL.
 */
void Si5351_pll_reset(si5351_pll target_pll)
{
	debug (RS_HFIQ, "Si5351_pll_reset:start target_pll=%d\n",target_pll);
	if(target_pll == SI5351_PLLA)
 	{
		Si5351_write(SI5351_PLL_RESET, SI5351_PLL_RESET_A);
	}
	else if(target_pll == SI5351_PLLB)
	{
		Si5351_write(SI5351_PLL_RESET, SI5351_PLL_RESET_B);
	}
	debug (RS_HFIQ, "Si5351_pll_reset:done\n");
}

/*
 * set_ms_source(enum si5351_clock clk, enum si5351_pll pll)
 *
 * clk - Clock output
 *   (use the si5351_clock enum)
 * pll - Which PLL to use as the source
 *     (use the si5351_pll enum)
 *
 * Set the desired PLL source for a multisynth.
 */
void Si5351_set_ms_source(si5351_clock clk, si5351_pll pll)
{
	debug (RS_HFIQ, "Si5351_set_ms_source:start clk=%d,pll=%x\n",clk,pll);
	uint8_t reg_val;

	reg_val = Si5351_read(SI5351_CLK0_CTRL + (uint8_t)clk);

	if(pll == SI5351_PLLA)
	{
		reg_val &= ~(SI5351_CLK_PLL_SELECT);
	}
	else if(pll == SI5351_PLLB)
	{
		reg_val |= SI5351_CLK_PLL_SELECT;
	}

	Si5351_write(SI5351_CLK0_CTRL + (uint8_t)clk, reg_val);
	debug (RS_HFIQ, "Si5351_set_ms_source:done\n");
}

/*
 * set_int(enum si5351_clock clk, uint8_t int_mode)
 *
 * clk - Clock output
 *   (use the si5351_clock enum)
 * enable - Set to 1 to enable, 0 to disable
 *
 * Set the indicated multisynth into integer mode.
 */
void Si5351_set_int(si5351_clock clk, uint8_t enable)
{
	debug (RS_HFIQ, "Si5351_set_int:start clk=%d,enable=%x\n",clk,enable);
	uint8_t reg_val;
	reg_val = Si5351_read(SI5351_CLK0_CTRL + (uint8_t)clk);

	if(enable == 1)
	{
		reg_val |= (SI5351_CLK_INTEGER_MODE);
	}
	else
	{
		reg_val &= ~(SI5351_CLK_INTEGER_MODE);
	}

	Si5351_write(SI5351_CLK0_CTRL + (uint8_t)clk, reg_val);

	// Integer mode indication
	switch(clk)
	{
	case SI5351_CLK0:
		clk0_int_mode = enable;
		break;
	case SI5351_CLK1:
		clk1_int_mode = enable;
		break;
	case SI5351_CLK2:
		clk2_int_mode = enable;
		break;
	default:
		break;
	}
	debug (RS_HFIQ, "Si5351_set_int:done\n");
}

/*
 * set_clock_pwr(enum si5351_clock clk, uint8_t pwr)
 *
 * clk - Clock output
 *   (use the si5351_clock enum)
 * pwr - Set to 1 to enable, 0 to disable
 *
 * Enable or disable power to a clock output (a power
 * saving feature).
 */
void Si5351_set_clock_pwr(si5351_clock clk, uint8_t pwr)
{
	debug (RS_HFIQ, "Si5351_set_clock_pwr:start clk=%d,pwr=%x\n",clk,pwr);
	uint8_t reg_val; //, reg;
	reg_val = Si5351_read(SI5351_CLK0_CTRL + (uint8_t)clk);

	if(pwr == 1)
	{
		reg_val &= 0b01111111;
	}
	else
	{
		reg_val |= 0b10000000;
	}

	Si5351_write(SI5351_CLK0_CTRL + (uint8_t)clk, reg_val);
	debug (RS_HFIQ, "Si5351_set_clock_pwr:done\n");
}

/*
 * set_clock_invert(enum si5351_clock clk, uint8_t inv)
 *
 * clk - Clock output
 *   (use the si5351_clock enum)
 * inv - Set to 1 to enable, 0 to disable
 *
 * Enable to invert the clock output waveform.
 */
void Si5351_set_clock_invert(si5351_clock clk, uint8_t inv)
{
	debug (RS_HFIQ, "Si5351_set_clock_invert:start clk=%d,inv=%x\n",clk,inv);
	uint8_t reg_val;
	reg_val = Si5351_read(SI5351_CLK0_CTRL + (uint8_t)clk);

	if(inv == 1)
	{
		reg_val |= (SI5351_CLK_INVERT);
	}
	else
	{
		reg_val &= ~(SI5351_CLK_INVERT);
	}

	Si5351_write(SI5351_CLK0_CTRL + (uint8_t)clk, reg_val);
	debug (RS_HFIQ, "Si5351_set_clock_invert:done\n");
}

/*
 * set_clock_source(enum si5351_clock clk, enum si5351_clock_source src)
 *
 * clk - Clock output
 *   (use the si5351_clock enum)
 * src - Which clock source to use for the multisynth
 *   (use the si5351_clock_source enum)
 *
 * Set the clock source for a multisynth (based on the options
 * presented for Registers 16-23 in the Silicon Labs AN619 document).
 * Choices are XTAL, CLKIN, MS0, or the multisynth associated with
 * the clock output.
 */
void Si5351_set_clock_source(si5351_clock clk, si5351_clock_source src)
{
	debug (RS_HFIQ, "Si5351_set_clock_source:start clk=%d,src=%x\n",clk,src);
	uint8_t reg_val;
	reg_val = Si5351_read(SI5351_CLK0_CTRL + (uint8_t)clk);

	// Clear the bits first
	reg_val &= ~(SI5351_CLK_INPUT_MASK);

	switch(src)
	{
	case SI5351_CLK_SRC_XTAL:
		reg_val |= (SI5351_CLK_INPUT_XTAL);
		break;
	case SI5351_CLK_SRC_CLKIN:
		reg_val |= (SI5351_CLK_INPUT_CLKIN);
		break;
	case SI5351_CLK_SRC_MS0:
		if(clk == SI5351_CLK0)
		{
			debug (RS_HFIQ, "Si5351_set_clock_source:done\n");
			return;
		}

		reg_val |= (SI5351_CLK_INPUT_MULTISYNTH_0_4);
		break;
	case SI5351_CLK_SRC_MS:
		reg_val |= (SI5351_CLK_INPUT_MULTISYNTH_N);
		break;
	default:
		debug (RS_HFIQ, "Si5351_set_clock_source:done\n");
		return;
	}

	Si5351_write(SI5351_CLK0_CTRL + (uint8_t)clk, reg_val);
	debug (RS_HFIQ, "Si5351_set_clock_source:done\n");
}

/*
 * set_clock_disable(enum si5351_clock clk, enum si5351_clock_disable dis_state)
 *
 * clk - Clock output
 *   (use the si5351_clock enum)
 * dis_state - Desired state of the output upon disable
 *   (use the si5351_clock_disable enum)
 *
 * Set the state of the clock output when it is disabled. Per page 27
 * of AN619 (Registers 24 and 25), there are four possible values: low,
 * high, high impedance, and never disabled.
 */
void Si5351_set_clock_disable(si5351_clock clk, si5351_clock_disable dis_state)
{
	debug (RS_HFIQ, "Si5351_set_clock_disable:start clk=%d,dis_state=%x\n",clk,dis_state);
	uint8_t reg_val, reg;

	if (clk >= SI5351_CLK0 && clk <= SI5351_CLK3)
	{
		reg = SI5351_CLK3_0_DISABLE_STATE;
	}
	else if (clk >= SI5351_CLK0 && clk <= SI5351_CLK3)
	{
		reg = SI5351_CLK7_4_DISABLE_STATE;
	}

	reg_val = Si5351_read(reg);

	if (clk >= SI5351_CLK0 && clk <= SI5351_CLK3)
	{
		reg_val &= ~(0b11 << (clk * 2));
		reg_val |= dis_state << (clk * 2);
	}
	else if (clk >= SI5351_CLK0 && clk <= SI5351_CLK3)
	{
		reg_val &= ~(0b11 << ((clk - 4) * 2));
		reg_val |= dis_state << ((clk - 4) * 2);
	}

	Si5351_write(reg, reg_val);
	debug (RS_HFIQ, "Si5351_set_clock_disable:done\n");
}

/*
 * set_clock_fanout(enum si5351_clock_fanout fanout, uint8_t enable)
 *
 * fanout - Desired clock fanout
 *   (use the si5351_clock_fanout enum)
 * enable - Set to 1 to enable, 0 to disable
 *
 * Use this function to enable or disable the clock fanout options
 * for individual clock outputs. If you intend to output the XO or
 * CLKIN on the clock outputs, enable this first.
 *
 * By default, only the Multisynth fanout is enabled at startup.
 */
void Si5351_set_clock_fanout(si5351_clock_fanout fanout, uint8_t enable)
{
	debug (RS_HFIQ, "Si5351_set_clock_fanout:start fanout=%x,enable=%x\n",fanout,enable);
	uint8_t reg_val;
	reg_val = Si5351_read(SI5351_FANOUT_ENABLE);

	switch(fanout)
	{
	case SI5351_FANOUT_CLKIN:
		if(enable)
		{
			reg_val |= SI5351_CLKIN_ENABLE;
		}
		else
		{
			reg_val &= ~(SI5351_CLKIN_ENABLE);
		}
		break;
	case SI5351_FANOUT_XO:
		if(enable)
		{
			reg_val |= SI5351_XTAL_ENABLE;
		}
		else
		{
			reg_val &= ~(SI5351_XTAL_ENABLE);
		}
		break;
	case SI5351_FANOUT_MS:
		if(enable)
		{
			reg_val |= SI5351_MULTISYNTH_ENABLE;
		}
		else
		{
			reg_val &= ~(SI5351_MULTISYNTH_ENABLE);
		}
		break;
	}

	Si5351_write(SI5351_FANOUT_ENABLE, reg_val);
	debug (RS_HFIQ, "Si5351_set_clock_fanout:done\n");
}

uint8_t Si5351_write_bulk(uint8_t addr, uint8_t bytes, uint8_t *data)
{
	debug (RS_HFIQ, "Si5351_write_bulk:start addr=%d,bytes=%x\n", addr, bytes);
	debug (RS_HFIQ, "   bytes: ");

//	Wire.beginTransmission(SI5351_BUS_BASE_ADDR);
//	Wire.write(addr);
	for(int i = 0; i < bytes; i++)
	{
//		Wire.write(data[i]);
		debug (RS_HFIQ, "%2x ", data[i]);
		I2C_WriteRegister( SI5351_BUS_BASE_ADDR, addr+i, data[i]);
	}
	debug (RS_HFIQ, "\n");
//	Wire.endTransmission();
	debug (RS_HFIQ, "Si5351_write_bulk:done\n");
	return 0;
}

uint8_t Si5351_write(uint8_t addr, uint8_t data)
{
	debug (RS_HFIQ, "Si5351_write:start addr=%d,data=%x\n", addr, data);
//	Wire.beginTransmission(SI5351_BUS_BASE_ADDR);
//	Wire.write(addr);
//	Wire.write(data);
//	Wire.endTransmission();
	I2C_WriteRegister( SI5351_BUS_BASE_ADDR, addr, data);
	debug (RS_HFIQ, "Si5351_write:done\n");
	return 0;
}

uint8_t Si5351_read(uint8_t addr)
{
	debug (RS_HFIQ, "Si5351_read:start addr=%d\n",addr);
	uint8_t reg_val;

//	Wire.beginTransmission(SI5351_BUS_BASE_ADDR);
//	Wire.write(addr);
//	Wire.endTransmission();

//	Wire.requestFrom(SI5351_BUS_BASE_ADDR, 1, false);

//	while(Wire.available())
//	{
//		reg_val = Wire.read();
//	}
	reg_val = I2C_ReadSlave(SI5351_BUS_BASE_ADDR, addr);
	debug (RS_HFIQ, "Si5351_read:done addr=%d,reg_val=%x\n", addr, reg_val);
	return reg_val;
}

/*********************/
/* Private functions */
/*********************/

uint64_t Si5351_pll_calc(uint64_t freq, struct Si5351RegSet *reg, int32_t correction)
{
	debug (RS_HFIQ, "Si5351_pll_calc:start freq=%x,correction=%x\n",freq,correction);
	uint64_t ref_freq = xtal_freq * SI5351_FREQ_MULT;
	uint32_t a, b, c, p1, p2, p3;
//	uint64_t lltmp, rfrac, denom;
	uint64_t lltmp, denom;
//	int64_t ref_temp;

	// Factor calibration value into nominal crystal frequency
	// Measured in parts-per-billion

	ref_freq = ref_freq + (int32_t)((((((int64_t)correction) << 31) / 1000000000LL) * ref_freq) >> 31);

	// PLL bounds checking
	if (freq < SI5351_PLL_VCO_MIN * SI5351_FREQ_MULT)
	{
		freq = SI5351_PLL_VCO_MIN * SI5351_FREQ_MULT;
	}
	if (freq > SI5351_PLL_VCO_MAX * SI5351_FREQ_MULT)
	{
		freq = SI5351_PLL_VCO_MAX * SI5351_FREQ_MULT;
	}

	// Determine integer part of feedback equation
	a = freq / ref_freq;

	if (a < SI5351_PLL_A_MIN)
	{
		freq = ref_freq * SI5351_PLL_A_MIN;
	}
	if (a > SI5351_PLL_A_MAX)
	{
		freq = ref_freq * SI5351_PLL_A_MAX;
	}

	// Find best approximation for b/c = fVCO mod fIN
	denom = (unsigned long long) 1000 * 1000;
	lltmp = freq % ref_freq;
	lltmp *= denom;
	do_div(lltmp, ref_freq);
//	rfrac = lltmp;

	b = (((uint64_t)(freq % ref_freq)) * RFRAC_DENOM) / ref_freq;
	c = b ? RFRAC_DENOM : 1;

	// Calculate parameters
    p1 = 128 * a + ((128 * b) / c) - 512;
    p2 = 128 * b - c * ((128 * b) / c);
    p3 = c;

	// Recalculate frequency as fIN * (a + b/c)
	lltmp  = ref_freq;
	lltmp *= b;
	do_div(lltmp, c);
	freq = lltmp;
	freq += ref_freq * a;

	reg->p1 = p1;
	reg->p2 = p2;
	reg->p3 = p3;

	debug (RS_HFIQ, "Si5351_pll_calc:done freq=%x\n",freq);
	return freq;
}

uint64_t Si5351_multisynth_calc(uint64_t freq, uint64_t pll_freq, struct Si5351RegSet *reg)
{
	debug (RS_HFIQ, "Si5351_multisynth_calc:start freq=%x,pll_freq=%x\n",freq,pll_freq);
	uint64_t lltmp;
	uint32_t a, b, c, p1, p2, p3;
	uint8_t divby4;
	uint8_t ret_val = 0;

	// Multisynth bounds checking
	if (freq > SI5351_MULTISYNTH_MAX_FREQ * SI5351_FREQ_MULT)
	{
		freq = SI5351_MULTISYNTH_MAX_FREQ * SI5351_FREQ_MULT;
	}
	if (freq < SI5351_MULTISYNTH_MIN_FREQ * SI5351_FREQ_MULT)
	{
		freq = SI5351_MULTISYNTH_MIN_FREQ * SI5351_FREQ_MULT;
	}

	divby4 = 0;
	if (freq >= SI5351_MULTISYNTH_DIVBY4_FREQ * SI5351_FREQ_MULT)
	{
		divby4 = 1;
	}

	if(pll_freq == 0)
	{
		// Find largest integer divider for max
		// VCO frequency and given target frequency
		if(divby4 == 0)
		{
			lltmp = SI5351_PLL_VCO_MAX * SI5351_FREQ_MULT;
			do_div(lltmp, freq);
			a = (uint32_t)lltmp;
		}
		else
		{
			a = 4;
		}

		b = 0;
		c = 1;
		pll_freq = a * freq;
	}
	else
	{
		// Preset PLL, so return the actual freq for these params instead of PLL freq
		ret_val = 1;

		// Determine integer part of feedback equation
		a = pll_freq / freq;

		if (a < SI5351_MULTISYNTH_A_MIN)
		{
			freq = pll_freq / SI5351_MULTISYNTH_A_MIN;
		}
		if (a > SI5351_MULTISYNTH_A_MAX)
		{
			freq = pll_freq / SI5351_MULTISYNTH_A_MAX;
		}

		b = (pll_freq % freq * RFRAC_DENOM) / freq;
		c = b ? RFRAC_DENOM : 1;
	}

	// Calculate parameters
	if (divby4 == 1)
	{
		p3 = 1;
		p2 = 0;
		p1 = 0;
	}
	else
	{
        p1 = 128 * a + ((128 * b) / c) - 512;
        p2 = 128 * b - c * ((128 * b) / c);
        p3 = c;
	}

	reg->p1 = p1;
	reg->p2 = p2;
	reg->p3 = p3;

	if(ret_val == 0)
	{
		debug (RS_HFIQ, "Si5351_multisynth_calc:done pll_freq=%x\n",pll_freq);
		return pll_freq;
	}
	else
	{
		debug (RS_HFIQ, "Si5351_multisynth_calc:done freq=%x\n",freq);
		return freq;
	}
}

void Si5351_update_sys_status(struct Si5351Status *status)
{
	debug (RS_HFIQ, "Si5351_update_sys_status:start\n");
  uint8_t reg_val = 0;

  reg_val = Si5351_read(SI5351_DEVICE_STATUS);

  // Parse the register
  status->SYS_INIT = (reg_val >> 7) & 0x01;
  status->LOL_B = (reg_val >> 6) & 0x01;
  status->LOL_A = (reg_val >> 5) & 0x01;
  status->LOS = (reg_val >> 4) & 0x01;
  status->REVID = reg_val & 0x03;
  debug (RS_HFIQ, "Si5351_update_sys_status:done\n");
}

void Si5351_update_int_status(struct Si5351IntStatus *int_status)
{
	debug (RS_HFIQ, "Si5351_update_int_status:start\n");
  uint8_t reg_val = 0;

  reg_val = Si5351_read(SI5351_DEVICE_STATUS);

  // Parse the register
  int_status->SYS_INIT_STKY = (reg_val >> 7) & 0x01;
  int_status->LOL_B_STKY = (reg_val >> 6) & 0x01;
  int_status->LOL_A_STKY = (reg_val >> 5) & 0x01;
  int_status->LOS_STKY = (reg_val >> 4) & 0x01;
  debug (RS_HFIQ, "Si5351_update_int_status:done\n");
}

void Si5351_ms_div(si5351_clock clk, uint8_t r_div, uint8_t div_by_4)
{
	debug (RS_HFIQ, "Si5351_ms_div:start clk=%d,r_div=%x,div_by_4=%x\n",clk,r_div,div_by_4);
	uint8_t reg_val, reg_addr=0;

	switch(clk)
	{
		case SI5351_CLK0:
			reg_addr = SI5351_CLK0_PARAMETERS + 2;
			break;
		case SI5351_CLK1:
			reg_addr = SI5351_CLK1_PARAMETERS + 2;
			break;
		case SI5351_CLK2:
			reg_addr = SI5351_CLK2_PARAMETERS + 2;
			break;
		case SI5351_CLK3:
			reg_addr = SI5351_CLK3_PARAMETERS + 2;
			break;
		case SI5351_CLK4:
			reg_addr = SI5351_CLK4_PARAMETERS + 2;
			break;
		case SI5351_CLK5:
			reg_addr = SI5351_CLK5_PARAMETERS + 2;
			break;
		case SI5351_CLK6:
			debug (RS_HFIQ, "Si5351_ms_div:done\n");
			return;
		case SI5351_CLK7:
			debug (RS_HFIQ, "Si5351_ms_div:done\n");
			return;
		default:
			debug (RS_HFIQ, "Si5351_ms_div:Error\n");
	}

	reg_val = Si5351_read(reg_addr);

	// Clear the relevant bits
	reg_val &= ~(0x7c);

	if(div_by_4 == 0)
	{
		reg_val &= ~(SI5351_OUTPUT_CLK_DIVBY4);
	}
	else
	{
		reg_val |= (SI5351_OUTPUT_CLK_DIVBY4);
	}

	reg_val |= (r_div << SI5351_OUTPUT_CLK_DIV_SHIFT);

	Si5351_write(reg_addr, reg_val);
	debug (RS_HFIQ, "Si5351_ms_div:done\n");
}

uint8_t Si5351_select_r_div(uint64_t *freq)
{
	debug (RS_HFIQ, "Si5351_select_r_div:start\n");
	uint8_t r_div = SI5351_OUTPUT_CLK_DIV_1;

	// Choose the correct R divider
	if((*freq >= SI5351_CLKOUT_MIN_FREQ * SI5351_FREQ_MULT) && (*freq < SI5351_CLKOUT_MIN_FREQ * SI5351_FREQ_MULT * 2))
	{
		r_div = SI5351_OUTPUT_CLK_DIV_128;
		*freq *= (unsigned long long) 128;
	}
	else if((*freq >= SI5351_CLKOUT_MIN_FREQ * SI5351_FREQ_MULT * 2) && (*freq < SI5351_CLKOUT_MIN_FREQ * SI5351_FREQ_MULT * 4))
	{
		r_div = SI5351_OUTPUT_CLK_DIV_64;
		*freq *= (unsigned long long) 64;
	}
	else if((*freq >= SI5351_CLKOUT_MIN_FREQ * SI5351_FREQ_MULT * 4) && (*freq < SI5351_CLKOUT_MIN_FREQ * SI5351_FREQ_MULT * 8))
	{
		r_div = SI5351_OUTPUT_CLK_DIV_32;
		*freq *= (unsigned long long) 32;
	}
	else if((*freq >= SI5351_CLKOUT_MIN_FREQ * SI5351_FREQ_MULT * 8) && (*freq < SI5351_CLKOUT_MIN_FREQ * SI5351_FREQ_MULT * 16))
	{
		r_div = SI5351_OUTPUT_CLK_DIV_16;
		*freq *= (unsigned long long) 16;
	}
	else if((*freq >= SI5351_CLKOUT_MIN_FREQ * SI5351_FREQ_MULT * 16) && (*freq < SI5351_CLKOUT_MIN_FREQ * SI5351_FREQ_MULT * 32))
	{
		r_div = SI5351_OUTPUT_CLK_DIV_8;
		*freq *= (unsigned long long) 8;
	}
	else if((*freq >= SI5351_CLKOUT_MIN_FREQ * SI5351_FREQ_MULT * 32) && (*freq < SI5351_CLKOUT_MIN_FREQ * SI5351_FREQ_MULT * 64))
	{
		r_div = SI5351_OUTPUT_CLK_DIV_4;
		*freq *= (unsigned long long) 4;
	}
	else if((*freq >= SI5351_CLKOUT_MIN_FREQ * SI5351_FREQ_MULT * 64) && (*freq < SI5351_CLKOUT_MIN_FREQ * SI5351_FREQ_MULT * 128))
	{
		r_div = SI5351_OUTPUT_CLK_DIV_2;
		*freq *= (unsigned long long) 2;
	}
	debug (RS_HFIQ, "Si5351_select_r_div:done r_div=%x\n",r_div);
	return r_div;
}

void Si5351_dump (void){
	char buf[15];
	debug (RS_HFIQ, "Si5351_dump:start\n");
	uint8_t reg_val;
	debug (RS_HFIQ, "         ");
	for (uint8_t i=0; i<10; i++){
		debug (RS_HFIQ, "%2x ", i);
	}
	debug (RS_HFIQ, "\n");

	for (uint8_t j=0; j<=18; j++){
		sprintf(buf, "   %*d : ", 3, j*10);
		debug (RS_HFIQ, buf);
		for (uint8_t i=0; i<10; i++){
			reg_val = I2C_ReadSlave(SI5351_BUS_BASE_ADDR, i+10*j);
			debug (RS_HFIQ, "%2x ", reg_val);
		}
		debug (RS_HFIQ, "\n");
	}
	debug (RS_HFIQ, "Si5351_dump:done\n");
}


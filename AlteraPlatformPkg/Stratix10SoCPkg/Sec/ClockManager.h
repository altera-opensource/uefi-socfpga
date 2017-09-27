/** @file

  Copyright (c) 2016, Intel Corporation. All rights reserved.

  Redistribution and use in source and binary forms, with or without modification,
  are permitted provided that the following conditions are met:

  1. Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

  2. Redistributions in binary form must reproduce the above copyright notice, this
  list of conditions and the following disclaimer in the documentation and/or other
  materials provided with the distribution.

  3. Neither the name of the copyright holder nor the names of its contributors may
  be used to endorse or promote products derived from this software without specific
  prior written permission.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
  EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
  OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT
  SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
  TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
  BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
  DAMAGE.

**/

#ifndef __CLOCKMANAGER_H__
#define __CLOCKMANAGER_H__

#include "Handoff.h"

#define CLKMGR_PLLGLOB_PD_MASK				0x00000001
#define CLKMGR_PLLGLOB_RST_MASK				0x00000002
#define CLKMGR_PLLGLOB_VCO_PSRC_MASK			0X3
#define CLKMGR_PLLGLOB_VCO_PSRC_OFFSET			16
#define CLKMGR_VCO_PSRC_EOSC1				0
#define CLKMGR_VCO_PSRC_INTOSC				1
#define CLKMGR_VCO_PSRC_F2S				2
#define CLKMGR_PLLGLOB_REFCLKDIV_MASK			0X3f
#define CLKMGR_PLLGLOB_REFCLKDIV_OFFSET			8

#define CLKMGR_CLKSRC_MASK				0x7
#define CLKMGR_CLKSRC_OFFSET				16
#define CLKMGR_CLKSRC_MAIN				0
#define CLKMGR_CLKSRC_PER				1
#define CLKMGR_CLKSRC_OSC1				2
#define CLKMGR_CLKSRC_INTOSC				3
#define CLKMGR_CLKSRC_FPGA				4
#define CLKMGR_CLKCNT_MSK				0x7ff

#define CLKMGR_FDBCK_MDIV_MASK				0xff
#define CLKMGR_FDBCK_MDIV_OFFSET			24

#define CLKMGR_PLLC0_DIV_MASK				0xff
#define CLKMGR_PLLC1_DIV_MASK				0xff
#define CLKMGR_PLLC0_EN_OFFSET				27
#define CLKMGR_PLLC1_EN_OFFSET				24

#define CLKMGR_NOCDIV_L4MAIN_OFFSET			0
#define CLKMGR_NOCDIV_L4MPCLK_OFFSET			8
#define CLKMGR_NOCDIV_L4SPCLK_OFFSET			16
#define CLKMGR_NOCDIV_CSATCLK_OFFSET			24
#define CLKMGR_NOCDIV_CSTRACECLK_OFFSET			26
#define CLKMGR_NOCDIV_CSPDBGCLK_OFFSET			28

#define CLKMGR_NOCDIV_L4SPCLK_MASK			0X3
#define CLKMGR_NOCDIV_DIV1				0
#define CLKMGR_NOCDIV_DIV2				1
#define CLKMGR_NOCDIV_DIV4				2
#define CLKMGR_NOCDIV_DIV8				3
#define CLKMGR_CSPDBGCLK_DIV1				0
#define CLKMGR_CSPDBGCLK_DIV4				1

#define CLKMGR_VCOCALIB_MSCNT_MASK			0xff
#define CLKMGR_VCOCALIB_MSCNT_OFFSET			9
#define CLKMGR_VCOCALIB_HSCNT_MASK			0xff

#define CLKMGR_EMACCTL_EMAC0SEL_OFFSET			26
#define CLKMGR_EMACCTL_EMAC1SEL_OFFSET			27
#define CLKMGR_EMACCTL_EMAC2SEL_OFFSET			28

#define CLKMGR_PERPLLGRP_EN_SDMMCCLK_MASK		0x00000020

// ------------------------------------------------------------------
// Clock Source Frequency
// ------------------------------------------------------------------
typedef struct {
  UINT32  clk_freq_of_eosc1;
  UINT32  clk_freq_of_f2h_free;
  UINT32  clk_freq_of_cb_intosc_ls;
} CLOCK_SOURCE_CONFIG;

// ------------------------------------------------------------------
// clock_manager.mainpll
// ------------------------------------------------------------------
typedef struct {
	UINT32 mpuclk;
	UINT32 nocclk;
	UINT32 cntr2clk;
	UINT32 cntr3clk;
	UINT32 cntr4clk;
	UINT32 cntr5clk;
	UINT32 cntr6clk;
	UINT32 cntr7clk;
	UINT32 cntr8clk;
	UINT32 cntr9clk;
	UINT32 nocdiv;
	UINT32 pllglob;
	UINT32 fdbck;
	UINT32 pllc0;
	UINT32 pllc1;
} CLOCK_MANAGER_MAIN_PLL_CONFIG;

// ------------------------------------------------------------------
// clock_manager.perpll
// ------------------------------------------------------------------
typedef struct {
	UINT32 cntr2clk;
	UINT32 cntr3clk;
	UINT32 cntr4clk;
	UINT32 cntr5clk;
	UINT32 cntr6clk;
	UINT32 cntr7clk;
	UINT32 cntr8clk;
	UINT32 cntr9clk;
	UINT32 emacctl;
	UINT32 gpiodiv;
	UINT32 pllglob;
	UINT32 fdbck;
	UINT32 pllc0;
	UINT32 pllc1;
} CLOCK_MANAGER_PER_PLL_CONFIG;


// ------------------------------------------------------------------
// clock_manager
// ------------------------------------------------------------------

typedef struct {
  CLOCK_MANAGER_MAIN_PLL_CONFIG    mainpll;
  CLOCK_MANAGER_PER_PLL_CONFIG     perpll;
} CLOCK_MANAGER_CONFIG;


// ==================================================================
// Functions Definition
// ==================================================================

//
// Public Functions
//
VOID ConfigureClockManager (VOID);
VOID ConfigureClockManagerHandoff (handoff *hoff_ptr);
VOID WaitPllLocked (VOID);

VOID DisplayClockManagerInfo(VOID);

VOID DisplayClockFrequencyInfo (VOID);
VOID DisplayMpuClockInfo (UINT32 main_ref_clk, UINT32 peri_ref_clk);
VOID DisplayNocClockInfo (UINT32 main_ref_clk, UINT32 peri_ref_clk);
VOID DisplayPllClockOuput (UINT32 main_ref_clk, UINT32 peri_ref_clk);
UINT32 GetMainRefClock (VOID);
UINT32 GetPeriRefClock (VOID);
#endif

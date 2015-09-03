/** @file

  Copyright (c) 2015, Altera Corporation. All rights reserved.

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

//
// Include files
//
#include <AlteraPlatform.h>
#include <Library/BaseMemoryLib.h>
#include <Library/DebugLib.h>
#include <Library/IoLib.h>
#include <Library/MemoryAllocationLib.h>
#include <Library/PcdLib.h>
#include <Library/PrintLib.h>
#include <Library/SerialPortPrintLib.h>
#include <Library/TimerLib.h>
#include "Assert.h"
#include "ClockManager.h"
#include "DeviceTree.h"

#if (FixedPcdGet32(PcdDebugMsg_ClockManager) == 0)
  #define ProgressPrint(FormatString, ...)    /* do nothing */
  #define InfoPrint(FormatString, ...)        /* do nothing */
#else
  #define ProgressPrint SerialPortPrint
  #define InfoPrint     SerialPortPrint
#endif

//
// Global Variables
//
CLOCK_SOURCE_CONFIG  mClkSrc;
CLOCK_MANAGER_CONFIG mClkCfg;

//
// Functions
//
VOID
EFIAPI
ConfigureClockManager (
  IN  CONST VOID*  Fdt
  )
{
  EFI_STATUS  Status;

  ProgressPrint ("Config Clock Manager\r\n");

  Status = GetClockSourceCfg  (Fdt, &mClkSrc);
  ASSERT_PLATFORM_INIT(!EFI_ERROR(Status));

  Status = GetClockManagerCfg (Fdt, &mClkCfg);
  ASSERT_PLATFORM_INIT(!EFI_ERROR(Status));

  SetClockManagerCfg (&mClkCfg);
}


VOID
EFIAPI
SetClockManagerCfg (
  IN CLOCK_MANAGER_CONFIG*  Cfg
  )
{
  UINT32 Data32;
  // ----------------------
  // Background Information:
  // ----------------------
  // Hardware-Managed clock V.S. Software-Managed clock
  //
  // A Hardware-Managed clock,
  // are automatically transitioned by the hardware to ensure glitch-free operation.
  //
  // List of Hardware-Managed clock:
  // mainpllgrp.en
  // Bit0 mpuclken     - mpu_clk, mpu_l2ram_clk  and mpu_periph_clk
  // Bit1 l4mainclken  - l4_main_clk
  // Bit2 l4mpclken    - l4_mp_clk
  // Bit3 l4spclken    - l4_sp_clk
  // Bit4 csclken      - cs_at_clk, cs_pdbg_clk, and cs_trace_clk
  // Bit5 cstimerclken - cs_timer_clk
  //
  // A Software-Managed clock,
  // software is responsible for gating off the clock,
  // waiting for a PLL lock if required, and gating the clock back on.
  //
  // List of Software-Managed clock:
  // mainpllgrp.en
  // Bit6 s2fuser0clken  - s2f_user0_clk
  // Bit7 hmcpllrefclken - hmc_pll_ref_clk
  // perpllgrp - all clocks in Peripheral PLL group are Software-Managed.
  //
  // Each clock group has EN, ENS, ENR register
  // ENS - Write One to Set corresonding fields in Enable Register
  // ENR - Write One to Clear corresponding fields in Enable Register.
  //

  // Disable mainpllgrp's software-managed clock
  MmioWrite32 (ALT_CLKMGR_MAINPLL_OFST +
               ALT_CLKMGR_MAINPLL_ENR_OFST,
               ALT_CLKMGR_MAINPLL_ENR_S2FUSER0CLKEN_SET_MSK |
               ALT_CLKMGR_MAINPLL_ENR_HMCPLLREFCLKEN_SET_MSK);
  // Disable perpllgrp's software-managed clock
  MmioWrite32 (ALT_CLKMGR_PERPLL_OFST +
               ALT_CLKMGR_PERPLL_ENR_OFST,
               ALT_CLKMGR_PERPLL_ENR_RESET);

  // Bypass Register has bypass, bypassS (set), bypassR (clear)
  // Bypassed all mainpllgrp's clocks
  MmioWrite32 (ALT_CLKMGR_MAINPLL_OFST +
               ALT_CLKMGR_MAINPLL_BYPASSS_OFST,
               ALT_CLKMGR_MAINPLL_BYPASSS_RESET);
  // Bypassed all perpllgrp's clocks
  MmioWrite32 (ALT_CLKMGR_PERPLL_OFST +
               ALT_CLKMGR_PERPLL_BYPASSS_OFST,
               ALT_CLKMGR_PERPLL_BYPASSS_RESET);

  // vco0 of mainpllgrp
  Data32 = MmioRead32 (ALT_CLKMGR_CLKMGR_OFST +
                       ALT_CLKMGR_CLKMGR_STAT_OFST);
  // Is secure clock if the source of boot_clk is cb_intosc_hs_div2_clk.
  if (ALT_CLKMGR_CLKMGR_STAT_BOOTCLKSRC_GET(Data32) == 1) {
    // Secure boot_clk
    MmioWrite32 (ALT_CLKMGR_MAINPLL_OFST +
                 ALT_CLKMGR_MAINPLL_VCO0_OFST,
                 ALT_CLKMGR_MAINPLL_VCO0_RESET |
                 ALT_CLKMGR_MAINPLL_VCO0_REGEXTSEL_SET(1) |
                 ALT_CLKMGR_MAINPLL_VCO0_PSRC_SET(ALT_CLKMGR_MAINPLL_VCO0_PSRC_E_INTOSC));
  } else {
    // Non-Secure boot_clk
    MmioWrite32 (ALT_CLKMGR_MAINPLL_OFST +
                 ALT_CLKMGR_MAINPLL_VCO0_OFST,
                 ALT_CLKMGR_MAINPLL_VCO0_RESET |
                 ALT_CLKMGR_MAINPLL_VCO0_REGEXTSEL_SET(1) |
                 ALT_CLKMGR_MAINPLL_VCO0_PSRC_SET(Cfg->mainpll.vco0_psrc));
  }
  // vco0 of perpllgrp
  MmioWrite32 (ALT_CLKMGR_PERPLL_OFST +
               ALT_CLKMGR_PERPLL_VCO0_OFST,
               ALT_CLKMGR_PERPLL_VCO0_RESET |
               ALT_CLKMGR_PERPLL_VCO0_REGEXTSEL_SET(1) |
               ALT_CLKMGR_PERPLL_VCO0_PSRC_SET(Cfg->perpll.vco0_psrc));

  // vco1 - Initialize denominator counter settings.
  // vco1 of mainpllgrp
  MmioAndThenOr32 (ALT_CLKMGR_MAINPLL_OFST +
                   ALT_CLKMGR_MAINPLL_VCO1_OFST,
                   ALT_CLKMGR_MAINPLL_VCO1_DENOM_CLR_MSK &
                   ALT_CLKMGR_MAINPLL_VCO1_NUMER_CLR_MSK,
                   ALT_CLKMGR_MAINPLL_VCO1_DENOM_SET(Cfg->mainpll.vco1_denom) |
                   ALT_CLKMGR_MAINPLL_VCO1_NUMER_SET(Cfg->mainpll.vco1_numer));
  // vco1 of perpllgrp
  MmioAndThenOr32 (ALT_CLKMGR_PERPLL_OFST +
                   ALT_CLKMGR_PERPLL_VCO1_OFST,
                   ALT_CLKMGR_PERPLL_VCO1_DENOM_CLR_MSK &
                   ALT_CLKMGR_PERPLL_VCO1_NUMER_CLR_MSK,
                   ALT_CLKMGR_PERPLL_VCO1_DENOM_SET(Cfg->perpll.vco1_denom) |
                   ALT_CLKMGR_PERPLL_VCO1_NUMER_SET(Cfg->perpll.vco1_numer));

  // #############################################################
  // Give enough time for software-managed clock to be reset
  MicroSecondDelay (5);
  // #############################################################

  // Power up Bandgap and Analog Circuitry
  MmioAnd32 (ALT_CLKMGR_MAINPLL_OFST +
             ALT_CLKMGR_MAINPLL_VCO0_OFST,
             ALT_CLKMGR_MAINPLL_VCO0_PWRDN_CLR_MSK &
             ALT_CLKMGR_MAINPLL_VCO0_BGPWRDN_CLR_MSK);
  MmioAnd32 (ALT_CLKMGR_PERPLL_OFST +
             ALT_CLKMGR_PERPLL_VCO0_OFST,
             ALT_CLKMGR_PERPLL_VCO0_PWRDN_CLR_MSK &
             ALT_CLKMGR_PERPLL_VCO0_BGPWRDN_CLR_MSK);

  // #############################################################
  // Give enough time for Bandgap and Analog Circuitry to power up
  MicroSecondDelay (7);
  // #############################################################

  // -----------------------------------------
  // Set Enable' bit and the 'External Regulator Input Select' together
  // It is strongly recommended to select the external regulator while the PLL is not enabled (in reset),
  // and then disable the external regulater once the PLL becomes enabled.
  // Software should simulateously update the 'Enable' bit and the 'External Regulator Input Select'
  // in the same write access to the VCO register.
  // When the 'Enable' bit is clear, the 'External Regulator Input Select' should be set, and vice versa.
  // mainpllgrp
  MmioAndThenOr32 (ALT_CLKMGR_MAINPLL_OFST +
                   ALT_CLKMGR_MAINPLL_VCO0_OFST,
                   ALT_CLKMGR_MAINPLL_VCO0_REGEXTSEL_CLR_MSK,
                   ALT_CLKMGR_MAINPLL_VCO0_EN_SET_MSK);
  // perpllgrp
  MmioAndThenOr32 (ALT_CLKMGR_PERPLL_OFST +
                   ALT_CLKMGR_PERPLL_VCO0_OFST,
                   ALT_CLKMGR_PERPLL_VCO0_REGEXTSEL_CLR_MSK,
                   ALT_CLKMGR_PERPLL_VCO0_EN_SET_MSK);
  // alteragrp.nocclk
  MmioWrite32 (ALT_CLKMGR_ALTERA_OFST +
               ALT_CLKMGR_NOCCLK_OFST,
               Cfg->alteragrp.nocclk);

  // Main PLL Clock Source and Counters/Divider
  // ------------------------------------------
  // mainpll.c0 - pll0_mpu_base_clk
  MmioWrite32 (ALT_CLKMGR_MAINPLL_OFST +
               ALT_CLKMGR_MAINPLL_MPUCLK_OFST,
               ALT_CLKMGR_MAINPLL_MPUCLK_CNT_SET(Cfg->mainpll.mpuclk_cnt) |
               ALT_CLKMGR_MAINPLL_MPUCLK_SRC_SET(Cfg->mainpll.mpuclk_src));
  // mainpll.c1 - pll0_noc _base_clk
  MmioWrite32 (ALT_CLKMGR_MAINPLL_OFST +
               ALT_CLKMGR_MAINPLL_NOCCLK_OFST,
               ALT_CLKMGR_MAINPLL_NOCCLK_CNT_SET(Cfg->mainpll.nocclk_cnt) |
               ALT_CLKMGR_MAINPLL_NOCCLK_SRC_SET(Cfg->mainpll.nocclk_src));
  // mainpll.c2 - pll0_emaca_clk
  MmioWrite32 (ALT_CLKMGR_MAINPLL_OFST +
               ALT_CLKMGR_MAINPLL_CNTR2CLK_OFST,
               ALT_CLKMGR_MAINPLL_CNTR2CLK_CNT_SET(Cfg->mainpll.cntr2clk_cnt));
  // mainpll.c3 - pll0_emacb_clk
  MmioWrite32 (ALT_CLKMGR_MAINPLL_OFST +
               ALT_CLKMGR_MAINPLL_CNTR3CLK_OFST,
               ALT_CLKMGR_MAINPLL_CNTR3CLK_CNT_SET(Cfg->mainpll.cntr3clk_cnt));
  // mainpll.c4 - pll0_emac_ptp_clk
  MmioWrite32 (ALT_CLKMGR_MAINPLL_OFST +
               ALT_CLKMGR_MAINPLL_CNTR4CLK_OFST,
               ALT_CLKMGR_MAINPLL_CNTR4CLK_CNT_SET(Cfg->mainpll.cntr4clk_cnt));
  // mainpll.c5 - pll0_gpio_db_clk
  MmioWrite32 (ALT_CLKMGR_MAINPLL_OFST +
               ALT_CLKMGR_MAINPLL_CNTR5CLK_OFST,
               ALT_CLKMGR_MAINPLL_CNTR5CLK_CNT_SET(Cfg->mainpll.cntr5clk_cnt));
  // mainpll.c6 - pll0_sdmmc_clk
  MmioWrite32 (ALT_CLKMGR_MAINPLL_OFST +
               ALT_CLKMGR_MAINPLL_CNTR6CLK_OFST,
               ALT_CLKMGR_MAINPLL_CNTR6CLK_CNT_SET(Cfg->mainpll.cntr6clk_cnt));
  // mainpll.c7 - pll0_h2f_user0_clk (A.K.A. s2f_user0_clk)
  MmioWrite32 (ALT_CLKMGR_MAINPLL_OFST +
               ALT_CLKMGR_MAINPLL_CNTR7CLK_OFST,
               ALT_CLKMGR_MAINPLL_CNTR7CLK_CNT_SET(Cfg->mainpll.cntr7clk_cnt) |
               ALT_CLKMGR_MAINPLL_CNTR7CLK_SRC_SET(Cfg->mainpll.cntr7clk_src));
  // mainpll.c8 - pll0_h2f_user1_clk (A.K.A. s2f_user1_clk)
  MmioWrite32 (ALT_CLKMGR_MAINPLL_OFST +
               ALT_CLKMGR_MAINPLL_CNTR8CLK_OFST,
               ALT_CLKMGR_MAINPLL_CNTR8CLK_CNT_SET(Cfg->mainpll.cntr8clk_cnt));
  // mainpll.c9 - pll0_hmc_pll_ref_clk
  MmioWrite32 (ALT_CLKMGR_MAINPLL_OFST +
               ALT_CLKMGR_MAINPLL_CNTR9CLK_OFST,
               ALT_CLKMGR_MAINPLL_CNTR9CLK_CNT_SET(Cfg->mainpll.cntr9clk_cnt) |
               ALT_CLKMGR_MAINPLL_CNTR9CLK_SRC_SET(Cfg->mainpll.cntr9clk_src));
  // mainpll.c15 - periph_ref_clk
  // Comment out because C15 input for PLL1 is not supported (2014.12.15 A10_5v4.PDF)
  // MmioWrite32 (ALT_CLKMGR_MAINPLL_OFST +
  //             ALT_CLKMGR_MAINPLL_CNTR15CLK_OFST,
  //             ALT_CLKMGR_MAINPLL_CNTR15CLK_CNT_SET(Cfg->mainpll.cntr15clk_cnt));
  // mainpll's NoC Clocks's divider
  MmioWrite32 (ALT_CLKMGR_MAINPLL_OFST +
               ALT_CLKMGR_MAINPLL_NOCDIV_OFST,
               ALT_CLKMGR_MAINPLL_NOCDIV_L4MAINCLK_SET(Cfg->mainpll.nocdiv_l4mainclk) |
               ALT_CLKMGR_MAINPLL_NOCDIV_L4MPCLK_SET(Cfg->mainpll.nocdiv_l4mpclk) |
               ALT_CLKMGR_MAINPLL_NOCDIV_L4SPCLK_SET(Cfg->mainpll.nocdiv_l4spclk) |
               ALT_CLKMGR_MAINPLL_NOCDIV_CSATCLK_SET(Cfg->mainpll.nocdiv_csatclk) |
               ALT_CLKMGR_MAINPLL_NOCDIV_CSTRACECLK_SET(Cfg->mainpll.nocdiv_cstraceclk) |
               ALT_CLKMGR_MAINPLL_NOCDIV_CSPDBGCLK_SET(Cfg->mainpll.nocdiv_cspdbgclk));

  // Peripheral PLL Clock Source and Counters/Divider
  // ------------------------------------------------
  // perpll.c2 - pll1_emaca_clk
  MmioWrite32 (ALT_CLKMGR_PERPLL_OFST +
               ALT_CLKMGR_PERPLL_CNTR2CLK_OFST,
               ALT_CLKMGR_PERPLL_CNTR2CLK_CNT_SET(Cfg->perpll.cntr2clk_cnt) |
               ALT_CLKMGR_PERPLL_CNTR2CLK_SRC_SET(Cfg->perpll.cntr2clk_src));
  // perpll.c3 - pll1_emacb_clk
  MmioWrite32 (ALT_CLKMGR_PERPLL_OFST +
               ALT_CLKMGR_PERPLL_CNTR3CLK_OFST,
               ALT_CLKMGR_PERPLL_CNTR3CLK_CNT_SET(Cfg->perpll.cntr3clk_cnt) |
               ALT_CLKMGR_PERPLL_CNTR3CLK_SRC_SET(Cfg->perpll.cntr3clk_src));
  // perpll.c4 - pll1_emac_ptp_clk
  MmioWrite32 (ALT_CLKMGR_PERPLL_OFST +
               ALT_CLKMGR_PERPLL_CNTR4CLK_OFST,
               ALT_CLKMGR_PERPLL_CNTR4CLK_CNT_SET(Cfg->perpll.cntr4clk_cnt) |
               ALT_CLKMGR_PERPLL_CNTR4CLK_SRC_SET(Cfg->perpll.cntr4clk_src));
  // perpll.c5 - pll1_gpio_db_clk
  MmioWrite32 (ALT_CLKMGR_PERPLL_OFST +
               ALT_CLKMGR_PERPLL_CNTR5CLK_OFST,
               ALT_CLKMGR_PERPLL_CNTR5CLK_CNT_SET(Cfg->perpll.cntr5clk_cnt) |
               ALT_CLKMGR_PERPLL_CNTR5CLK_SRC_SET(Cfg->perpll.cntr5clk_src));
  // perpll.c6 - pll1_sdmmc_clk
  MmioWrite32 (ALT_CLKMGR_PERPLL_OFST +
               ALT_CLKMGR_PERPLL_CNTR6CLK_OFST,
               ALT_CLKMGR_PERPLL_CNTR6CLK_CNT_SET(Cfg->perpll.cntr6clk_cnt) |
               ALT_CLKMGR_PERPLL_CNTR6CLK_SRC_SET(Cfg->perpll.cntr6clk_src));
  // perpll.c7 - pll1_h2f_user0_clk (A.K.A. s2f_user0_clk)
  MmioWrite32 (ALT_CLKMGR_PERPLL_OFST +
               ALT_CLKMGR_PERPLL_CNTR7CLK_OFST,
               ALT_CLKMGR_PERPLL_CNTR7CLK_CNT_SET(Cfg->perpll.cntr7clk_cnt));
  // perpll.c8 - pll1_h2f_user1_clk (A.K.A. s2f_user1_clk)
  MmioWrite32 (ALT_CLKMGR_PERPLL_OFST +
               ALT_CLKMGR_PERPLL_CNTR8CLK_OFST,
               ALT_CLKMGR_PERPLL_CNTR8CLK_CNT_SET(Cfg->perpll.cntr8clk_cnt) |
               ALT_CLKMGR_PERPLL_CNTR8CLK_SRC_SET(Cfg->perpll.cntr8clk_src));
  // perpll.c9 - pll1_hmc_pll_ref_clk
  MmioWrite32 (ALT_CLKMGR_PERPLL_OFST +
               ALT_CLKMGR_PERPLL_CNTR9CLK_OFST,
               ALT_CLKMGR_PERPLL_CNTR9CLK_CNT_SET(Cfg->perpll.cntr9clk_cnt));

  // Select EMAC clock source
  MmioWrite32 (ALT_CLKMGR_PERPLL_OFST +
               ALT_CLKMGR_PERPLL_EMACCTL_OFST,
               ALT_CLKMGR_PERPLL_EMACCTL_EMAC0SEL_SET(Cfg->perpll.emacctl_emac0sel) |
               ALT_CLKMGR_PERPLL_EMACCTL_EMAC1SEL_SET(Cfg->perpll.emacctl_emac1sel) |
               ALT_CLKMGR_PERPLL_EMACCTL_EMAC2SEL_SET(Cfg->perpll.emacctl_emac2sel));

  // Init GPIO De-bounce Clock Divider
  MmioWrite32 (ALT_CLKMGR_PERPLL_OFST +
               ALT_CLKMGR_PERPLL_GPIODIV_OFST,
               ALT_CLKMGR_PERPLL_GPIODIV_GPIODBCLK_SET(Cfg->perpll.gpiodiv_gpiodbclk));

  // #############################################################
  // Wait until both the Main PLL and the Peripheral PLL is locked
  do {
    Data32 = MmioRead32 (ALT_CLKMGR_CLKMGR_OFST +
                         ALT_CLKMGR_CLKMGR_STAT_OFST);
  } while ((ALT_CLKMGR_CLKMGR_STAT_MAINPLLLOCKED_GET(Data32) == 0) ||
           (ALT_CLKMGR_CLKMGR_STAT_PERPLLLOCKED_GET(Data32) == 0));
  // #############################################################


  // Now PLL is locked, but before releasing Bypass,
  // All Output Counter Reset must be set and cleared by software for correct clock operation.
  // mainpll - Assert 'outresetall'
  MmioOr32 (ALT_CLKMGR_MAINPLL_OFST +
            ALT_CLKMGR_MAINPLL_VCO0_OFST,
            ALT_CLKMGR_MAINPLL_VCO0_OUTRSTALL_SET_MSK);
  // perpll - Assert 'outresetall'
  MmioOr32 (ALT_CLKMGR_PERPLL_OFST +
            ALT_CLKMGR_PERPLL_VCO0_OFST,
            ALT_CLKMGR_PERPLL_VCO0_OUTRSTALL_SET_MSK);
  // mainpll - Deassert 'outresetall'
  MmioAnd32 (ALT_CLKMGR_MAINPLL_OFST +
             ALT_CLKMGR_MAINPLL_VCO0_OFST,
             ALT_CLKMGR_MAINPLL_VCO0_OUTRSTALL_CLR_MSK);
  // perpll - Deassert 'outresetall'
  MmioAnd32 (ALT_CLKMGR_PERPLL_OFST +
             ALT_CLKMGR_PERPLL_VCO0_OFST,
             ALT_CLKMGR_PERPLL_VCO0_OUTRSTALL_CLR_MSK);


  // Release all PLL from Bypass and then clear Clock Manager's Bootmode bit
  // Note: Bypass Register has bypass, bypassS (set), bypassR (clear)

  // =============================================================
  // Release from Bypass all mainpllgrp's clocks
  MmioWrite32 (ALT_CLKMGR_MAINPLL_OFST +
               ALT_CLKMGR_MAINPLL_BYPASSR_OFST,
               ALT_CLKMGR_MAINPLL_BYPASSR_RESET);
  // Immediately following writes to
  // CTRL.BOOTMODE, MAINPLLGRP.BYPASS.MPU or MAINPLLGRP.BYPASS.NOC register bits
  // SW should wait 0.5 usecs and then poll this BUSY bit until it is IDLE
  // before proceeding with any other register writes in the Clock Manager.
  NanoSecondDelay(500);
  do {
    Data32 = MmioRead32 (ALT_CLKMGR_CLKMGR_OFST +
                         ALT_CLKMGR_CLKMGR_STAT_OFST);
  } while ((ALT_CLKMGR_CLKMGR_STAT_BUSY_GET(Data32) != ALT_CLKMGR_CLKMGR_STAT_BUSY_E_IDLE));
  // =============================================================


  // =============================================================
  // Release from Bypass all perpllgrp's clocks
  MmioWrite32 (ALT_CLKMGR_PERPLL_OFST +
               ALT_CLKMGR_PERPLL_BYPASSR_OFST,
               ALT_CLKMGR_PERPLL_BYPASSR_RESET);
  // Immediately following writes to
  // CTRL.BOOTMODE, MAINPLLGRP.BYPASS.MPU or MAINPLLGRP.BYPASS.NOC register bits
  // SW should wait 0.5 usecs and then poll this BUSY bit until it is IDLE
  // before proceeding with any other register writes in the Clock Manager.
  NanoSecondDelay(500);
  do {
    Data32 = MmioRead32 (ALT_CLKMGR_CLKMGR_OFST +
                         ALT_CLKMGR_CLKMGR_STAT_OFST);
  } while ((ALT_CLKMGR_CLKMGR_STAT_BUSY_GET(Data32) != ALT_CLKMGR_CLKMGR_STAT_BUSY_E_IDLE));
  // =============================================================


  // =============================================================
  // Clear Clock Manager's Bootmode bit
  MmioAnd32 (ALT_CLKMGR_CLKMGR_OFST +
             ALT_CLKMGR_CLKMGR_CTL_OFST,
             ALT_CLKMGR_CLKMGR_CTL_BOOTMOD_CLR_MSK);
  // Immediately following writes to
  // CTRL.BOOTMODE, MAINPLLGRP.BYPASS.MPU or MAINPLLGRP.BYPASS.NOC register bits
  // SW should wait 0.5 usecs and then poll this BUSY bit until it is IDLE
  // before proceeding with any other register writes in the Clock Manager.
  NanoSecondDelay(500);
  do {
    Data32 = MmioRead32 (ALT_CLKMGR_CLKMGR_OFST +
                         ALT_CLKMGR_CLKMGR_STAT_OFST);
  } while ((ALT_CLKMGR_CLKMGR_STAT_BUSY_GET(Data32) != ALT_CLKMGR_CLKMGR_STAT_BUSY_E_IDLE));
  // =============================================================

  // Enable mainpllgrp's software-managed clock
  MmioWrite32 (ALT_CLKMGR_MAINPLL_OFST +
               ALT_CLKMGR_MAINPLL_ENS_OFST,
               ALT_CLKMGR_MAINPLL_ENS_S2FUSER0CLKEN_SET_MSK |
               ALT_CLKMGR_MAINPLL_ENS_HMCPLLREFCLKEN_SET_MSK);
  // Enable perpllgrp's software-managed clock
  MmioWrite32 (ALT_CLKMGR_PERPLL_OFST +
               ALT_CLKMGR_PERPLL_ENS_OFST,
               ALT_CLKMGR_PERPLL_ENS_RESET);

  // Clear all interrupt status register, loss lock and slip bits might set during configuration
  MmioWrite32 (ALT_CLKMGR_CLKMGR_OFST +
               ALT_CLKMGR_CLKMGR_INTR_OFST,
               ALT_CLKMGR_CLKMGR_INTR_MAINPLLLOST_SET_MSK |
               ALT_CLKMGR_CLKMGR_INTR_PERPLLLOST_SET_MSK |
               ALT_CLKMGR_CLKMGR_INTR_MAINPLLRFSLIP_SET_MSK |
               ALT_CLKMGR_CLKMGR_INTR_PERPLLRFSLIP_SET_MSK |
               ALT_CLKMGR_CLKMGR_INTR_MAINPLLFBSLIP_SET_MSK |
               ALT_CLKMGR_CLKMGR_INTR_PERPLLFBSLIP_SET_MSK |
               ALT_CLKMGR_CLKMGR_INTR_MAINPLLACHIEVED_SET_MSK |
               ALT_CLKMGR_CLKMGR_INTR_PERPLLACHIEVED_SET_MSK);
}


VOID
EFIAPI
DisplayClockManagerInfo (
  VOID
  )
{
  UINTN         i;

  // Char8Str is use to speed up Semihosting printing speed
  CHAR8   Char8Str[2048];
  CHAR8*  Char8Ptr = &Char8Str[0];

  Char8Ptr += AsciiSPrint (Char8Ptr, 1024,
    "Clock Manager Info:\r\n");

  Char8Ptr += AsciiSPrint (Char8Ptr, 1024,
    "\t mainpll.vco0     :\t0x%08x\r\n"
    "\t mainpll.vco1     :\t0x%08x\r\n"
    "\t mainpll.mpuclk   :\t0x%08x\r\n"
    "\t mainpll.nocclk   :\t0x%08x\r\n",
    MmioRead32 (ALT_CLKMGR_MAINPLL_OFST + ALT_CLKMGR_MAINPLL_VCO0_OFST),
    MmioRead32 (ALT_CLKMGR_MAINPLL_OFST + ALT_CLKMGR_MAINPLL_VCO1_OFST),
    MmioRead32 (ALT_CLKMGR_MAINPLL_OFST + ALT_CLKMGR_MAINPLL_MPUCLK_OFST),
    MmioRead32 (ALT_CLKMGR_MAINPLL_OFST + ALT_CLKMGR_MAINPLL_NOCCLK_OFST));

  for (i = 0; i < 8; i++)
  {
    Char8Ptr += AsciiSPrint (Char8Ptr, 1024,
      "\t mainpll.cntr%dclk :\t0x%08x\r\n", (i + 2),
      MmioRead32 (ALT_CLKMGR_MAINPLL_OFST + ALT_CLKMGR_MAINPLL_CNTR2CLK_OFST + (i * 4)));
  }

  InfoPrint ("%a", Char8Str); Char8Ptr = &Char8Str[0];

  Char8Ptr += AsciiSPrint (Char8Ptr, 1024,
    "\t mainpll.nocdiv   :\t0x%08x\r\n"
    "\t perpll.vco0      :\t0x%08x\r\n"
    "\t perpll.vco1      :\t0x%08x\r\n",
     MmioRead32 (ALT_CLKMGR_MAINPLL_OFST + ALT_CLKMGR_MAINPLL_NOCDIV_OFST),
     MmioRead32 (ALT_CLKMGR_PERPLL_OFST + ALT_CLKMGR_PERPLL_VCO0_OFST),
     MmioRead32 (ALT_CLKMGR_PERPLL_OFST + ALT_CLKMGR_PERPLL_VCO1_OFST));

  for (i = 0; i < 8; i++)
  {
    Char8Ptr += AsciiSPrint (Char8Ptr, 1024,
      "\t perpll.cntr%dclk  :\t0x%08x\r\n", (i + 2),
      MmioRead32 (ALT_CLKMGR_PERPLL_OFST + ALT_CLKMGR_PERPLL_CNTR2CLK_OFST + (i * 4)));
  }

  Char8Ptr += AsciiSPrint (Char8Ptr, 1024,
    "\t perpll.emacctl   :\t0x%08x\r\n"
    "\t perpll.gpiodiv   :\t0x%08x\r\n"
    "\t alteragrp.mpuclk :\t0x%08x\r\n"
    "\t alteragrp.nocclk :\t0x%08x\r\n",
     MmioRead32 (ALT_CLKMGR_PERPLL_OFST + ALT_CLKMGR_PERPLL_EMACCTL_OFST),
     MmioRead32 (ALT_CLKMGR_PERPLL_OFST + ALT_CLKMGR_PERPLL_GPIODIV_OFST),
     MmioRead32 (ALT_CLKMGR_ALTERA_OFST + ALT_CLKMGR_MPUCLK_OFST),
     MmioRead32 (ALT_CLKMGR_ALTERA_OFST + ALT_CLKMGR_NOCCLK_OFST));

  InfoPrint ("%a", Char8Str); Char8Ptr = &Char8Str[0];

  DisplayClockFrequencyInfo ();
}

VOID
EFIAPI
DisplayClockFrequencyInfo (
  VOID
  )
{
  // Char8Str is use to speed up Semihosting printing speed
  CHAR8   Char8Str[2048];
  CHAR8*  Char8Ptr = &Char8Str[0];

#if (FixedPcdGet32(PcdDebugMsg_ClockManager) != 0)
  UINT32  Div14[2] = {1, 4};
#endif
  UINT32  Div1248[4] = {1, 2, 4, 8};

  UINT32  Data32;
  UINT32  PLL0InputClock;
  UINT32  PLL1InputClock;
  UINT32  PLL0VcoFreqInMhz;
  UINT32  PLL1VcoFreqInMhz;
  UINT32  PLL0_mpu_base_clk_InMhz;
  UINT32  PLL0_noc_base_clk_InMhz;
  UINT32  mpu_free_clk_InMhz;
  UINT32  mpu_clk_InMhz;
  UINT32  cs_at_clk_InMhz;
  UINT32  l3_main_free_clk_InMhz;
  UINT32  l4_sys_free_clk_InMhz;
  UINT32  l4_main_clk_InMhz;
  UINT32  l4_mp_clk_InMhz;
  UINT32  l4_sp_clk_InMhz;
  UINT32  emaca_clk_InMhz;
  UINT32  emacb_clk_InMhz;
  UINT32  emac_ptp_clk_InMhz;
  UINT32  gpio_db_clk_InMhz;
  UINT32  sdmmc_clk_InMhz;
  UINT32  s2f_user0_clk_InMhz;
  UINT32  s2f_user1_clk_InMhz;
  UINT32  hmc_pll_ref_clk_InMhz;

  CLOCK_MANAGER_CONFIG*  Cfg;
  Cfg = &mClkCfg;

  emaca_clk_InMhz = 0;
  emacb_clk_InMhz = 0;
  emac_ptp_clk_InMhz = 0;
  gpio_db_clk_InMhz = 0;
  sdmmc_clk_InMhz = 0;
  s2f_user0_clk_InMhz = 0;
  s2f_user1_clk_InMhz = 0;
  hmc_pll_ref_clk_InMhz = 0;

  Char8Ptr += AsciiSPrint (Char8Ptr, 1024,
    "Clock Frequency Info:\r\n");

  // Boot Clock
  // ----------
  // The status of the hps_clk_f security fuse in the Security Manager determines if boot_clk should be secure:
  // If set, then boot_clk is cb_intosc_hs_clk divided by 2.
  // If clear, then boot_clk is EOSC1.
  Char8Ptr += AsciiSPrint (Char8Ptr, 1024,
    "\t List of possible Boot Clock Source:\r\n"
    "\t\t EOSC1            : %d MHz\r\n"
    "\t\t cb_intosc_hs_clk : 120-400 MHz \r\n",
     mClkSrc.clk_freq_of_eosc1/1000/1000);

  // If swctrlbtclken set, then Software will take control of the boot_clk mux select else it will follow fuse.
  Data32 = MmioRead32 (ALT_CLKMGR_CLKMGR_OFST + ALT_CLKMGR_CLKMGR_CTL_OFST);
  if (ALT_CLKMGR_CLKMGR_CTL_SWCTLBTCLKEN_GET(Data32) == 1) {
      Char8Ptr += AsciiSPrint (Char8Ptr, 1024,
        "\t\t Software took control of the boot_clk mux select\r\n");
    if (ALT_CLKMGR_CLKMGR_CTL_SWCTLBTCLKSEL_GET(Data32) == 1)
      Char8Ptr += AsciiSPrint (Char8Ptr, 1024,
        "\t\t The boot_clk is cb_intosc_hs_clk divided by 2\r\n");
    else
      Char8Ptr += AsciiSPrint (Char8Ptr, 1024,
        "\t\t The boot_clk is EOSC1\r\n");
  } else {
    Data32 = MmioRead32 (ALT_SEC_MGR_OFST + ALT_SEC_MGR_CURSECSTATE_OFST);
    if (ALT_SEC_MGR_CURSECSTATE_STATE_CLK_GET(Data32) ==
        ALT_SEC_MGR_CURSECSTATE_STATE_CLK_E_IOSC_CLK)
      Char8Ptr += AsciiSPrint (Char8Ptr, 1024,
        "\t\t The boot_clk is cb_intosc_hs_clk divided by 2\r\n");
    else
      Char8Ptr += AsciiSPrint (Char8Ptr, 1024,
        "\t\t The boot_clk is EOSC1\r\n");
  }

  // PLL Clocks
  // ----------
  Char8Ptr += AsciiSPrint (Char8Ptr, 1024,
    "\t List of possible Clock Manager PLLs Clock Source:\r\n"
    "\t\t EOSC1                 : %d MHz\r\n"
    "\t\t cb_intosc_ls_clk      : 30-100 MHz\r\n"
    "\t\t f2h_free_clk          : %d MHz\r\n",
    mClkSrc.clk_freq_of_eosc1/1000/1000,
    mClkSrc.clk_freq_of_f2h_free/1000/1000);

  Char8Ptr += AsciiSPrint (Char8Ptr, 1024,
    "\t\t The pll0_ref_clk (mainpllgrp) is ");
  Data32 = MmioRead32 (ALT_CLKMGR_MAINPLL_OFST + ALT_CLKMGR_MAINPLL_VCO0_OFST);
  switch (ALT_CLKMGR_MAINPLL_VCO0_PSRC_GET(Data32))
  {
    case ALT_CLKMGR_MAINPLL_VCO0_PSRC_E_EOSC1:
      PLL0InputClock = mClkSrc.clk_freq_of_eosc1;
      Char8Ptr += AsciiSPrint (Char8Ptr, 1024, "EOSC1\r\n");
      break;
    case ALT_CLKMGR_MAINPLL_VCO0_PSRC_E_INTOSC:
      PLL0InputClock = mClkSrc.clk_freq_of_cb_intosc_ls;
      Char8Ptr += AsciiSPrint (Char8Ptr, 1024, "cb_intosc_ls_clk\r\n");
      break;
    case ALT_CLKMGR_MAINPLL_VCO0_PSRC_E_F2S:
      PLL0InputClock = mClkSrc.clk_freq_of_f2h_free;
      Char8Ptr += AsciiSPrint (Char8Ptr, 1024, "f2h_free_clk\r\n");
      break;
    default:
      // We should have covered all possible conditions above
      PLL0InputClock = 0;
      ASSERT_PLATFORM_INIT(0);
      break;
  }

  Char8Ptr += AsciiSPrint (Char8Ptr, 1024,
    "\t\t The pll1_ref_clk (perpllgrp) is ");
  Data32 = MmioRead32 (ALT_CLKMGR_PERPLL_OFST + ALT_CLKMGR_PERPLL_VCO0_OFST);
  switch (ALT_CLKMGR_PERPLL_VCO0_PSRC_GET(Data32))
  {
    case ALT_CLKMGR_PERPLL_VCO0_PSRC_E_EOSC1:
      PLL1InputClock = mClkSrc.clk_freq_of_eosc1;
      Char8Ptr += AsciiSPrint (Char8Ptr, 1024, "EOSC1\r\n");
      break;
    case ALT_CLKMGR_PERPLL_VCO0_PSRC_E_INTOSC:
      PLL1InputClock = mClkSrc.clk_freq_of_cb_intosc_ls;
      Char8Ptr += AsciiSPrint (Char8Ptr, 1024, "cb_intosc_ls_clk\r\n");
      break;
    case ALT_CLKMGR_PERPLL_VCO0_PSRC_E_F2S:
      PLL1InputClock = mClkSrc.clk_freq_of_f2h_free;
      Char8Ptr += AsciiSPrint (Char8Ptr, 1024, "f2h_free_clk\r\n");
      break;
    default:
      // We should have covered all supported conditions above
      PLL1InputClock = 0;
      ASSERT_PLATFORM_INIT(0);
      break;
  }

  InfoPrint ("%a", Char8Str); Char8Ptr = &Char8Str[0];

  // Calculate PLL's VCO Frequency
  PLL0VcoFreqInMhz = PLL0InputClock;
  PLL0VcoFreqInMhz = PLL0VcoFreqInMhz / (Cfg->mainpll.vco1_denom + 1);
  PLL0VcoFreqInMhz = PLL0VcoFreqInMhz * (Cfg->mainpll.vco1_numer + 1);
  PLL0VcoFreqInMhz = PLL0VcoFreqInMhz / (1000000);
  PLL1VcoFreqInMhz = PLL1InputClock;
  PLL1VcoFreqInMhz = PLL1VcoFreqInMhz / (Cfg->perpll.vco1_denom + 1);
  PLL1VcoFreqInMhz = PLL1VcoFreqInMhz * (Cfg->perpll.vco1_numer + 1);
  PLL1VcoFreqInMhz = PLL1VcoFreqInMhz / (1000000);
  Data32 = MmioRead32 (ALT_CLKMGR_ALTERA_OFST + ALT_CLKMGR_MPUCLK_OFST);
  PLL0_mpu_base_clk_InMhz = (PLL0VcoFreqInMhz / (ALT_CLKMGR_MPUCLK_MAINCNT_GET(Data32)+1));
  Data32 = MmioRead32 (ALT_CLKMGR_ALTERA_OFST + ALT_CLKMGR_NOCCLK_OFST);
  PLL0_noc_base_clk_InMhz = (PLL0VcoFreqInMhz / (ALT_CLKMGR_NOCCLK_MAINCNT_GET(Data32)+1));

  Char8Ptr += AsciiSPrint (Char8Ptr, 1024, "\t PLL 0 Output Clock Frequency: \r\n");
  Char8Ptr += AsciiSPrint (Char8Ptr, 1024, "\t\t pll%d_mpu_base_clk    (C0): %d MHz\r\n", 0, PLL0_mpu_base_clk_InMhz);
  Char8Ptr += AsciiSPrint (Char8Ptr, 1024, "\t\t pll%d_noc_base_clk    (C1): %d MHz\r\n", 0, PLL0_noc_base_clk_InMhz);
  Char8Ptr += AsciiSPrint (Char8Ptr, 1024, "\t\t pll%d_emaca_clk       (C2): ", 0);
  if (Cfg->perpll.cntr2clk_src == 0)
  {
    emaca_clk_InMhz = (PLL0VcoFreqInMhz) / (Cfg->mainpll.cntr2clk_cnt + 1);
    Char8Ptr += AsciiSPrint (Char8Ptr, 1024, "%d MHz\r\n", emaca_clk_InMhz);
  } else {
    Char8Ptr += AsciiSPrint (Char8Ptr, 1024, "Unused\r\n");
  }
    Char8Ptr += AsciiSPrint (Char8Ptr, 1024, "\t\t pll%d_emacb_clk       (C3): ", 0);
  if (Cfg->perpll.cntr3clk_src == 0)
  {
    emacb_clk_InMhz = (PLL0VcoFreqInMhz) / (Cfg->mainpll.cntr3clk_cnt + 1);
    Char8Ptr += AsciiSPrint (Char8Ptr, 1024, "%d MHz\r\n", emacb_clk_InMhz);
  } else {
    Char8Ptr += AsciiSPrint (Char8Ptr, 1024, "Unused\r\n");
  }
  Char8Ptr += AsciiSPrint (Char8Ptr, 1024, "\t\t pll%d_emac_ptp_clk    (C4): ", 0);
  if (Cfg->perpll.cntr4clk_src == 0)
  {
    emac_ptp_clk_InMhz = (PLL0VcoFreqInMhz) / (Cfg->mainpll.cntr4clk_cnt + 1);
    Char8Ptr += AsciiSPrint (Char8Ptr, 1024, "%d MHz\r\n", emac_ptp_clk_InMhz);
  } else {
    Char8Ptr += AsciiSPrint (Char8Ptr, 1024, "Unused\r\n");
  }
  Char8Ptr += AsciiSPrint (Char8Ptr, 1024, "\t\t pll%d_gpio_db_clk     (C5): ", 0);
  if (Cfg->perpll.cntr5clk_src == 0)
  {
    gpio_db_clk_InMhz  = (PLL0VcoFreqInMhz) / (Cfg->mainpll.cntr5clk_cnt + 1);
    Char8Ptr += AsciiSPrint (Char8Ptr, 1024, "%d MHz\r\n", gpio_db_clk_InMhz);
  } else {
    Char8Ptr += AsciiSPrint (Char8Ptr, 1024, "Unused\r\n");
  }
  Char8Ptr += AsciiSPrint (Char8Ptr, 1024, "\t\t pll%d_sdmmc_clk       (C6): ", 0);
  if (Cfg->perpll.cntr6clk_src == 0)
  {
    sdmmc_clk_InMhz = (PLL0VcoFreqInMhz) / (Cfg->mainpll.cntr6clk_cnt + 1);
    Char8Ptr += AsciiSPrint (Char8Ptr, 1024, "%d MHz\r\n", sdmmc_clk_InMhz);
  } else {
    Char8Ptr += AsciiSPrint (Char8Ptr, 1024, "Unused\r\n");
  }
  Char8Ptr += AsciiSPrint (Char8Ptr, 1024, "\t\t pll%d_h2f_user0_clk   (C7): ", 0);
  if (Cfg->mainpll.cntr7clk_src == 0)
  {
    s2f_user0_clk_InMhz = (PLL0VcoFreqInMhz) / (Cfg->mainpll.cntr7clk_cnt + 1);
    Char8Ptr += AsciiSPrint (Char8Ptr, 1024, "%d MHz\r\n", s2f_user0_clk_InMhz);
  } else {
    Char8Ptr += AsciiSPrint (Char8Ptr, 1024, "Unused\r\n");
  }
  Char8Ptr += AsciiSPrint (Char8Ptr, 1024, "\t\t pll%d_h2f_user1_clk   (C8): ", 0);
  if (Cfg->perpll.cntr8clk_src == 0)
  {
    s2f_user1_clk_InMhz = (PLL0VcoFreqInMhz) / (Cfg->mainpll.cntr8clk_cnt + 1);
    Char8Ptr += AsciiSPrint (Char8Ptr, 1024, "%d MHz\r\n", s2f_user1_clk_InMhz);
  } else {
    Char8Ptr += AsciiSPrint (Char8Ptr, 1024, "Unused\r\n");
  }
  Char8Ptr += AsciiSPrint (Char8Ptr, 1024, "\t\t pll%d_hmc_pll_ref_clk (C9): ", 0);
  if (Cfg->mainpll.cntr9clk_src == 0)
  {
    hmc_pll_ref_clk_InMhz = (PLL0VcoFreqInMhz) / (Cfg->mainpll.cntr9clk_cnt + 1);
    Char8Ptr += AsciiSPrint (Char8Ptr, 1024, "%d MHz\r\n", hmc_pll_ref_clk_InMhz);
  } else {
    Char8Ptr += AsciiSPrint (Char8Ptr, 1024, "Unused\r\n");
  }

  InfoPrint ("%a", Char8Str); Char8Ptr = &Char8Str[0];

  Char8Ptr += AsciiSPrint (Char8Ptr, 1024, "\t PLL 1 Output Clock Frequency: \r\n");
  Char8Ptr += AsciiSPrint (Char8Ptr, 1024, "\t\t pll%d_emaca_clk       (C2): ", 1);
  if (Cfg->perpll.cntr2clk_src == 1)
  {
    emaca_clk_InMhz = (PLL1VcoFreqInMhz) / (Cfg->perpll.cntr2clk_cnt + 1);
    Char8Ptr += AsciiSPrint (Char8Ptr, 1024, "%d MHz\r\n", emaca_clk_InMhz);
  } else {
    Char8Ptr += AsciiSPrint (Char8Ptr, 1024, "Unused\r\n");
  }
  Char8Ptr += AsciiSPrint (Char8Ptr, 1024, "\t\t pll%d_emacb_clk       (C3): ", 1);
  if (Cfg->perpll.cntr3clk_src == 1)
  {
    emacb_clk_InMhz = (PLL1VcoFreqInMhz) / (Cfg->perpll.cntr3clk_cnt + 1);
    Char8Ptr += AsciiSPrint (Char8Ptr, 1024, "%d MHz\r\n", emacb_clk_InMhz);
  } else {
    Char8Ptr += AsciiSPrint (Char8Ptr, 1024, "Unused\r\n");
  }
  Char8Ptr += AsciiSPrint (Char8Ptr, 1024, "\t\t pll%d_emac_ptp_clk    (C4): ", 1);
  if (Cfg->perpll.cntr4clk_src == 1)
  {
    emac_ptp_clk_InMhz = (PLL1VcoFreqInMhz) / (Cfg->perpll.cntr4clk_cnt + 1);
    Char8Ptr += AsciiSPrint (Char8Ptr, 1024, "%d MHz\r\n", emac_ptp_clk_InMhz);
  } else {
    Char8Ptr += AsciiSPrint (Char8Ptr, 1024, "Unused\r\n");
  }
  Char8Ptr += AsciiSPrint (Char8Ptr, 1024, "\t\t pll%d_gpio_db_clk     (C5): ", 1);
  if (Cfg->perpll.cntr5clk_src == 1)
  {
    gpio_db_clk_InMhz = (PLL1VcoFreqInMhz) / (Cfg->perpll.cntr5clk_cnt + 1);
    Char8Ptr += AsciiSPrint (Char8Ptr, 1024, "%d MHz\r\n", gpio_db_clk_InMhz);
  } else {
    Char8Ptr += AsciiSPrint (Char8Ptr, 1024, "Unused\r\n");
  }
  Char8Ptr += AsciiSPrint (Char8Ptr, 1024, "\t\t pll%d_sdmmc_clk       (C6): ", 1);
  if (Cfg->perpll.cntr6clk_src == 1)
  {
    sdmmc_clk_InMhz = (PLL1VcoFreqInMhz) / (Cfg->perpll.cntr6clk_cnt + 1);
    Char8Ptr += AsciiSPrint (Char8Ptr, 1024, "%d MHz\r\n", sdmmc_clk_InMhz);
  } else {
    Char8Ptr += AsciiSPrint (Char8Ptr, 1024, "Unused\r\n");
  }
  Char8Ptr += AsciiSPrint (Char8Ptr, 1024, "\t\t pll%d_h2f_user0_clk   (C7): ", 1);
  if (Cfg->mainpll.cntr7clk_src == 1)
  {
    s2f_user0_clk_InMhz = (PLL1VcoFreqInMhz) / (Cfg->perpll.cntr7clk_cnt + 1);
    Char8Ptr += AsciiSPrint (Char8Ptr, 1024, "%d MHz\r\n", s2f_user0_clk_InMhz);
  } else {
    Char8Ptr += AsciiSPrint (Char8Ptr, 1024, "Unused\r\n");
  }
  Char8Ptr += AsciiSPrint (Char8Ptr, 1024, "\t\t pll%d_h2f_user1_clk   (C8): ", 1);
  if (Cfg->perpll.cntr8clk_src == 1)
  {
    s2f_user1_clk_InMhz = (PLL1VcoFreqInMhz) / (Cfg->perpll.cntr8clk_cnt + 1);
    Char8Ptr += AsciiSPrint (Char8Ptr, 1024, "%d MHz\r\n", s2f_user1_clk_InMhz);
  } else {
    Char8Ptr += AsciiSPrint (Char8Ptr, 1024, "Unused\r\n");
  }
  Char8Ptr += AsciiSPrint (Char8Ptr, 1024, "\t\t pll%d_hmc_pll_ref_clk (C9): ", 1);
  if (Cfg->mainpll.cntr9clk_src == 1)
  {
    hmc_pll_ref_clk_InMhz = (PLL1VcoFreqInMhz) / (Cfg->perpll.cntr9clk_cnt + 1);
    Char8Ptr += AsciiSPrint (Char8Ptr, 1024, "%d MHz\r\n", hmc_pll_ref_clk_InMhz);
  } else {
    Char8Ptr += AsciiSPrint (Char8Ptr, 1024, "Unused\r\n");
  }

  InfoPrint ("%a", Char8Str); Char8Ptr = &Char8Str[0];

  InfoPrint ("\t MPU Clock Group : \r\n");
  mpu_free_clk_InMhz = PLL0_mpu_base_clk_InMhz;
  InfoPrint ("\t\t mpu_free_clk clock source is ");
  switch (Cfg->mainpll.mpuclk_src)
  {
    case 0:
      InfoPrint ("pll0_mpu_base_clk\r\n");
      break;
    case 1:
      // Not supported
      InfoPrint ("PLL1.C0 (Not supported)\r\n");
      ASSERT_PLATFORM_INIT(0);
      break;
    case 2:
      InfoPrint ("EOSC1 pin\r\n");
      mpu_free_clk_InMhz = mClkSrc.clk_freq_of_eosc1/1000/1000;
      break;
    case 3:
      InfoPrint ("cb_intosc_hs_div2_clk (60-200 MHz)\r\n");
      mpu_free_clk_InMhz = 100;
      break;
    case 4:
      InfoPrint ("h2f_free_clk\r\n");
      mpu_free_clk_InMhz = mClkSrc.clk_freq_of_f2h_free/1000/1000;
      break;
    default:
      // We should have covered all supported conditions above
      ASSERT_PLATFORM_INIT(0);
      break;
  }

  mpu_clk_InMhz = PLL0_mpu_base_clk_InMhz / (Cfg->mainpll.mpuclk_cnt + 1);
  l3_main_free_clk_InMhz = PLL0_noc_base_clk_InMhz / (Cfg->mainpll.nocclk_cnt + 1);
  l4_sys_free_clk_InMhz = l3_main_free_clk_InMhz / 4;
  l4_main_clk_InMhz = l3_main_free_clk_InMhz / Div1248[Cfg->mainpll.nocdiv_l4mainclk];
  l4_mp_clk_InMhz = l3_main_free_clk_InMhz / Div1248[Cfg->mainpll.nocdiv_l4mpclk];
  l4_sp_clk_InMhz = l3_main_free_clk_InMhz / Div1248[Cfg->mainpll.nocdiv_l4spclk];
  cs_at_clk_InMhz = l3_main_free_clk_InMhz / Div1248[Cfg->mainpll.nocdiv_csatclk];


  InfoPrint ("\t\t mpu_clk               : %d MHz\r\n"
             "\t\t mpu_l2_ram_clk        : %d MHz\r\n"
             "\t\t mpu_periph_clk        : %d MHz\r\n",
             mpu_clk_InMhz,
             mpu_clk_InMhz/2,
             mpu_clk_InMhz/4);

  InfoPrint ("\t Interconnect Clock Group : \r\n"
             "\t\t noc_free_clk clock source is \r\n"
             "\t\t l3_main_free_clk      : %d MHz\r\n"
             "\t\t l4_sys_free_clk       : %d MHz\r\n"
             "\t\t l4_main_clk           : %d MHz\r\n"
             "\t\t l4_mp_clk             : %d MHz\r\n"
             "\t\t l4_sp_clk             : %d MHz\r\n"
             "\t\t cs_at_clk             : %d MHz\r\n"
             "\t\t cs_timer_clk          : %d MHz\r\n"
             "\t\t cs_pdbg_clk           : %d MHz\r\n"
             "\t\t cs_trace_clk          : %d MHz\r\n",
             l3_main_free_clk_InMhz,
             l4_sys_free_clk_InMhz,
             l4_main_clk_InMhz,
             l4_mp_clk_InMhz,
             l4_sp_clk_InMhz,
             cs_at_clk_InMhz,
             cs_at_clk_InMhz,
             cs_at_clk_InMhz / Div14[Cfg->mainpll.nocdiv_cspdbgclk],
             cs_at_clk_InMhz/Div1248[Cfg->mainpll.nocdiv_cstraceclk]);

  InfoPrint ("\t HPS Managerial Blocks : \r\n"
             "\t\t Clock Manager         - l4_sys_free_clk    : %d MHz\r\n"
             "\t\t FPGA Manager          - l4_sys_free_clk    : %d MHz\r\n"
             "\t\t Reset Manager         - l4_sys_free_clk    : %d MHz\r\n"
             "\t\t System Manager        - l4_sys_free_clk    : %d MHz\r\n"
             "\t\t Security Manager      - cb_intosc_hs_clk   : 120-400 MHz\r\n",
             l4_sys_free_clk_InMhz,
             l4_sys_free_clk_InMhz,
             l4_sys_free_clk_InMhz,
             l4_sys_free_clk_InMhz);

  InfoPrint ("\t Memory related Blocks : \r\n"
             "\t\t Boot ROM              - l3_main_free_clk   : %d MHz\r\n"
             "\t\t On Chip RAM           - l3_main_free_clk   : %d MHz\r\n"
             "\t\t DMA Controller        - l4_main_clk        : %d MHz\r\n"
             "\t\t SDRAM L3 Interconnect - hmc_free_clk       : HMC's clock\r\n"
             "\t\t FPGA-to-SDRAM         - f2h_sdram_clk[2:0] : DDR's clock\r\n",
             l3_main_free_clk_InMhz,
             l3_main_free_clk_InMhz,
             l4_main_clk_InMhz);

  InfoPrint ("\t Bridges Blocks : \r\n"
             "\t\t FPGA-to-HPS           - fpga2hps_clk \r\n"
             "\t\t HPS-to-FPGA           - hps2fpga_clk \r\n"
             "\t\t HPS-to-FPGA LW        - lwh2fpga_clk \r\n");

  InfoPrint ("\t Timer Peripherals : \r\n"
             "\t\t OSC Timer 0/1         - l4_sys_free_clk    : %d MHz\r\n"
             "\t\t SP Timer 0/1          - l4_sp_clk          : %d MHz\r\n"
             "\t\t Watchdog Timer 0/1    - l4_sys_free_clk    : %d MHz\r\n",
             l4_sys_free_clk_InMhz,
             l4_sp_clk_InMhz,
             l4_sys_free_clk_InMhz);

  InfoPrint ("\t Interface Peripherals : \r\n"
             "\t\t Ethernet MAC 0/1/2    - l4_mp_clk          : %d MHz\r\n"
             "\t\t USB 2.0 On-The-Go 0/1 - l4_mp_clk          : %d MHz\r\n"
             "\t\t I2C Controllers 0-4   - l4_sp_clk          : %d MHz\r\n"
             "\t\t UART Controllers 0/1  - l4_sp_clk          : %d MHz\r\n"
             "\t\t SPI Master 0/1        - l4_main_clk        : %d MHz\r\n"
             "\t\t SPI Slave 0/1         - l4_main_clk        : %d MHz\r\n"
             "\t\t GPIO Controllers 0-2  - l4_sp_clk          : %d MHz\r\n",
             l4_mp_clk_InMhz,
             l4_mp_clk_InMhz,
             l4_sp_clk_InMhz,
             l4_sp_clk_InMhz,
             l4_main_clk_InMhz,
             l4_main_clk_InMhz,
             l4_sp_clk_InMhz);

  InfoPrint ("\t NAND Controller : \r\n"
             "\t\t NAND Input Clock      - l4_mp_clk          : %d MHz\r\n"
             "\t\t NAND Output Clock     - nand_clk           : %d MHz\r\n"
             "\t\t NAND Output Clock     - nand_x_clk         : %d MHz\r\n",
             l4_mp_clk_InMhz,
             l4_mp_clk_InMhz / 4,
             l4_mp_clk_InMhz);

  InfoPrint ("\t Quad SPI Controller : \r\n"
             "\t\t QSPI Input Clock      - l4_mp_clk          : %d MHz\r\n"
             "\t\t QSPI Input Clock      - l4_main_clk        : %d MHz\r\n"
             "\t\t QSPI Output Clock     - sclk_out           : l4_main_clk / bauddiv\r\n",
              l4_mp_clk_InMhz,
              l4_main_clk_InMhz);

  InfoPrint ("\t SD/MMC Controller : \r\n"
             "\t\t SD/MMC Input Clock    - l4_mp_clk          : %d MHz\r\n"
             "\t\t SD/MMC Input Clock    - sdmmc_clk          : %d MHz\r\n"
             "\t\t SD/MMC CIU Clock      - cclk_in            : %d MHz\r\n"
             "\t\t SD/MMC Card Clock     - cclk_out           : cclk_in / (clk_divider0*2)\r\n",
             l4_mp_clk_InMhz,
             sdmmc_clk_InMhz,
             sdmmc_clk_InMhz / 4);

}




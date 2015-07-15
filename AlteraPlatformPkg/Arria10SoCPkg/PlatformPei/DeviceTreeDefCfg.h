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

#ifndef __DEVICE_TREE_DEF_CFG_H__
#define __DEVICE_TREE_DEF_CFG_H__

#include "AlteraSdMmcPei/AlteraSdMmcPei.h"

typedef struct {
  CONST CHAR8 *PropName;
} PROPERTY_NAME ;

// ------------------------------------------------------------------
// RBF Filename
// ------------------------------------------------------------------
#define MAX_NumOfRbfFileParts 2
typedef struct {
  UINT32      NumOfRbfFileParts;
  CHAR8*      RBF_FileName[MAX_NumOfRbfFileParts];
  FAT32_FILE  FileHandler[MAX_NumOfRbfFileParts];
} RBF_FILE_CONFIG;

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
  UINT32  vco0_psrc;
  UINT32  vco1_denom;
  UINT32  vco1_numer;
  UINT32  mpuclk_cnt;
  UINT32  mpuclk_src;
  UINT32  nocclk_cnt;
  UINT32  nocclk_src;
  UINT32  cntr2clk_cnt;
  UINT32  cntr3clk_cnt;
  UINT32  cntr4clk_cnt;
  UINT32  cntr5clk_cnt;
  UINT32  cntr6clk_cnt;
  UINT32  cntr7clk_cnt;
  UINT32  cntr7clk_src;
  UINT32  cntr8clk_cnt;
  UINT32  cntr9clk_cnt;
  UINT32  cntr9clk_src;
  UINT32  cntr15clk_cnt;
  UINT32  nocdiv_l4mainclk;
  UINT32  nocdiv_l4mpclk;
  UINT32  nocdiv_l4spclk;
  UINT32  nocdiv_csatclk;
  UINT32  nocdiv_cstraceclk;
  UINT32  nocdiv_cspdbgclk;
} CLOCK_MANAGER_MAIN_PLL_CONFIG;

// ------------------------------------------------------------------
// clock_manager.perpll
// ------------------------------------------------------------------
typedef struct {
  UINT32  vco0_psrc;
  UINT32  vco1_denom;
  UINT32  vco1_numer;
  UINT32  cntr2clk_cnt;
  UINT32  cntr2clk_src;
  UINT32  cntr3clk_cnt;
  UINT32  cntr3clk_src;
  UINT32  cntr4clk_cnt;
  UINT32  cntr4clk_src;
  UINT32  cntr5clk_cnt;
  UINT32  cntr5clk_src;
  UINT32  cntr6clk_cnt;
  UINT32  cntr6clk_src;
  UINT32  cntr7clk_cnt;
  UINT32  cntr8clk_cnt;
  UINT32  cntr8clk_src;
  UINT32  cntr9clk_cnt;
  UINT32  emacctl_emac0sel;
  UINT32  emacctl_emac1sel;
  UINT32  emacctl_emac2sel;
  UINT32  gpiodiv_gpiodbclk;
} CLOCK_MANAGER_PER_PLL_CONFIG;

// ------------------------------------------------------------------
// clock_manager.alteragrp
// ------------------------------------------------------------------
typedef struct {
  UINT32  nocclk;
} CLOCK_MANAGER_ALTERA_GRP_CONFIG;

// ------------------------------------------------------------------
// clock_manager
// ------------------------------------------------------------------

typedef struct {
  CLOCK_MANAGER_MAIN_PLL_CONFIG    mainpll;
  CLOCK_MANAGER_PER_PLL_CONFIG     perpll;
  CLOCK_MANAGER_ALTERA_GRP_CONFIG  alteragrp;
} CLOCK_MANAGER_CONFIG;

// ------------------------------------------------------------------
// firewall
// ------------------------------------------------------------------

typedef struct {
  UINT32  base;
  UINT32  limit;
  UINT32  enable;
} FIREWALL_PROP ;

typedef struct {
  FIREWALL_PROP  mpu0;
  FIREWALL_PROP  mpu1;
  FIREWALL_PROP  mpu2;
  FIREWALL_PROP  mpu3;
  FIREWALL_PROP  l3_0;
  FIREWALL_PROP  l3_1;
  FIREWALL_PROP  l3_2;
  FIREWALL_PROP  l3_3;
  FIREWALL_PROP  l3_4;
  FIREWALL_PROP  l3_5;
  FIREWALL_PROP  l3_6;
  FIREWALL_PROP  l3_7;
  FIREWALL_PROP  fpga2sdram0_0;
  FIREWALL_PROP  fpga2sdram0_1;
  FIREWALL_PROP  fpga2sdram0_2;
  FIREWALL_PROP  fpga2sdram0_3;
  FIREWALL_PROP  fpga2sdram1_0;
  FIREWALL_PROP  fpga2sdram1_1;
  FIREWALL_PROP  fpga2sdram1_2;
  FIREWALL_PROP  fpga2sdram1_3;
  FIREWALL_PROP  fpga2sdram2_0;
  FIREWALL_PROP  fpga2sdram2_1;
  FIREWALL_PROP  fpga2sdram2_2;
  FIREWALL_PROP  fpga2sdram2_3;
} FIREWALL_CONFIG;

#endif

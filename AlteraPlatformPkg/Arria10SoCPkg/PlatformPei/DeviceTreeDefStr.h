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

#ifndef __DEVICE_TREE_DEF_STR_H__
#define __DEVICE_TREE_DEF_STR_H__

// ------------------------------------------------------------------
// pinmux
// ------------------------------------------------------------------

#define COMP_pinmux "pinctrl-single"
#define PROP_pinmux_baseaddress "reg"
#define PROP_pinmux_offsetandvalue "pinctrl-single,pins"

// ------------------------------------------------------------------
// RBF Filename
// ------------------------------------------------------------------
#define NODE_chosen   "chosen"
#define NODE_cff_file "cff-file"
#define NODE_ext_fpga_config "external-fpga-config"
#define NODE_cff_offset "cff-offset"

// ------------------------------------------------------------------
// clock source
// ------------------------------------------------------------------
#define NODE_clocks           "clocks"

#define NODE_clock_src_eosc1  "altera_arria10_hps_eosc1"
#define NODE_clock_src_f2h    "altera_arria10_hps_f2h_free"
#define NODE_clock_src_intosc "altera_arria10_hps_cb_intosc_ls"

#define PROP_clock_frequency  "clock-frequency"

// ------------------------------------------------------------------
// clock_manager
// ------------------------------------------------------------------
#define COMP_clock_manager  "altr,socfpga-a10-clk-init"

// ------------------------------------------------------------------
// clock_manager.mainpll
// ------------------------------------------------------------------
#define NODE_mainpll  "mainpll"

PROPERTY_NAME ClockManagerMainPllCfgStr[] = {
  { "vco0-psrc" },
  { "vco1-denom" },
  { "vco1-numer" },
  { "mpuclk-cnt" },
  { "mpuclk-src" },
  { "nocclk-cnt"  },
  { "nocclk-src" },
  { "cntr2clk-cnt" },
  { "cntr3clk-cnt" },
  { "cntr4clk-cnt" },
  { "cntr5clk-cnt" },
  { "cntr6clk-cnt" },
  { "cntr7clk-cnt" },
  { "cntr7clk-src" },
  { "cntr8clk-cnt" },
  { "cntr9clk-cnt" },
  { "cntr9clk-src" },
  { "cntr15clk-cnt" },
  { "nocdiv-l4mainclk" },
  { "nocdiv-l4mpclk" },
  { "nocdiv-l4spclk" },
  { "nocdiv-csatclk" },
  { "nocdiv-cstraceclk" },
  { "nocdiv-cspdbgclk"},
};

// ------------------------------------------------------------------
// clock_manager.perpll
// ------------------------------------------------------------------
#define NODE_perpll  "perpll"

PROPERTY_NAME ClockManagerPerPllCfgStr[] = {
  { "vco0-psrc" },
  { "vco1-denom" },
  { "vco1-numer" },
  { "cntr2clk-cnt" },
  { "cntr2clk-src" },
  { "cntr3clk-cnt" },
  { "cntr3clk-src" },
  { "cntr4clk-cnt" },
  { "cntr4clk-src" },
  { "cntr5clk-cnt" },
  { "cntr5clk-src" },
  { "cntr6clk-cnt" },
  { "cntr6clk-src" },
  { "cntr7clk-cnt" },
  { "cntr8clk-cnt" },
  { "cntr8clk-src" },
  { "cntr9clk-cnt" },
  { "emacctl-emac0sel" },
  { "emacctl-emac1sel" },
  { "emacctl-emac2sel"},
  { "gpiodiv-gpiodbclk"},
  };

// ------------------------------------------------------------------
// clock_manager.alteragrp
// ------------------------------------------------------------------
#define NODE_alteragrp  "alteragrp"

PROPERTY_NAME ClockManagerAlteraGrpCfgStr[] = {
  { "nocclk" },
};

// ------------------------------------------------------------------
// firewall
// ------------------------------------------------------------------
#define COMP_firewall  "altr,socfpga-a10-noc"
#define NODE_firewall  "firewall"

PROPERTY_NAME FirewallCfgStr[] = {
  { "mpu0" },
  { "mpu1" },
  { "mpu2" },
  { "mpu3" },
  { "l3-0" },
  { "l3-1" },
  { "l3-2" },
  { "l3-3" },
  { "l3-4" },
  { "l3-5" },
  { "l3-6" },
  { "l3-7" },
  { "fpga2sdram0-0" },
  { "fpga2sdram0-1" },
  { "fpga2sdram0-2" },
  { "fpga2sdram0-3" },
  { "fpga2sdram1-0" },
  { "fpga2sdram1-1" },
  { "fpga2sdram1-2" },
  { "fpga2sdram1-3" },
  { "fpga2sdram2-0" },
  { "fpga2sdram2-1" },
  { "fpga2sdram2-2" },
  { "fpga2sdram2-3" },
  };

#endif

/** @file
  Header defining Altera HPS constants

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

#ifndef __ALTERA_HPS_SOCAL_H__
#define __ALTERA_HPS_SOCAL_H__

#include <Base.h>

#include "Socal_Base.h"

#include "Socal/alt_clkmgr.h"
#include "Socal/alt_clkmgr_altera.h"
#include "Socal/alt_clkmgr_mainpll.h"
#include "Socal/alt_clkmgr_perpll.h"
#include "Socal/alt_noc_ccu_ios_ccu_ios_main_qos.h"
#include "Socal/alt_noc_ccu_ios_cs_obs_at_main_atbendpt.h"
#include "Socal/alt_noc_ccu_ios_dma_tbu_m_main_qos.h"
#include "Socal/alt_noc_ccu_ios_emac_tbu_m_main_qos.h"
#include "Socal/alt_noc_ccu_ios_io_tbu_m_main_qos.h"
#include "Socal/alt_noc_ccu_ios_prb_iom_main_prb.h"
#include "Socal/alt_noc_ccu_ios_sdm_tbu_m_main_qos.h"
#include "Socal/alt_noc_ccu_emac_main_prb.h"
#include "Socal/alt_noc_ccu_emac_tbu_transtatfilt.h"
#include "Socal/alt_noc_ccu_h2f_main_prb.h"
#include "Socal/alt_noc_ccu_l4_link_rate_adptr.h"
#include "Socal/alt_noc_ccu_main_prb.h"
#include "Socal/alt_noc_ccu_prb_emac_tbu_transtatprof.h"
#include "Socal/alt_noc_fw_h2f_scr.h"
#include "Socal/alt_noc_fw_l4_per_scr.h"
#include "Socal/alt_noc_fw_l4_sys_scr.h"
#include "Socal/alt_noc_fw_lwh2f_scr.h"
#include "Socal/alt_noc_fw_mmap_priv.h"
#include "Socal/alt_noc_fw_tcu_scr.h"
#include "Socal/alt_soc_noc_fw_ddr_f2sdr_scr.h"
#include "Socal/alt_soc_noc_fw_ddr_scr.h"
#include "Socal/alt_soc_noc_fw_mpfe_csr.h"
#include "Socal/alt_psi.h"
#include "Socal/alt_smmu_secure.h"
#include "Socal/alt_mpfe.h"
#include "Socal/alt_gic.h"
#include "Socal/alt_ecc.h"
#include "Socal/alt_emac.h"
#include "Socal/alt_gpio.h"
#include "Socal/alt_i2c.h"
#include "Socal/alt_wdt.h"
#include "Socal/alt_nand.h"
#include "Socal/alt_qspi.h"
#include "Socal/alt_pinmux.h"
#include "Socal/alt_rstmgr.h"
#include "Socal/alt_sdmmc.h"
#include "Socal/alt_spim.h"
#include "Socal/alt_spis.h"
#include "Socal/alt_sysmgr.h"
#include "Socal/alt_tmr.h"
#include "Socal/alt_uart.h"
#include "Socal/alt_usb.h"
#include "Socal/alt_dma.h"
#include "Socal/alt_doorbell_in.h"
#include "Socal/alt_doorbell_out.h"
#include "Socal/alt_mbox.h"
#include "Socal/alt_mbox_stream.h"
#include "Socal/alt_ccu_noc.h"
#include "Socal/hps.h"
#include "Socal/socal.h"

/* The SILICONID1 register for different silicon. */
#define ALT_SYSMGR_SILICONID1_ES1       0x00010001
#define ALT_SYSMGR_SILICONID1_ES2       0x00010002

// The base address byte offset for the start of the ARM'S MPU L2 component.
// Ref: ARM's CoreLink L2C-310 Technical Reference Manual 3.3.12 Address filtering
#define ARM_MPUL2_OFST                       0xfffff000
// The address filtering start and end registers in the L2 cache controller define the SDRAM window boundaries.
// When enabled, addresses within the boundaries route to the SDRAM master.
// and addresses outside the boundaries route to the system interconnect master.
// When disabled, SDRAM are not visible to MPU
#define ARM_MPUL2_ADDR_FILTERING_START_OFST  0xC00
#define ARM_MPUL2_ADDR_FILTERING_END_OFST    0xC04
#define ARM_MPUL2_ADDR_FILTERING_START_EN_GET(value) (((value) & 0x00000001) >> 0)
#define ARM_MPUL2_ADDR_FILTERING_START_EN_SET(value) (((value) << 0) & 0x00000001)
#define ARM_MPUL2_ADDR_FILTERING_DISABLED    0x00
#define ARM_MPUL2_ADDR_FILTERING_ENABLED     0x01
// The boundaries are megabyte-aligned.
#define ARM_MPUL2_ADDR_FILTERING_ADDR_GET(value) ((value) & 0xfff00000)
#define ARM_MPUL2_ADDR_FILTERING_ADDR_SET(value) ((value) & 0xfff00000)



#endif


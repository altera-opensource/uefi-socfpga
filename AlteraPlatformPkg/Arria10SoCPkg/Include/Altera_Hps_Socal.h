/** @file
  Header defining Altera HPS constants

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

#ifndef __ALTERA_HPS_SOCAL_H__
#define __ALTERA_HPS_SOCAL_H__

#include <Base.h>

#include "Socal_Base.h"

#include "Socal/alt_clkmgr.h"
#include "Socal/alt_ecc_dmac.h"
#include "Socal/alt_ecc_emac0_rx_ecc.h"
#include "Socal/alt_ecc_emac0_tx_ecc.h"
#include "Socal/alt_ecc_emac1_rx_ecc.h"
#include "Socal/alt_ecc_emac1_tx_ecc.h"
#include "Socal/alt_ecc_emac2_rx_ecc.h"
#include "Socal/alt_ecc_emac2_tx_ecc.h"
#include "Socal/alt_ecc_hmc_ocp.h"
#include "Socal/alt_ecc_nand.h"
#include "Socal/alt_ecc_nandr.h"
#include "Socal/alt_ecc_nandw.h"
#include "Socal/alt_ecc_ocram_ecc.h"
#include "Socal/alt_ecc_otg0_ecc.h"
#include "Socal/alt_ecc_otg1_ecc.h"
#include "Socal/alt_ecc_qspi.h"
#include "Socal/alt_ecc_sdmmc.h"
#include "Socal/alt_emac.h"
#include "Socal/alt_fpgamgr.h"
#include "Socal/alt_fpgamgrdata.h"
#include "Socal/alt_gpio.h"
#include "Socal/alt_i2c.h"
#include "Socal/alt_io48_hmc_mmr.h"
#include "Socal/alt_l4wd.h"
#include "Socal/alt_nand.h"
#include "Socal/alt_noc_fw_ddr_l3_scr.h"
#include "Socal/alt_noc_fw_ddr_mpu_f2sdr_ddr_scr.h"
#include "Socal/alt_noc_fw_h2f_scr.h"
#include "Socal/alt_noc_fw_l4_per_scr.h"
#include "Socal/alt_noc_fw_l4_sys_scr.h"
#include "Socal/alt_noc_fw_ocram_scr.h"
#include "Socal/alt_noc_l4_priv_flt.h"
#include "Socal/alt_noc_mpu_acp_rate_ad_main_rate.h"
#include "Socal/alt_noc_mpu_cs.h"
#include "Socal/alt_noc_mpu_ddr.h"
#include "Socal/alt_noc_mpu_dma_m0_qos.h"
#include "Socal/alt_noc_mpu_emac0.h"
#include "Socal/alt_noc_mpu_emac1.h"
#include "Socal/alt_noc_mpu_emac2.h"
#include "Socal/alt_noc_mpu_f2h_axi128_qos.h"
#include "Socal/alt_noc_mpu_f2h_axi32_qos.h"
#include "Socal/alt_noc_mpu_f2h_axi64_qos.h"
#include "Socal/alt_noc_mpu_f2h_rate_ad_main_rate.h"
#include "Socal/alt_noc_mpu_f2sdr0_axi128_qos.h"
#include "Socal/alt_noc_mpu_f2sdr0_axi32_qos.h"
#include "Socal/alt_noc_mpu_f2sdr0_axi64_qos.h"
#include "Socal/alt_noc_mpu_f2sdr1_axi32_qos.h"
#include "Socal/alt_noc_mpu_f2sdr1_axi64_qos.h"
#include "Socal/alt_noc_mpu_f2sdr2_axi128_qos.h"
#include "Socal/alt_noc_mpu_f2sdr2_axi32_qos.h"
#include "Socal/alt_noc_mpu_f2sdr2_axi64_qos.h"
#include "Socal/alt_noc_mpu_l3toh2fresp_main_rate.h"
#include "Socal/alt_noc_mpu_l4.h"
#include "Socal/alt_noc_mpu_m0_main_qos.h"
#include "Socal/alt_noc_mpu_m0_rate_adresp_main_rate.h"
#include "Socal/alt_noc_mpu_m1_main_qos.h"
#include "Socal/alt_noc_mpu_m1toddrresp_main_rate.h"
#include "Socal/alt_noc_mpu_nand_m_main_qos.h"
#include "Socal/alt_noc_mpu_prb.h"
#include "Socal/alt_noc_mpu_sdmmc_m_main_qos.h"
#include "Socal/alt_noc_mpu_usb0_m_main_qos.h"
#include "Socal/alt_noc_mpu_usb1_m_main_qos.h"
#include "Socal/alt_pinmux.h"
#include "Socal/alt_qspi.h"
#include "Socal/alt_rstmgr.h"
#include "Socal/alt_sdmmc.h"
#include "Socal/alt_spim.h"
#include "Socal/alt_spis.h"
#include "Socal/alt_sysmgr.h"
#include "Socal/alt_tmr.h"
#include "Socal/alt_uart.h"
#include "Socal/alt_usb.h"
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

//
// Definition of io48_hmc_mmr.niosreserve(N) registers (case:270660)
//
// EMIF IP Info:
// niosreserve0[7:0] = DRAM interface IO Size in number of bits.
#define ALT_IO48_HMC_MMR_NIOSRESERVE0_IOSIZE_GET(value) (((value) & 0x000000ff) >> 0)
// niosreserve0[8] = ‘1’ if user-mode OCT workaround is present in the design
#define ALT_IO48_HMC_MMR_NIOSRESERVE0_BIT8_GET(value) (((value) & 0x00000100) >> 8)
// niosreserve0[9] = ‘1’ if warm reset support is compiled into the EMIF calibration code
#define ALT_IO48_HMC_MMR_NIOSRESERVE0_BIT9_GET(value) (((value) & 0x00000200) >> 9)
// niosreserve0[10] = ‘1’ if warm reset support is enabled during generation in the EMIF calibration code
#define ALT_IO48_HMC_MMR_NIOSRESERVE0_BIT10_GET(value) (((value) & 0x00000400) >> 10)
// EMIF IP ACDS version:
// niosreserve1[2:0] = encodes whether this is a special variant
//                       0 = not a special variant,
//                       1 = FAE beta,
//                       2 = Customer beta
//                       3 = Early access program,
//                       4-6 reserved
//                       7 = unrecongized variant.
#define ALT_IO48_HMC_MMR_NIOSRESERVE1_ACDS_VARIANT_GET(value) (((value) & 0x00000007) >> 0)
// niosreserve1[5:3] = encodes the service pack number (e.g. 2 if 15.1sp2)
#define ALT_IO48_HMC_MMR_NIOSRESERVE1_ACDS_SP_VER_GET(value) (((value) & 0x00000038) >> 3)
// niosreserve1[9:6] = encodes the major release number (e.g. 1 if 15.1sp2)
#define ALT_IO48_HMC_MMR_NIOSRESERVE1_ACDS_MAJOR_VER_GET(value) (((value) & 0x000003c0) >> 6)
// niosreserve1[14:10] = encodes the major release number (e.g. 15 if 15.1sp2)
#define ALT_IO48_HMC_MMR_NIOSRESERVE1_ACDS_MINOR_VER_GET(value) (((value) & 0x00007c00) >> 10)
// niosreserve1[15] = reserved

//
// Security Manager
// alt_sec_mgr.h is not release in SocEDS
// here only list limited registers that are useful for debug purposes
//
#define ALT_SEC_MGR_OFST                                                   0xffd02000
#define ALT_SEC_MGR_CURSECSTATE_OFST                                       0x5c
#define ALT_SEC_MGR_CURSECSTATE_STATE_FPGA_BOOT_GET(value)                 (((value) & 0x00000040) >> 6)
#define ALT_SEC_MGR_CURSECSTATE_STATE_FPGA_BOOT_E_FPGA_BOOT_DIS            0x0
#define ALT_SEC_MGR_CURSECSTATE_STATE_FPGA_BOOT_E_FPGA_BOOT_EN             0x1
#define ALT_SEC_MGR_CURSECSTATE_STATE_CLK_GET(value)                       (((value) & 0x00000080) >> 7)
#define ALT_SEC_MGR_CURSECSTATE_STATE_CLK_E_EOSC1_CLK                      0x0
#define ALT_SEC_MGR_CURSECSTATE_STATE_CLK_E_IOSC_CLK                       0x1
#define ALT_SEC_MGR_FUSESEC_OFST                                           0x8
#define ALT_SEC_MGR_FUSESEC2_OFST                                          0xc
#define ALT_SEC_MGR_FPGA_FUSESEC_OFST                                      0x10
#define ALT_SEC_MGR_CURSECSTATE_STATE_AUTHEN_EN_GET(value)                 (((value) & 0x00000001) >> 0)
#define ALT_SEC_MGR_CURSECSTATE_STATE_AUTHEN_EN_E_AUTH_DIS                 0x0
#define ALT_SEC_MGR_CURSECSTATE_STATE_AUTHEN_EN_E_AUTH_EN                  0x1
#define ALT_SEC_MGR_CURSECSTATE_STATE_AES_EN_GET(value)                    (((value) & 0x00000020) >> 5)
#define ALT_SEC_MGR_CURSECSTATE_STATE_AES_EN_E_AES_DIS                     0x0
#define ALT_SEC_MGR_CURSECSTATE_STATE_AES_EN_E_AES_EN                      0x1
#define ALT_SEC_MGR_CURSECSTATE_STATE_OC_BOOT_GET(value)                   (((value) & 0x00000100) >> 8)
#define ALT_SEC_MGR_CURSECSTATE_STATE_OC_BOOT_E_OCRAM_BOOT_EN              0x0
#define ALT_SEC_MGR_CURSECSTATE_STATE_OC_BOOT_E_OCRAM_BOOT_DIS             0x1
#define ALT_SEC_MGR_CURSECSTATE_STATE_CLR_RAM_WARM_GET(value)              (((value) & 0x00000200) >> 9)
#define ALT_SEC_MGR_CURSECSTATE_STATE_CLR_RAM_WARM_E_CLR_RAM_WARM_DIS      0x0
#define ALT_SEC_MGR_CURSECSTATE_STATE_CLR_RAM_WARM_E_CLR_RAM_WARM_EN       0x1
#define ALT_SEC_MGR_CURSECSTATE_STATE_CLR_RAM_COLD_GET(value)              (((value) & 0x00000400) >> 10)
#define ALT_SEC_MGR_CURSECSTATE_STATE_CLR_RAM_COLD_E_CLR_RAM_COLD_DIS      0x0
#define ALT_SEC_MGR_CURSECSTATE_STATE_CLR_RAM_COLD_E_CLR_RAM_COLD_EN       0x1
#define ALT_SEC_MGR_CURSECSTATE_STATE_CLR_RAM_ORDER_GET(value)             (((value) & 0x00000800) >> 11)
#define ALT_SEC_MGR_CURSECSTATE_STATE_CLR_RAM_ORDER_E_CLR_RAM_ORDER_PAR    0x0
#define ALT_SEC_MGR_CURSECSTATE_STATE_CLR_RAM_ORDER_E_CLR_RAM_ORDER_SER    0x1
#define ALT_SEC_MGR_SWOPTSET_OFST                                          0x68
#define ALT_SEC_MGR_SEC_SWROMCODE_OFST                                     0x78
#define ALT_SEC_MGR_SEC_FPGACHK_OFST                                       0x8c
#define ALT_SEC_MGR_SEC_HPSCHK_OFST                                        0x90

//
// Clock Manager
//
#define ALT_CLKMGR_MPUCLK_OFST        0x0
#define ALT_CLKMGR_MPUCLK_PERICNT_GET(value) (((value) & 0x07ff0000) >> 16)
#define ALT_CLKMGR_MPUCLK_MAINCNT_GET(value) (((value) & 0x000007ff) >> 0)
#define ISW_HANDOFF_SLOT8_L4_SP_CLK_IN_MHZ_OFST  (sizeof(UINT32) * 7)

#endif


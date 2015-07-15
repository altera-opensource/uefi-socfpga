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
#include "DeviceTree.h"
#include "Firewall.h"

#if (FixedPcdGet32(PcdDebugMsg_Firewall) == 0)
  #define ProgressPrint(FormatString, ...)    /* do nothing */
  #define InfoPrint(FormatString, ...)        /* do nothing */
#else
  #define ProgressPrint SerialPortPrint
  #define InfoPrint     SerialPortPrint
#endif

//
// Functions
//

EFI_STATUS
EFIAPI
InitFirewall (
  IN  CONST VOID*  Fdt
  )
{
  EFI_STATUS      Status;

  FIREWALL_CONFIG        Cfg;

  ProgressPrint ("Initializing Firewall\r\n");

  // Get Device Tree Settings
  GetFirewallCfg (Fdt, &Cfg);

  // Configure Firewall Base & Limit definition
  Status = ConfigureDdrMpuFpga2sdramFirewallRegisters (&Cfg);
  if (Status != EFI_SUCCESS) {
    return Status;
  }

  Status = ConfigureDdrL3FirewallRegisters (&Cfg);
  if (Status != EFI_SUCCESS) {
    return Status;
  }

  return Status;
}

EFI_STATUS
EFIAPI
ConfigureDdrMpuFpga2sdramFirewallRegisters (
  IN  FIREWALL_CONFIG*   Cfg
  )
{
  EFI_STATUS             Status;
  UINT32                 EnableRegion;

  Status = EFI_SUCCESS;
  EnableRegion = 0;

  ProgressPrint ("\t Init DDR MPU F2SDR Firewall.\r\n");

  // Step 1 - Disable all regions firewall
  // Use the enable_clear Write one to clear register
  MmioWrite32 (ALT_NOC_FW_DDR_MPU_F2SDR_DDR_SCR_OFST +
               ALT_NOC_FW_DDR_MPU_F2SDR_DDR_SCR_EN_CLR_OFST,
               ALT_NOC_FW_DDR_MPU_F2SDR_DDR_SCR_EN_CLR_MPUREG0EN_SET_MSK |
               ALT_NOC_FW_DDR_MPU_F2SDR_DDR_SCR_EN_CLR_MPUREG1EN_SET_MSK |
               ALT_NOC_FW_DDR_MPU_F2SDR_DDR_SCR_EN_CLR_MPUREG2EN_SET_MSK |
               ALT_NOC_FW_DDR_MPU_F2SDR_DDR_SCR_EN_CLR_MPUREG3EN_SET_MSK |
               ALT_NOC_FW_DDR_MPU_F2SDR_DDR_SCR_EN_CLR_F2SDR0REG0EN_SET_MSK |
               ALT_NOC_FW_DDR_MPU_F2SDR_DDR_SCR_EN_CLR_F2SDR0REG1EN_SET_MSK |
               ALT_NOC_FW_DDR_MPU_F2SDR_DDR_SCR_EN_CLR_F2SDR0REG2EN_SET_MSK |
               ALT_NOC_FW_DDR_MPU_F2SDR_DDR_SCR_EN_CLR_F2SDR0REG3EN_SET_MSK |
               ALT_NOC_FW_DDR_MPU_F2SDR_DDR_SCR_EN_CLR_F2SDR1REG0EN_SET_MSK |
               ALT_NOC_FW_DDR_MPU_F2SDR_DDR_SCR_EN_CLR_F2SDR1REG1EN_SET_MSK |
               ALT_NOC_FW_DDR_MPU_F2SDR_DDR_SCR_EN_CLR_F2SDR1REG2EN_SET_MSK |
               ALT_NOC_FW_DDR_MPU_F2SDR_DDR_SCR_EN_CLR_F2SDR1REG3EN_SET_MSK |
               ALT_NOC_FW_DDR_MPU_F2SDR_DDR_SCR_EN_CLR_F2SDR2REG0EN_SET_MSK |
               ALT_NOC_FW_DDR_MPU_F2SDR_DDR_SCR_EN_CLR_F2SDR2REG1EN_SET_MSK |
               ALT_NOC_FW_DDR_MPU_F2SDR_DDR_SCR_EN_CLR_F2SDR2REG2EN_SET_MSK |
               ALT_NOC_FW_DDR_MPU_F2SDR_DDR_SCR_EN_CLR_F2SDR2REG3EN_SET_MSK);

  // Step 2 - Set Base and Limit definition based on information from DTB file.

  // MPU 0 - 3
  if (Cfg->mpu0.enable == 1) {
    MmioWrite32 (ALT_NOC_FW_DDR_MPU_F2SDR_DDR_SCR_OFST +
                 ALT_NOC_FW_DDR_MPU_F2SDR_DDR_SCR_MPUREG0ADDR_OFST,
                 ALT_NOC_FW_DDR_MPU_F2SDR_DDR_SCR_MPUREG0ADDR_BASE_SET(
                                                 Cfg->mpu0.base) |
                 ALT_NOC_FW_DDR_MPU_F2SDR_DDR_SCR_MPUREG0ADDR_LIMIT_SET(
                                                 Cfg->mpu0.limit));
    EnableRegion |=
          ALT_NOC_FW_DDR_MPU_F2SDR_DDR_SCR_EN_SET_MPUREG0EN_SET_MSK;
  }
  if (Cfg->mpu1.enable == 1) {
    MmioWrite32 (ALT_NOC_FW_DDR_MPU_F2SDR_DDR_SCR_OFST +
                 ALT_NOC_FW_DDR_MPU_F2SDR_DDR_SCR_MPUREG1ADDR_OFST,
                 ALT_NOC_FW_DDR_MPU_F2SDR_DDR_SCR_MPUREG1ADDR_BASE_SET(
                                                 Cfg->mpu1.base) |
                 ALT_NOC_FW_DDR_MPU_F2SDR_DDR_SCR_MPUREG1ADDR_LIMIT_SET(
                                                 Cfg->mpu1.limit));
    EnableRegion |=
          ALT_NOC_FW_DDR_MPU_F2SDR_DDR_SCR_EN_SET_MPUREG1EN_SET_MSK;
  }
  if (Cfg->mpu2.enable == 1) {
    MmioWrite32 (ALT_NOC_FW_DDR_MPU_F2SDR_DDR_SCR_OFST +
                 ALT_NOC_FW_DDR_MPU_F2SDR_DDR_SCR_MPUREG2ADDR_OFST,
                 ALT_NOC_FW_DDR_MPU_F2SDR_DDR_SCR_MPUREG2ADDR_BASE_SET(
                                                 Cfg->mpu2.base) |
                 ALT_NOC_FW_DDR_MPU_F2SDR_DDR_SCR_MPUREG2ADDR_LIMIT_SET(
                                                 Cfg->mpu2.limit));
    EnableRegion |=
          ALT_NOC_FW_DDR_MPU_F2SDR_DDR_SCR_EN_SET_MPUREG2EN_SET_MSK;
  }
  if (Cfg->mpu3.enable == 1) {
    MmioWrite32 (ALT_NOC_FW_DDR_MPU_F2SDR_DDR_SCR_OFST +
                 ALT_NOC_FW_DDR_MPU_F2SDR_DDR_SCR_MPUREG3ADDR_OFST,
                 ALT_NOC_FW_DDR_MPU_F2SDR_DDR_SCR_MPUREG3ADDR_BASE_SET(
                                                 Cfg->mpu3.base) |
                 ALT_NOC_FW_DDR_MPU_F2SDR_DDR_SCR_MPUREG3ADDR_LIMIT_SET(
                                                 Cfg->mpu3.limit));
    EnableRegion |=
          ALT_NOC_FW_DDR_MPU_F2SDR_DDR_SCR_EN_SET_MPUREG3EN_SET_MSK;
  }

  // FPGA2SDRAM0 Region 0 - 3
  if (Cfg->fpga2sdram0_0.enable == 1) {
    MmioWrite32 (ALT_NOC_FW_DDR_MPU_F2SDR_DDR_SCR_OFST +
                 ALT_NOC_FW_DDR_MPU_F2SDR_DDR_SCR_F2SDR0REG0ADDR_OFST,
                 ALT_NOC_FW_DDR_MPU_F2SDR_DDR_SCR_F2SDR0REG0ADDR_BASE_SET(
                                          Cfg->fpga2sdram0_0.base) |
                 ALT_NOC_FW_DDR_MPU_F2SDR_DDR_SCR_F2SDR0REG0ADDR_LIMIT_SET(
                                          Cfg->fpga2sdram0_0.limit));
    EnableRegion |=
          ALT_NOC_FW_DDR_MPU_F2SDR_DDR_SCR_EN_SET_F2SDR0REG0EN_SET_MSK;
  }
  if (Cfg->fpga2sdram0_1.enable == 1) {
    MmioWrite32 (ALT_NOC_FW_DDR_MPU_F2SDR_DDR_SCR_OFST +
                 ALT_NOC_FW_DDR_MPU_F2SDR_DDR_SCR_F2SDR0REG1ADDR_OFST,
                 ALT_NOC_FW_DDR_MPU_F2SDR_DDR_SCR_F2SDR0REG1ADDR_BASE_SET(
                                          Cfg->fpga2sdram0_1.base) |
                 ALT_NOC_FW_DDR_MPU_F2SDR_DDR_SCR_F2SDR0REG1ADDR_LIMIT_SET(
                                          Cfg->fpga2sdram0_1.limit));
    EnableRegion |=
          ALT_NOC_FW_DDR_MPU_F2SDR_DDR_SCR_EN_SET_F2SDR0REG1EN_SET_MSK;
  }
  if (Cfg->fpga2sdram0_2.enable == 1) {
    MmioWrite32 (ALT_NOC_FW_DDR_MPU_F2SDR_DDR_SCR_OFST +
                 ALT_NOC_FW_DDR_MPU_F2SDR_DDR_SCR_F2SDR0REG2ADDR_OFST,
                 ALT_NOC_FW_DDR_MPU_F2SDR_DDR_SCR_F2SDR0REG2ADDR_BASE_SET(
                                          Cfg->fpga2sdram0_2.base) |
                 ALT_NOC_FW_DDR_MPU_F2SDR_DDR_SCR_F2SDR0REG2ADDR_LIMIT_SET(
                                          Cfg->fpga2sdram0_2.limit));
    EnableRegion |=
          ALT_NOC_FW_DDR_MPU_F2SDR_DDR_SCR_EN_SET_F2SDR0REG2EN_SET_MSK;
  }
  if (Cfg->fpga2sdram0_3.enable == 1) {
    MmioWrite32 (ALT_NOC_FW_DDR_MPU_F2SDR_DDR_SCR_OFST +
                 ALT_NOC_FW_DDR_MPU_F2SDR_DDR_SCR_F2SDR0REG3ADDR_OFST,
                 ALT_NOC_FW_DDR_MPU_F2SDR_DDR_SCR_F2SDR0REG3ADDR_BASE_SET(
                                          Cfg->fpga2sdram0_3.base) |
                 ALT_NOC_FW_DDR_MPU_F2SDR_DDR_SCR_F2SDR0REG3ADDR_LIMIT_SET(
                                          Cfg->fpga2sdram0_3.limit));
    EnableRegion |=
          ALT_NOC_FW_DDR_MPU_F2SDR_DDR_SCR_EN_SET_F2SDR0REG3EN_SET_MSK;
  }

  // FPGA2SDRAM1 Region 0 - 3
  if (Cfg->fpga2sdram1_0.enable == 1) {
    MmioWrite32 (ALT_NOC_FW_DDR_MPU_F2SDR_DDR_SCR_OFST +
                 ALT_NOC_FW_DDR_MPU_F2SDR_DDR_SCR_F2SDR1REG0ADDR_OFST,
                 ALT_NOC_FW_DDR_MPU_F2SDR_DDR_SCR_F2SDR1REG0ADDR_BASE_SET(
                                          Cfg->fpga2sdram1_0.base) |
                 ALT_NOC_FW_DDR_MPU_F2SDR_DDR_SCR_F2SDR1REG0ADDR_LIMIT_SET(
                                          Cfg->fpga2sdram1_0.limit));
    EnableRegion |=
          ALT_NOC_FW_DDR_MPU_F2SDR_DDR_SCR_EN_SET_F2SDR1REG0EN_SET_MSK;
  }
  if (Cfg->fpga2sdram1_1.enable == 1) {
    MmioWrite32 (ALT_NOC_FW_DDR_MPU_F2SDR_DDR_SCR_OFST +
                 ALT_NOC_FW_DDR_MPU_F2SDR_DDR_SCR_F2SDR1REG1ADDR_OFST,
                 ALT_NOC_FW_DDR_MPU_F2SDR_DDR_SCR_F2SDR1REG1ADDR_BASE_SET(
                                          Cfg->fpga2sdram1_1.base) |
                 ALT_NOC_FW_DDR_MPU_F2SDR_DDR_SCR_F2SDR1REG1ADDR_LIMIT_SET(
                                          Cfg->fpga2sdram1_1.limit));
    EnableRegion |=
          ALT_NOC_FW_DDR_MPU_F2SDR_DDR_SCR_EN_SET_F2SDR1REG1EN_SET_MSK;
  }
  if (Cfg->fpga2sdram1_2.enable == 1) {
    MmioWrite32 (ALT_NOC_FW_DDR_MPU_F2SDR_DDR_SCR_OFST +
                 ALT_NOC_FW_DDR_MPU_F2SDR_DDR_SCR_F2SDR1REG2ADDR_OFST,
                 ALT_NOC_FW_DDR_MPU_F2SDR_DDR_SCR_F2SDR1REG2ADDR_BASE_SET(
                                          Cfg->fpga2sdram1_2.base) |
                 ALT_NOC_FW_DDR_MPU_F2SDR_DDR_SCR_F2SDR1REG2ADDR_LIMIT_SET(
                                          Cfg->fpga2sdram1_2.limit));
    EnableRegion |=
          ALT_NOC_FW_DDR_MPU_F2SDR_DDR_SCR_EN_SET_F2SDR1REG2EN_SET_MSK;
  }
  if (Cfg->fpga2sdram1_3.enable == 1) {
    MmioWrite32 (ALT_NOC_FW_DDR_MPU_F2SDR_DDR_SCR_OFST +
                 ALT_NOC_FW_DDR_MPU_F2SDR_DDR_SCR_F2SDR1REG3ADDR_OFST,
                 ALT_NOC_FW_DDR_MPU_F2SDR_DDR_SCR_F2SDR1REG3ADDR_BASE_SET(
                                          Cfg->fpga2sdram1_3.base) |
                 ALT_NOC_FW_DDR_MPU_F2SDR_DDR_SCR_F2SDR1REG3ADDR_LIMIT_SET(
                                          Cfg->fpga2sdram1_3.limit));
    EnableRegion |=
          ALT_NOC_FW_DDR_MPU_F2SDR_DDR_SCR_EN_SET_F2SDR1REG3EN_SET_MSK;
  }

  // FPGA2SDRAM2 Region 0 - 3
  if (Cfg->fpga2sdram2_0.enable == 1) {
    MmioWrite32 (ALT_NOC_FW_DDR_MPU_F2SDR_DDR_SCR_OFST +
                 ALT_NOC_FW_DDR_MPU_F2SDR_DDR_SCR_F2SDR2REG0ADDR_OFST,
                 ALT_NOC_FW_DDR_MPU_F2SDR_DDR_SCR_F2SDR2REG0ADDR_BASE_SET(
                                          Cfg->fpga2sdram2_0.base) |
                 ALT_NOC_FW_DDR_MPU_F2SDR_DDR_SCR_F2SDR2REG0ADDR_LIMIT_SET(
                                          Cfg->fpga2sdram2_0.limit));
    EnableRegion |=
          ALT_NOC_FW_DDR_MPU_F2SDR_DDR_SCR_EN_SET_F2SDR2REG0EN_SET_MSK;
  }
  if (Cfg->fpga2sdram2_1.enable == 1) {
    MmioWrite32 (ALT_NOC_FW_DDR_MPU_F2SDR_DDR_SCR_OFST +
                 ALT_NOC_FW_DDR_MPU_F2SDR_DDR_SCR_F2SDR2REG1ADDR_OFST,
                 ALT_NOC_FW_DDR_MPU_F2SDR_DDR_SCR_F2SDR2REG1ADDR_BASE_SET(
                                          Cfg->fpga2sdram2_1.base) |
                 ALT_NOC_FW_DDR_MPU_F2SDR_DDR_SCR_F2SDR2REG1ADDR_LIMIT_SET(
                                          Cfg->fpga2sdram2_1.limit));
    EnableRegion |=
          ALT_NOC_FW_DDR_MPU_F2SDR_DDR_SCR_EN_SET_F2SDR2REG1EN_SET_MSK;
  }
  if (Cfg->fpga2sdram2_2.enable == 1) {
    MmioWrite32 (ALT_NOC_FW_DDR_MPU_F2SDR_DDR_SCR_OFST +
                 ALT_NOC_FW_DDR_MPU_F2SDR_DDR_SCR_F2SDR2REG2ADDR_OFST,
                 ALT_NOC_FW_DDR_MPU_F2SDR_DDR_SCR_F2SDR2REG2ADDR_BASE_SET(
                                          Cfg->fpga2sdram2_2.base) |
                 ALT_NOC_FW_DDR_MPU_F2SDR_DDR_SCR_F2SDR2REG2ADDR_LIMIT_SET(
                                          Cfg->fpga2sdram2_2.limit));
    EnableRegion |=
          ALT_NOC_FW_DDR_MPU_F2SDR_DDR_SCR_EN_SET_F2SDR2REG2EN_SET_MSK;
  }
  if (Cfg->fpga2sdram2_3.enable == 1) {
    MmioWrite32 (ALT_NOC_FW_DDR_MPU_F2SDR_DDR_SCR_OFST +
                 ALT_NOC_FW_DDR_MPU_F2SDR_DDR_SCR_F2SDR2REG3ADDR_OFST,
                 ALT_NOC_FW_DDR_MPU_F2SDR_DDR_SCR_F2SDR2REG3ADDR_BASE_SET(
                                          Cfg->fpga2sdram2_3.base) |
                 ALT_NOC_FW_DDR_MPU_F2SDR_DDR_SCR_F2SDR2REG3ADDR_LIMIT_SET(
                                          Cfg->fpga2sdram2_3.limit));
    EnableRegion |=
          ALT_NOC_FW_DDR_MPU_F2SDR_DDR_SCR_EN_SET_F2SDR2REG3EN_SET_MSK;
  }

  // Step 3 - Enable firewall regions
  // Use the enable_set Write one to set register
  MmioWrite32 (ALT_NOC_FW_DDR_MPU_F2SDR_DDR_SCR_OFST +
               ALT_NOC_FW_DDR_MPU_F2SDR_DDR_SCR_EN_SET_OFST,
               EnableRegion);

  return Status;
}

EFI_STATUS
EFIAPI
ConfigureDdrL3FirewallRegisters (
  IN  FIREWALL_CONFIG*   Cfg
  )
{
  EFI_STATUS             Status;
  UINT32                 EnableRegion;

  Status = EFI_SUCCESS;
  EnableRegion = 0;

  ProgressPrint ("\t Init DDR L3 Firewall.\r\n");

  // Step 1 - Disable all regions firewall
  // Use the enable_clear Write one to clear register
  MmioWrite32 (ALT_NOC_FW_DDR_L3_SCR_OFST +
               ALT_NOC_FW_DDR_L3_SCR_EN_CLR_OFST,
               ALT_NOC_FW_DDR_L3_SCR_EN_CLR_HPSREG0EN_SET_MSK |
               ALT_NOC_FW_DDR_L3_SCR_EN_CLR_HPSREG1EN_SET_MSK |
               ALT_NOC_FW_DDR_L3_SCR_EN_CLR_HPSREG2EN_SET_MSK |
               ALT_NOC_FW_DDR_L3_SCR_EN_CLR_HPSREG3EN_SET_MSK |
               ALT_NOC_FW_DDR_L3_SCR_EN_CLR_HPSREG4EN_SET_MSK |
               ALT_NOC_FW_DDR_L3_SCR_EN_CLR_HPSREG5EN_SET_MSK |
               ALT_NOC_FW_DDR_L3_SCR_EN_CLR_HPSREG6EN_SET_MSK |
               ALT_NOC_FW_DDR_L3_SCR_EN_CLR_HPSREG7EN_SET_MSK );

  // Step 2 - Set Base and Limit definition based on information from DTB file.

  // HPS 0 - 7
  if (Cfg->l3_0.enable == 1) {
    MmioWrite32 (ALT_NOC_FW_DDR_L3_SCR_OFST +
                 ALT_NOC_FW_DDR_L3_SCR_HPSREG0ADDR_OFST,
                 ALT_NOC_FW_DDR_L3_SCR_HPSREG0ADDR_BASE_SET(
                                     Cfg->l3_0.base) |
                 ALT_NOC_FW_DDR_L3_SCR_HPSREG0ADDR_LIMIT_SET(
                                     Cfg->l3_0.limit));
    EnableRegion |=
          ALT_NOC_FW_DDR_L3_SCR_EN_SET_HPSREG0EN_SET_MSK;
  }
  if (Cfg->l3_1.enable == 1) {
    MmioWrite32 (ALT_NOC_FW_DDR_L3_SCR_OFST +
                 ALT_NOC_FW_DDR_L3_SCR_HPSREG1ADDR_OFST,
                 ALT_NOC_FW_DDR_L3_SCR_HPSREG1ADDR_BASE_SET(
                                     Cfg->l3_1.base) |
                 ALT_NOC_FW_DDR_L3_SCR_HPSREG1ADDR_LIMIT_SET(
                                     Cfg->l3_1.limit));
    EnableRegion |=
          ALT_NOC_FW_DDR_L3_SCR_EN_SET_HPSREG1EN_SET_MSK;
  }
  if (Cfg->l3_2.enable == 1) {
    MmioWrite32 (ALT_NOC_FW_DDR_L3_SCR_OFST +
                 ALT_NOC_FW_DDR_L3_SCR_HPSREG2ADDR_OFST,
                 ALT_NOC_FW_DDR_L3_SCR_HPSREG2ADDR_BASE_SET(
                                     Cfg->l3_2.base) |
                 ALT_NOC_FW_DDR_L3_SCR_HPSREG2ADDR_LIMIT_SET(
                                     Cfg->l3_2.limit));
    EnableRegion |=
          ALT_NOC_FW_DDR_L3_SCR_EN_SET_HPSREG2EN_SET_MSK;
  }
  if (Cfg->l3_3.enable == 1) {
    MmioWrite32 (ALT_NOC_FW_DDR_L3_SCR_OFST +
                 ALT_NOC_FW_DDR_L3_SCR_HPSREG3ADDR_OFST,
                 ALT_NOC_FW_DDR_L3_SCR_HPSREG3ADDR_BASE_SET(
                                     Cfg->l3_3.base) |
                 ALT_NOC_FW_DDR_L3_SCR_HPSREG3ADDR_LIMIT_SET(
                                     Cfg->l3_3.limit));
    EnableRegion |=
          ALT_NOC_FW_DDR_L3_SCR_EN_SET_HPSREG3EN_SET_MSK;
  }

  if (Cfg->l3_4.enable == 1) {
    MmioWrite32 (ALT_NOC_FW_DDR_L3_SCR_OFST +
                 ALT_NOC_FW_DDR_L3_SCR_HPSREG4ADDR_OFST,
                 ALT_NOC_FW_DDR_L3_SCR_HPSREG4ADDR_BASE_SET(
                                     Cfg->l3_4.base) |
                 ALT_NOC_FW_DDR_L3_SCR_HPSREG4ADDR_LIMIT_SET(
                                     Cfg->l3_4.limit));
    EnableRegion |=
          ALT_NOC_FW_DDR_L3_SCR_EN_SET_HPSREG4EN_SET_MSK;
  }
  if (Cfg->l3_5.enable == 1) {
    MmioWrite32 (ALT_NOC_FW_DDR_L3_SCR_OFST +
                 ALT_NOC_FW_DDR_L3_SCR_HPSREG5ADDR_OFST,
                 ALT_NOC_FW_DDR_L3_SCR_HPSREG5ADDR_BASE_SET(
                                     Cfg->l3_5.base) |
                 ALT_NOC_FW_DDR_L3_SCR_HPSREG5ADDR_LIMIT_SET(
                                     Cfg->l3_5.limit));
    EnableRegion |=
          ALT_NOC_FW_DDR_L3_SCR_EN_SET_HPSREG5EN_SET_MSK;
  }
  if (Cfg->l3_6.enable == 1) {
    MmioWrite32 (ALT_NOC_FW_DDR_L3_SCR_OFST +
                 ALT_NOC_FW_DDR_L3_SCR_HPSREG6ADDR_OFST,
                 ALT_NOC_FW_DDR_L3_SCR_HPSREG6ADDR_BASE_SET(
                                     Cfg->l3_6.base) |
                 ALT_NOC_FW_DDR_L3_SCR_HPSREG6ADDR_LIMIT_SET(
                                     Cfg->l3_6.limit));
    EnableRegion |=
          ALT_NOC_FW_DDR_L3_SCR_EN_SET_HPSREG6EN_SET_MSK;
  }
  if (Cfg->l3_7.enable == 1) {
    MmioWrite32 (ALT_NOC_FW_DDR_L3_SCR_OFST +
                 ALT_NOC_FW_DDR_L3_SCR_HPSREG7ADDR_OFST,
                 ALT_NOC_FW_DDR_L3_SCR_HPSREG7ADDR_BASE_SET(
                                     Cfg->l3_7.base) |
                 ALT_NOC_FW_DDR_L3_SCR_HPSREG7ADDR_LIMIT_SET(
                                     Cfg->l3_7.limit));
    EnableRegion |=
          ALT_NOC_FW_DDR_L3_SCR_EN_SET_HPSREG7EN_SET_MSK;
  }

  // Step 3 - Enable firewall regions
  // Use the enable_set Write one to set register
  MmioWrite32 (ALT_NOC_FW_DDR_L3_SCR_OFST +
               ALT_NOC_FW_DDR_L3_SCR_EN_SET_OFST,
               EnableRegion);

  return Status;
}


VOID
EFIAPI
DisplayFirewallInfo (
  VOID
  )
{
  UINTN   i, j;
  UINT32  Data32;

  // Char8Str is use to speed up Semihosting printing speed
  CHAR8   Char8Str[2048];
  CHAR8*  Char8Ptr = &Char8Str[0];

  InfoPrint (
    "Firewall Info:\r\n"
    "\t Master Region Enable field:\r\n");
  Data32 = MmioRead32 (ALT_NOC_FW_DDR_MPU_F2SDR_DDR_SCR_OFST +
                       ALT_NOC_FW_DDR_MPU_F2SDR_DDR_SCR_EN_OFST);
  InfoPrint (
    "\t\t MPU REG 0\t: %d\r\n"
    "\t\t MPU REG 1\t: %d\r\n"
    "\t\t MPU REG 2\t: %d\r\n"
    "\t\t MPU REG 3\t: %d\r\n"
    "\t\t F2SDR 0 REG 0\t: %d\r\n"
    "\t\t F2SDR 0 REG 1\t: %d\r\n"
    "\t\t F2SDR 0 REG 2\t: %d\r\n"
    "\t\t F2SDR 0 REG 3\t: %d\r\n"
    "\t\t F2SDR 1 REG 0\t: %d\r\n"
    "\t\t F2SDR 1 REG 1\t: %d\r\n"
    "\t\t F2SDR 1 REG 2\t: %d\r\n"
    "\t\t F2SDR 1 REG 3\t: %d\r\n"
    "\t\t F2SDR 2 REG 0\t: %d\r\n"
    "\t\t F2SDR 2 REG 1\t: %d\r\n"
    "\t\t F2SDR 2 REG 2\t: %d\r\n"
    "\t\t F2SDR 2 REG 3\t: %d\r\n",
    ALT_NOC_FW_DDR_MPU_F2SDR_DDR_SCR_EN_MPUREG0EN_GET(Data32),
    ALT_NOC_FW_DDR_MPU_F2SDR_DDR_SCR_EN_MPUREG1EN_GET(Data32),
    ALT_NOC_FW_DDR_MPU_F2SDR_DDR_SCR_EN_MPUREG2EN_GET(Data32),
    ALT_NOC_FW_DDR_MPU_F2SDR_DDR_SCR_EN_MPUREG3EN_GET(Data32),
    ALT_NOC_FW_DDR_MPU_F2SDR_DDR_SCR_EN_F2SDR0REG0EN_GET(Data32),
    ALT_NOC_FW_DDR_MPU_F2SDR_DDR_SCR_EN_F2SDR0REG1EN_GET(Data32),
    ALT_NOC_FW_DDR_MPU_F2SDR_DDR_SCR_EN_F2SDR0REG2EN_GET(Data32),
    ALT_NOC_FW_DDR_MPU_F2SDR_DDR_SCR_EN_F2SDR0REG3EN_GET(Data32),
    ALT_NOC_FW_DDR_MPU_F2SDR_DDR_SCR_EN_F2SDR1REG0EN_GET(Data32),
    ALT_NOC_FW_DDR_MPU_F2SDR_DDR_SCR_EN_F2SDR1REG1EN_GET(Data32),
    ALT_NOC_FW_DDR_MPU_F2SDR_DDR_SCR_EN_F2SDR1REG2EN_GET(Data32),
    ALT_NOC_FW_DDR_MPU_F2SDR_DDR_SCR_EN_F2SDR1REG3EN_GET(Data32),
    ALT_NOC_FW_DDR_MPU_F2SDR_DDR_SCR_EN_F2SDR2REG0EN_GET(Data32),
    ALT_NOC_FW_DDR_MPU_F2SDR_DDR_SCR_EN_F2SDR2REG1EN_GET(Data32),
    ALT_NOC_FW_DDR_MPU_F2SDR_DDR_SCR_EN_F2SDR2REG2EN_GET(Data32),
    ALT_NOC_FW_DDR_MPU_F2SDR_DDR_SCR_EN_F2SDR2REG3EN_GET(Data32));

  Data32 = MmioRead32 (ALT_NOC_FW_DDR_L3_SCR_OFST +
                       ALT_NOC_FW_DDR_L3_SCR_EN_OFST);
  InfoPrint (
    "\t\t HPS REG 0\t: %d\r\n"
    "\t\t HPS REG 1\t: %d\r\n"
    "\t\t HPS REG 2\t: %d\r\n"
    "\t\t HPS REG 3\t: %d\r\n"
    "\t\t HPS REG 4\t: %d\r\n"
    "\t\t HPS REG 5\t: %d\r\n"
    "\t\t HPS REG 6\t: %d\r\n"
    "\t\t HPS REG 7\t: %d\r\n",
    ALT_NOC_FW_DDR_L3_SCR_EN_HPSREG0EN_GET(Data32),
    ALT_NOC_FW_DDR_L3_SCR_EN_HPSREG1EN_GET(Data32),
    ALT_NOC_FW_DDR_L3_SCR_EN_HPSREG2EN_GET(Data32),
    ALT_NOC_FW_DDR_L3_SCR_EN_HPSREG3EN_GET(Data32),
    ALT_NOC_FW_DDR_L3_SCR_EN_HPSREG4EN_GET(Data32),
    ALT_NOC_FW_DDR_L3_SCR_EN_HPSREG5EN_GET(Data32),
    ALT_NOC_FW_DDR_L3_SCR_EN_HPSREG6EN_GET(Data32),
    ALT_NOC_FW_DDR_L3_SCR_EN_HPSREG7EN_GET(Data32));

  for (i = 0; i < 4; i++)
  {
    Data32 = MmioRead32 (ALT_NOC_FW_DDR_MPU_F2SDR_DDR_SCR_OFST +
                         ALT_NOC_FW_DDR_MPU_F2SDR_DDR_SCR_MPUREG0ADDR_OFST +
                         (i * 4));
    Char8Ptr += AsciiSPrint (Char8Ptr, 1024,
      "\t MPU REG %d\tBase=0x%x\tLimit=0x%x\r\n", i,
      ALT_NOC_FW_DDR_MPU_F2SDR_DDR_SCR_MPUREG0ADDR_BASE_GET(Data32),
      ALT_NOC_FW_DDR_MPU_F2SDR_DDR_SCR_MPUREG0ADDR_LIMIT_GET(Data32));
  }

  for (j = 0; j < 3; j++)
  {
    for (i = 0; i < 4; i++)
    {
    Data32 = MmioRead32 (ALT_NOC_FW_DDR_MPU_F2SDR_DDR_SCR_OFST +
                         ALT_NOC_FW_DDR_MPU_F2SDR_DDR_SCR_F2SDR0REG0ADDR_OFST +
                         (i * 4) + (j*4*4));
      Char8Ptr += AsciiSPrint (Char8Ptr, 1024,
        "\t F2SDR %d REG %d\tBase=0x%x\tLimit=0x%x\r\n", j, i,
        ALT_NOC_FW_DDR_MPU_F2SDR_DDR_SCR_F2SDR0REG0ADDR_BASE_GET(Data32),
        ALT_NOC_FW_DDR_MPU_F2SDR_DDR_SCR_F2SDR0REG0ADDR_LIMIT_GET(Data32));
    }
  }

  for (i = 0; i < 8; i++)
  {
    Data32 = MmioRead32 (ALT_NOC_FW_DDR_L3_SCR_OFST +
                         ALT_NOC_FW_DDR_L3_SCR_HPSREG0ADDR_OFST +
                         (i * 4));
    Char8Ptr += AsciiSPrint (Char8Ptr, 1024,
      "\t HPS REG %d\tBase=0x%x\tLimit=0x%x\r\n", i,
      ALT_NOC_FW_DDR_L3_SCR_HPSREG0ADDR_BASE_GET(Data32),
      ALT_NOC_FW_DDR_L3_SCR_HPSREG0ADDR_LIMIT_GET(Data32));
  }

  InfoPrint ("%a", Char8Str); Char8Ptr = &Char8Str[0];
}



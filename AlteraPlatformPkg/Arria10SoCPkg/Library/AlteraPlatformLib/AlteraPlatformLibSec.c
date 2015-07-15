/** @file

  Portions of the code modified by Altera to support SoC devices are licensed as follows:
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

  The original software modules are licensed as follows:

  Copyright (c) 2011-2012, ARM Limited. All rights reserved.

  This program and the accompanying materials
  are licensed and made available under the terms and conditions of the BSD License
  which accompanies this distribution.  The full text of the license may be found at
  http://opensource.org/licenses/bsd-license.php

  THE PROGRAM IS DISTRIBUTED UNDER THE BSD 3 Clause LICENSE ON AN "AS IS" BASIS,
  WITHOUT WARRANTIES OR REPRESENTATIONS OF ANY KIND, EITHER EXPRESS OR IMPLIED.

**/

#include <Library/ArmLib.h>
#include <Library/ArmPlatformLib.h>
#include <Library/DebugLib.h>
#include <Library/IoLib.h>
#include <Library/PcdLib.h>
#include <Ppi/ArmMpCoreInfo.h>
#include <AlteraPlatform.h>

//-----------------------------------------------------------------------------------------
// BEGIN ALTERA SOC FPGA RELATED CODE
//-----------------------------------------------------------------------------------------

VOID
AssertWatchDogTimerZeroReset (
  VOID
  )
{
  // Assert the Reset signal of Watchdog Timer 0 which may have been enabled by BootROM
  MmioOr32 (ALT_RSTMGR_OFST +
            ALT_RSTMGR_PER1MODRST_OFST,
            ALT_RSTMGR_PER1MODRST_WD0_SET_MSK);
}

VOID
AssertPeripheralsReset (
  VOID
  )
{
  UINT32 EccOcpMask;

  //
  // Assert Reset to all Peripheral Group 1 components except Watchdog Timer0
  //
  MmioWrite32 (ALT_RSTMGR_OFST +
               ALT_RSTMGR_PER1MODRST_OFST,
             ~(ALT_RSTMGR_PER1MODRST_WD0_SET_MSK));

  // Assert Reset to all Peripheral Group 0 using two step:
  // First, all none ECC_OCP components, followed by ECC_OCP components
  // NOTE:
  // When asserting reset, we have to set the ECC OCP last
  // When deasserting reset, we have to set the ECC OCP first
  EccOcpMask = ALT_RSTMGR_PER0MODRST_EMAC0OCP_SET_MSK |
               ALT_RSTMGR_PER0MODRST_EMAC1OCP_SET_MSK |
               ALT_RSTMGR_PER0MODRST_EMAC2OCP_SET_MSK |
               ALT_RSTMGR_PER0MODRST_USB0OCP_SET_MSK |
               ALT_RSTMGR_PER0MODRST_USB1OCP_SET_MSK |
               ALT_RSTMGR_PER0MODRST_NANDOCP_SET_MSK |
               ALT_RSTMGR_PER0MODRST_QSPIOCP_SET_MSK |
               ALT_RSTMGR_PER0MODRST_SDMMCOCP_SET_MSK |
               ALT_RSTMGR_PER0MODRST_DMAOCP_SET_MSK;

  // Assert reset to none ECC_OCP components
  MmioOr32 (ALT_RSTMGR_OFST +
            ALT_RSTMGR_PER0MODRST_OFST, ~EccOcpMask);

  // Assert reset to ECC_OCP components
  MmioOr32 (ALT_RSTMGR_OFST +
            ALT_RSTMGR_PER0MODRST_OFST, EccOcpMask);

}

VOID
DeassertSystemTimerZeroReset (
  VOID
  )
{
  // Assert the Reset signal of Watchdog Timer 0 which may have been enabled by BootROM
  MmioAnd32 (ALT_RSTMGR_OFST +
             ALT_RSTMGR_PER1MODRST_OFST,
             ALT_RSTMGR_PER1MODRST_L4SYSTMR0_CLR_MSK);
}

VOID
InitMpuDramWindowBoundary (
  VOID
  )
{
  UINT32  DramStartAddress;

  if (PcdGet32(PcdRemapOnChipRamTo1stOneMB) == 1) {
    // Open a 1MB Window to remap HPS On-Chip RAM at 0x0
    // Although only the first 256KB is usable,
    // The window still need to be a minimum 1MB due to MPUL2 filtering address must be 1MB aligned.
    DramStartAddress = 0x00100000;
    // Set the noc_addr_remap_value to 3 (b'11)
    MmioWrite32 (
      ALT_SYSMGR_OFST +
      ALT_SYSMGR_NOC_ADDR_REMAP_SET_OFST,
      ALT_SYSMGR_NOC_ADDR_REMAP_SET_REMAP0_SET_MSK |
      ALT_SYSMGR_NOC_ADDR_REMAP_SET_REMAP1_SET_MSK
      );
  } else {
    // BootROM is mapped to address 0x0 on reset, Remap it to make DRAM start from 0 instead.
    DramStartAddress = 0x00000000;
    // Clear the noc_addr_remap_value to zero
    MmioWrite32 (
      ALT_SYSMGR_OFST +
      ALT_SYSMGR_NOC_ADDR_REMAP_CLR_OFST,
      ALT_SYSMGR_NOC_ADDR_REMAP_CLR_REMAP0_SET_MSK |
      ALT_SYSMGR_NOC_ADDR_REMAP_CLR_REMAP1_SET_MSK
      );
  }

  // Switch address within the filtering range
  // from M0 system interconnect to M1 SDRAM L3 Interconnect.
  MmioWrite32 (
    ARM_MPUL2_OFST +
    ARM_MPUL2_ADDR_FILTERING_START_OFST,
    ARM_MPUL2_ADDR_FILTERING_ADDR_SET(DramStartAddress) |
    ARM_MPUL2_ADDR_FILTERING_START_EN_SET(ARM_MPUL2_ADDR_FILTERING_ENABLED)
    );
}

VOID
SetSecurityControlRegisters (
  VOID
  )
{
  // Set OCRAM Security Control Registers
  MmioWrite32 (ALT_NOC_FW_OCRAM_SCR_OFST +
               ALT_NOC_FW_OCRAM_SCR_REG0ADDR_OFST,
			   ALT_NOC_FW_OCRAM_SCR_REG0ADDR_LIMIT_SET_MSK);
  MmioWrite32 (ALT_NOC_FW_OCRAM_SCR_OFST +
               ALT_NOC_FW_OCRAM_SCR_EN_OFST,
			   ALT_NOC_FW_OCRAM_SCR_EN_CLR_REG0EN_SET_MSK);
  // Set DDR Security Control Registers
  MmioWrite32 (ALT_NOC_FW_DDR_L3_SCR_OFST +
               ALT_NOC_FW_DDR_L3_SCR_HPSREG0ADDR_OFST,
               ALT_NOC_FW_DDR_L3_SCR_HPSREG0ADDR_LIMIT_SET_MSK);
  MmioWrite32 (ALT_NOC_FW_DDR_L3_SCR_OFST +
               ALT_NOC_FW_DDR_L3_SCR_EN_OFST,
               ALT_NOC_FW_DDR_L3_SCR_EN_CLR_HPSREG0EN_SET_MSK);
}

/**
  Initialize controllers that must setup before entering PEI MAIN

  This function is called by the
  AlteraPlatformPkg/PrePeiCore/PrePeiCore.c

**/
RETURN_STATUS
ArmPlatformInitialize (
  IN  UINTN                     MpId
  )
{
  //
  // ArmPlatformPrePeiCore.efi will call this function
  // This is still SEC pahse, we want to do most SoC init in PEI phase instead
  //

  // Assert the reset to Disable Watchdog Timer 0 which has been enabled by BootROM
  AssertWatchDogTimerZeroReset ();

  // Reset is asserted here and not in PlatformInit PEI module because of
  // PeiMain first init Sys Timer 0 and install MicroSecondDelay/NanoSecondDelay functions
  AssertPeripheralsReset ();
  DeassertSystemTimerZeroReset ();

  // BootRom is mapped to address 0x0 when handover.
  // Define the MPU DRAM window boundaries.
  InitMpuDramWindowBoundary ();

  // Set Security Control Registers
  SetSecurityControlRegisters ();

  return EFI_SUCCESS;
}

//-----------------------------------------------------------------------------------------
// END ALTERA SOC FPGA RELATED CODE
//-----------------------------------------------------------------------------------------


//-----------------------------------------------------------------------------------------
// BEGIN ARM CPU RELATED CODE
//-----------------------------------------------------------------------------------------

// This Table will be consume by Hob init code to publish it into HOB as MPCore Info
// Hob init code will retrieve it by calling PrePeiCoreGetMpCoreInfo via Ppi
ARM_CORE_INFO mArmPlatformNullMpCoreInfoTable[] = {
  {
    // Cluster 0, Core 0
    0x0, 0x0,

    // MP Core MailBox Set/Get/Clear Addresses and Clear Value
    (EFI_PHYSICAL_ADDRESS)0,
    (EFI_PHYSICAL_ADDRESS)0,
    (EFI_PHYSICAL_ADDRESS)0,
    (UINT64)0xFFFFFFFF
  },
  {
    // Cluster 0, Core 1
    0x0, 0x1,

    // MP Core MailBox Set/Get/Clear Addresses and Clear Value
    (EFI_PHYSICAL_ADDRESS)0,
    (EFI_PHYSICAL_ADDRESS)0,
    (EFI_PHYSICAL_ADDRESS)0,
    (UINT64)0xFFFFFFFF
  },
  {
    // Cluster 0, Core 2
    0x0, 0x2,

    // MP Core MailBox Set/Get/Clear Addresses and Clear Value
    (EFI_PHYSICAL_ADDRESS)0,
    (EFI_PHYSICAL_ADDRESS)0,
    (EFI_PHYSICAL_ADDRESS)0,
    (UINT64)0xFFFFFFFF
  },
  {
    // Cluster 0, Core 3
    0x0, 0x3,

    // MP Core MailBox Set/Get/Clear Addresses and Clear Value
    (EFI_PHYSICAL_ADDRESS)0,
    (EFI_PHYSICAL_ADDRESS)0,
    (EFI_PHYSICAL_ADDRESS)0,
    (UINT64)0xFFFFFFFF
  }
};

EFI_STATUS
PrePeiCoreGetMpCoreInfo (
  OUT UINTN                   *CoreCount,
  OUT ARM_CORE_INFO           **ArmCoreTable
  )
{
  if (ArmIsMpCore()) {
    *CoreCount    = sizeof(mArmPlatformNullMpCoreInfoTable) / sizeof(ARM_CORE_INFO);
    *ArmCoreTable = mArmPlatformNullMpCoreInfoTable;
    return EFI_SUCCESS;
  } else {
    return EFI_UNSUPPORTED;
  }
}

// This will be consume by PrePeiCore to install Ppi
// Needs to be declared in the file. Otherwise gArmMpCoreInfoPpiGuid is undefined in the contect of PrePeiCore
EFI_GUID mArmMpCoreInfoPpiGuid = ARM_MP_CORE_INFO_PPI_GUID;
ARM_MP_CORE_INFO_PPI mMpCoreInfoPpi = { PrePeiCoreGetMpCoreInfo };

EFI_PEI_PPI_DESCRIPTOR      gPlatformPpiTable[] = {
  {
    EFI_PEI_PPI_DESCRIPTOR_PPI,
    &mArmMpCoreInfoPpiGuid,
    &mMpCoreInfoPpi
  }
};

VOID
ArmPlatformGetPlatformPpiList (
  OUT UINTN                   *PpiListSize,
  OUT EFI_PEI_PPI_DESCRIPTOR  **PpiList
  )
{
  if (ArmIsMpCore()) {
    *PpiListSize = sizeof(gPlatformPpiTable);
    *PpiList = gPlatformPpiTable;
  } else {
    *PpiListSize = 0;
    *PpiList = NULL;
  }
}

//-----------------------------------------------------------------------------------------
// END ARM CPU RELATED CODE
//-----------------------------------------------------------------------------------------


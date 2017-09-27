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
//
// Include files
//
#include <AlteraPlatform.h>
#include <Library/BaseMemoryLib.h>
#include <Library/DebugLib.h>
#include <Library/IoLib.h>
#include <Library/MemoryAllocationLib.h>
#include <Library/PcdLib.h>
#include <Library/SerialPortPrintLib.h>

#define InfoPrint  SerialPortPrint
#include "Ethernet.h"

STATIC CONST UINT32 mLookUpTable_ALT_EMAC_OFST[] = {
  ALT_EMAC0_OFST,
  ALT_EMAC1_OFST,
  ALT_EMAC2_OFST
};

STATIC CONST UINT32 mLookUpTable_ALT_SYSMGR_CORE_EMAC_OFST[] = {
  ALT_SYSMGR_CORE_EMAC0_OFST,
  ALT_SYSMGR_CORE_EMAC1_OFST,
  ALT_SYSMGR_CORE_EMAC2_OFST
};

STATIC CONST UINT32 mLookUpTable_ALT_RSTMGR_PER0MODRST_EMAC_SET_MSK[] = {
  ALT_RSTMGR_PER0MODRST_EMAC0_SET_MSK,
  ALT_RSTMGR_PER0MODRST_EMAC1_SET_MSK,
  ALT_RSTMGR_PER0MODRST_EMAC2_SET_MSK
};

STATIC CONST UINT32 mLookUpTable_ALT_RSTMGR_PER0MODRST_EMAC_CLR_MSK[] = {
  ALT_RSTMGR_PER0MODRST_EMAC0_CLR_MSK,
  ALT_RSTMGR_PER0MODRST_EMAC1_CLR_MSK,
  ALT_RSTMGR_PER0MODRST_EMAC2_CLR_MSK
};

STATIC CONST UINT32 mLookUpTable_ALT_SYSMGR_CORE_FPGAINTF_EN_3_EMAC_CLR_MSK[] = {
  ALT_SYSMGR_CORE_FPGAINTF_EN_3_EMAC_0_CLR_MSK,
  ALT_SYSMGR_CORE_FPGAINTF_EN_3_EMAC_1_CLR_MSK,
  ALT_SYSMGR_CORE_FPGAINTF_EN_3_EMAC_2_CLR_MSK
};

STATIC CONST UINT32 mLookUpTable_ALT_SYSMGR_CORE_EMAC_PHY_INTF_SEL_CLR_MSK[] = {
  ALT_SYSMGR_CORE_EMAC0_PHY_INTF_SEL_CLR_MSK,
  ALT_SYSMGR_CORE_EMAC1_PHY_INTF_SEL_CLR_MSK,
  ALT_SYSMGR_CORE_EMAC2_PHY_INTF_SEL_CLR_MSK
};

UINT32 mALT_EMAC_OFST;
UINT32 mALT_SYSMGR_CORE_EMAC_OFST;
UINT32 mALT_RSTMGR_PER0MODRST_EMAC_SET_MSK;
UINT32 mALT_RSTMGR_PER0MODRST_EMAC_CLR_MSK;
UINT32 mALT_SYSMGR_CORE_FPGAINTF_EN_3_EMAC_CLR_MSK;
UINT32 mALT_SYSMGR_CORE_EMAC_PHY_INTF_SEL_CLR_MSK;


// emac config phy interface setting
#define PHY_INTERFACE_MODE_GMII      0
#define PHY_INTERFACE_MODE_MII       1
#define PHY_INTERFACE_MODE_RGMII     2
#define PHY_INTERFACE_MODE_RMII      3

VOID
EFIAPI
EmacChooseInterface (
  IN UINT32 MacInterface
  )
{
  InfoPrint ("SNP:MAC: %s (%d)\r\n", __FUNCTION__, MacInterface);

  mALT_EMAC_OFST                              = mLookUpTable_ALT_EMAC_OFST[MacInterface];
  mALT_SYSMGR_CORE_EMAC_OFST                  = mLookUpTable_ALT_SYSMGR_CORE_EMAC_OFST[MacInterface];
  mALT_SYSMGR_CORE_FPGAINTF_EN_3_EMAC_CLR_MSK = mLookUpTable_ALT_SYSMGR_CORE_FPGAINTF_EN_3_EMAC_CLR_MSK[MacInterface];
  mALT_SYSMGR_CORE_EMAC_PHY_INTF_SEL_CLR_MSK  = mLookUpTable_ALT_SYSMGR_CORE_EMAC_PHY_INTF_SEL_CLR_MSK[MacInterface];
  mALT_RSTMGR_PER0MODRST_EMAC_SET_MSK         = mLookUpTable_ALT_RSTMGR_PER0MODRST_EMAC_SET_MSK[MacInterface];
  mALT_RSTMGR_PER0MODRST_EMAC_CLR_MSK         = mLookUpTable_ALT_RSTMGR_PER0MODRST_EMAC_CLR_MSK[MacInterface];
}

VOID
EFIAPI
EmacConfigPhyInterface (
  IN UINT32 PhyMode
  )
{
  UINT32 Val;

  InfoPrint ("SNP:MAC: %s ()\r\n", __FUNCTION__);

  Val = ALT_SYSMGR_CORE_EMAC0_PHY_INTF_SEL_E_GMII_MII;
  switch (PhyMode) {
    case PHY_INTERFACE_MODE_GMII:
      Val = ALT_SYSMGR_CORE_EMAC0_PHY_INTF_SEL_E_GMII_MII;
      break;
    case PHY_INTERFACE_MODE_MII:
      Val = ALT_SYSMGR_CORE_EMAC0_PHY_INTF_SEL_E_GMII_MII;
      break;
    case PHY_INTERFACE_MODE_RGMII:
      Val = ALT_SYSMGR_CORE_EMAC0_PHY_INTF_SEL_E_RGMII;
      break;
    case PHY_INTERFACE_MODE_RMII:
      Val = ALT_SYSMGR_CORE_EMAC0_PHY_INTF_SEL_E_RMII;
      break;
    default:
      InfoPrint("SNP:MAC: bad phymode %x\n", PhyMode);
      Val = ALT_SYSMGR_CORE_EMAC0_PHY_INTF_SEL_E_GMII_MII;
      break;
    }

  MmioAndThenOr32 (ALT_SYSMGR_CORE_OFST +
                   mALT_SYSMGR_CORE_EMAC_OFST,
                   mALT_SYSMGR_CORE_EMAC_PHY_INTF_SEL_CLR_MSK,
                   Val);
}

VOID
EFIAPI
AlteraHpsInitialization (
  VOID
  )
{
  InfoPrint ("SNP:MAC: %s ()\r\n", __FUNCTION__);

  // To initialize the Ethernet controller to use the HPS interface, specific software steps must be followed
  // including selecting the correct PHY interface through the System Manager.
  // In general, the Ethernet Controller must be in a reset state during static configuration and the clock must
  // be active and valid before the Ethernet Controller is brought out of reset.

  // 1. After the HPS is released from cold or warm reset, reset the Ethernet Controller module by setting the
  // appropriate emac* bit in the per0modrst register in the Reset Manager.
  MmioOr32 (ALT_RSTMGR_OFST +
            ALT_RSTMGR_PER0MODRST_OFST,
            mALT_RSTMGR_PER0MODRST_EMAC_SET_MSK);
  // 2. Configure the EMAC Controller clock to 250 MHz by programming the appropriate registers in the Clock Manager.

  // 3. Bring the Ethernet PHY out of reset to verify that there are RX PHY clocks.

  // 4. When all the clocks are valid, program the following clock settings:
  // a. Program the phy_intf_sel field of the emac* register in the System Manager to 0x1 or 0x2 to select
  // RGMII or RMII PHY interface.
  EmacConfigPhyInterface (PHY_INTERFACE_MODE_RGMII);

  // b. Disable the Ethernet Controller FPGA interface by clearing the emac_* bit in the fpgaintf_en_3
  // register of the System Manager.
  MmioAnd32 (ALT_SYSMGR_CORE_OFST +
             ALT_SYSMGR_CORE_FPGAINTF_EN_3_OFST,
             mALT_SYSMGR_CORE_FPGAINTF_EN_3_EMAC_CLR_MSK);

  // 5. Configure all of the EMAC static settings if the user requires a different setting from the default value.
  // These settings include the AxPROT[1:0] and AxCACHE signal values, which are programmed in the
  // emac* register of the System Manager.

  // 6. Execute a register read back to confirm the clock and static configuration settings are valid.

  // 7. After confirming the settings are valid, software can clear the emac* bit in the per0modrst register of
  // the Reset Manager to bring the EMAC out of reset.
   MmioAnd32 (ALT_RSTMGR_OFST +
              ALT_RSTMGR_PER0MODRST_OFST,
              mALT_RSTMGR_PER0MODRST_EMAC_CLR_MSK);
  // When these steps are completed, general Ethernet controller and DMA software initialization and
  // configuration can continue.

}


void EmacInit (void)
{
	EmacChooseInterface(0);
	AlteraHpsInitialization ();
	
	
}

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

  Copyright (c) 2012 - 2014, ARM Limited. All rights reserved.
  Copyright (c) 2004 - 2010, Intel Corporation. All rights reserved.

  This program and the accompanying materials
  are licensed and made available under the terms and conditions of the BSD License
  which accompanies this distribution.  The full text of the license may be found at
  http://opensource.org/licenses/bsd-license.php

  THE PROGRAM IS DISTRIBUTED UNDER THE BSD 3 Clause LICENSE ON AN "AS IS" BASIS,
  WITHOUT WARRANTIES OR REPRESENTATIONS OF ANY KIND, EITHER EXPRESS OR IMPLIED.

**/

#include "PhyDxeUtil.h"
#include "EmacDxeUtil.h"

#include <Library/SerialPortPrintLib.h>
#if (FixedPcdGet32(PcdDebugMsg_EmacSnpDxe) == 0)
  #define ProgressPrint                       SerialPortPrint
  #define InfoPrint(FormatString, ...)        /* do nothing */
#else
  #define ProgressPrint SerialPortPrint
  #define InfoPrint     SerialPortPrint
#endif

EFI_STATUS
EFIAPI
PhyDxeInitialization (
  IN PHY_DRIVER* PhyDriver
  )
{
  InfoPrint ("SNP:PHY: %a ()\r\n", __FUNCTION__);

  // initialize the phyaddr
  PhyDriver->PhyAddr = 0;
  PhyDriver->PhyCurrentLink = LINK_DOWN;
  PhyDriver->PhyOldLink = LINK_DOWN;

  if (PhyDetectDevice (PhyDriver) != EFI_SUCCESS) {
    return EFI_NOT_FOUND;
  }

  PhyConfig (PhyDriver);

  return EFI_SUCCESS;
}


// PHY detect device
EFI_STATUS
EFIAPI
PhyDetectDevice (
  IN PHY_DRIVER* PhyDriver
  )
{
  UINT32 PhyAddr;

  InfoPrint ("SNP:PHY: %a ()\r\n", __FUNCTION__);

  for (PhyAddr = 0; PhyAddr < 32; PhyAddr++) {
    if (PhyReadId (PhyAddr) == EFI_SUCCESS) {
      PhyDriver->PhyAddr = PhyAddr;
      return EFI_SUCCESS;
    }
  }
  ProgressPrint("SNP:PHY: Fail to detect Ethernet PHY!\r\n");
  return EFI_NOT_FOUND;
}


EFI_STATUS
EFIAPI
PhyConfig (
  IN PHY_DRIVER* PhyDriver
  )
{
  EFI_STATUS Status;

  InfoPrint ("SNP:PHY: %a ()\r\n", __FUNCTION__);

  Status = PhySoftReset (PhyDriver);
  if (EFI_ERROR (Status)) {
    return EFI_DEVICE_ERROR;
  }

  // Configure TX/RX Skew
  PhyConfigSkew (PhyDriver);

  // Read back and display Skew settings
  PhyDisplayConfigSkew (PhyDriver);

  // Configure AN and Advertise
  PhyAutoNego (PhyDriver);

  return EFI_SUCCESS;
}


// Perform PHY software reset
EFI_STATUS
EFIAPI
PhySoftReset (
  IN PHY_DRIVER* PhyDriver
  )
{
  UINT32        TimeOut;
  UINT32        Data32;
  EFI_STATUS    Status;

  InfoPrint ("SNP:PHY: %a ()\r\n", __FUNCTION__);

  // PHY Basic Control Register reset
  PhyWrite (PhyDriver->PhyAddr, PHY_BASIC_CTRL, PHYCTRL_RESET);

  // Wait for completion
  TimeOut = 0;
  do {
    // Read PHY_BASIC_CTRL register from PHY
    Status = PhyRead (PhyDriver->PhyAddr, PHY_BASIC_CTRL, &Data32);
    if (EFI_ERROR(Status)) {
      return Status;
    }
    // Wait until PHYCTRL_RESET become zero
    if ((Data32 & PHYCTRL_RESET) == 0) {
      break;
    }
    MicroSecondDelay(1);
  } while (TimeOut++ < PHY_TIMEOUT);
  if (TimeOut >= PHY_TIMEOUT) {
    InfoPrint("SNP:PHY: ERROR! PhySoftReset timeout\n");
    return EFI_TIMEOUT;
  }

  return EFI_SUCCESS;
}


// PHY read ID
EFI_STATUS
EFIAPI
PhyReadId (
  IN UINT32  PhyAddr
  )
{
  EFI_STATUS    Status;
  UINT32        PhyId1;
  UINT32        PhyId2;

  //InfoPrint ("SNP:PHY: %a (0x%02X)\r\n", __FUNCTION__, PhyAddr);

  Status = PhyRead (PhyAddr, PHY_ID1, &PhyId1);
  if (EFI_ERROR(Status)) return Status;
  Status = PhyRead (PhyAddr, PHY_ID2, &PhyId2);
  if (EFI_ERROR(Status)) return Status;

  if (PhyId1 == PHY_INVALID_ID || PhyId2 == PHY_INVALID_ID) {
    return EFI_NOT_FOUND;
  }

  InfoPrint("SNP:PHY: Ethernet PHY detected. PHY_ID1=0x%04X, PHY_ID2=0x%04X, PHY_ADDR=0x%02X\r\n", PhyId1, PhyId2, PhyAddr);
  return EFI_SUCCESS;
}


VOID
EFIAPI
PhyConfigSkew (
  IN PHY_DRIVER* PhyDriver
  )
{
  Phy9031ExtendedWrite(PhyDriver, PHY_KSZ9031_MOD_DATA_NO_POST_INC, PHY_KSZ9031RN_DEV_ADDR, PHY_KSZ9031RN_CONTROL_PAD_SKEW_REG, PHY_KSZ9031RN_CONTROL_PAD_SKEW_VALUE);
  Phy9031ExtendedWrite(PhyDriver, PHY_KSZ9031_MOD_DATA_NO_POST_INC, PHY_KSZ9031RN_DEV_ADDR, PHY_KSZ9031RN_CLK_PAD_SKEW_REG,     PHY_KSZ9031RN_CLK_PAD_SKEW_VALUE);
  Phy9031ExtendedWrite(PhyDriver, PHY_KSZ9031_MOD_DATA_NO_POST_INC, PHY_KSZ9031RN_DEV_ADDR, PHY_KSZ9031RN_RX_DATA_PAD_SKEW_REG, PHY_KSZ9031RN_RX_DATA_PAD_SKEW_VALUE);
  Phy9031ExtendedWrite(PhyDriver, PHY_KSZ9031_MOD_DATA_NO_POST_INC, PHY_KSZ9031RN_DEV_ADDR, PHY_KSZ9031RN_TX_DATA_PAD_SKEW_REG, PHY_KSZ9031RN_TX_DATA_PAD_SKEW_VALUE);

}


VOID
EFIAPI
PhyDisplayConfigSkew (
  IN PHY_DRIVER* PhyDriver
  )
{
  // Display skew configuration
  InfoPrint("SNP:PHY: Control Signal Pad Skew = 0x%04X\r\n", Phy9031ExtendedRead(PhyDriver, PHY_KSZ9031_MOD_DATA_NO_POST_INC, PHY_KSZ9031RN_DEV_ADDR, PHY_KSZ9031RN_CONTROL_PAD_SKEW_REG));
  InfoPrint("SNP:PHY: RGMII Clock Pad Skew    = 0x%04X\r\n", Phy9031ExtendedRead(PhyDriver, PHY_KSZ9031_MOD_DATA_NO_POST_INC, PHY_KSZ9031RN_DEV_ADDR, PHY_KSZ9031RN_CLK_PAD_SKEW_REG));
  InfoPrint("SNP:PHY: RGMII RX Data Pad Skew  = 0x%04X\r\n", Phy9031ExtendedRead(PhyDriver, PHY_KSZ9031_MOD_DATA_NO_POST_INC, PHY_KSZ9031RN_DEV_ADDR, PHY_KSZ9031RN_RX_DATA_PAD_SKEW_REG));
  InfoPrint("SNP:PHY: RGMII TX Data Pad Skew  = 0x%04X\r\n", Phy9031ExtendedRead(PhyDriver, PHY_KSZ9031_MOD_DATA_NO_POST_INC, PHY_KSZ9031RN_DEV_ADDR, PHY_KSZ9031RN_TX_DATA_PAD_SKEW_REG));
}


// Do auto-negotiation
EFI_STATUS
EFIAPI
PhyAutoNego (
  IN PHY_DRIVER* PhyDriver
  )
{
  EFI_STATUS    Status;
  UINT32        PhyControl;
  UINT32        PhyStatus;
  UINT32        Features;

  InfoPrint ("SNP:PHY: %a ()\r\n", __FUNCTION__);

  // Read PHY Status
  Status = PhyRead (PhyDriver->PhyAddr, PHY_BASIC_STATUS, &PhyStatus);
  if (EFI_ERROR(Status)) return Status;

  // Check PHY Status if auto-negotiation is supported
  if ((PhyStatus & PHYSTS_AUTO_CAP) == 0) {
    InfoPrint("SNP:PHY: Auto-negotiation is not supported.\n");
    return EFI_DEVICE_ERROR;
  }

  // Read PHY Auto-Nego Advertise capabilities register for 10/100 Base-T
  Status = PhyRead (PhyDriver->PhyAddr, PHY_AUTO_NEG_ADVERT, &Features);
  if (EFI_ERROR(Status)) return Status;

  // Set Advertise capabilities for 10Base-T/10Base-T full-duplex/100Base-T/100Base-T full-duplex
  Features |= (PHYANA_10BASET | PHYANA_10BASETFD | PHYANA_100BASETX | PHYANA_100BASETXFD);
  PhyWrite (PhyDriver->PhyAddr, PHY_AUTO_NEG_ADVERT, Features);

  // Read PHY Auto-Nego Advertise capabilities register for 1000 Base-T
  Status = PhyRead (PhyDriver->PhyAddr, PHY_1000BASE_T_CONTROL, &Features);
  if (EFI_ERROR(Status)) return Status;

  // Set Advertise capabilities for 1000 Base-T/1000 Base-T full-duplex
  Features |= (PHYADVERTISE_1000FULL | PHYADVERTISE_1000HALF);
  PhyWrite (PhyDriver->PhyAddr, PHY_1000BASE_T_CONTROL, Features);

  // Read control register
  Status = PhyRead (PhyDriver->PhyAddr, PHY_BASIC_CTRL, &PhyControl);
  if (EFI_ERROR(Status)) return Status;

  // Enable Auto-Negotiation
  PhyControl |= PHYCTRL_AUTO_EN;
  // Restart auto-negotiation
  PhyControl |= PHYCTRL_RST_AUTO;
  // Write this configuration
  PhyWrite (PhyDriver->PhyAddr, PHY_BASIC_CTRL, PhyControl);

  return EFI_SUCCESS;
}


EFI_STATUS
EFIAPI
PhyLinkAdjustEmacConfig (
  IN PHY_DRIVER* PhyDriver
  )
{
  UINT32 Speed;
  UINT32 Duplex;
  EFI_STATUS Status;

  //InfoPrint ("SNP:PHY: %a ()\r\n", __FUNCTION__);

  Status = EFI_SUCCESS;
  Speed = SPEED_10;
  Duplex = DUPLEX_HALF;

  if (PhyCheckLinkStatus (PhyDriver) == EFI_SUCCESS) {
    PhyDriver->PhyCurrentLink = LINK_UP;
  } else {
    PhyDriver->PhyCurrentLink = LINK_DOWN;
  }

  if (PhyDriver->PhyOldLink != PhyDriver->PhyCurrentLink) {
    if (PhyDriver->PhyCurrentLink == LINK_UP) {
      ProgressPrint("SNP:PHY: Link is up - Network Cable is Plugged\r\n");
      PhyReadCapability (PhyDriver, &Speed, &Duplex);
      EmacConfigAdjust (Speed, Duplex);
      Status = EFI_SUCCESS;
    } else {
      ProgressPrint("SNP:PHY: Link is Down - Network Cable is Unplugged?\r\n");
      Status = EFI_NOT_READY;
    }
  } else if (PhyDriver->PhyCurrentLink == LINK_DOWN) {
    Status = EFI_NOT_READY;
  }

  PhyDriver->PhyOldLink = PhyDriver->PhyCurrentLink;

  return Status;
}


EFI_STATUS
EFIAPI
PhyCheckLinkStatus (
  IN PHY_DRIVER* PhyDriver
  )
{
  EFI_STATUS    Status;
  UINT32        Data32;
  UINTN         TimeOut;
  UINT32        PhyBasicStatus;

  //InfoPrint ("SNP:PHY: %a ()\r\n", __FUNCTION__);

  // Get the PHY Status
  Status = PhyRead (PhyDriver->PhyAddr, PHY_BASIC_STATUS, &PhyBasicStatus);
  if (EFI_ERROR(Status)) return Status;

  // if Link is already up then dont need to proceed anymore
  if (PhyBasicStatus & PHYSTS_LINK_STS) {
    return EFI_SUCCESS;
  }

  // Wait until it is up or until Time Out
  TimeOut = 0;
  do {
    // Read PHY_BASIC_STATUS register from PHY
    Status = PhyRead (PhyDriver->PhyAddr, PHY_BASIC_STATUS, &Data32);
    if (EFI_ERROR(Status)) {
      return Status;
    }
    // Wait until PHYSTS_LINK_STS become one
    if (Data32 & PHYSTS_LINK_STS) {
      // Link is up
      break;
    }
    MicroSecondDelay(1);
  } while (TimeOut++ < PHY_TIMEOUT);
  if (TimeOut >= PHY_TIMEOUT) {
    // Link is down
    return EFI_TIMEOUT;
  }

  // Wait until autonego process has completed
  TimeOut = 0;
  do {
    // Read PHY_BASIC_STATUS register from PHY
    Status = PhyRead (PhyDriver->PhyAddr, PHY_BASIC_STATUS, &Data32);
    if (EFI_ERROR(Status)) {
      return Status;
    }
    // Wait until PHYSTS_AUTO_COMP become one
    if (Data32 & PHYSTS_AUTO_COMP) {
      InfoPrint("SNP:PHY: Auto Negotiation completed\r\n");
      break;
    }
    MicroSecondDelay(1);
  } while (TimeOut++ < PHY_TIMEOUT);
  if (TimeOut >= PHY_TIMEOUT) {
    InfoPrint("SNP:PHY: Error! Auto Negotiation timeout\n");
    return EFI_TIMEOUT;
  }

  return EFI_SUCCESS;
}


EFI_STATUS
EFIAPI
PhyReadCapability (
  IN PHY_DRIVER* PhyDriver,
  IN UINT32* Speed,
  IN UINT32* Duplex
  )
{
  EFI_STATUS    Status;
  UINT32        PartnerAbilityGb;
  UINT32        AdvertisingGb;
  UINT32        CommonAbilityGb;
  UINT32        PartnerAbility;
  UINT32        Advertising;
  UINT32        CommonAbility;

  //InfoPrint ("SNP:PHY: %a ()\r\n", __FUNCTION__);

  // For 1000 Base-T

  Status = PhyRead (PhyDriver->PhyAddr, PHY_1000BASE_T_STATUS, &PartnerAbilityGb);
  if (EFI_ERROR(Status)) return Status;

  Status = PhyRead (PhyDriver->PhyAddr, PHY_1000BASE_T_CONTROL, &AdvertisingGb);
  if (EFI_ERROR(Status)) return Status;

  CommonAbilityGb = PartnerAbilityGb & (AdvertisingGb << 2);

  // For 10/100 Base-T

  Status = PhyRead (PhyDriver->PhyAddr, PHY_AUTO_NEG_LINK_ABILITY, &PartnerAbility);
  if (EFI_ERROR(Status)) return Status;

  Status = PhyRead (PhyDriver->PhyAddr, PHY_AUTO_NEG_EXP, &Advertising);
  if (EFI_ERROR(Status)) return Status;

  CommonAbility = PartnerAbility & Advertising;

  // Determine the Speed and Duplex
  if (PartnerAbilityGb & (PHYLPA_1000FULL | PHYLPA_1000HALF)) {
    *Speed = SPEED_1000;
    if (CommonAbilityGb & PHYLPA_1000FULL) {
      *Duplex = DUPLEX_FULL;
    }
  } else if (CommonAbility & (PHYLPA_100FULL | PHYLPA_100HALF)) {
    *Speed = SPEED_100;
    if (CommonAbility & PHYLPA_100FULL) {
      *Duplex = DUPLEX_FULL;
    } else if (CommonAbility & PHYLPA_10FULL) {
      *Duplex = DUPLEX_FULL;
    }
  }

  PhyDisplayAbility (*Speed, *Duplex);

  return EFI_SUCCESS;
}


VOID
EFIAPI
PhyDisplayAbility (
  IN UINT32 Speed,
  IN UINT32 Duplex
  )
{
  //InfoPrint ("SNP:PHY: %a ()\r\n", __FUNCTION__);

  ProgressPrint("SNP:PHY: ");
  switch (Speed) {
    case SPEED_1000:
      ProgressPrint("1 Gbps - ");
      break;
    case SPEED_100:
      ProgressPrint("100 Mbps - ");
      break;
    case SPEED_10:
      ProgressPrint("10 Mbps - ");
      break;
    default:
      ProgressPrint("Invalid link speed");
      break;
    }

  switch (Duplex) {
    case DUPLEX_FULL:
      ProgressPrint("Full Duplex\n");
      break;
    case DUPLEX_HALF:
      ProgressPrint("Half Duplex\n");
      break;
    default:
      ProgressPrint("Invalid duplex mode\n");
      break;
    }
}


// Function to read from MII register (PHY Access)
EFI_STATUS
EFIAPI
PhyRead (
  IN  UINT32  Addr,
  IN  UINT32  Reg,
  OUT UINT32 *Data
  )
{
  UINT32        MiiConfig;
  UINT32        Count;

  //InfoPrint ("SNP:PHY: %a ()\r\n", __FUNCTION__);

  // Check it is a valid Reg
  ASSERT(Reg < 31);

  MiiConfig = ((Addr << MIIADDRSHIFT) & MII_ADDRMSK) |
              ((Reg << MIIREGSHIFT) & MII_REGMSK)|
               MII_CLKRANGE_150_250M |
               MII_BUSY;

  // write this config to register
  MmioWrite32(ALT_EMAC0_OFST + ALT_EMAC_GMAC_GMII_ADDR_OFST, MiiConfig);

  // Wait for busy bit to clear
  Count = 0;
  while (Count < 10000) {
    if (!(ALT_EMAC_GMAC_GMII_ADDR_GB_GET(MmioRead32(ALT_EMAC0_OFST + ALT_EMAC_GMAC_GMII_ADDR_OFST))))
	{
      *Data = ALT_EMAC_GMAC_GMII_DATA_GD_GET(MmioRead32(ALT_EMAC0_OFST + ALT_EMAC_GMAC_GMII_DATA_OFST));
      return EFI_SUCCESS;
	}
    Count++;
  };
  InfoPrint("SNP:PHY: MDIO busy bit timeout\r\n");
  return EFI_TIMEOUT;
}


// Function to write to the MII register (PHY Access)
EFI_STATUS
EFIAPI
PhyWrite (
  IN UINT32 Addr,
  IN UINT32 Reg,
  IN UINT32 Data
  )
{
  UINT32 MiiConfig;
  UINT32 Count;

  //InfoPrint ("SNP:PHY: %a ()\r\n", __FUNCTION__);

  // Check it is a valid Reg
  ASSERT(Reg < 31);

  MiiConfig = ((Addr << MIIADDRSHIFT) & MII_ADDRMSK) |
              ((Reg << MIIREGSHIFT) & MII_REGMSK)|
               MII_WRITE |
               MII_CLKRANGE_150_250M |
               MII_BUSY;
  // Write the desired value to the register first
  MmioWrite32 (ALT_EMAC0_OFST + ALT_EMAC_GMAC_GMII_DATA_OFST, (Data & 0xFFFF));

  // write this config to register
  MmioWrite32(ALT_EMAC0_OFST + ALT_EMAC_GMAC_GMII_ADDR_OFST, MiiConfig);

  // Wait for busy bit to clear
  Count = 0;
  while (Count < 1000) {
      if (!(ALT_EMAC_GMAC_GMII_ADDR_GB_GET(MmioRead32(ALT_EMAC0_OFST + ALT_EMAC_GMAC_GMII_ADDR_OFST))))
        return EFI_SUCCESS;
      Count++;
    };

  return EFI_TIMEOUT;
}


EFI_STATUS
EFIAPI
Phy9031ExtendedWrite (
  IN PHY_DRIVER* PhyDriver,
  IN UINT32      Mode,
  IN UINT32      DevAddr,
  IN UINT32      Regnum,
  IN UINT16      Val
  )
{
  //InfoPrint ("SNP:PHY: %a ()\r\n", __FUNCTION__);

  PhyWrite(PhyDriver->PhyAddr, PHY_KSZ9031RN_MMD_CTRL_REG, DevAddr);
  PhyWrite(PhyDriver->PhyAddr, PHY_KSZ9031RN_MMD_REGDATA_REG, Regnum);
  PhyWrite(PhyDriver->PhyAddr, PHY_KSZ9031RN_MMD_CTRL_REG, (Mode << 14) | DevAddr);
  return PhyWrite(PhyDriver->PhyAddr, PHY_KSZ9031RN_MMD_REGDATA_REG, Val);
}


UINT32
EFIAPI
Phy9031ExtendedRead (
  IN PHY_DRIVER* PhyDriver,
  IN UINT32      Mode,
  IN UINT32      DevAddr,
  IN UINT32      Regnum
  )
{
  EFI_STATUS    Status;
  UINT32        Data32;

  //InfoPrint ("SNP:PHY: %a ()\r\n", __FUNCTION__);

  PhyWrite(PhyDriver->PhyAddr, PHY_KSZ9031RN_MMD_CTRL_REG, DevAddr);
  PhyWrite(PhyDriver->PhyAddr, PHY_KSZ9031RN_MMD_REGDATA_REG, Regnum);
  PhyWrite(PhyDriver->PhyAddr, PHY_KSZ9031RN_MMD_CTRL_REG, (Mode << 14) | DevAddr);

  Status = PhyRead (PhyDriver->PhyAddr, PHY_KSZ9031RN_MMD_REGDATA_REG, &Data32);
  if (EFI_ERROR(Status)) return 0;

  return Data32;
}



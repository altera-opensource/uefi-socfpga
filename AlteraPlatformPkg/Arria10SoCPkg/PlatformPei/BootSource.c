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
#include <Library/SerialPortPrintLib.h>
#include <Library/TimerLib.h>
#include "Assert.h"
#include "BootSource.h"
#include "DeviceTree.h"

#if (FixedPcdGet32(PcdDebugMsg_BootSource) == 0)
  #define ProgressPrint(FormatString, ...)    /* do nothing */
  #define InfoPrint(FormatString, ...)        /* do nothing */
  #define Delay1Second()                      /* no delay */
#else
  #define ProgressPrint SerialPortPrint
  #define InfoPrint     SerialPortPrint
  #define Delay1Second()  MicroSecondDelay (1000000)
#endif


//
// Functions
//
#if (FixedPcdGet32(PcdIsAlteraSoCFPGADevelopmentBoards) == 1)
VOID
EFIAPI
TryToGrabUsersAttentionOnThisMessage (
  VOID
  )
{
  // Print only if Serial Port is ready
  // this is to avoid user wondering why nothing is happening when no serial port
  #define R_UART_LCR  (3 * 4)
  if (PcdGet64 (PcdSerialRegisterBase) != 0) // Have serial port?
  {
    if ((MmioRead32 (PcdGet64 (PcdSerialRegisterBase) + R_UART_LCR) & 0x3F) == (PcdGet8 (PcdSerialLineControl) & 0x3F)) {
      ProgressPrint ("Counting down...10");
      Delay1Second ();ProgressPrint ("...9");
      Delay1Second ();ProgressPrint ("...8");
      Delay1Second ();ProgressPrint ("...7");
      Delay1Second ();ProgressPrint ("...6");
      Delay1Second ();ProgressPrint ("...5");
      Delay1Second ();ProgressPrint ("...4");
      Delay1Second ();ProgressPrint ("...3");
      Delay1Second ();ProgressPrint ("...2");
      Delay1Second ();ProgressPrint ("...1");
      Delay1Second ();ProgressPrint ("\r\n");
    }
  }
}
#endif

BOOT_SOURCE_TYPE
EFIAPI
GetBootSourceType (
  VOID
  )
{
  BOOT_SOURCE_TYPE         ReturnType;
  FUSE_BOOT_SOURCE_TYPE    FuseType;
  BSEL_BOOT_SOURCE_TYPE    BselType;
  PINMUX_BOOT_SOURCE_TYPE  PinmuxType;

  // Get Boot Source type from hardware registers
  FuseType   = GetBootSourceTypeViaFuse ();
  BselType   = GetBootSourceTypeViaBsel ();
  PinmuxType = GetBootSourceTypeViaDedicatedIoMuxSelects ();

  ProgressPrint ("Boot Source Type = ");

  // Boot based on FUSE ?
  if (FuseType == FUSE_BOOT_SOURCE_NONE)
  {
    // Boot based on BSEL ?
    // and use this opportunity to check the BSEL == PINMUX consistency
    switch (BselType)
    {
      case BSEL_BOOT_SOURCE_NAND:
        ProgressPrint ("NAND based on BSEL\r\n");
        if (PinmuxType != PINMUX_BOOT_SOURCE_NAND)
        {
          ErrorHandlerBselIsNandButDtbPinMuxIsNotForNand();
        }
        ReturnType = BOOT_SOURCE_NAND;
        break;
      case BSEL_BOOT_SOURCE_SDMMC:
        ProgressPrint ("SD/MMC based on BSEL\r\n");
        if (PinmuxType != PINMUX_BOOT_SOURCE_SDMMC)
        {
          ErrorHandlerBselIsSdMmcButDtbPinMuxIsNotForSdMmc();
        }
        ReturnType = BOOT_SOURCE_SDMMC;
        break;
      case BSEL_BOOT_SOURCE_QSPI:
        ProgressPrint ("QSPI based on BSEL\r\n");
        if (PinmuxType != PINMUX_BOOT_SOURCE_QSPI)
        {
          ErrorHandlerBselIsQspiButDtbPinMuxIsNotForQspi();
        }
        ReturnType = BOOT_SOURCE_QSPI;
        break;
      case BSEL_BOOT_SOURCE_RSVD:
        // Boot based on Pin Mux ?
        if (PinmuxType == PINMUX_BOOT_SOURCE_NAND)
        {
          ProgressPrint ("NAND based on FDT PinMux\r\n");
          ReturnType = BOOT_SOURCE_NAND;
        } else if (PinmuxType == PINMUX_BOOT_SOURCE_SDMMC)
        {
          ProgressPrint ("SD/MMC based on FDT PinMux\r\n");
          ReturnType = BOOT_SOURCE_SDMMC;
        } else if (PinmuxType == PINMUX_BOOT_SOURCE_QSPI)
        {
          ProgressPrint ("QSPI based on FDT PinMux\r\n");
          ReturnType = BOOT_SOURCE_QSPI;
        } else {
          ProgressPrint ("RSVD based on BSEL\r\n");
          ReturnType = BOOT_SOURCE_RSVD;
        }
#if (FixedPcdGet32(PcdIsAlteraSoCFPGADevelopmentBoards) == 1)
        if (ReturnType != BOOT_SOURCE_RSVD)
        {
          // This is Altera Soc FPGA Development Board
          // Override Pin Mux settings per Daughter Card

          // Delay long enough to annoy the developer
          // so that they know they build the wrong DTB
          // then only we override the pin mux for user's convenience
          ProgressPrint ("\r\nPLEASE CHECK:\r\n");
          ProgressPrint ("Is the FLASH Daughter Card plugin correctly? (because BSEL is RSVD)\r\n");
          TryToGrabUsersAttentionOnThisMessage ();
        }
#endif
        break;
      case BSEL_BOOT_SOURCE_FPGA:
        ProgressPrint ("FPGA based on BSEL\r\n");
        ReturnType = BOOT_SOURCE_FPGA;
        break;
      default:
        // Shoud not come here
        ASSERT_PLATFORM_INIT(0);
        ReturnType = BOOT_SOURCE_RSVD;
        break;
    }
  } else if (FuseType == FUSE_BOOT_SOURCE_FPGA)
  {
    ProgressPrint ("FPGA based on FUSE\r\n");
    ReturnType = BOOT_SOURCE_FPGA;
  } else {
    // Shoud not come here
    ASSERT_PLATFORM_INIT(0);
    ReturnType = BOOT_SOURCE_RSVD;
  }
  return ReturnType;
}


FUSE_BOOT_SOURCE_TYPE
EFIAPI
GetBootSourceTypeViaFuse (
  VOID
  )
{
  UINT32                 Data32;
  FUSE_BOOT_SOURCE_TYPE  FuseBootSourceType;

  Data32 = MmioRead32 (ALT_SEC_MGR_OFST + ALT_SEC_MGR_CURSECSTATE_OFST);

  if (ALT_SEC_MGR_CURSECSTATE_STATE_FPGA_BOOT_GET(Data32) ==
      ALT_SEC_MGR_CURSECSTATE_STATE_FPGA_BOOT_E_FPGA_BOOT_EN)
    FuseBootSourceType = FUSE_BOOT_SOURCE_FPGA;
  else
    FuseBootSourceType = FUSE_BOOT_SOURCE_NONE;

  return FuseBootSourceType;
}


BSEL_BOOT_SOURCE_TYPE
EFIAPI
GetBootSourceTypeViaBsel (
  VOID
  )
{
  UINT32                 Data32;
  BSEL_BOOT_SOURCE_TYPE  BootSourceType;

  ProgressPrint ("BSEL is set to ");

  // Decode BSEL from bootinfo Register
  Data32 = MmioRead32 (ALT_SYSMGR_OFST + ALT_SYSMGR_BOOT_OFST);
  switch (ALT_SYSMGR_BOOT_BSEL_GET(Data32))
  {
    case ALT_SYSMGR_BOOT_BSEL_E_NAND_FLSH_1_8V:
    case ALT_SYSMGR_BOOT_BSEL_E_NAND_FLSH_3_0V:
      BootSourceType = BSEL_BOOT_SOURCE_NAND;
      ProgressPrint ("NAND\r\n");
      break;
    case ALT_SYSMGR_BOOT_BSEL_E_SD_MMC_EXTERNAL_TRANSCEIVER_1_8V:
    case ALT_SYSMGR_BOOT_BSEL_E_SD_MMC_INTERNAL_TRANSCEIVER_3_0V:
      BootSourceType = BSEL_BOOT_SOURCE_SDMMC;
      ProgressPrint ("SD/MMC\r\n");
      break;
    case ALT_SYSMGR_BOOT_BSEL_E_QSPI_FLSH_1_8V:
    case ALT_SYSMGR_BOOT_BSEL_E_QSPI_FLSH_3_0V:
      BootSourceType = BSEL_BOOT_SOURCE_QSPI;
      ProgressPrint ("QSPI\r\n");
      break;
    case ALT_SYSMGR_BOOT_BSEL_E_FPGA:
      BootSourceType = BSEL_BOOT_SOURCE_FPGA;
      ProgressPrint ("FPGA\r\n");
      break;
    case ALT_SYSMGR_BOOT_BSEL_E_RSVDX:
      BootSourceType = BSEL_BOOT_SOURCE_RSVD;
      ProgressPrint ("RSVD\r\n");
      break;
    default:
      // Shoud not come here
      ASSERT_PLATFORM_INIT(0);
      BootSourceType = BSEL_BOOT_SOURCE_RSVD;
      break;
  }
  return BootSourceType;
}


PINMUX_BOOT_SOURCE_TYPE
EFIAPI
GetBootSourceTypeViaDedicatedIoMuxSelects (
  VOID
  )
{
  UINT32   Data32;
  UINT32   i;
  BOOLEAN  IsPinmuxSetConsistentlyForThatParticularBootType;
  BOOLEAN  SdMmcPinMuxSupport4BitMode;
  BOOLEAN  SdMmcPinMuxSupport8BitMode;
  BOOLEAN  SdMmcPinCfgSupport4BitMode;
  BOOLEAN  SdMmcPinCfgSupport8BitMode;

  ProgressPrint ("Boot Flash's dedicated IO Pin is Mux to ");

  IsPinmuxSetConsistentlyForThatParticularBootType = TRUE;
  SdMmcPinMuxSupport4BitMode = TRUE;
  SdMmcPinMuxSupport8BitMode = TRUE;
  SdMmcPinCfgSupport4BitMode = TRUE;
  SdMmcPinCfgSupport8BitMode = TRUE;

  Data32 = MmioRead32 (ALT_PINMUX_DCTD_IO_GRP_OFST +
                       ALT_PINMUX_DCTD_IO_4_OFST);

  switch (ALT_PINMUX_DCTD_IO_4_SEL_GET(Data32)) {
    case DCTD_IO_4to9_qspi:
      // Pinmux Consistency check
      // To boot QSPI:
      //    mux[4 to 9]    must be 4
      //    mux[11,16,17]  must be 8
      // QSPI interface signals include both mux select 4 and mux select 8 (qspi.ss1/2/3).
      // mux[4 to 9]
      for (i = 0; i < 6; i++)
      {
        Data32 = MmioRead32 (ALT_PINMUX_DCTD_IO_GRP_OFST +
                             ALT_PINMUX_DCTD_IO_4_OFST + (i * 4));
        if (ALT_PINMUX_DCTD_IO_4_SEL_GET(Data32) != DCTD_IO_4to9_qspi) {
          IsPinmuxSetConsistentlyForThatParticularBootType = FALSE;
        }
      }
      /*---------------------------------------------
      // Comment out because
      // QSPI_SS[1,2,3] is optional signal
      // You may uncomment and customize this code when needed.
      // mux[11,16,17]
      for (i = 7; i < 7; i++)
      {
        if (((i + 4) == 11) ||
            ((i + 4) == 16) ||
            ((i + 4) == 17))
        {
          Data32 = MmioRead32 (ALT_PINMUX_DCTD_IO_GRP_OFST +
                               ALT_PINMUX_DCTD_IO_4_OFST + (i * 4));
          if (ALT_PINMUX_DCTD_IO_4_SEL_GET(Data32) != DCTD_IO_11_16_17_qspi) {
            IsPinmuxSetConsistentlyForThatParticularBootType = FALSE;
            ProgressPrint ("\r\nBoot Source Mux Select io_%d should be for QSPI!\r\n", (i + 4));
          }
        }
      }
      ---------------------------------------------*/
      ASSERT_PLATFORM_INIT(IsPinmuxSetConsistentlyForThatParticularBootType == TRUE);
      ProgressPrint ("QSPI\r\n");
      return PINMUX_BOOT_SOURCE_QSPI;
      // break; //statement is unreachable
    case DCTD_IO_sdmmc:
      // Pinmux Consistency check
      // To boot SD/MMC:
      //    mux[4 to 15, except 10 & 11] must be 8
      for (i = 0; i < 12; i++)
      {
        if (((i + 4) != 10) && ((i + 4) != 11))
        {
          Data32 = MmioRead32 (ALT_PINMUX_DCTD_IO_GRP_OFST +
                               ALT_PINMUX_DCTD_IO_4_OFST + (i * 4));
          if (ALT_PINMUX_DCTD_IO_4_SEL_GET(Data32) != DCTD_IO_sdmmc) {
            if (((i + 4) == 7) || ((i + 4) == 8) || ((i + 4) == 9)) {
              SdMmcPinMuxSupport4BitMode = FALSE;
            } if (((i + 4) == 12) || ((i + 4) == 13) || ((i + 4) == 14) || ((i + 4) == 15)) {
              SdMmcPinMuxSupport8BitMode = FALSE;
            } else {
              IsPinmuxSetConsistentlyForThatParticularBootType = FALSE;
              ProgressPrint ("\r\nBoot Source Mux Select io_%d should be for SDMMC!\r\n", (i + 4));
            }
          }
        }
      }
      ProgressPrint ("1");
      if (SdMmcPinMuxSupport4BitMode == TRUE)
      {
        // Check weak pull up is disabled for sdmmc.data1, sdmmc.data2, sdmmc.data3
        for (i = 0; i < 3; i++)
        {
            Data32 = MmioRead32 (ALT_PINMUX_DCTD_IO_GRP_OFST +
                                 ALT_PINMUX_DCTD_IO_CFG_7_OFST + (i * 4));
            if (ALT_PINMUX_DCTD_IO_CFG_7_WK_PU_EN_GET(Data32) == 1) {
              SdMmcPinCfgSupport4BitMode = FALSE;
            }
        }
        if (SdMmcPinCfgSupport4BitMode == TRUE)
        {
          ProgressPrint (",4");
        }
      }
      if ((SdMmcPinMuxSupport4BitMode == TRUE) && (SdMmcPinMuxSupport8BitMode == TRUE))
      {
        // Check weak pull up is disabled for sdmmc.data4, sdmmc.data5, sdmmc.data6, sdmmc.data7
        for (i = 0; i < 4; i++)
        {
            Data32 = MmioRead32 (ALT_PINMUX_DCTD_IO_GRP_OFST +
                                 ALT_PINMUX_DCTD_IO_CFG_12_OFST + (i * 4));
            if (ALT_PINMUX_DCTD_IO_CFG_12_WK_PU_EN_GET(Data32) == 1) {
              SdMmcPinCfgSupport8BitMode = FALSE;
            }
        }
        if (SdMmcPinCfgSupport8BitMode == TRUE)
        {
          ProgressPrint (",8");
        }
      }
      ProgressPrint (" bit bus mode SD/MMC\r\n");
      ASSERT_PLATFORM_INIT(IsPinmuxSetConsistentlyForThatParticularBootType == TRUE);
      return PINMUX_BOOT_SOURCE_SDMMC;
      // break; //statement is unreachable
    case DCTD_IO_nand:
      // Pinmux Consistency check
      // To boot NAND:
      //    mux[4 to 17] must be 14
      for (i = 0; i < 14; i++)
      {
        Data32 = MmioRead32 (ALT_PINMUX_DCTD_IO_GRP_OFST +
                             ALT_PINMUX_DCTD_IO_4_OFST + (i * 4));
        if (ALT_PINMUX_DCTD_IO_4_SEL_GET(Data32) != DCTD_IO_nand) {
          IsPinmuxSetConsistentlyForThatParticularBootType = FALSE;
          ProgressPrint ("\r\nBoot Source Mux Select io_%d should be for NAND!\r\n", (i + 4));
        }
      }
      ProgressPrint ("NAND\r\n");
      ASSERT_PLATFORM_INIT(IsPinmuxSetConsistentlyForThatParticularBootType == TRUE);
      return PINMUX_BOOT_SOURCE_NAND;
      // break; //statement is unreachable
    default:
      // Others
      ProgressPrint ("None Flash device\r\n");
      return PINMUX_BOOT_SOURCE_UNKNOWN;
      // break; //statement is unreachable
  }
}


VOID
EFIAPI
ErrorHandlerBselIsNandButDtbPinMuxIsNotForNand (
  VOID
  )
{
#if (FixedPcdGet32(PcdIsAlteraSoCFPGADevelopmentBoards) == 1)
  // This is Altera Soc FPGA Development Board
  // Override Pin Mux settings per Daughter Card

  // Delay long enough to annoy the developer
  // so that they know they build the wrong DTB
  // then only we override the pin mux for user's convenience
  ProgressPrint ("\r\nPLEASE TAKE NOTE:\r\n");
  ProgressPrint ("BSEL is NAND but the DTB build in this firmware is not for NAND\r\n");
  TryToGrabUsersAttentionOnThisMessage ();

  ProgressPrint ("\r\nOverriding Pin Mux setting to Boot NAND Flash Daughter Card\r\n\r\n");
  // Boot NAND Flash Daughter Card
  // FDT:dedicated
  MmioWrite32(ALT_PINMUX_DCTD_IO_GRP_OFST + ALT_PINMUX_DCTD_IO_4_OFST,  0x0000000e); //nand.adq0
  MmioWrite32(ALT_PINMUX_DCTD_IO_GRP_OFST + ALT_PINMUX_DCTD_IO_5_OFST,  0x0000000e); //nand.adq1
  MmioWrite32(ALT_PINMUX_DCTD_IO_GRP_OFST + ALT_PINMUX_DCTD_IO_6_OFST,  0x0000000e); //nand.we_n
  MmioWrite32(ALT_PINMUX_DCTD_IO_GRP_OFST + ALT_PINMUX_DCTD_IO_7_OFST,  0x0000000e); //nand.re_n
  MmioWrite32(ALT_PINMUX_DCTD_IO_GRP_OFST + ALT_PINMUX_DCTD_IO_8_OFST,  0x0000000e); //nand.adq2
  MmioWrite32(ALT_PINMUX_DCTD_IO_GRP_OFST + ALT_PINMUX_DCTD_IO_9_OFST,  0x0000000e); //nand.adq3
  MmioWrite32(ALT_PINMUX_DCTD_IO_GRP_OFST + ALT_PINMUX_DCTD_IO_10_OFST, 0x0000000e); //nand.cle
  MmioWrite32(ALT_PINMUX_DCTD_IO_GRP_OFST + ALT_PINMUX_DCTD_IO_11_OFST, 0x0000000e); //nand.ale
  MmioWrite32(ALT_PINMUX_DCTD_IO_GRP_OFST + ALT_PINMUX_DCTD_IO_12_OFST, 0x0000000e); //nand.rb
  MmioWrite32(ALT_PINMUX_DCTD_IO_GRP_OFST + ALT_PINMUX_DCTD_IO_13_OFST, 0x0000000e); //nand.ce_n
  MmioWrite32(ALT_PINMUX_DCTD_IO_GRP_OFST + ALT_PINMUX_DCTD_IO_14_OFST, 0x0000000e); //nand.adq4
  MmioWrite32(ALT_PINMUX_DCTD_IO_GRP_OFST + ALT_PINMUX_DCTD_IO_15_OFST, 0x0000000e); //nand.adq5
  MmioWrite32(ALT_PINMUX_DCTD_IO_GRP_OFST + ALT_PINMUX_DCTD_IO_16_OFST, 0x0000000e); //nand.adq6
  MmioWrite32(ALT_PINMUX_DCTD_IO_GRP_OFST + ALT_PINMUX_DCTD_IO_17_OFST, 0x0000000e); //nand.adq7
  // FDT:dedicated_cfg
  MmioWrite32(ALT_PINMUX_DCTD_IO_GRP_OFST + ALT_PINMUX_DCTD_IO_CFG_4_OFST,  0x000a0304); //nand.adq0
  MmioWrite32(ALT_PINMUX_DCTD_IO_GRP_OFST + ALT_PINMUX_DCTD_IO_CFG_5_OFST,  0x000a0304); //nand.adq1
  MmioWrite32(ALT_PINMUX_DCTD_IO_GRP_OFST + ALT_PINMUX_DCTD_IO_CFG_6_OFST,  0x00090304); //nand.we_n
  MmioWrite32(ALT_PINMUX_DCTD_IO_GRP_OFST + ALT_PINMUX_DCTD_IO_CFG_7_OFST,  0x00090304); //nand.re_n
  MmioWrite32(ALT_PINMUX_DCTD_IO_GRP_OFST + ALT_PINMUX_DCTD_IO_CFG_8_OFST,  0x000a0304); //nand.adq2
  MmioWrite32(ALT_PINMUX_DCTD_IO_GRP_OFST + ALT_PINMUX_DCTD_IO_CFG_9_OFST,  0x000a0304); //nand.adq3
  MmioWrite32(ALT_PINMUX_DCTD_IO_GRP_OFST + ALT_PINMUX_DCTD_IO_CFG_10_OFST, 0x00080304); //nand.cle
  MmioWrite32(ALT_PINMUX_DCTD_IO_GRP_OFST + ALT_PINMUX_DCTD_IO_CFG_11_OFST, 0x00080304); //nand.ale
  MmioWrite32(ALT_PINMUX_DCTD_IO_GRP_OFST + ALT_PINMUX_DCTD_IO_CFG_12_OFST, 0x000a0008); //nand.rb
  MmioWrite32(ALT_PINMUX_DCTD_IO_GRP_OFST + ALT_PINMUX_DCTD_IO_CFG_13_OFST, 0x00090304); //nand.ce_n
  MmioWrite32(ALT_PINMUX_DCTD_IO_GRP_OFST + ALT_PINMUX_DCTD_IO_CFG_14_OFST, 0x000a0304); //nand.adq4
  MmioWrite32(ALT_PINMUX_DCTD_IO_GRP_OFST + ALT_PINMUX_DCTD_IO_CFG_15_OFST, 0x000a0304); //nand.adq5
  MmioWrite32(ALT_PINMUX_DCTD_IO_GRP_OFST + ALT_PINMUX_DCTD_IO_CFG_16_OFST, 0x000a0304); //nand.adq6
  MmioWrite32(ALT_PINMUX_DCTD_IO_GRP_OFST + ALT_PINMUX_DCTD_IO_CFG_17_OFST, 0x000a0304); //nand.adq7
#else
  // This is other Board without Boot Flash Daughter Card slot
  ASSERT_PLATFORM_INIT(0);
#endif
}


VOID
EFIAPI
ErrorHandlerBselIsSdMmcButDtbPinMuxIsNotForSdMmc (
  VOID
  )
{
#if (FixedPcdGet32(PcdIsAlteraSoCFPGADevelopmentBoards) == 1)
  // This is Altera Soc FPGA Development Board
  // Override Pin Mux settings per Daughter Card

  ProgressPrint ("\r\nPLEASE TAKE NOTE:\r\n");
  ProgressPrint ("BSEL is SDMMC but the DTB build in this firmware is not for SDMMC\r\n");
  TryToGrabUsersAttentionOnThisMessage ();

  ProgressPrint ("\r\nOverriding Pin Mux setting to Boot MicroSD Flash Daughter Card\r\n\r\n");
  // Boot MicroSD Flash Daughter Card
  // FDT:dedicated
  MmioWrite32(ALT_PINMUX_DCTD_IO_GRP_OFST + ALT_PINMUX_DCTD_IO_4_OFST,  0x00000008); //sdmmc.data0
  MmioWrite32(ALT_PINMUX_DCTD_IO_GRP_OFST + ALT_PINMUX_DCTD_IO_5_OFST,  0x00000008); //sdmmc.cmd
  MmioWrite32(ALT_PINMUX_DCTD_IO_GRP_OFST + ALT_PINMUX_DCTD_IO_6_OFST,  0x00000008); //sdmmc.cclk
  MmioWrite32(ALT_PINMUX_DCTD_IO_GRP_OFST + ALT_PINMUX_DCTD_IO_7_OFST,  0x00000008); //sdmmc.data1
  MmioWrite32(ALT_PINMUX_DCTD_IO_GRP_OFST + ALT_PINMUX_DCTD_IO_8_OFST,  0x00000008); //sdmmc.data2
  MmioWrite32(ALT_PINMUX_DCTD_IO_GRP_OFST + ALT_PINMUX_DCTD_IO_9_OFST,  0x00000008); //sdmmc.data3
  MmioWrite32(ALT_PINMUX_DCTD_IO_GRP_OFST + ALT_PINMUX_DCTD_IO_10_OFST, 0x00000008); //sdmmc.pwr_ena
  MmioWrite32(ALT_PINMUX_DCTD_IO_GRP_OFST + ALT_PINMUX_DCTD_IO_12_OFST, 0x00000008); //sdmmc.data4
  MmioWrite32(ALT_PINMUX_DCTD_IO_GRP_OFST + ALT_PINMUX_DCTD_IO_13_OFST, 0x00000008); //sdmmc.data5
  MmioWrite32(ALT_PINMUX_DCTD_IO_GRP_OFST + ALT_PINMUX_DCTD_IO_14_OFST, 0x00000008); //sdmmc.data6
  MmioWrite32(ALT_PINMUX_DCTD_IO_GRP_OFST + ALT_PINMUX_DCTD_IO_15_OFST, 0x00000008); //sdmmc.data7
  // FDT:dedicated_cfg
  MmioWrite32(ALT_PINMUX_DCTD_IO_GRP_OFST + ALT_PINMUX_DCTD_IO_CFG_4_OFST,  0x000a0304); //sdmmc.data0
  MmioWrite32(ALT_PINMUX_DCTD_IO_GRP_OFST + ALT_PINMUX_DCTD_IO_CFG_5_OFST,  0x000a0304); //sdmmc.cmd
  MmioWrite32(ALT_PINMUX_DCTD_IO_GRP_OFST + ALT_PINMUX_DCTD_IO_CFG_6_OFST,  0x000a0304); //sdmmc.cclk
  MmioWrite32(ALT_PINMUX_DCTD_IO_GRP_OFST + ALT_PINMUX_DCTD_IO_CFG_7_OFST,  0x000a0304); //sdmmc.data1
  MmioWrite32(ALT_PINMUX_DCTD_IO_GRP_OFST + ALT_PINMUX_DCTD_IO_CFG_8_OFST,  0x000a0304); //sdmmc.data2
  MmioWrite32(ALT_PINMUX_DCTD_IO_GRP_OFST + ALT_PINMUX_DCTD_IO_CFG_9_OFST,  0x000a0304); //sdmmc.data3
  MmioWrite32(ALT_PINMUX_DCTD_IO_GRP_OFST + ALT_PINMUX_DCTD_IO_CFG_10_OFST, 0x00090304); //sdmmc.pwr_ena
  MmioWrite32(ALT_PINMUX_DCTD_IO_GRP_OFST + ALT_PINMUX_DCTD_IO_CFG_12_OFST, 0x000b0304); //sdmmc.data4
  MmioWrite32(ALT_PINMUX_DCTD_IO_GRP_OFST + ALT_PINMUX_DCTD_IO_CFG_13_OFST, 0x000b0304); //sdmmc.data5
  MmioWrite32(ALT_PINMUX_DCTD_IO_GRP_OFST + ALT_PINMUX_DCTD_IO_CFG_14_OFST, 0x000b0304); //sdmmc.data6
  MmioWrite32(ALT_PINMUX_DCTD_IO_GRP_OFST + ALT_PINMUX_DCTD_IO_CFG_15_OFST, 0x000b0304); //sdmmc.data7

#else
  // This is other Board without Boot Flash Daughter Card slot
  ASSERT_PLATFORM_INIT(0);
#endif
}


VOID
EFIAPI
ErrorHandlerBselIsQspiButDtbPinMuxIsNotForQspi (
  VOID
  )
{
#if (FixedPcdGet32(PcdIsAlteraSoCFPGADevelopmentBoards) == 1)
  // This is Altera Soc FPGA Development Board
  // Override Pin Mux settings per Daughter Card

  // Delay long enough to annoy the developer
  // so that they know they build the wrong DTB
  // then only we override the pin mux for user's convenience
  ProgressPrint ("\r\nPLEASE TAKE NOTE:\r\n");
  ProgressPrint ("BSEL is QSPI but the DTB build in this firmware is not for QSPI\r\n");
  TryToGrabUsersAttentionOnThisMessage ();

  ProgressPrint ("\r\nOverriding Pin Mux setting to Boot QSPI Flash Daughter Card\r\n\r\n");
  // Boot QSPI Flash Daughter Card
  // FDT:dedicated
  MmioWrite32(ALT_PINMUX_DCTD_IO_GRP_OFST + ALT_PINMUX_DCTD_IO_4_OFST,  0x00000004); //qspi.clk
  MmioWrite32(ALT_PINMUX_DCTD_IO_GRP_OFST + ALT_PINMUX_DCTD_IO_5_OFST,  0x00000004); //qspi.io0
  MmioWrite32(ALT_PINMUX_DCTD_IO_GRP_OFST + ALT_PINMUX_DCTD_IO_6_OFST,  0x00000004); //qspi.ss0
  MmioWrite32(ALT_PINMUX_DCTD_IO_GRP_OFST + ALT_PINMUX_DCTD_IO_7_OFST,  0x00000004); //qspi.io1
  MmioWrite32(ALT_PINMUX_DCTD_IO_GRP_OFST + ALT_PINMUX_DCTD_IO_8_OFST,  0x00000004); //qspi.io2_wpn
  MmioWrite32(ALT_PINMUX_DCTD_IO_GRP_OFST + ALT_PINMUX_DCTD_IO_9_OFST,  0x00000004); //qspi.io3_hold
  MmioWrite32(ALT_PINMUX_DCTD_IO_GRP_OFST + ALT_PINMUX_DCTD_IO_11_OFST, 0x00000008); //qspi.ss1
  // FDT:dedicated_cfg
  MmioWrite32(ALT_PINMUX_DCTD_IO_GRP_OFST + ALT_PINMUX_DCTD_IO_CFG_4_OFST,  0x00080304); //qspi.clk
  MmioWrite32(ALT_PINMUX_DCTD_IO_GRP_OFST + ALT_PINMUX_DCTD_IO_CFG_5_OFST,  0x000a0304); //qspi.io0
  MmioWrite32(ALT_PINMUX_DCTD_IO_GRP_OFST + ALT_PINMUX_DCTD_IO_CFG_6_OFST,  0x00090304); //qspi.ss0
  MmioWrite32(ALT_PINMUX_DCTD_IO_GRP_OFST + ALT_PINMUX_DCTD_IO_CFG_7_OFST,  0x000a0304); //qspi.io1
  MmioWrite32(ALT_PINMUX_DCTD_IO_GRP_OFST + ALT_PINMUX_DCTD_IO_CFG_8_OFST,  0x000a0304); //qspi.io2_wpn
  MmioWrite32(ALT_PINMUX_DCTD_IO_GRP_OFST + ALT_PINMUX_DCTD_IO_CFG_9_OFST,  0x000a0304); //qspi.io3_hold
  MmioWrite32(ALT_PINMUX_DCTD_IO_GRP_OFST + ALT_PINMUX_DCTD_IO_CFG_11_OFST, 0x00090304); //qspi.ss1

#else
  // This is other Board without Boot Flash Daughter Card slot
  ASSERT_PLATFORM_INIT(0);
#endif
}


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

#include <AlteraPlatform.h>
#include <Library/BaseMemoryLib.h>
#include <Library/DebugLib.h>
#include <Library/IoLib.h>
#include <Library/MemoryAllocationLib.h>
#include <Library/PcdLib.h>
#include <Library/PrintLib.h>
#include <Library/SerialPortPrintLib.h>
#include <Library/TimerLib.h>
#include "NandLib.h"
#include "Assert.h"

#if (FixedPcdGet32(PcdDebugMsg_Nand) == 0)
  #define ProgressPrint(FormatString, ...)    /* do nothing */
  #define InfoPrint(FormatString, ...)        /* do nothing */
  #define MmioHexDump(BaseAddr, Data32Size)   /* do nothing */
#else
  #define ProgressPrint SerialPortPrint
  #define InfoPrint     SerialPortPrint
  #define MmioHexDump SerialPortMmioHexDump
#endif

NAND_FLASH_CHARACTERISTIC mFlash;

//
// Lookup table for ALT_NAND_STAT_INTR_STATUS[n]_OFST
//
UINT32 mLUT_ALT_NAND_STAT_INTR_STATUSn_OFST[] = {
  ALT_HPS_NAND_STATUS_OFST + ALT_NAND_STAT_INTR_STATUS0_OFST,
  ALT_HPS_NAND_STATUS_OFST + ALT_NAND_STAT_INTR_STATUS1_OFST,
  ALT_HPS_NAND_STATUS_OFST + ALT_NAND_STAT_INTR_STATUS2_OFST,
  ALT_HPS_NAND_STATUS_OFST + ALT_NAND_STAT_INTR_STATUS3_OFST
};

//
UINT32 mLUT_ALT_NAND_STAT_INTR_ENn_OFST[] = {
  ALT_HPS_NAND_STATUS_OFST + ALT_NAND_STAT_INTR_EN0_OFST,
  ALT_HPS_NAND_STATUS_OFST + ALT_NAND_STAT_INTR_EN1_OFST,
  ALT_HPS_NAND_STATUS_OFST + ALT_NAND_STAT_INTR_EN2_OFST,
  ALT_HPS_NAND_STATUS_OFST + ALT_NAND_STAT_INTR_EN3_OFST
};
// Lookup table for ALT_NAND_CFG_DEVICE_RESET_BANK[n]_SET_MSK
//
UINT32 mLUT_ALT_NAND_CFG_DEVICE_RESET_BANKn_SET_MSK[] = {
  ALT_NAND_CFG_DEVICE_RESET_BANK0_SET_MSK,
  ALT_NAND_CFG_DEVICE_RESET_BANK1_SET_MSK,
  ALT_NAND_CFG_DEVICE_RESET_BANK2_SET_MSK,
  ALT_NAND_CFG_DEVICE_RESET_BANK3_SET_MSK
};

//
// Lookup table for ALT_NAND_CFG_RB_PIN_ENABLED_BANK[n]_CLR_MSK
//
UINT32 mLUT_ALT_NAND_CFG_RB_PIN_ENABLED_BANKn_CLR_MSK[] = {
  ALT_NAND_CFG_RB_PIN_ENABLED_BANK0_CLR_MSK,
  ALT_NAND_CFG_RB_PIN_ENABLED_BANK1_CLR_MSK,
  ALT_NAND_CFG_RB_PIN_ENABLED_BANK2_CLR_MSK,
  ALT_NAND_CFG_RB_PIN_ENABLED_BANK3_CLR_MSK
};

EFI_STATUS
EFIAPI
NandInit (
  VOID
  )
{
  UINTN  bootstrap_noinit;
  UINTN  bootstrap_noloadb0p0;
  UINTN  bootstrap_tworowaddr;
  UINTN  bootstrap_page512;
  UINTN  bootstrap_page512_x16;
  UINTN  Bank;
  UINT32 InterruptMask;

  ProgressPrint ("Initializing NAND\r\n");

  // Hold Nand Controller in Reset
  MmioOr32 (
    ALT_RSTMGR_OFST +
    ALT_RSTMGR_PER0MODRST_OFST,
    ALT_RSTMGR_PER0MODRST_NAND_SET_MSK
    );

  // Setup Nand Bootstrap Control Register
  // These Bootstrap fields are sampled by NAND Flash Controller when released from reset.

  // Bootstrap Inhibit Initialization
  // If 1, inhibits NAND Flash Controller from performing initialization when coming
  // out of reset. Instead, software must program all registers pertaining to device
  // parameters like page size, width, etc.
  bootstrap_noinit = 0;

  // Bootstrap Inhibit Load Block 0 Page 0
  // If 1, inhibits NAND Flash Controller from loading page 0 of block 0 of the NAND
  // device as part of the initialization procedure.
  bootstrap_noloadb0p0 = 0;

  // Bootstrap Two Row Address Cycles
  // If 1, NAND device requires only 2 row address cycles instead of the normal 3 row
  // address cycles.
  bootstrap_tworowaddr = 0;

  // Bootstrap 512 Byte Device
  // If 1, NAND device has a 512 byte page size.
  bootstrap_page512 = 0;

  // Bootstrap 512 Byte and I/O width 16 bits Device
  // If 1, NAND device has 512 bytes page size and I/O width is 16 bits.
  bootstrap_page512_x16 = 0;

  // Set Nand Bootstrap Control Register
  MmioAndThenOr32 (
    ALT_SYSMGR_CORE_OFST +
    ALT_SYSMGR_CORE_NAND_BOOTSTRAP_OFST,
    ALT_SYSMGR_CORE_NAND_BOOTSTRAP_NOINIT_CLR_MSK &
    ALT_SYSMGR_CORE_NAND_BOOTSTRAP_NOLOADB0P0_CLR_MSK &
    ALT_SYSMGR_CORE_NAND_BOOTSTRAP_TWOROWADDR_CLR_MSK &
    ALT_SYSMGR_CORE_NAND_BOOTSTRAP_PAGE512_CLR_MSK &
    ALT_SYSMGR_CORE_NAND_BOOTSTRAP_PAGE512_X16_CLR_MSK,
    ALT_SYSMGR_CORE_NAND_BOOTSTRAP_NOINIT_SET(bootstrap_noinit) |
    ALT_SYSMGR_CORE_NAND_BOOTSTRAP_NOLOADB0P0_SET(bootstrap_noloadb0p0) |
    ALT_SYSMGR_CORE_NAND_BOOTSTRAP_TWOROWADDR_SET(bootstrap_tworowaddr) |
    ALT_SYSMGR_CORE_NAND_BOOTSTRAP_PAGE512_SET(bootstrap_page512) |
    ALT_SYSMGR_CORE_NAND_BOOTSTRAP_PAGE512_X16_SET(bootstrap_page512_x16)
    );

  // Release Nand Controller from Reset
  MmioAnd32 (
    ALT_RSTMGR_OFST +
    ALT_RSTMGR_PER0MODRST_OFST,
    ALT_RSTMGR_PER0MODRST_NAND_CLR_MSK
    );

  // NAND Controller to sends a RESET command to device at each bank.
  // Reset Bank 0
  for (Bank = 0; Bank < NAND_NUMBER_OF_BANK; Bank++)
  {
    NandResetBank (Bank);
  }

  // Set ECC correction register
  MmioAndThenOr32 (
    ALT_HPS_NAND_CFG_OFST +
    ALT_NAND_CFG_ECC_CORRECTION_OFST,
    ALT_NAND_CFG_ECC_CORRECTION_VALUE_CLR_MSK &
    ALT_NAND_CFG_ECC_CORRECTION_ERASE_THRESHOLD_CLR_MSK,
    ALT_NAND_CFG_ECC_CORRECTION_VALUE_SET(ALT_NAND_CFG_ECC_CORRECTION_VALUE_RESET) |
    ALT_NAND_CFG_ECC_CORRECTION_ERASE_THRESHOLD_SET(ALT_NAND_CFG_ECC_CORRECTION_ERASE_THRESHOLD_RESET)
    );

  // ECC related: Set number of bytes to skip from beginning of Spare Area
  MmioWrite32(
    ALT_HPS_NAND_CFG_OFST +
    ALT_NAND_CFG_SPARE_AREA_SKIP_BYTES_OFST,
    NAND_SPARE_AREA_SKIP_BYTES
    );

  // Enables controller ECC capabilities
  MmioOr32(
    ALT_HPS_NAND_CFG_OFST +
    ALT_NAND_CFG_ECC_ENABLE_OFST,
    ALT_NAND_CFG_ECC_ENABLE_FLAG_SET_MSK);

  MmioWrite32(
    ALT_HPS_NAND_CFG_OFST +
    ALT_NAND_CFG_TRANSFER_SPARE_REG_OFST,
    ALT_NAND_CFG_TRANSFER_SPARE_REG_RESET
    );
  MmioWrite32(ALT_HPS_NAND_CFG_OFST +
              ALT_NAND_CFG_RB_PIN_ENABLED_OFST,
              ALT_NAND_CFG_RB_PIN_ENABLED_BANK0_SET_MSK |
              ALT_NAND_CFG_RB_PIN_ENABLED_BANK1_SET_MSK |
              ALT_NAND_CFG_RB_PIN_ENABLED_BANK2_SET_MSK |
              ALT_NAND_CFG_RB_PIN_ENABLED_BANK3_SET_MSK
              );
  MmioWrite32(ALT_HPS_NAND_CFG_OFST +
              ALT_NAND_CFG_CHIP_ENABLE_DONT_CARE_OFST,
              ALT_NAND_CFG_CHIP_ENABLE_DONT_CARE_FLAG_SET_MSK);
  MmioWrite32(ALT_HPS_NAND_CFG_OFST +
              ALT_NAND_CFG_SPARE_AREA_MARKER_OFST,
              ALT_NAND_CFG_SPARE_AREA_MARKER_VALUE_RESET);
  // Read NAND Flash Characteristic
  NandGetFlashInfo ();

  // Set next plane starts Block number In case the device is a multi plane device
  // Note: This can only be set after calling NandGetFlashInfo
  MmioWrite32(
    ALT_HPS_NAND_CFG_OFST +
    ALT_NAND_CFG_FIRST_BLOCK_OF_NEXT_PLANE_OFST,
    mFlash.FirstBlockOfNextPlane
    );

  MmioWrite32 (ALT_HPS_NAND_CFG_OFST +
               ALT_NAND_CFG_GLOBAL_INT_ENABLE_OFST,
               ALT_NAND_CFG_GLOBAL_INT_ENABLE_RESET);

  for (Bank = 0; Bank < NAND_NUMBER_OF_BANK; Bank++) {
    MmioWrite32(mLUT_ALT_NAND_STAT_INTR_STATUSn_OFST[Bank], MAX_UINT32);
  }
  InterruptMask = ALT_NAND_STAT_INTR_EN0_ECC_UNCOR_ERR_SET_MSK |
                  ALT_NAND_STAT_INTR_EN0_DMA_CMD_COMP_SET_MSK  |
                  ALT_NAND_STAT_INTR_EN0_TIME_OUT_SET_MSK      |
                  ALT_NAND_STAT_INTR_EN0_PROGRAM_FAIL_SET_MSK  |
                  ALT_NAND_STAT_INTR_EN0_ERASE_FAIL_SET_MSK    |
                  ALT_NAND_STAT_INTR_EN0_LOAD_COMP_SET_MSK       |
                  ALT_NAND_STAT_INTR_EN0_PROGRAM_COMP_SET_MSK  |
                  ALT_NAND_STAT_INTR_EN0_ERASE_COMP_SET_MSK    |
                  ALT_NAND_STAT_INTR_EN0_LOCKED_BLK_SET_MSK    |
                  ALT_NAND_STAT_INTR_EN0_INT_ACT_SET_MSK       |
                  ALT_NAND_STAT_INTR_EN0_RST_COMP_SET_MSK;
  for (Bank = 0; Bank < NAND_NUMBER_OF_BANK; Bank++) {
    MmioWrite32(mLUT_ALT_NAND_STAT_INTR_ENn_OFST[Bank], InterruptMask);
  }
  return EFI_SUCCESS;
}
VOID
EFIAPI
NandEnableDma (
  IN BOOLEAN Enable
  )
{
  if (Enable == TRUE) {
  // Enables data DMA operation in the controller
  MmioOr32 (
    ALT_HPS_NAND_DMA_OFST +
    ALT_NAND_DMA_DMA_ENABLE_OFST,
    ALT_NAND_DMA_DMA_ENABLE_FLAG_SET_MSK
    );
  } else {
    MmioAnd32 (
      ALT_HPS_NAND_DMA_OFST +
      ALT_NAND_DMA_DMA_ENABLE_OFST,
      ALT_NAND_DMA_DMA_ENABLE_FLAG_CLR_MSK
      );
  }
}


VOID
EFIAPI
NandResetBank (
  IN  UINT32 Bank
  )
{
  UINT32  IntStatReg;
  UINT32  DevResetBankMask;

  InfoPrint ("\t Reset NAND device on bank %d\r\n", Bank);

  // Get the registers offset for this Bank number
  IntStatReg = mLUT_ALT_NAND_STAT_INTR_STATUSn_OFST[Bank];
  DevResetBankMask = mLUT_ALT_NAND_CFG_DEVICE_RESET_BANKn_SET_MSK[Bank];

  // Clear Interrupt status register for this Bank
  MmioWrite32(IntStatReg, MAX_UINT32);

  // Tell NAND controller to sends a RESET command to device on this Bank number
  // NAND Controller will clear the bit after reset command is issued to device.
  MmioOr32(
    ALT_HPS_NAND_CFG_OFST +
    ALT_NAND_CFG_DEVICE_RESET_OFST,
    DevResetBankMask
    );

  // Poll until NAND Controller finished reset and initialization process
  NandPollForIntStat(IntStatReg, ALT_NAND_STAT_INTR_STATUS0_RST_COMP_SET_MSK);

  // Disable Ready/Busy pin for bank 0. We are using polling mode.
  MmioAnd32(
    ALT_HPS_NAND_CFG_OFST +
    ALT_NAND_CFG_RB_PIN_ENABLED_OFST,
    mLUT_ALT_NAND_CFG_RB_PIN_ENABLED_BANKn_CLR_MSK[Bank]
    );

  // Clear Interrupt status register for this Bank number
  MmioWrite32(IntStatReg, MAX_UINT32);

}


UINT32
EFIAPI
NandPollForIntStat (
  IN  UINT32 IntStatReg,
  IN  UINT32 IntStatMask
  )
{
  UINTN  WaitCount;
  UINT32 Data32;
  WaitCount = 0;
  do {
    Data32 = MmioRead32 (IntStatReg);
    // Break if NAND controller responded with the interrupt we are waiting for or error
    if (Data32 & (IntStatMask))
      break;
    MicroSecondDelay (1);
  } while ( (WaitCount++ < NAND_POLL_FOR_INT_STAT_TIMEOUT) );
  if (WaitCount >= NAND_POLL_FOR_INT_STAT_TIMEOUT) {
    InfoPrint ("NAND: Timeout 0x%08X != 0x%08X\r\n", Data32, IntStatMask);
    ASSERT_PLATFORM_INIT(0);
  }

  return Data32;
}


/*
 * Count the consecutive zero bits (trailing) on the right in parallel
 *
 * Some bit fiddleing stuff.
 * From... http://graphics.stanford.edu/~seander/bithacks.html
 */
UINT32
EFIAPI
Ffs32 (
  IN  UINT32 v
  )
{
  UINT32 r = 0;
  do {
    if(v == 0)
      break;

    if(v & 0xFFFF0000){v >>= 16;r |= 16;}
    if(v & 0x0000FF00){v >>=  8;r |=  8;}
    if(v & 0x000000F0){v >>=  4;r |=  4;}
    if(v & 0x0000000C){v >>=  2;r |=  2;}
    if(v & 0x00000002){        ;r |=  1;}
  } while(0);

  return(r);
}


VOID
EFIAPI
NandGetFlashInfo (
  VOID
  )
{
  UINT32 Data32;

  // Read device parameters from NAND Flash
  mFlash.ManufacturerId             = MmioRead32(ALT_HPS_NAND_PARAM_OFST + ALT_NAND_PARAM_MANUFACTURER_ID_OFST);
  mFlash.DeviceId                   = MmioRead32(ALT_HPS_NAND_PARAM_OFST + ALT_NAND_PARAM_DEVICE_ID_OFST);
  mFlash.DeviceParam0               = MmioRead32(ALT_HPS_NAND_PARAM_OFST + ALT_NAND_PARAM_DEVICE_PARAM_0_OFST);
  mFlash.DeviceParam1               = MmioRead32(ALT_HPS_NAND_PARAM_OFST + ALT_NAND_PARAM_DEVICE_PARAM_1_OFST);
  mFlash.DeviceParam2               = MmioRead32(ALT_HPS_NAND_PARAM_OFST + ALT_NAND_PARAM_DEVICE_PARAM_2_OFST);
  mFlash.PageSize                   = MmioRead32(ALT_HPS_NAND_PARAM_OFST + ALT_NAND_PARAM_LOGICAL_PAGE_DATA_SIZE_OFST);
  mFlash.SpareSize                  = MmioRead32(ALT_HPS_NAND_PARAM_OFST + ALT_NAND_PARAM_LOGICAL_PAGE_SPARE_SIZE_OFST);
  mFlash.Revision                   = MmioRead32(ALT_HPS_NAND_PARAM_OFST + ALT_NAND_PARAM_REVISION_OFST);
  mFlash.OnfiDeviceFeatures         = MmioRead32(ALT_HPS_NAND_PARAM_OFST + ALT_NAND_PARAM_ONFI_DEVICE_FEATURES_OFST);
  mFlash.OnfiOptionalCommands       = MmioRead32(ALT_HPS_NAND_PARAM_OFST + ALT_NAND_PARAM_ONFI_OPTIONAL_COMMANDS_OFST);
  mFlash.OnfiTimingMode             = MmioRead32(ALT_HPS_NAND_PARAM_OFST + ALT_NAND_PARAM_ONFI_TIMING_MODE_OFST);
  mFlash.OnfiPgmCacheTimingMode     = MmioRead32(ALT_HPS_NAND_PARAM_OFST + ALT_NAND_PARAM_ONFI_PGM_CACHE_TIMING_MODE_OFST);
  mFlash.OnfiCompliant              = MmioRead32(ALT_HPS_NAND_PARAM_OFST + ALT_NAND_PARAM_ONFI_DEVICE_NO_OF_LUNS_OFST) >> 8;
  mFlash.OnfiDeviceNoOfLuns         = MmioRead32(ALT_HPS_NAND_PARAM_OFST + ALT_NAND_PARAM_ONFI_DEVICE_NO_OF_LUNS_OFST) & 0xff;
  mFlash.OnfiDeviceNoOfBlocksPerLun =(MmioRead32(ALT_HPS_NAND_PARAM_OFST + ALT_NAND_PARAM_ONFI_DEVICE_NO_OF_BLOCKS_PER_LUN_U_OFST) << 16)
                                    + MmioRead32(ALT_HPS_NAND_PARAM_OFST + ALT_NAND_PARAM_ONFI_DEVICE_NO_OF_BLOCKS_PER_LUN_L_OFST);
  mFlash.Features                   = MmioRead32(ALT_HPS_NAND_PARAM_OFST + ALT_NAND_PARAM_FEATURES_OFST);

  // Read NAND Controller Configurations
  mFlash.PagesPerBlock              = MmioRead32(ALT_HPS_NAND_CFG_OFST + ALT_NAND_CFG_PAGES_PER_BLOCK_OFST);
  mFlash.DeviceWidth                = MmioRead32(ALT_HPS_NAND_CFG_OFST + ALT_NAND_CFG_DEVICE_WIDTH_OFST);
  mFlash.SpareAreaSkipBytes         = MmioRead32(ALT_HPS_NAND_CFG_OFST + ALT_NAND_CFG_SPARE_AREA_SKIP_BYTES_OFST);
  Data32                            = MmioRead32(ALT_HPS_NAND_CFG_OFST + ALT_NAND_CFG_NUMBER_OF_PLANES_OFST);
  switch (Data32) {
    case 0:
      mFlash.NumberOfPlanes = 1;
      break;
    case 1:
      mFlash.NumberOfPlanes = 2;
      break;
    case 3:
      mFlash.NumberOfPlanes = 4;
      break;
    case 7:
      mFlash.NumberOfPlanes = 4;
      break;
    default:
      mFlash.NumberOfPlanes = 1;
      break;
  }
  mFlash.FirstBlockOfNextPlane      = mFlash.OnfiDeviceNoOfBlocksPerLun / mFlash.NumberOfPlanes;

  // Derived useful data from available information
  mFlash.BlockSize  = mFlash.PagesPerBlock * mFlash.PageSize;
  mFlash.PageSize32 = mFlash.PageSize / sizeof(UINT32);
  mFlash.PageShift  = Ffs32(mFlash.PageSize);
  mFlash.BlockShift = Ffs32(mFlash.PagesPerBlock);

  InfoPrint("\t Manufacturer ID             : 0x%08x\r\n", mFlash.ManufacturerId);
  InfoPrint("\t Device ID                   : 0x%08x\r\n", mFlash.DeviceId);
  InfoPrint("\t Page Size                   : 0x%08x\r\n", mFlash.PageSize);
  InfoPrint("\t Spare Size                  : 0x%08x\r\n", mFlash.SpareSize);
  InfoPrint("\t Number of pages in a Block  : 0x%08x\r\n", mFlash.PagesPerBlock);
  InfoPrint("\t Block Size                  : 0x%08x\r\n", mFlash.BlockSize);
  InfoPrint("\t Revision                    : 0x%08x\r\n", mFlash.Revision);
  InfoPrint("\t ONFI Device Features        : 0x%08x\r\n", mFlash.OnfiDeviceFeatures);
  InfoPrint("\t ONFI Optional Commands      : 0x%08x\r\n", mFlash.OnfiOptionalCommands);
  InfoPrint("\t ONFI Timing Mode            : 0x%08x\r\n", mFlash.OnfiTimingMode);
  InfoPrint("\t ONFI Pgm Cache Timing Mode  : 0x%08x\r\n", mFlash.OnfiPgmCacheTimingMode);
  InfoPrint("\t ONFI Compliant              : 0x%08x\r\n", mFlash.OnfiCompliant);
  InfoPrint("\t Number of LUN in device     : 0x%08x\r\n", mFlash.OnfiDeviceNoOfLuns);
  InfoPrint("\t Number of Block per LUN     : 0x%08x\r\n", mFlash.OnfiDeviceNoOfBlocksPerLun);
  InfoPrint("\t Number of Planes            : 0x%08x\r\n", mFlash.NumberOfPlanes);
  InfoPrint("\t First Block of Next Plane   : 0x%08x\r\n", mFlash.FirstBlockOfNextPlane);
  InfoPrint("\t Page Shift                  : 0x%08x\r\n", mFlash.PageShift);
  InfoPrint("\t Block Shift                 : 0x%08x\r\n", mFlash.BlockShift);
  InfoPrint("\t DeviceParam0                : 0x%08x\r\n", mFlash.DeviceParam0);
  InfoPrint("\t DeviceParam1                : 0x%08x\r\n", mFlash.DeviceParam1);
  InfoPrint("\t DeviceParam2                : 0x%08x\r\n", mFlash.DeviceParam2);
  InfoPrint("\t Features                    : 0x%08x\r\n", mFlash.Features);
  InfoPrint("\t DeviceWidth                 : 0x%08x\r\n", mFlash.DeviceWidth);
  InfoPrint("\t SpareAreaSkipBytes          : 0x%08x\r\n", mFlash.SpareAreaSkipBytes);
  InfoPrint("\t PageSize32                  : 0x%08x\r\n", mFlash.PageSize32);

}


EFI_STATUS
EFIAPI
NandRead (
  OUT VOID*   Buffer,
  IN  UINT32  Offset,
  IN  UINT32  Size
  )
{
  EFI_STATUS  Status;
  UINT32      Bank;
  UINT32      Page;
  UINT32      Block;
  UINT8*      BufferPtr;
  UINT32      NextReadOffset;
  UINT32      NumberOfBytesLeftToRead;
  UINT32      NumberOfBytesReadInThisLoop;
  UINT8       TempReadBuffer[4096 + 128];
  UINT32      FirstLoopUnAlignedOffsetAdjustment;

  ASSERT_PLATFORM_INIT(mFlash.PageSize <= (4096 + 128));

  Status = EFI_SUCCESS;
  Bank = ALT_NAND_FLASH_MEM_BANK_0;
  BufferPtr = (UINT8*) Buffer;
  NextReadOffset = Offset;
  NumberOfBytesLeftToRead = Size;
  FirstLoopUnAlignedOffsetAdjustment = Offset % mFlash.PageSize;

  do {
    // Read a page of data into temporary buffer
    Block  = NandBlockAddressGet(NextReadOffset);
    Page   = NandPageAddressGet(NextReadOffset);

	InfoPrint("NAND: Read bank %d, block %d, page %d\r\n", Bank, Block, Page);
    Status = NandDmaPageRead (Bank, Block, Page, (UINT32)(UINTN) &TempReadBuffer[0]);
    if (EFI_ERROR(Status)) return Status;

    // Copy the data to caller's buffer
    NumberOfBytesReadInThisLoop = MIN(NumberOfBytesLeftToRead, mFlash.PageSize - FirstLoopUnAlignedOffsetAdjustment);
    CopyMem ((VOID*) BufferPtr, (VOID*) &TempReadBuffer[FirstLoopUnAlignedOffsetAdjustment], NumberOfBytesReadInThisLoop);

    // Setup the source flash offset and destination buffer pointer for next page of data
    NextReadOffset += NumberOfBytesReadInThisLoop;
    BufferPtr += NumberOfBytesReadInThisLoop;
    NumberOfBytesLeftToRead -= NumberOfBytesReadInThisLoop;
    FirstLoopUnAlignedOffsetAdjustment = 0;

  } while (NumberOfBytesLeftToRead);

  return Status;
}


EFI_STATUS
EFIAPI
NandErase (
  IN  UINT32  Offset,
  IN  UINT32  Size
  )
{
  EFI_STATUS  Status;
  UINT32      NextEraseOffset;
  UINT32      NumberOfBytesLeftToErase;
  UINT32      NumberOfBytesErasedInThisLoop;
  UINT32      FirstLoopUnAlignedOffsetAdjustment;
  UINT32      Block;

  Status = EFI_SUCCESS;
  NextEraseOffset = Offset;
  NumberOfBytesLeftToErase = Size;
  FirstLoopUnAlignedOffsetAdjustment = Offset % mFlash.BlockSize;

  do {
    // Erase a block of data
    Block  = NandBlockAddressGet(NextEraseOffset);
    InfoPrint("NAND Erasing Block %d\r\n", Block);
    Status = NandFlashBlockErase (Block);
    if (EFI_ERROR(Status)) return Status;

    // Setup for block of data
    NumberOfBytesErasedInThisLoop = MIN(NumberOfBytesLeftToErase, mFlash.BlockSize - FirstLoopUnAlignedOffsetAdjustment);
    NextEraseOffset += NumberOfBytesErasedInThisLoop;
    NumberOfBytesLeftToErase -= NumberOfBytesErasedInThisLoop;
    FirstLoopUnAlignedOffsetAdjustment = 0;

  } while (NumberOfBytesLeftToErase);

  return Status;
}


EFI_STATUS
EFIAPI
NandWrite (
  OUT VOID*   Buffer,
  IN  UINT32  Offset,
  IN  UINT32  Size
  )
{
  EFI_STATUS  Status;
  UINT32      Bank;
  UINT32      Page;
  UINT32      Block;
  UINT8*      BufferPtr;
  UINT32      NextWriteOffset;
  UINT32      NumberOfBytesLeftToWrite;
  UINT32      NumberOfBytesWriteInThisLoop;
  UINT8       TempReadBuffer[4096 + 128];
  UINT32      FirstLoopUnAlignedOffsetAdjustment;

  ASSERT_PLATFORM_INIT(mFlash.PageSize <= (4096 + 128));

  Status = EFI_SUCCESS;
  Bank = ALT_NAND_FLASH_MEM_BANK_0;
  BufferPtr = (UINT8*) Buffer;
  NextWriteOffset = Offset;
  NumberOfBytesLeftToWrite = Size;
  FirstLoopUnAlignedOffsetAdjustment = Offset % mFlash.PageSize;

  do {
    // Copy the data from caller's buffer into temporary buffer
    SetMem ((VOID*) &TempReadBuffer[0], sizeof(TempReadBuffer), 0xFF);
    NumberOfBytesWriteInThisLoop = MIN(NumberOfBytesLeftToWrite, mFlash.PageSize - FirstLoopUnAlignedOffsetAdjustment);
    CopyMem ((VOID*) &TempReadBuffer[FirstLoopUnAlignedOffsetAdjustment], (VOID*) BufferPtr, NumberOfBytesWriteInThisLoop);

    // Write a page of data into temporary buffer
    Block  = NandBlockAddressGet(NextWriteOffset);
    Page   = NandPageAddressGet(NextWriteOffset);
    InfoPrint("NAND Writing %d bytes to Block %d Page %d \r\n", NumberOfBytesWriteInThisLoop, Block, Page);
    Status = NandDmaPageWrite (Bank, Block, Page, (UINT32)(UINTN) &TempReadBuffer[0]);
    if (EFI_ERROR(Status)) return Status;

    // Setup for next page of data
    NextWriteOffset += NumberOfBytesWriteInThisLoop;
    BufferPtr += NumberOfBytesWriteInThisLoop;
    NumberOfBytesLeftToWrite -= NumberOfBytesWriteInThisLoop;
    FirstLoopUnAlignedOffsetAdjustment = 0;

  } while (NumberOfBytesLeftToWrite);

  return Status;
}

EFI_STATUS
EFIAPI
NandUpdate (
  IN  VOID*   Buffer,
  IN  UINT32  Offset,
  IN  UINT32  Size
  )
{
  EFI_STATUS Status = EFI_SUCCESS;;
  Status = NandErase (Offset, Size);
  if (Status != EFI_SUCCESS) {
    return Status;
  }
  return NandWrite (Buffer, Offset, Size);
}

EFI_STATUS
EFIAPI
NandDmaPageRead (
  IN UINT32  Bank,
  IN UINT32  BlockAddr,
  IN UINT32  PageAddr,
  IN UINT32  MemAddr
  )
{
  EFI_STATUS  Status = EFI_SUCCESS;
  UINT32      IntStat;
  BOOLEAN     IsReadOp = TRUE;
  UINT32      PageCount = 1;
  UINT32      BurstLenInBytes = 64;
  UINT32      IntStatRegOfst = mLUT_ALT_NAND_STAT_INTR_STATUSn_OFST[Bank];

  // enable ECC
  MmioOr32(
    ALT_HPS_NAND_CFG_OFST +
    ALT_NAND_CFG_ECC_ENABLE_OFST,
    ALT_NAND_CFG_ECC_ENABLE_FLAG_SET_MSK);
  // Clear Interrupt status register for this Bank
  MmioWrite32(IntStatRegOfst, MAX_UINT32);
  // enable DMA
  NandEnableDma (TRUE);
  // DMA READ PageCount amount of data from SRC = Flash (Bank, BlockAddr, PageAddr) to DST = MemAddr.
  NandDmaWriteCmdStructure (Bank, BlockAddr, PageAddr, PageCount, MemAddr, IsReadOp, BurstLenInBytes);
  // Poll until DMA command completed or errored
  IntStat = NandPollForIntStat(
              IntStatRegOfst,
              ALT_NAND_STAT_INTR_STATUS0_DMA_CMD_COMP_SET_MSK
              );
  if ( !(IntStat & ALT_NAND_STAT_INTR_STATUS0_DMA_CMD_COMP_SET_MSK) ) {
    InfoPrint("NAND: DMA read error at block 0x%x page 0x%x ! 0x%08X\r\n", BlockAddr, PageAddr, IntStat);
    Status = EFI_DEVICE_ERROR;
  }
  if(IntStat & ALT_NAND_STAT_INTR_STATUS0_ECC_UNCOR_ERR_SET_MSK) {
    InfoPrint("NAND: DMA read ECC uncorrectable error\r\n");
    MicroSecondDelay(100);
    Status = EFI_DEVICE_ERROR;
  }

  // disable DMA
  NandEnableDma (FALSE);
  return Status;
}


EFI_STATUS
EFIAPI
NandDmaPageWrite (
  IN UINT32  Bank,
  IN UINT32  BlockAddr,
  IN UINT32  PageAddr,
  IN UINT32  MemAddr
  )
{
  EFI_STATUS  Status = EFI_SUCCESS;
  UINT32      IntStat;
  BOOLEAN     IsReadOp = FALSE;
  UINT32      PageCount = 1;
  UINT32      BurstLenInBytes = 64;
  UINT32      IntStatRegOfst = mLUT_ALT_NAND_STAT_INTR_STATUSn_OFST[Bank];
  UINT32      Map10CmdAddr;

  // send Map10 command to access main + spare area
  Map10CmdAddr = NandComposeMap10CmdAddr (BlockAddr, PageAddr);
  MmioWrite32(ALT_HPS_NAND_DATA_OFST + ALT_NANDDATA_CTRL_OFST, Map10CmdAddr);
  MmioWrite32(ALT_HPS_NAND_DATA_OFST + ALT_NANDDATA_DATA_OFST, MAP10_CMD_MAIN_AREA_ACCESS);

  // Enables controller ECC capabilities
  MmioOr32(
    ALT_HPS_NAND_CFG_OFST +
    ALT_NAND_CFG_ECC_ENABLE_OFST,
    ALT_NAND_CFG_ECC_ENABLE_FLAG_SET_MSK);

  // disable spare access
  MmioAnd32(
    ALT_HPS_NAND_CFG_OFST +
    ALT_NAND_CFG_TRANSFER_SPARE_REG_OFST,
    ALT_NAND_CFG_TRANSFER_SPARE_REG_FLAG_CLR_MSK
  );

  // enable DMA
  NandEnableDma (TRUE);
  // Clear Interrupt status register for this Bank
  MmioWrite32(IntStatRegOfst, MAX_UINT32);

  // DMA Write PageCount amount of data from SRC = MemAddr to DST = Flash (Bank, BlockAddr, PageAddr)
  NandDmaWriteCmdStructure (Bank, BlockAddr, PageAddr, PageCount, MemAddr, IsReadOp, BurstLenInBytes);

  // Poll until DMA command completed or errored
  IntStat = NandPollForIntStat(
              IntStatRegOfst,
              ALT_NAND_STAT_INTR_STATUS0_DMA_CMD_COMP_SET_MSK );

  if ( !(IntStat & ALT_NAND_STAT_INTR_STATUS0_DMA_CMD_COMP_SET_MSK) ) {
    InfoPrint( "NAND: DMA WRITE ERROR! 0x%08X\r\n", IntStat );
    Status = EFI_DEVICE_ERROR;
  }

  if (IntStat & ALT_NAND_STAT_INTR_STATUS0_LOCKED_BLK_SET_MSK) {
    InfoPrint("NAND: DMA write failed as write to locked block\r\n");
    Status = EFI_DEVICE_ERROR;
  }
  // disable DMA
  NandEnableDma (FALSE);
  return Status;
}


VOID
EFIAPI
NandDmaWriteCmdStructure (
  IN UINT32  Bank,
  IN UINT32  BlockAddr,
  IN UINT32  PageAddr,
  IN UINT32  PageCount,
  IN UINT32  MemAddr,
  IN UINT32  IsReadOp,
  IN UINT32  BurstLen
  )
{
  UINT32  Map10CmdAddr;

  Map10CmdAddr = NandComposeMap10CmdAddr (BlockAddr, PageAddr );

  // Multi-transaction DMA Command Transaction 1 of 4
  // Table 13-10: Command-Data Pair 1
  // Command: 31:28 = 0, 27:26 = 2 - MAP10_CMD, 25:24 = 0, 23:<M> Block address, (<M> – 1):0 Page address
  MmioWrite32(ALT_HPS_NAND_DATA_OFST + ALT_NANDDATA_CTRL_OFST, Map10CmdAddr);
  // Data: 31:16 = 0, 15:12 = 2, 11:8 = 0-Read/1-Write, 7:0 = Number of pages
  MmioWrite32(
    ALT_HPS_NAND_DATA_OFST +
    ALT_NANDDATA_DATA_OFST,
    0x2000 |
    ((IsReadOp ? 0 : 1) << 8) |
    PageCount
    );

  // Multi-transaction DMA Command Transaction 2 of 4
  // Table 13-11: Command-Data Pair 2
  // Command: 31:28 = 0, 27:26 = 2 - MAP10_CMD, 25:24 = 0, 23:8 = Memory address high, 7:0 = 0
  // The buffer address in host memory, must be aligned to 32 bits.
  Map10CmdAddr = MAP10_CMD | ((UINT16)(MemAddr >> 16) << 8);
  MmioWrite32(ALT_HPS_NAND_DATA_OFST + ALT_NANDDATA_CTRL_OFST, Map10CmdAddr);
  // Data: 31:16 = 0, 15:12 = 2, 11:8 = 2, 7:0 = 0
  MmioWrite32(ALT_HPS_NAND_DATA_OFST + ALT_NANDDATA_DATA_OFST, 0x2200);

  // Multi-transaction DMA Command Transaction 3 of 4
  // Table 13-12: Command-Data Pair 3
  // Command: 31:28 = 0, 27:26 = 2 - MAP10_CMD, 25:24 = 0, 23:8 = Memory address low, 7:0 = 0
  Map10CmdAddr = MAP10_CMD | ((UINT16)MemAddr << 8);
  MmioWrite32(ALT_HPS_NAND_DATA_OFST + ALT_NANDDATA_CTRL_OFST, Map10CmdAddr);
  // Data: 31:16 = 0, 15:12 = 2, 11:8 = 3, 7:0 = 0
  MmioWrite32(ALT_HPS_NAND_DATA_OFST + ALT_NANDDATA_DATA_OFST, 0x2300);

  // Multi-transaction DMA Command Transaction 4 of 4
  // Table 13-13: Command-Data Pair 4
  // Command: 31:28 = 0, 27:26 = 2 - MAP10_CMD, 25:24 = 0, 23:17 = 0
  //          16 = INT, 15:8 = Burst length, 7:0 = 0
  Map10CmdAddr = MAP10_CMD |
                 0x10000 |       // Enable INTR_STATUS_DMA_CMD_COMP
                 BurstLen << 8;  // Set Burst length
  MmioWrite32(ALT_HPS_NAND_DATA_OFST + ALT_NANDDATA_CTRL_OFST, Map10CmdAddr);
  // Data: 31:16 = 0, 15:12 = 2, 11:8 = 4, 7:0 = 0
  MmioWrite32(ALT_HPS_NAND_DATA_OFST + ALT_NANDDATA_DATA_OFST, 0x2400);

}


UINT32
EFIAPI
NandComposeMap10CmdAddr (
  IN UINT32 BlockAddr,
  IN UINT32 PageAddr
  )
{
  UINT32 block_addr_mask;
  UINT32 page_addr_mask;
  UINT32 Map10CmdAddr;

  block_addr_mask = (1 << (MAP10_01_CMD_BLK_ADDR_MSB_INDEX - mFlash.BlockShift + 1)) - 1;
  page_addr_mask  = (1 << mFlash.BlockShift) - 1;
  Map10CmdAddr    = MAP10_CMD |
                    ((BlockAddr & block_addr_mask) << mFlash.BlockShift) |
                    (PageAddr  & page_addr_mask);

  return Map10CmdAddr;
}


EFI_STATUS
EFIAPI
NandFlashBlockErase (
  IN  UINT32 BlockAddr
  )
{
  EFI_STATUS  Status;
  UINT32      IntStatRegOfst;
  UINT32      IntStat;
  UINT32      Bank;
  UINT32      PageAddr;
  UINT32      Map10CmdAddr;

  Status = EFI_SUCCESS;
  Bank = ALT_NAND_FLASH_MEM_BANK_0;
  IntStatRegOfst = mLUT_ALT_NAND_STAT_INTR_STATUSn_OFST[Bank];
  PageAddr = 0;
  Map10CmdAddr = NandComposeMap10CmdAddr (BlockAddr, PageAddr);

  // Clear Interrupt status register for this Bank
  MmioWrite32(IntStatRegOfst, MAX_UINT32);

  // Sets block address for erase
  MmioWrite32(
    ALT_HPS_NAND_DATA_OFST +
    ALT_NANDDATA_CTRL_OFST,
    Map10CmdAddr);

  // Initiates block erase operation.
  MmioWrite32(
    ALT_HPS_NAND_DATA_OFST +
    ALT_NANDDATA_DATA_OFST,
    MAP10_CMD_ERASE_BLOCK
    );

  // Poll until DMA command completed or errored
  IntStat = NandPollForIntStat(
              IntStatRegOfst,
              ALT_NAND_STAT_INTR_STATUS0_TIME_OUT_SET_MSK |
              ALT_NAND_STAT_INTR_STATUS0_ERASE_COMP_SET_MSK |
              ALT_NAND_STAT_INTR_STATUS0_ERASE_FAIL_SET_MSK
              );

  if ( !(IntStat & ALT_NAND_STAT_INTR_STATUS0_ERASE_COMP_SET_MSK) ) {
    InfoPrint( "NAND: BLOCK %d ERASE ERROR! 0x%08X\r\n", IntStat, BlockAddr);
    Status = EFI_DEVICE_ERROR;
  }

  return Status;
}


UINT32
EFIAPI
NandComposeMap01CmdAddr (
  IN UINT32 BlockAddr,
  IN UINT32 PageAddr
  )
{
  UINT32 BlockAddrMask;
  UINT32 PageAddrMask;
  UINT32 Map01CmdAddr;

  BlockAddrMask = (1 << (MAP10_01_CMD_BLK_ADDR_MSB_INDEX - mFlash.BlockShift + 1)) - 1;
  PageAddrMask  = (1 << mFlash.BlockShift) - 1;
  Map01CmdAddr    = MAP01_CMD |
                   ((BlockAddr & BlockAddrMask) << mFlash.BlockShift) |
                    (PageAddr  & PageAddrMask);

  return Map01CmdAddr;
}
EFI_STATUS
EFIAPI
NandDmaPageReadRaw (
  IN  UINT32  Bank,
  IN  UINT32  BlockAddr,
  IN  UINT32  PageAddr,
  OUT UINT32  MemAddr
  )
{
  EFI_STATUS  Status = EFI_SUCCESS;
  UINT32      IntStat;
  BOOLEAN     IsReadOp = TRUE;
  UINT32      PageCount = 1;
  UINT32      BurstLenInBytes = 64;
  UINT32      IntStatRegOfst = mLUT_ALT_NAND_STAT_INTR_STATUSn_OFST[Bank];
  UINT32      MainAreaAddr[4096];
  UINT32      SpareAreaAddr[128];
  UINT32      Map10CmdAddr;
  MmioAnd32(
    ALT_HPS_NAND_CFG_OFST +
    ALT_NAND_CFG_ECC_ENABLE_OFST,
    ALT_NAND_CFG_ECC_ENABLE_FLAG_CLR_MSK);
  MmioWrite32(
    ALT_HPS_NAND_CFG_OFST +
    ALT_NAND_CFG_TRANSFER_SPARE_REG_OFST,
    ALT_NAND_CFG_TRANSFER_SPARE_REG_FLAG_SET_MSK
  );
  Map10CmdAddr = NandComposeMap10CmdAddr (BlockAddr, PageAddr);
  MmioWrite32(ALT_HPS_NAND_DATA_OFST + ALT_NANDDATA_CTRL_OFST, Map10CmdAddr);
  MmioWrite32(ALT_HPS_NAND_DATA_OFST + ALT_NANDDATA_DATA_OFST, MAP10_CMD_MAIN_SPARE_AREA_ACCESS);
  NandEnableDma (TRUE);
  MmioWrite32(IntStatRegOfst, MAX_UINT32);
  NandDmaWriteCmdStructure (Bank, BlockAddr, PageAddr, PageCount, MemAddr, IsReadOp, BurstLenInBytes);
  IntStat = NandPollForIntStat(
              IntStatRegOfst,
              ALT_NAND_STAT_INTR_STATUS0_DMA_CMD_COMP_SET_MSK
              );
  if ( !(IntStat & ALT_NAND_STAT_INTR_STATUS0_DMA_CMD_COMP_SET_MSK) ) {
    InfoPrint( "NAND: DMA read raw command cannot be completed with timeout\r\n");
    Status = EFI_DEVICE_ERROR;
  }
  if(IntStat & ALT_NAND_STAT_INTR_STATUS0_ECC_UNCOR_ERR_SET_MSK) {
    InfoPrint("NAND: DMA read ecc uncorrectable error\r\n");
    MicroSecondDelay(100);
    Status = EFI_DEVICE_ERROR;
  }
  NandEnableDma (FALSE);
  CopyMem ((VOID*) (UINTN) MainAreaAddr, (VOID*) (UINTN) MemAddr, mFlash.PageSize);
  CopyMem ((VOID*) (UINTN) SpareAreaAddr, (VOID*) (UINTN) (MemAddr + mFlash.PageSize), mFlash.SpareSize);
  return Status;
}
EFI_STATUS
EFIAPI
NandDmaPageWriteRaw (
  IN UINT32  Bank,
  IN UINT32  BlockAddr,
  IN UINT32  PageAddr,
  IN UINT32  MemAddr
  )
{
  EFI_STATUS  Status = EFI_SUCCESS;
  UINT32      IntStat;
  BOOLEAN     IsReadOp = FALSE;
  UINT32      PageCount = 1;
  UINT32      BurstLenInBytes = 64;
  UINT32      IntStatRegOfst = mLUT_ALT_NAND_STAT_INTR_STATUSn_OFST[Bank];
  UINT32      Map10CmdAddr;
  MmioAnd32(
    ALT_HPS_NAND_CFG_OFST +
    ALT_NAND_CFG_ECC_ENABLE_OFST,
    ALT_NAND_CFG_ECC_ENABLE_FLAG_CLR_MSK);
  Map10CmdAddr = NandComposeMap10CmdAddr (BlockAddr, PageAddr);
  MmioWrite32(ALT_HPS_NAND_DATA_OFST + ALT_NANDDATA_CTRL_OFST, Map10CmdAddr);
  MmioWrite32(ALT_HPS_NAND_DATA_OFST + ALT_NANDDATA_DATA_OFST, MAP10_CMD_MAIN_SPARE_AREA_ACCESS);
  NandEnableDma (TRUE);
  MmioWrite32(IntStatRegOfst, MAX_UINT32);
  NandDmaWriteCmdStructure (Bank, BlockAddr, PageAddr, PageCount, MemAddr, IsReadOp, BurstLenInBytes);
  IntStat = NandPollForIntStat(
              IntStatRegOfst,
              ALT_NAND_STAT_INTR_STATUS0_DMA_CMD_COMP_SET_MSK |
              ALT_NAND_STAT_INTR_STATUS0_PROGRAM_FAIL_SET_MSK |
              ALT_NAND_STAT_INTR_STATUS0_LOCKED_BLK_SET_MSK
              );
  if ( !(IntStat & ALT_NAND_STAT_INTR_STATUS0_DMA_CMD_COMP_SET_MSK) ) {
    InfoPrint( "NAND: DMA write raw command cannot be completed with timeout\r\n");
    Status = EFI_DEVICE_ERROR;
  }
  NandEnableDma (FALSE);
  return Status;
}
EFI_STATUS
EFIAPI
NandReadSpareData(
  OUT UINT32* Data,
  IN  UINT32  BlockAddr,
  IN  UINT32  PageAddr
  )
{
  UINT32      IntStatRegOfst;
  UINT32      IntStat;
  UINT32      Bank;
  UINT32      Map10CmdAddr;
  UINT32      Map01CmdAddr;
  BOOLEAN     IsReadOp = TRUE;
  UINT32      PageCount = 1;
  UINTN       i;
  UINT32*     SpareData;
  Bank = ALT_NAND_FLASH_MEM_BANK_0;
  IntStatRegOfst = mLUT_ALT_NAND_STAT_INTR_STATUSn_OFST[Bank];
  SpareData = Data;
  MmioWrite32(IntStatRegOfst, MAX_UINT32);
  MmioAnd32(
    ALT_HPS_NAND_CFG_OFST +
    ALT_NAND_CFG_ECC_ENABLE_OFST,
    ALT_NAND_CFG_ECC_ENABLE_FLAG_CLR_MSK);
  MmioWrite32(
    ALT_HPS_NAND_CFG_OFST +
    ALT_NAND_CFG_TRANSFER_SPARE_REG_OFST,
    ALT_NAND_CFG_TRANSFER_SPARE_REG_FLAG_SET_MSK
  );
  NandEnableDma (FALSE);
  Map10CmdAddr = NandComposeMap10CmdAddr (BlockAddr, PageAddr);
  MmioWrite32(ALT_HPS_NAND_DATA_OFST + ALT_NANDDATA_CTRL_OFST, Map10CmdAddr);
  MmioWrite32(ALT_HPS_NAND_DATA_OFST + ALT_NANDDATA_DATA_OFST, MAP10_CMD_SPARE_AREA_ACCESS);
  MmioWrite32(ALT_HPS_NAND_DATA_OFST + ALT_NANDDATA_CTRL_OFST, Map10CmdAddr);
  MmioWrite32(ALT_HPS_NAND_DATA_OFST + ALT_NANDDATA_DATA_OFST,
              0x2000 | ((IsReadOp ? 0 : 1) << 8) | PageCount);
  Map01CmdAddr = NandComposeMap01CmdAddr (BlockAddr, PageAddr);
  MmioWrite32(ALT_HPS_NAND_DATA_OFST + ALT_NANDDATA_CTRL_OFST, Map01CmdAddr);
  IntStat = NandPollForIntStat(
              IntStatRegOfst,
              ALT_NAND_STAT_INTR_STATUS0_LOAD_COMP_SET_MSK
              );
  if ( !(IntStat & ALT_NAND_STAT_INTR_STATUS0_LOAD_COMP_SET_MSK) ) {
    InfoPrint( "NAND: Spare read command cannot be completed with timeout\r\n");
    return EFI_DEVICE_ERROR;
  }
  for (i = 0; i < (mFlash.SpareSize / 4); i++)
    *SpareData++ = MmioRead32(ALT_HPS_NAND_DATA_OFST + ALT_NANDDATA_DATA_OFST);
  IntStat = NandPollForIntStat(
              IntStatRegOfst,
              ALT_NAND_STAT_INTR_STATUS0_LOAD_COMP_SET_MSK
              );
  if ( !(IntStat & ALT_NAND_STAT_INTR_STATUS0_LOAD_COMP_SET_MSK) ) {
    InfoPrint( "NAND: Spare read command cannot be completed with timeout\r\n");
    return EFI_DEVICE_ERROR;
  }
  Map10CmdAddr = NandComposeMap10CmdAddr (BlockAddr, PageAddr);
  MmioWrite32(ALT_HPS_NAND_DATA_OFST + ALT_NANDDATA_CTRL_OFST, Map10CmdAddr);
  MmioWrite32(ALT_HPS_NAND_DATA_OFST + ALT_NANDDATA_DATA_OFST, MAP10_CMD_MAIN_AREA_ACCESS);
  return EFI_SUCCESS;
}
EFI_STATUS
EFIAPI
NandWriteSpareData (
  IN  UINT32  BlockAddr,
  IN  UINT32  PageAddr,
  IN  UINT32* Data
  )
{
  UINT32      IntStatRegOfst;
  UINT32      IntStat;
  UINT32      Bank;
  UINT32      Map10CmdAddr;
  UINT32      Map01CmdAddr;
  BOOLEAN     IsReadOp = FALSE;
  UINT32      PageCount = 1;
  UINTN       i;
  UINT32*     SpareData;
  Bank = ALT_NAND_FLASH_MEM_BANK_0;
  IntStatRegOfst = mLUT_ALT_NAND_STAT_INTR_STATUSn_OFST[Bank];
  SpareData = Data;
  MmioWrite32(IntStatRegOfst, MAX_UINT32);
  MmioAnd32(
    ALT_HPS_NAND_CFG_OFST +
    ALT_NAND_CFG_ECC_ENABLE_OFST,
    ALT_NAND_CFG_ECC_ENABLE_FLAG_CLR_MSK);
  MmioWrite32(
    ALT_HPS_NAND_CFG_OFST +
    ALT_NAND_CFG_TRANSFER_SPARE_REG_OFST,
    ALT_NAND_CFG_TRANSFER_SPARE_REG_FLAG_SET_MSK
  );
  NandEnableDma (FALSE);
  Map10CmdAddr = NandComposeMap10CmdAddr (BlockAddr, PageAddr);
  MmioWrite32(ALT_HPS_NAND_DATA_OFST + ALT_NANDDATA_CTRL_OFST, Map10CmdAddr);
  MmioWrite32(ALT_HPS_NAND_DATA_OFST + ALT_NANDDATA_DATA_OFST, MAP10_CMD_SPARE_AREA_ACCESS);
  MmioWrite32(ALT_HPS_NAND_DATA_OFST + ALT_NANDDATA_CTRL_OFST, Map10CmdAddr);
  MmioWrite32(ALT_HPS_NAND_DATA_OFST + ALT_NANDDATA_DATA_OFST,
              0x2000 |((IsReadOp ? 0 : 1) << 8) | PageCount);
  Map01CmdAddr = NandComposeMap01CmdAddr (BlockAddr, PageAddr);
  MmioWrite32(ALT_HPS_NAND_DATA_OFST + ALT_NANDDATA_CTRL_OFST, Map01CmdAddr);
  for (i = 0; i < (mFlash.SpareSize / 4); i++)
    MmioWrite32(ALT_HPS_NAND_DATA_OFST + ALT_NANDDATA_DATA_OFST, *SpareData++);
  IntStat = NandPollForIntStat(
              IntStatRegOfst,
              ALT_NAND_STAT_INTR_STATUS0_LOAD_COMP_SET_MSK
              );
  if ( !(IntStat & ALT_NAND_STAT_INTR_STATUS0_LOAD_COMP_SET_MSK) ) {
    InfoPrint( "NAND: Write spare command cannot be completed with timeout\r\n");
    return EFI_DEVICE_ERROR;
  }
  Map10CmdAddr = NandComposeMap10CmdAddr (BlockAddr, PageAddr);
  MmioWrite32(ALT_HPS_NAND_DATA_OFST + ALT_NANDDATA_CTRL_OFST, Map10CmdAddr);
  MmioWrite32(ALT_HPS_NAND_DATA_OFST + ALT_NANDDATA_DATA_OFST, MAP10_CMD_MAIN_AREA_ACCESS);
  return EFI_SUCCESS;
}
EFI_STATUS
EFIAPI
NandReadRaw (
  OUT VOID*   Buffer,
  IN  UINT32  Offset,
  IN  UINT32  Size
  )
{
  EFI_STATUS  Status;
  UINT32      Bank;
  UINT32      Page;
  UINT32      Block;
  UINT8*      BufferPtr;
  UINT32      NextReadOffset;
  UINT32      NumberOfBytesLeftToRead;
  UINT32      NumberOfBytesReadInThisLoop;
  UINT8       TempReadBuffer[4096 + 128];
  UINT32      FirstLoopUnAlignedOffsetAdjustment;
  ASSERT_PLATFORM_INIT(mFlash.PageSize <= (4096 + 128));
  Status = EFI_SUCCESS;
  Bank = ALT_NAND_FLASH_MEM_BANK_0;
  BufferPtr = (UINT8*) Buffer;
  NextReadOffset = Offset;
  NumberOfBytesLeftToRead = Size;
  FirstLoopUnAlignedOffsetAdjustment = Offset % mFlash.PageSize;
  do {
    Block  = NandBlockAddressGet(NextReadOffset);
    Page   = NandPageAddressGet(NextReadOffset);
    Status = NandDmaPageReadRaw (Bank, Block, Page, (UINT32)(UINTN) &TempReadBuffer[0]);
    if (EFI_ERROR(Status)) return Status;
    NumberOfBytesReadInThisLoop = MIN(NumberOfBytesLeftToRead, mFlash.PageSize + mFlash.SpareSize - FirstLoopUnAlignedOffsetAdjustment);
    CopyMem ((VOID*) BufferPtr, (VOID*) &TempReadBuffer[FirstLoopUnAlignedOffsetAdjustment], NumberOfBytesReadInThisLoop);
    NextReadOffset += NumberOfBytesReadInThisLoop;
    BufferPtr += NumberOfBytesReadInThisLoop;
    NumberOfBytesLeftToRead -= NumberOfBytesReadInThisLoop;
    FirstLoopUnAlignedOffsetAdjustment = 0;
  } while (NumberOfBytesLeftToRead);
  return Status;
}
BOOLEAN
EFIAPI
NandBlockIsBad (
  IN UINT32 BlockAddr
  )
{
  EFI_STATUS Status;
  UINT8     ReadBuffer[128]; // max spare size
  UINT8     FirstByteFirstPageSpare;
  UINT8     FirstByteSecondPageSpare;
  UINT8     FirstByteLastPageSpare;
  Status = NandReadSpareData ((UINT32*)&ReadBuffer[0], BlockAddr, 0);
  if (EFI_ERROR(Status)) {
    InfoPrint("NAND: Error read 1st page oob\r\n");
    return TRUE;
  }
  FirstByteFirstPageSpare = ReadBuffer[0];
  Status = NandReadSpareData ((UINT32*)&ReadBuffer[0], BlockAddr, 1);
  if (EFI_ERROR(Status)) {
    InfoPrint("NAND: Error read 2nd page oob\r\n");
    return TRUE;
  }
  FirstByteSecondPageSpare = ReadBuffer[0];
  Status = NandReadSpareData ((UINT32*)&ReadBuffer[0], BlockAddr, mFlash.PagesPerBlock - 1);
  if (EFI_ERROR(Status)) {
    InfoPrint("NAND: Error read last page oob\r\n");
    return TRUE;
  }
  FirstByteLastPageSpare = ReadBuffer[0];
  if ((FirstByteFirstPageSpare  != 0xFF) ||
      (FirstByteSecondPageSpare != 0xFF) ||
      (FirstByteLastPageSpare   != 0xFF))
    return TRUE;
  return FALSE;
}
EFI_STATUS
EFIAPI
NandEraseSkipBadBlock (
  IN  UINT32  Offset,
  IN  UINT32  Size
  )
{
  EFI_STATUS  Status;
  UINT32      NextEraseOffset;
  UINT32      NumberOfBytesLeftToErase;
  UINT32      NumberOfBytesErasedInThisLoop;
  UINT32      FirstLoopUnAlignedOffsetAdjustment;
  UINT32      Block;
  UINT32      NumberOfBadBlock;
  Status = EFI_SUCCESS;
  NextEraseOffset = Offset;
  NumberOfBytesLeftToErase = Size;
  FirstLoopUnAlignedOffsetAdjustment = Offset % mFlash.BlockSize;
  NumberOfBadBlock = 0;
  do {
    Block  = NandBlockAddressGet(NextEraseOffset);
    if (NandBlockIsBad (Block) == TRUE) {
      InfoPrint ("NAND: Skip erase block %d as it is bad block\r\n", Block);
      NumberOfBadBlock++;
      if ((NumberOfBadBlock > PcdGet32 (PcdNandStopIfMoreThanThisNumberBadBlocks))) {
        InfoPrint("NAND: More than %d block are bad, skip whole process\r\n", PcdGet32 (PcdNandStopIfMoreThanThisNumberBadBlocks));
        return EFI_DEVICE_ERROR;
      }
      goto NextBlock;
    }
    InfoPrint("NAND: Erasing Block %d\r\n", Block);
    Status = NandFlashBlockErase (Block);
    if (EFI_ERROR(Status)) return Status;
  NextBlock:
    NumberOfBytesErasedInThisLoop = MIN(NumberOfBytesLeftToErase, mFlash.BlockSize - FirstLoopUnAlignedOffsetAdjustment);
    NextEraseOffset += NumberOfBytesErasedInThisLoop;
    NumberOfBytesLeftToErase -= NumberOfBytesErasedInThisLoop;
    FirstLoopUnAlignedOffsetAdjustment = 0;
  } while (NumberOfBytesLeftToErase);
  return Status;
}
EFI_STATUS
EFIAPI
NandReadSkipBadBlock (
  OUT VOID*   Buffer,
  IN  UINT32  Offset,
  IN  UINT32  Size
  )
{
  EFI_STATUS  Status;
  UINT32      Bank;
  UINT32      Page;
  UINT32      Block;
  UINT8*      BufferPtr;
  UINT32      NextReadOffset;
  UINT32      NumberOfBytesLeftToRead;
  UINT32      NumberOfBytesReadInThisLoop;
  UINT8       TempReadBuffer[4096 + 128];
  UINT32      FirstLoopUnAlignedOffsetAdjustment;
  UINT32      NumberOfBadBlock;
  ASSERT_PLATFORM_INIT(mFlash.PageSize <= (4096 + 128));
  Status = EFI_SUCCESS;
  Bank = ALT_NAND_FLASH_MEM_BANK_0;
  BufferPtr = (UINT8*) Buffer;
  NextReadOffset = Offset;
  NumberOfBytesLeftToRead = Size;
  FirstLoopUnAlignedOffsetAdjustment = Offset % mFlash.PageSize;
  NumberOfBadBlock = 0;
  do {
    // Read a page of data into temporary buffer
    Block  = NandBlockAddressGet(NextReadOffset);
    Page   = NandPageAddressGet(NextReadOffset);
    // check bad block
    if (NandBlockIsBad (Block) == TRUE) {
      InfoPrint ("NAND: Skip read block %d as it is bad block\r\n", Block);
      NumberOfBadBlock++;
      if ((NumberOfBadBlock > PcdGet32 (PcdNandStopIfMoreThanThisNumberBadBlocks))) {
        InfoPrint("NAND: More than %d block are bad, skip whole process\r\n", PcdGet32 (PcdNandStopIfMoreThanThisNumberBadBlocks));
        return EFI_DEVICE_ERROR;
      }
      // jump to next block
      NextReadOffset = (Block + 1) * mFlash.BlockSize;
      FirstLoopUnAlignedOffsetAdjustment = 0;
      continue;
    }
    Status = NandDmaPageRead (Bank, Block, Page, (UINT32)(UINTN) &TempReadBuffer[0]);
    if (EFI_ERROR(Status)) return Status;

    // Copy the data to caller's buffer
    NumberOfBytesReadInThisLoop = MIN(NumberOfBytesLeftToRead, mFlash.PageSize - FirstLoopUnAlignedOffsetAdjustment);
    CopyMem ((VOID*) BufferPtr, (VOID*) &TempReadBuffer[FirstLoopUnAlignedOffsetAdjustment], NumberOfBytesReadInThisLoop);

   // Setup the source flash offset and destination buffer pointer for next page of data
    NextReadOffset += NumberOfBytesReadInThisLoop;
    BufferPtr += NumberOfBytesReadInThisLoop;
    NumberOfBytesLeftToRead -= NumberOfBytesReadInThisLoop;
    FirstLoopUnAlignedOffsetAdjustment = 0;

  } while (NumberOfBytesLeftToRead);

  return Status;
}


EFI_STATUS
EFIAPI
NandWriteSkipBadBlock (
  OUT VOID*   Buffer,
  IN  UINT32  Offset,
  IN  UINT32  Size
  )
{
  EFI_STATUS  Status;
  UINT32      Bank;
  UINT32      Page;
  UINT32      Block;
  UINT8*      BufferPtr;
  UINT32      NextWriteOffset;
  UINT32      NumberOfBytesLeftToWrite;
  UINT32      NumberOfBytesWriteInThisLoop;
  UINT8       TempReadBuffer[4096 + 128];  // 4096 bytes data + 128 bytes spare for ECC + bad block marker
  UINT32      FirstLoopUnAlignedOffsetAdjustment;
  UINT32      NumberOfBadBlock;

  ASSERT_PLATFORM_INIT(mFlash.PageSize <= (4096 + 128));

  Status = EFI_SUCCESS;
  Bank = ALT_NAND_FLASH_MEM_BANK_0;
  BufferPtr = (UINT8*) Buffer;
  NextWriteOffset = Offset;
  NumberOfBytesLeftToWrite = Size;
  FirstLoopUnAlignedOffsetAdjustment = Offset % mFlash.PageSize;
  NumberOfBadBlock = 0;

  do {
    // Copy the data from caller's buffer into temporary buffer
    SetMem ((VOID*) &TempReadBuffer[0], sizeof(TempReadBuffer), 0xFF);
    NumberOfBytesWriteInThisLoop = MIN(NumberOfBytesLeftToWrite, mFlash.PageSize - FirstLoopUnAlignedOffsetAdjustment);
    CopyMem ((VOID*) &TempReadBuffer[FirstLoopUnAlignedOffsetAdjustment], (VOID*) BufferPtr, NumberOfBytesWriteInThisLoop);

    // Write a page of data into temporary buffer
    Block  = NandBlockAddressGet(NextWriteOffset);
    Page   = NandPageAddressGet(NextWriteOffset);
    // check bad block
    if (NandBlockIsBad (Block) == TRUE) {
      InfoPrint ("NAND: Skip write block %d as it is bad block\r\n", Block);
      NumberOfBadBlock++;
      if ((NumberOfBadBlock > PcdGet32 (PcdNandStopIfMoreThanThisNumberBadBlocks))) {
        InfoPrint("NAND: More than %d block are bad, skip whole process\r\n", PcdGet32 (PcdNandStopIfMoreThanThisNumberBadBlocks));
        return EFI_DEVICE_ERROR;
      }
      NextWriteOffset = (Block + 1) * mFlash.BlockSize;
      FirstLoopUnAlignedOffsetAdjustment = 0;
      continue;
    }
    InfoPrint("NAND: Writing %d bytes to Block %d Page %d or offset 0x%x \r\n", NumberOfBytesWriteInThisLoop, Block, Page, NextWriteOffset);
    Status = NandDmaPageWrite (Bank, Block, Page, (UINT32)(UINTN) &TempReadBuffer[0]);
    if (EFI_ERROR(Status)) return Status;

    // Setup for next page of data
    NextWriteOffset += NumberOfBytesWriteInThisLoop;
    BufferPtr += NumberOfBytesWriteInThisLoop;
    NumberOfBytesLeftToWrite -= NumberOfBytesWriteInThisLoop;
    FirstLoopUnAlignedOffsetAdjustment = 0;

  } while (NumberOfBytesLeftToWrite);

  return Status;
}

EFI_STATUS
EFIAPI
NandUpdateSkipBadBlock (
  IN  VOID*   Buffer,
  IN  UINT32  Offset,
  IN  UINT32  Size
  )
{
  EFI_STATUS Status = EFI_SUCCESS;;
  Status = NandEraseSkipBadBlock (Offset, Size);
  if (Status != EFI_SUCCESS) {
    return Status;
  }
  return NandWriteSkipBadBlock (Buffer, Offset, Size);
}

// scan bad block for whole chip
VOID
EFIAPI
NandScanBadBlockWholeChip (
  VOID
  )
{
  UINT32   Block;
  UINT32   NumberOfBlocks;
  UINT32   Count;

  Count = 0;
  NumberOfBlocks = mFlash.OnfiDeviceNoOfBlocksPerLun * mFlash.OnfiDeviceNoOfLuns;
  for (Block = 0; Block < NumberOfBlocks; Block++) {
    if (NandBlockIsBad (Block) == TRUE) {
      InfoPrint ("NAND: Block %d is bad block\r\n", Block);
      Count++;
    }
  }
  InfoPrint("NAND: Total numbers of bad block : %d blocks\r\n", Count);
}


// pipeline read/write
EFI_STATUS
EFIAPI
NandPipelinePageWrite (
  IN UINT32  Bank,
  IN UINT32  BlockAddr,
  IN UINT32  PageAddr,
  IN UINT32  MemAddr
  )
{
  EFI_STATUS  Status = EFI_SUCCESS;
  UINT32      IntStat;
  BOOLEAN     IsReadOp = FALSE;
  UINT32      PageCount = 1;
  UINT32      IntStatRegOfst = mLUT_ALT_NAND_STAT_INTR_STATUSn_OFST[Bank];
  UINT32      Map10CmdAddr;
  UINT32*     WriteData;
  UINT32      Map01CmdAddr;
  UINTN       i;

  WriteData = (UINT32*)(UINTN)MemAddr;

  // Enables controller ECC capabilities
  MmioOr32(
    ALT_HPS_NAND_CFG_OFST +
    ALT_NAND_CFG_ECC_ENABLE_OFST,
    ALT_NAND_CFG_ECC_ENABLE_FLAG_SET_MSK);

  // disable spare access
  MmioAnd32(
    ALT_HPS_NAND_CFG_OFST +
    ALT_NAND_CFG_TRANSFER_SPARE_REG_OFST,
    ALT_NAND_CFG_TRANSFER_SPARE_REG_FLAG_CLR_MSK
  );

  // disble DMA
  NandEnableDma (FALSE);

  // Clear Interrupt status register for this Bank
  MmioWrite32(IntStatRegOfst, MAX_UINT32);

  // send Map10 command to access main area
  Map10CmdAddr = NandComposeMap10CmdAddr (BlockAddr, PageAddr);
  MmioWrite32(ALT_HPS_NAND_DATA_OFST + ALT_NANDDATA_CTRL_OFST, Map10CmdAddr);
  MmioWrite32(ALT_HPS_NAND_DATA_OFST + ALT_NANDDATA_DATA_OFST, MAP10_CMD_MAIN_AREA_ACCESS);

  // 2. setup the read pipeline command
  // Command: 31:28 = 0, 27:26 = 2 - MAP10_CMD, 25:24 = 0, 23:<M> Block address, (<M> – 1):0 Page address
  MmioWrite32(ALT_HPS_NAND_DATA_OFST + ALT_NANDDATA_CTRL_OFST, Map10CmdAddr);
  // Data: 31:16 = 0, 15:12 = 2, 11:8 = 0-Read/1-Write, 7:0 = Number of pages
  MmioWrite32(ALT_HPS_NAND_DATA_OFST + ALT_NANDDATA_DATA_OFST,
              0x2000 | ((IsReadOp ? 0 : 1) << 8) | PageCount);

  // 3. set up command 01
  Map01CmdAddr = NandComposeMap01CmdAddr (BlockAddr, PageAddr);
  MmioWrite32(ALT_HPS_NAND_DATA_OFST + ALT_NANDDATA_CTRL_OFST, Map01CmdAddr);

  // write data
  for (i = 0; i < (mFlash.PageSize / 4); i++)
    MmioWrite32(ALT_HPS_NAND_DATA_OFST + ALT_NANDDATA_DATA_OFST, *WriteData++);

 // Poll until command completed
  IntStat = NandPollForIntStat(
              IntStatRegOfst,
              ALT_NAND_STAT_INTR_STATUS0_LOAD_COMP_SET_MSK
              );

  if ( !(IntStat & ALT_NAND_STAT_INTR_STATUS0_LOAD_COMP_SET_MSK) ) {
    InfoPrint( "NAND: Pipeline write command cannot be completed with timeout\r\n");
    return EFI_DEVICE_ERROR;
  }

  return Status;
}

EFI_STATUS
EFIAPI
NandPipelinePageRead (
  IN UINT32  Bank,
  IN UINT32  BlockAddr,
  IN UINT32  PageAddr,
  IN UINT32  MemAddr
  )
{
  EFI_STATUS  Status = EFI_SUCCESS;
  UINT32      IntStat;
  BOOLEAN     IsReadOp = TRUE;
  UINT32      PageCount = 1;
  UINT32      IntStatRegOfst = mLUT_ALT_NAND_STAT_INTR_STATUSn_OFST[Bank];
  UINT32      Map10CmdAddr;
  UINT32*     ReadData;
  UINT32      Map01CmdAddr;
  UINTN       i;

  ReadData = (UINT32*)(UINTN) MemAddr;

  // Enables controller ECC capabilities
  MmioOr32(
    ALT_HPS_NAND_CFG_OFST +
    ALT_NAND_CFG_ECC_ENABLE_OFST,
    ALT_NAND_CFG_ECC_ENABLE_FLAG_SET_MSK);

  // disable spare access
  MmioAnd32(
    ALT_HPS_NAND_CFG_OFST +
    ALT_NAND_CFG_TRANSFER_SPARE_REG_OFST,
    ALT_NAND_CFG_TRANSFER_SPARE_REG_FLAG_CLR_MSK
  );

  // disable DMA
  NandEnableDma (FALSE);

  // Clear Interrupt status register for this Bank
  MmioWrite32(IntStatRegOfst, MAX_UINT32);

  // send Map10 command to access main area
  Map10CmdAddr = NandComposeMap10CmdAddr (BlockAddr, PageAddr);
  MmioWrite32(ALT_HPS_NAND_DATA_OFST + ALT_NANDDATA_CTRL_OFST, Map10CmdAddr);
  MmioWrite32(ALT_HPS_NAND_DATA_OFST + ALT_NANDDATA_DATA_OFST, MAP10_CMD_MAIN_AREA_ACCESS);

  // 2. setup the read pipeline command
  // Command: 31:28 = 0, 27:26 = 2 - MAP10_CMD, 25:24 = 0, 23:<M> Block address, (<M> – 1):0 Page address
  MmioWrite32(ALT_HPS_NAND_DATA_OFST + ALT_NANDDATA_CTRL_OFST, Map10CmdAddr);
  // Data: 31:16 = 0, 15:12 = 2, 11:8 = 0-Read/1-Write, 7:0 = Number of pages
  MmioWrite32(ALT_HPS_NAND_DATA_OFST + ALT_NANDDATA_DATA_OFST,
              0x2000 | ((IsReadOp ? 0 : 1) << 8) | PageCount);

  // 3. set up command 01
  Map01CmdAddr = NandComposeMap01CmdAddr (BlockAddr, PageAddr);
  MmioWrite32(ALT_HPS_NAND_DATA_OFST + ALT_NANDDATA_CTRL_OFST, Map01CmdAddr);

 // Poll until command completed
  IntStat = NandPollForIntStat(
              IntStatRegOfst,
              ALT_NAND_STAT_INTR_STATUS0_LOAD_COMP_SET_MSK
              );

  if ( !(IntStat & ALT_NAND_STAT_INTR_STATUS0_LOAD_COMP_SET_MSK) ) {
    InfoPrint( "NAND: Pipeline read command cannot be completed with timeout\r\n");
    return EFI_DEVICE_ERROR;
  }

  // read data
  for (i = 0; i < (mFlash.SpareSize / 4); i++)
    *ReadData++ = MmioRead32(ALT_HPS_NAND_DATA_OFST + ALT_NANDDATA_DATA_OFST);

 // Poll until command completed
  IntStat = NandPollForIntStat(
              IntStatRegOfst,
              ALT_NAND_STAT_INTR_STATUS0_LOAD_COMP_SET_MSK
              );

  if ( !(IntStat & ALT_NAND_STAT_INTR_STATUS0_LOAD_COMP_SET_MSK) ) {
    InfoPrint( "NAND: Pipeline read command cannot be completed with timeout\r\n");
    return EFI_DEVICE_ERROR;
  }

  return Status;
}

UINT32
EFIAPI
TrimFfsAtEndOfBlock (
  IN  UINT8*   Buffer,
  IN  UINT32   BlockLength
  )
{
  UINT32 Length;
  INTN  i;
  UINT32 AlignedPage;

  for (i = BlockLength - 1; i >=0; i--)
    if (Buffer[i] != 0xFF)
        break;

  Length = i + 1;

  // Align with page size
  AlignedPage = (Length + mFlash.PageSize - 1) / mFlash.PageSize;
  Length = AlignedPage * mFlash.PageSize;

  return Length;

}

EFI_STATUS
EFIAPI
NandWriteTrimFfsSkipBadBlock (
  IN  VOID*   Buffer,
  IN  UINT32  Offset,
  IN  UINT32  Size
  )
{
  EFI_STATUS   Status;
  UINT32      NextWriteOffset;
  UINT32      NumberOfBytesLeftToWrite;
  UINT32      NumberOfBytesWritedInThisLoop;
  UINT32      FirstLoopUnAlignedOffsetAdjustment;
  UINT32      Block;
  UINT32      NumberOfBadBlock;
  UINT8*      BufferPtr;
  UINT32      TruncatedWriteSize;

  BufferPtr = (UINT8*) Buffer;
  Status = EFI_SUCCESS;
  NextWriteOffset = Offset;
  NumberOfBytesLeftToWrite = Size;
  FirstLoopUnAlignedOffsetAdjustment = Offset % mFlash.BlockSize;
  NumberOfBadBlock = 0;

  do {
    NumberOfBytesWritedInThisLoop = MIN(NumberOfBytesLeftToWrite, mFlash.BlockSize - FirstLoopUnAlignedOffsetAdjustment);
   // Write a block of data
    Block  = NandBlockAddressGet(NextWriteOffset);
    // Check bad block
    if (NandBlockIsBad (Block) == TRUE) {
      InfoPrint ("NAND: Skip write block %d as it is bad block\r\n", Block);
      NumberOfBadBlock++;
      if ((NumberOfBadBlock > PcdGet32 (PcdNandStopIfMoreThanThisNumberBadBlocks))) {
        InfoPrint("NAND: More than %d block are bad, skip whole process\r\n", PcdGet32 (PcdNandStopIfMoreThanThisNumberBadBlocks));
        return EFI_DEVICE_ERROR;
      }
    NextWriteOffset += NumberOfBytesWritedInThisLoop;
    continue;
    }
     // Check 0xFF pages at the end of each block
    TruncatedWriteSize = TrimFfsAtEndOfBlock (BufferPtr, NumberOfBytesWritedInThisLoop);
    if (TruncatedWriteSize < NumberOfBytesWritedInThisLoop)
      InfoPrint("Empty papge found at Block %d, TruncatedWriteSize %d\r\n", Block, TruncatedWriteSize);

    Status = NandWrite (BufferPtr,NextWriteOffset, TruncatedWriteSize);
    if (EFI_ERROR(Status)) return Status;

    // Update for next loop
    NextWriteOffset += NumberOfBytesWritedInThisLoop;
    NumberOfBytesLeftToWrite -= NumberOfBytesWritedInThisLoop;
    BufferPtr += NumberOfBytesWritedInThisLoop;
    FirstLoopUnAlignedOffsetAdjustment = 0;
  } while (NumberOfBytesLeftToWrite);

  return Status;
}

  
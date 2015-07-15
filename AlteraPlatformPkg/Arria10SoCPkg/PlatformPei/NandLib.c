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
// Lookup table for ALT_NAND_STAT_INTR_STAT[n]_OFST
//
UINT32 mLUT_ALT_NAND_STAT_INTR_STATn_OFST[] = {
  ALT_NAND_STAT_OFST + ALT_NAND_STAT_INTR_STAT0_OFST,
  ALT_NAND_STAT_OFST + ALT_NAND_STAT_INTR_STAT1_OFST,
  ALT_NAND_STAT_OFST + ALT_NAND_STAT_INTR_STAT2_OFST,
  ALT_NAND_STAT_OFST + ALT_NAND_STAT_INTR_STAT3_OFST
};

//
// Lookup table for ALT_NAND_CFG_DEVICE_RST_BANK[n]_SET_MSK
//
UINT32 mLUT_ALT_NAND_CFG_DEVICE_RST_BANKn_SET_MSK[] = {
  ALT_NAND_CFG_DEVICE_RST_BANK0_SET_MSK,
  ALT_NAND_CFG_DEVICE_RST_BANK1_SET_MSK,
  ALT_NAND_CFG_DEVICE_RST_BANK2_SET_MSK,
  ALT_NAND_CFG_DEVICE_RST_BANK3_SET_MSK
};

//
// Lookup table for ALT_NAND_CFG_RB_PIN_END_BANK[n]_CLR_MSK
//
UINT32 mLUT_ALT_NAND_CFG_RB_PIN_END_BANKn_CLR_MSK[] = {
  ALT_NAND_CFG_RB_PIN_END_BANK0_CLR_MSK,
  ALT_NAND_CFG_RB_PIN_END_BANK1_CLR_MSK,
  ALT_NAND_CFG_RB_PIN_END_BANK2_CLR_MSK,
  ALT_NAND_CFG_RB_PIN_END_BANK3_CLR_MSK
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
    ALT_SYSMGR_OFST +
    ALT_SYSMGR_NAND_BOOTSTRAP_OFST,
    ALT_SYSMGR_NAND_BOOTSTRAP_NOINIT_CLR_MSK &
    ALT_SYSMGR_NAND_BOOTSTRAP_NOLDB0P0_CLR_MSK &
    ALT_SYSMGR_NAND_BOOTSTRAP_TWOROWADDR_CLR_MSK &
    ALT_SYSMGR_NAND_BOOTSTRAP_PAGE512_CLR_MSK &
    ALT_SYSMGR_NAND_BOOTSTRAP_PAGE512_X16_CLR_MSK,
    ALT_SYSMGR_NAND_BOOTSTRAP_NOINIT_SET(bootstrap_noinit) |
    ALT_SYSMGR_NAND_BOOTSTRAP_NOLDB0P0_SET(bootstrap_noloadb0p0) |
    ALT_SYSMGR_NAND_BOOTSTRAP_TWOROWADDR_SET(bootstrap_tworowaddr) |
    ALT_SYSMGR_NAND_BOOTSTRAP_PAGE512_SET(bootstrap_page512) |
    ALT_SYSMGR_NAND_BOOTSTRAP_PAGE512_X16_SET(bootstrap_page512_x16)
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
    ALT_NAND_CFG_OFST +
    ALT_NAND_CFG_ECC_CORRECTION_OFST,
    ALT_NAND_CFG_ECC_CORRECTION_VALUE_CLR_MSK &
    ALT_NAND_CFG_ECC_CORRECTION_ERASE_THRESHOLD_CLR_MSK,
    ALT_NAND_CFG_ECC_CORRECTION_VALUE_SET(ALT_NAND_CFG_ECC_CORRECTION_VALUE_RESET) |
    ALT_NAND_CFG_ECC_CORRECTION_ERASE_THRESHOLD_SET(ALT_NAND_CFG_ECC_CORRECTION_ERASE_THRESHOLD_RESET)
    );

  // ECC related: Set number of bytes to skip from beginning of Spare Area
  MmioWrite32(
    ALT_NAND_CFG_OFST +
    ALT_NAND_CFG_SPARE_AREA_SKIP_BYTES_OFST,
    NAND_SPARE_AREA_SKIP_BYTES
    );

  // Enables controller ECC capabilities
  MmioOr32(
    ALT_NAND_CFG_OFST +
    ALT_NAND_CFG_ECC_EN_OFST,
    ALT_NAND_CFG_ECC_EN_FLAG_SET_MSK);

  // Read NAND Flash Characteristic
  NandGetFlashInfo ();

  // Set next plane starts Block number In case the device is a multi plane device
  // Note: This can only be set after calling NandGetFlashInfo
  MmioWrite32(
    ALT_NAND_CFG_OFST +
    ALT_NAND_CFG_FIRST_BLOCK_OF_NEXT_PLANE_OFST,
    mFlash.FirstBlockOfNextPlane
    );

  // Enables data DMA operation in the controller
  MmioOr32 (
    ALT_NAND_DMA_OFST +
    ALT_NAND_DMA_DMA_EN_OFST,
    ALT_NAND_DMA_DMA_EN_FLAG_SET_MSK
    );

  return EFI_SUCCESS;
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
  IntStatReg = mLUT_ALT_NAND_STAT_INTR_STATn_OFST[Bank];
  DevResetBankMask = mLUT_ALT_NAND_CFG_DEVICE_RST_BANKn_SET_MSK[Bank];

  // Clear Interrupt status register for this Bank
  MmioWrite32(IntStatReg, MAX_UINT32);

  // Tell NAND controller to sends a RESET command to device on this Bank number
  // NAND Controller will clear the bit after reset command is issued to device.
  MmioOr32(
    ALT_NAND_CFG_OFST +
    ALT_NAND_CFG_DEVICE_RST_OFST,
    DevResetBankMask
    );

  // Poll until NAND Controller finished reset and initialization process
  NandPollForIntStat(IntStatReg, ALT_NAND_STAT_INTR_STAT0_RST_COMP_SET_MSK);

  // Disable Ready/Busy pin for bank 0. We are using polling mode.
  MmioAnd32(
    ALT_NAND_CFG_OFST +
    ALT_NAND_CFG_RB_PIN_END_OFST,
    mLUT_ALT_NAND_CFG_RB_PIN_END_BANKn_CLR_MSK[Bank]
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
    if (Data32 & (IntStatMask | ALT_NAND_STAT_INTR_STAT0_UNSUP_CMD_SET_MSK ))
      break;
  } while ( (WaitCount++ < NAND_POLL_FOR_INT_STAT_TIMEOUT) );
  if (WaitCount >= NAND_POLL_FOR_INT_STAT_TIMEOUT) {
    InfoPrint ("NAND: Timeout 0x%08X != 0x%08X\r\n", Data32, IntStatMask);
    ASSERT_PLATFORM_INIT(0);
  }
  if ( Data32 & ALT_NAND_STAT_INTR_EN0_UNSUP_CMD_SET_MSK) {
    InfoPrint ( "NAND: Unsupported CMD!\r\n" );
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
  mFlash.ManufacturerId             = MmioRead32(ALT_NAND_PARAM_OFST + ALT_NAND_PARAM_MANUFACTURER_ID_OFST);
  mFlash.DeviceId                   = MmioRead32(ALT_NAND_PARAM_OFST + ALT_NAND_PARAM_DEVICE_ID_OFST);
  mFlash.DeviceParam0               = MmioRead32(ALT_NAND_PARAM_OFST + ALT_NAND_PARAM_DEVICE_PARAM_0_OFST);
  mFlash.DeviceParam1               = MmioRead32(ALT_NAND_PARAM_OFST + ALT_NAND_PARAM_DEVICE_PARAM_1_OFST);
  mFlash.DeviceParam2               = MmioRead32(ALT_NAND_PARAM_OFST + ALT_NAND_PARAM_DEVICE_PARAM_2_OFST);
  mFlash.PageSize                   = MmioRead32(ALT_NAND_PARAM_OFST + ALT_NAND_PARAM_LOGICAL_PAGE_DATA_SIZE_OFST);
  mFlash.SpareSize                  = MmioRead32(ALT_NAND_PARAM_OFST + ALT_NAND_PARAM_LOGICAL_PAGE_SPARE_SIZE_OFST);
  mFlash.Revision                   = MmioRead32(ALT_NAND_PARAM_OFST + ALT_NAND_PARAM_REVISION_OFST);
  mFlash.OnfiDeviceFeatures         = MmioRead32(ALT_NAND_PARAM_OFST + ALT_NAND_PARAM_ONFI_DEV_FEATURES_OFST);
  mFlash.OnfiOptionalCommands       = MmioRead32(ALT_NAND_PARAM_OFST + ALT_NAND_PARAM_ONFI_OPTIONAL_CMDS_OFST);
  mFlash.OnfiTimingMode             = MmioRead32(ALT_NAND_PARAM_OFST + ALT_NAND_PARAM_ONFI_TIMING_MOD_OFST);
  mFlash.OnfiPgmCacheTimingMode     = MmioRead32(ALT_NAND_PARAM_OFST + ALT_NAND_PARAM_ONFI_PGM_CACHE_TIMING_MOD_OFST);
  mFlash.OnfiCompliant              = MmioRead32(ALT_NAND_PARAM_OFST + ALT_NAND_PARAM_ONFI_DEV_NO_OF_LUNS_OFST) >> 8;
  mFlash.OnfiDeviceNoOfLuns         = MmioRead32(ALT_NAND_PARAM_OFST + ALT_NAND_PARAM_ONFI_DEV_NO_OF_LUNS_OFST) & 0xff;
  mFlash.OnfiDeviceNoOfBlocksPerLun =(MmioRead32(ALT_NAND_PARAM_OFST + ALT_NAND_PARAM_ONFI_DEV_BLKS_PER_LUN_U_OFST) << 16)
                                    + MmioRead32(ALT_NAND_PARAM_OFST + ALT_NAND_PARAM_ONFI_DEV_BLKS_PER_LUN_L_OFST);
  mFlash.Features                   = MmioRead32(ALT_NAND_PARAM_OFST + ALT_NAND_PARAM_FEATURES_OFST);

  // Read NAND Controller Configurations
  mFlash.PagesPerBlock              = MmioRead32(ALT_NAND_CFG_OFST + ALT_NAND_CFG_PAGES_PER_BLOCK_OFST);
  mFlash.DeviceWidth                = MmioRead32(ALT_NAND_CFG_OFST + ALT_NAND_CFG_DEVICE_WIDTH_OFST);
  mFlash.SpareAreaSkipBytes         = MmioRead32(ALT_NAND_CFG_OFST + ALT_NAND_CFG_SPARE_AREA_SKIP_BYTES_OFST);
  Data32                            = MmioRead32(ALT_NAND_CFG_OFST + ALT_NAND_CFG_NUMBER_OF_PLANES_OFST);
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
    Status = NandDmaPageRead (Bank, Block, Page, (UINT32) &TempReadBuffer[0]);
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
    Status = NandDmaPageWrite (Bank, Block, Page, (UINT32) &TempReadBuffer[0]);
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
  UINT32      IntStatRegOfst = mLUT_ALT_NAND_STAT_INTR_STATn_OFST[Bank];

  // Clear Interrupt status register for this Bank
  MmioWrite32(IntStatRegOfst, MAX_UINT32);

  // DMA READ PageCount amount of data from SRC = Flash (Bank, BlockAddr, PageAddr) to DST = MemAddr.
  NandDmaWriteCmdStructure (Bank, BlockAddr, PageAddr, PageCount, MemAddr, IsReadOp, BurstLenInBytes);

  // Poll until DMA command completed or errored
  IntStat = NandPollForIntStat(
              IntStatRegOfst,
              ALT_NAND_STAT_INTR_STAT0_DMA_CMD_COMP_SET_MSK |
              ALT_NAND_STAT_INTR_STAT0_ECC_UNCOR_ERR_SET_MSK
              );

  if ( !(IntStat & ALT_NAND_STAT_INTR_STAT0_DMA_CMD_COMP_SET_MSK) ) {
    InfoPrint( "NAND: DMA READ ERROR! 0x%08X\r\n", IntStat );
    Status = EFI_DEVICE_ERROR;
  }

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
  UINT32      IntStatRegOfst = mLUT_ALT_NAND_STAT_INTR_STATn_OFST[Bank];

  // Clear Interrupt status register for this Bank
  MmioWrite32(IntStatRegOfst, MAX_UINT32);

  // DMA Write PageCount amount of data from SRC = MemAddr to DST = Flash (Bank, BlockAddr, PageAddr)
  NandDmaWriteCmdStructure (Bank, BlockAddr, PageAddr, PageCount, MemAddr, IsReadOp, BurstLenInBytes);

  // Poll until DMA command completed or errored
  IntStat = NandPollForIntStat(
              IntStatRegOfst,
              ALT_NAND_STAT_INTR_STAT0_DMA_CMD_COMP_SET_MSK |
              ALT_NAND_STAT_INTR_STAT0_PROGRAM_FAIL_SET_MSK |
              ALT_NAND_STAT_INTR_STAT0_LOCKED_BLK_SET_MSK
              );

  if ( !(IntStat & ALT_NAND_STAT_INTR_STAT0_DMA_CMD_COMP_SET_MSK) ) {
    InfoPrint( "NAND: DMA WRITE ERROR! 0x%08X\r\n", IntStat );
    Status = EFI_DEVICE_ERROR;
  }

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

  Map10CmdAddr = NandComposeMap10CmdAddr (Bank, BlockAddr, PageAddr );

  // Multi-transaction DMA Command Transaction 1 of 4
  // Table 13-10: Command-Data Pair 1
  // Command: 31:28 = 0, 27:26 = 2 - MAP10_CMD, 25:24 = 0, 23:<M> Block address, (<M> – 1):0 Page address
  MmioWrite32(ALT_NANDDATA_OFST + ALT_NANDDATA_CTRL_OFST, Map10CmdAddr);
  // Data: 31:16 = 0, 15:12 = 2, 11:8 = 0-Read/1-Write, 7:0 = Number of pages
  MmioWrite32(
    ALT_NANDDATA_OFST +
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
  MmioWrite32(ALT_NANDDATA_OFST + ALT_NANDDATA_CTRL_OFST, Map10CmdAddr);
  // Data: 31:16 = 0, 15:12 = 2, 11:8 = 2, 7:0 = 0
  MmioWrite32(ALT_NANDDATA_OFST + ALT_NANDDATA_DATA_OFST, 0x2200);

  // Multi-transaction DMA Command Transaction 3 of 4
  // Table 13-12: Command-Data Pair 3
  // Command: 31:28 = 0, 27:26 = 2 - MAP10_CMD, 25:24 = 0, 23:8 = Memory address low, 7:0 = 0
  Map10CmdAddr = MAP10_CMD | ((UINT16)MemAddr << 8);
  MmioWrite32(ALT_NANDDATA_OFST + ALT_NANDDATA_CTRL_OFST, Map10CmdAddr);
  // Data: 31:16 = 0, 15:12 = 2, 11:8 = 3, 7:0 = 0
  MmioWrite32(ALT_NANDDATA_OFST + ALT_NANDDATA_DATA_OFST, 0x2300);

  // Multi-transaction DMA Command Transaction 4 of 4
  // Table 13-13: Command-Data Pair 4
  // Command: 31:28 = 0, 27:26 = 2 - MAP10_CMD, 25:24 = 0, 23:17 = 0
  //          16 = INT, 15:8 = Burst length, 7:0 = 0
  Map10CmdAddr = MAP10_CMD |
                 0x10000 |       // Enable INTR_STATUS_DMA_CMD_COMP
                 BurstLen << 8;  // Set Burst length
  MmioWrite32(ALT_NANDDATA_OFST + ALT_NANDDATA_CTRL_OFST, Map10CmdAddr);
  // Data: 31:16 = 0, 15:12 = 2, 11:8 = 4, 7:0 = 0
  MmioWrite32(ALT_NANDDATA_OFST + ALT_NANDDATA_DATA_OFST, 0x2400);

}


UINT32
EFIAPI
NandComposeMap10CmdAddr (
  IN UINT32 Bank,
  IN UINT32 BlockAddr,
  IN UINT32 PageAddr
  )
{
  UINT32 bank_mask;
  UINT32 block_addr_mask;
  UINT32 page_addr_mask;
  UINT32 Map10CmdAddr;

  bank_mask       =  MAP10_CMD_BANK_SEL_MASK;
  block_addr_mask = (1 << (MAP10_CMD_BLK_ADDR_MSB_INDEX - mFlash.BlockShift + 1)) - 1;
  page_addr_mask  = (1 << mFlash.BlockShift) - 1;
  Map10CmdAddr    = MAP10_CMD |
                   /*
                      HPS System Technical Reference Manual said 25:24 is reserved for future, no Bank Sel
                     ((Bank      & bank_mask)       << MAP10_CMD_BANK_SEL_LSB_INDEX) |
                   */
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
  IntStatRegOfst = mLUT_ALT_NAND_STAT_INTR_STATn_OFST[Bank];
  PageAddr = 0;
  Map10CmdAddr = NandComposeMap10CmdAddr (Bank, BlockAddr, PageAddr);

  // Clear Interrupt status register for this Bank
  MmioWrite32(IntStatRegOfst, MAX_UINT32);

  // Sets block address for erase
  MmioWrite32(
    ALT_NANDDATA_OFST +
    ALT_NANDDATA_CTRL_OFST,
    Map10CmdAddr);

  // Initiates block erase operation.
  MmioWrite32(
    ALT_NANDDATA_OFST +
    ALT_NANDDATA_DATA_OFST,
    MAP10_CMD_ERASE_BLOCK
    );

  // Poll until DMA command completed or errored
  IntStat = NandPollForIntStat(
              IntStatRegOfst,
              ALT_NAND_STAT_INTR_STAT0_TIME_OUT_SET_MSK |
              ALT_NAND_STAT_INTR_STAT0_ERASE_COMP_SET_MSK |
              ALT_NAND_STAT_INTR_STAT0_ERASE_FAIL_SET_MSK
              );

  if ( !(IntStat & ALT_NAND_STAT_INTR_STAT0_ERASE_COMP_SET_MSK) ) {
    InfoPrint( "NAND: BLOCK %d ERASE ERROR! 0x%08X\r\n", IntStat, BlockAddr);
    Status = EFI_DEVICE_ERROR;
  }

  return Status;
}





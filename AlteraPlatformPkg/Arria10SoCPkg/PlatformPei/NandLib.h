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

#ifndef __NAND_LIB_H__
#define __NAND_LIB_H__

#define ALT_NANDDATA_CTRL_OFST              0x00
#define ALT_NANDDATA_DATA_OFST              0x10

#define NAND_POLL_FOR_INT_STAT_TIMEOUT     (100000)
#define NAND_NUMBER_OF_BANK                 4
#define NAND_SPARE_AREA_SKIP_BYTES          2

//
// NAND Flash Characteristic
//
typedef struct
{
  UINT32    ManufacturerId;
  UINT32    DeviceId;
  UINT32    DeviceParam0;
  UINT32    DeviceParam1;
  UINT32    DeviceParam2;
  UINT32    PageSize;
  UINT32    SpareSize;
  UINT32    Revision;
  UINT32    OnfiDeviceFeatures;
  UINT32    OnfiOptionalCommands;
  UINT32    OnfiTimingMode;
  UINT32    OnfiPgmCacheTimingMode;
  UINT32    OnfiCompliant;
  UINT32    OnfiDeviceNoOfLuns;
  UINT32    OnfiDeviceNoOfBlocksPerLun;
  UINT32    Features;
  UINT32    NumberOfPlanes;
  UINT32    PagesPerBlock;
  UINT32    DeviceWidth;
  UINT32    BlockSize;
  UINT32    SpareAreaSkipBytes;
  UINT32    FirstBlockOfNextPlane;
  UINT32    PageSize32;
  UINT32    PageShift;
  UINT32    BlockShift;
} NAND_FLASH_CHARACTERISTIC;


#define NandBlockAddressGet(Addr)                (Addr >> (mFlash.BlockShift + mFlash.PageShift))
#define NandPageAddressGet(Addr)                 (Addr >> mFlash.PageShift & ((1 << mFlash.BlockShift) - 1))
#define NandFlashAddrCompose(BlockNum, PageNum) ((BlockNum << (mFlash.BlockShift + mFlash.PageShift)) + \
                                                 (PageNum  <<  mFlash.PageShift))

#define ALT_NAND_FLASH_MEM_BANK_0                (0)
#define ALT_NAND_FLASH_MEM_BANK_1                (1)
#define ALT_NAND_FLASH_MEM_BANK_2                (2)
#define ALT_NAND_FLASH_MEM_BANK_3                (3)

#define MAP00_CMD                                (0 << 26)  // MAP 00 Buff Access
#define MAP01_CMD                                (1 << 26)  // MAP 01 Array Access
#define MAP10_CMD                                (2 << 26)  // MAP 10 Control Access
#define MAP11_CMD                                (3 << 26)  // MAP 11 Direct Access

#define MAP10_CMD_BANK_SEL_LSB_INDEX             (24)
#define MAP10_CMD_BANK_SEL_MASK                  (3)
#define MAP10_CMD_BLK_ADDR_MSB_INDEX             (23)
#define MAP10_CMD_PAGE_ADDR_LSB_INDEX            (0)

#define MAP10_CMD_ERASE_BLOCK                    (0x01)

EFI_STATUS
EFIAPI
NandInit (
  VOID
  );

EFI_STATUS
EFIAPI
NandErase (
  IN  UINT32  Offset,
  IN  UINT32  Size
  );

EFI_STATUS
EFIAPI
NandRead (
  OUT VOID*   Buffer,
  IN  UINT32  Offset,
  IN  UINT32  Size
  );

EFI_STATUS
EFIAPI
NandWrite (
  OUT VOID*   Buffer,
  IN  UINT32  Offset,
  IN  UINT32  Size
  );

EFI_STATUS
EFIAPI
NandUpdate (
  OUT VOID*   Buffer,
  IN  UINT32  Offset,
  IN  UINT32  Size
  );

VOID
EFIAPI
NandResetBank (
  IN  UINT32 Bank
  );

UINT32
EFIAPI
NandPollForIntStat (
  IN  UINT32 IntStatReg,
  IN  UINT32 IntStatMask
  );

VOID
EFIAPI
NandGetFlashInfo (
  VOID
  );

EFI_STATUS
EFIAPI
NandDmaPageRead (
  IN UINT32  Bank,
  IN UINT32  BlockAddr,
  IN UINT32  PageAddr,
  IN UINT32  MemAddr
  );

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
  );

UINT32
EFIAPI
NandComposeMap10CmdAddr (
  IN UINT32 Bank,
  IN UINT32 BlockAddr,
  IN UINT32 PageAddr
  );


EFI_STATUS
EFIAPI
NandFlashBlockErase (
  IN  UINT32 Offset
  );

EFI_STATUS
EFIAPI
NandDmaPageWrite (
  IN UINT32  Bank,
  IN UINT32  BlockAddr,
  IN UINT32  PageAddr,
  IN UINT32  MemAddr
  );

#endif  /* __NAND_LIB_H__ */



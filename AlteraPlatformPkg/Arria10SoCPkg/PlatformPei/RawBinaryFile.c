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
#include "AlteraSdMmcPei/AlteraSdMmcPei.h"
#include "Assert.h"
#include "BootSource.h"
#include "DeviceTree.h"
#include "NandLib.h"
#include "QspiLib.h"
#include "MkimageHeader.h"
#include "RawBinaryFile.h"
#include "PitStopUtility.h"
#include "SdMmc.h"

#if (FixedPcdGet32(PcdDebugMsg_Rbf) == 0)
  #define ProgressPrint(FormatString, ...)    /* do nothing */
  #define InfoPrint(FormatString, ...)        /* do nothing */
  #define MmioHexDumpEx(BaseAddr, Data32Size, PrintLineStartAddress)   /* do nothing */
#else
  #define ProgressPrint SerialPortPrint
  #define InfoPrint     SerialPortPrint
  #define MmioHexDumpEx SerialPortMmioHexDumpEx
#endif

// ==================================================================
// Module level variables:
// ==================================================================

BOOT_SOURCE_TYPE  mBootSourceType = BOOT_SOURCE_RSVD;

// SDMMC will store RBF files in FAT32 partition
// (FDT->chosen->cff-file->"file1, file2, ...)
RBF_FILE_CONFIG   mFdtRbfCfg;
UINT32            mQspiNandRbfOffset;

// ==================================================================
// Functions Implementation:
// ==================================================================

EFI_STATUS
EFIAPI
FlashWrite (
  IN  UINT32  Offset,
  IN  VOID*   Buffer,
  IN  UINT32  BufferSize
  )
{
  EFI_STATUS    Status;
  Status = EFI_SUCCESS;

  // Flash Device
  switch (mBootSourceType) {
    case BOOT_SOURCE_NAND:
      Status = NandWrite (Buffer, Offset, BufferSize);
      break;
    case BOOT_SOURCE_QSPI:
      Status = QspiWrite (Buffer, Offset, BufferSize);
      break;
    case BOOT_SOURCE_SDMMC:
    case BOOT_SOURCE_RSVD:
    case BOOT_SOURCE_FPGA:
    default:
      Status = EFI_UNSUPPORTED;
      break;
  }
  return Status;
}


EFI_STATUS
EFIAPI
FlashRead (
  IN  UINT32  Offset,
  OUT VOID*   Buffer,
  IN  UINT32  BufferSize
  )
{
  EFI_STATUS    Status;
  Status = EFI_SUCCESS;

  // Flash Device
  switch (mBootSourceType)
  {
    case BOOT_SOURCE_NAND:
      Status = NandRead (Buffer, Offset, BufferSize);
      break;
    case BOOT_SOURCE_QSPI:
      Status = QspiRead (Buffer, Offset, BufferSize);
      break;
    case BOOT_SOURCE_SDMMC:
    case BOOT_SOURCE_RSVD:
    case BOOT_SOURCE_FPGA:
    default:
      Status = EFI_UNSUPPORTED;
      break;
  }
  return Status;
}


EFI_STATUS
EFIAPI
OpenRawBinaryFile(
  IN  VOID*             Fdt,
  IN  BOOT_SOURCE_TYPE  BootSourceType,
  OUT UINT32*           RbfSize
  )
{
  EFI_STATUS        Status;
  INTN              i;
  // QSPI and NAND RBF files will have an MKIMAGE header
  // (MKIMAGE Header + file1 + file2 + ...)
  MKIMG_HEADER      ImgHdr;
  RBF_FILE_CONFIG   RbfCfg;
  UINT32            RbfOffset;

  mBootSourceType = BootSourceType;
  Status = EFI_UNSUPPORTED;

  switch (mBootSourceType)
  {
    case BOOT_SOURCE_NAND:
    case BOOT_SOURCE_QSPI:
      GetRbfOffset (Fdt, &RbfOffset);
      mQspiNandRbfOffset = RbfOffset;
      Status = FlashRead(mQspiNandRbfOffset, &ImgHdr, sizeof(ImgHdr));
      if (EFI_ERROR(Status)) return Status;

      Status = ValidateMkimageHeader(&ImgHdr);
      if (EFI_ERROR(Status)) {
        InfoPrint("mkimage header for RBF not found!\r\n");
        MmioHexDumpEx((UINTN)(&ImgHdr), sizeof(ImgHdr)/4, mQspiNandRbfOffset);
        return Status;
      }
      *RbfSize = ImgHdr.DataSize;
      break;

    case BOOT_SOURCE_SDMMC:
      *RbfSize = 0;
      GetRbfFileCfg (Fdt, &RbfCfg);
      mFdtRbfCfg = RbfCfg;
      if (mFdtRbfCfg.NumOfRbfFileParts == 0) {
        return EFI_NOT_FOUND;
      }
      // Try to open each RBF file(s)
      // Use count down method because the last opened file need to be Part1
      for (i = (mFdtRbfCfg.NumOfRbfFileParts - 1); i >= 0; i--)
      {
        mFdtRbfCfg.FileHandler[i].FileName = mFdtRbfCfg.RBF_FileName[i];
        OpenFileInRootDirectory (&mFdtRbfCfg.FileHandler[i]);
        if (mFdtRbfCfg.FileHandler[i].Found == FALSE)
          return EFI_NOT_FOUND;
        *RbfSize += mFdtRbfCfg.FileHandler[i].FileSize;
      }
      InfoPrint ("Total RBF size = %d\r\n", *RbfSize);
      Status = EFI_SUCCESS;
      break;

    case BOOT_SOURCE_RSVD:
    case BOOT_SOURCE_FPGA:
    default:
      Status = EFI_UNSUPPORTED;
      break;
  }

  return Status;
}


EFI_STATUS
EFIAPI
ReadRawBinaryFile(
  IN  UINT32  Offset,
  IN  UINTN   ReadSize,
  OUT VOID*   Buffer
  )
{
  EFI_STATUS    Status;

  // The following variables is used when boot souce is SDMMC only
  STATIC UINTN  CurrentFile = 0;
  STATIC UINTN  NumOfBytesLeftInBufferFromPreviousRead = 0;
  STATIC VOID*  LeftOverBufferFromPreviousRead = NULL;
  UINTN         DataSizeLeftToRead;
  UINTN         NumOfBytesReadToCopyToDestBuffer;
  UINTN         SourceDataSize;
  VOID*         SourceBuffer;
  VOID*         DestBuffer;

  Status = EFI_SUCCESS;
  DestBuffer = Buffer;

  switch (mBootSourceType)
  {
    case BOOT_SOURCE_NAND:
    case BOOT_SOURCE_QSPI:
      Status = FlashRead (mQspiNandRbfOffset + Offset + sizeof(MKIMG_HEADER), DestBuffer, ReadSize);
      break;

    case BOOT_SOURCE_SDMMC:
      // Special handling when calling from GetCdRatio function
      if ((ReadSize == 4) && (Offset < 4096))
      {
        // InfoPrint ("Reading RbfData32[%d]\r\n", Offset/4);

        // GetCdRatio will call this function first
        CurrentFile = 0;

        // Make sure we are at the begining of the file
        OpenFileByCluster (mFdtRbfCfg.FileHandler[0].FileFirstCluster);

        // Read first block of data
        Status = ReadFileNextCluster ();

        // Copy to caller's buffer
        CopyMem (DestBuffer, (mFdtRbfCfg.FileHandler[0].FileData + Offset), ReadSize);

        // Reset file pointer to the begining of the file
        OpenFileByCluster (mFdtRbfCfg.FileHandler[0].FileFirstCluster);

      } else {
        // Started sending bitstream, so just keep fetching next block of data to fill the buffer
        // InfoPrint ("Reading Offset 0x%08x, ReadSize=0x%08x\r\n", Offset, ReadSize);

        NumOfBytesReadToCopyToDestBuffer = 0;
        DataSizeLeftToRead = ReadSize;

        while (DataSizeLeftToRead > 0) {

          // Read next block of data
          if (NumOfBytesLeftInBufferFromPreviousRead == 0)
          {
            // InfoPrint ("Read from Card\r\n");
            Status = ReadFileNextData (&mFdtRbfCfg.FileHandler[CurrentFile]);
            SourceBuffer = mFdtRbfCfg.FileHandler[CurrentFile].FileData;
            SourceDataSize = mFdtRbfCfg.FileHandler[CurrentFile].LastReadDataSize;
          } else {
            // InfoPrint ("Use left over data\r\n");
            // InfoPrint ("NumOfBytesLeftInBufferFromPreviousRead = %d\r\n", NumOfBytesLeftInBufferFromPreviousRead);
            Status = EFI_SUCCESS;
            SourceBuffer = LeftOverBufferFromPreviousRead;
            SourceDataSize = NumOfBytesLeftInBufferFromPreviousRead;
            NumOfBytesLeftInBufferFromPreviousRead = 0;
          }

          // Do we read more/equal/less than ReadSize?
          if (SourceDataSize <= DataSizeLeftToRead)
          {
            // InfoPrint ("Got %d <= Req %d\r\n", SourceDataSize, DataSizeLeftToRead);
            // we have read less or equal to the size needed
            NumOfBytesReadToCopyToDestBuffer = SourceDataSize;
          } else {
            // InfoPrint ("Got %d > Req %d\r\n", SourceDataSize, DataSizeLeftToRead);
            // read more data than requested
            NumOfBytesReadToCopyToDestBuffer = DataSizeLeftToRead;
            NumOfBytesLeftInBufferFromPreviousRead = SourceDataSize - DataSizeLeftToRead;
            LeftOverBufferFromPreviousRead = (VOID*)((UINT8*)SourceBuffer + NumOfBytesReadToCopyToDestBuffer);
          }

          // Copy to caller's buffer
          // InfoPrint ("Copying to buffer offset 0x%08x size 0x%08x\r\n", (UINTN)DestBuffer - (UINTN)Buffer, NumOfBytesReadToCopyToDestBuffer);
          CopyMem (DestBuffer, SourceBuffer, NumOfBytesReadToCopyToDestBuffer);
          DestBuffer = (VOID*)((UINT8*)DestBuffer + NumOfBytesReadToCopyToDestBuffer);

          // Decrease number of bytes left to read
          DataSizeLeftToRead -= NumOfBytesReadToCopyToDestBuffer;

          // Do we reach end of file?
          if (mFdtRbfCfg.FileHandler[CurrentFile].EndOfFile == TRUE)
          {
            CurrentFile++;
            if (CurrentFile < mFdtRbfCfg.NumOfRbfFileParts)
            {
              // Open next file
              // InfoPrint ("Opening next file\r\n");
              OpenFileByCluster (mFdtRbfCfg.FileHandler[CurrentFile].FileFirstCluster);
            } else {
              // InfoPrint ("End of RBF file(s)\r\n");
              ASSERT_PLATFORM_INIT(DataSizeLeftToRead == 0);
            }
          }

        }
      }
      break;

    case BOOT_SOURCE_RSVD:
    case BOOT_SOURCE_FPGA:
    default:
      Status = EFI_UNSUPPORTED;
      break;
  }

  return Status;
}

VOID
EFIAPI
LoadCoreRbfImageToRam (
  IN  UINTN            DestinationMemoryBase,
  IN  UINT32           RbfSize
  )
{
  EFI_STATUS        Status;
  BOOT_SOURCE_TYPE  BootSourceType;
  UINT32            FileSize;

  // Detect Boot Source Type
  BootSourceType = GetBootSourceType ();
  // Flash Device Type
  switch (BootSourceType)
  {
    case BOOT_SOURCE_SDMMC:
      // Read core RBF file from root folder of FAT32 partition on SD/MMC card
      LoadFileToMemory (
        mFdtRbfCfg.RBF_FileName[0],
        DestinationMemoryBase,
        &FileSize);  // this is just the dummy FileSize that wont need to return back value to caller functions.

      break;
    case BOOT_SOURCE_NAND:
    case BOOT_SOURCE_QSPI:
      // Print message that we are going to read core RBF from QSPI or NAND flash
      ProgressPrint ("Copying Core RBF from Flash Offset 0x%08lx to RAM Address 0x%08lx with image size 0x%08x\r\n",
        (UINT64) mQspiNandRbfOffset,
        (UINT64) DestinationMemoryBase,
        (UINT32) RbfSize);
       if (BootSourceType == BOOT_SOURCE_QSPI)
      {
        // Read from QSPI
        Status = QspiRead(
          (VOID *) DestinationMemoryBase,
          mQspiNandRbfOffset,
          RbfSize);
      } else {
        // Read from NAND
        Status = NandRead(
          (VOID *) DestinationMemoryBase,
          mQspiNandRbfOffset,
          RbfSize);
      }
      // Error Checking
      if (EFI_ERROR(Status)) {
        InfoPrint ("Error Reading Core RBF image\r\n");
      }
      break;
    case BOOT_SOURCE_RSVD:
    case BOOT_SOURCE_FPGA:
    default:
      // No Flash device.
      InfoPrint ("No flash devices\r\n");
      break;
  }

}

VOID
EFIAPI
GetRbfOffset (
  IN  CONST VOID*                  Fdt,
  OUT       UINT32*                RbfOffset
  )
{
  if (Fdt != NULL) {
    // get peri or combine RBF offset from DTB
    GetRbfOffsetFromDeviceTree (Fdt, RbfOffset);
  } else if (PcdGet32 (PcdAutoProgramCoreRbf) == 1) {
    // get core RBF offset from PCD
    *RbfOffset = (UINT32) PcdGet32(PcdQspiOrNand_CORE_RBF_ADDR);
  } else {
    //get core/peri/combine rbf offset from pitstop
    GetRbfOffsetFromPitStop (RbfOffset);
  }
}

VOID
EFIAPI
GetRbfFileCfg (
  IN  CONST VOID*                  Fdt,
  OUT       RBF_FILE_CONFIG*        RbfCfg
  )
{
  if (Fdt != NULL) {
    // get core/peri rbf file name from dtb
    GetRbfFileCfgFromDeviceTree (Fdt, RbfCfg);
  } else if (PcdGet32 (PcdAutoProgramCoreRbf) == 1) {
    // get core rbf filename from PCD
    RbfCfg->NumOfRbfFileParts = 1;
    RbfCfg->RBF_FileName[0] = (CHAR8*) PcdGetPtr (PcdFileName_CORE_RBF);
  } else {
    //get core/peri/combine rbf offset from pitstop
    RbfCfg->NumOfRbfFileParts = 1;
    GetRbfFileNameFromPitStop(&RbfCfg->RBF_FileName[0]);
  }
}

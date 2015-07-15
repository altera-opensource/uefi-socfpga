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
#include <libfdt.h>
#include <Library/ArmGicLib.h>
#include <Library/ArmLib.h>
#include <Library/BaseMemoryLib.h>
#include <Library/DebugLib.h>
#include <Library/IoLib.h>
#include <Library/MemoryAllocationLib.h>
#include <Library/PcdLib.h>
#include <Library/SerialPortPrintLib.h>
#include <Library/TimerLib.h>
#include "AlteraSdMmcPei/AlteraSdMmcPei.h"
#include "Assert.h"
#include "DeviceTree.h"
#include "MkimageHeader.h"
#include "SdMmc.h"

#if (FixedPcdGet32(PcdDebugMsg_SdMmc) == 0)
  //#define ProgressPrint(FormatString, ...)    /* do nothing */
  #define ProgressPrint                       SerialPortPrint
  #define InfoPrint(FormatString, ...)        /* do nothing */
  #define MmioHexDump(BaseAddr, Data32Size)   /* do nothing */
#else
  #define ProgressPrint SerialPortPrint
  #define InfoPrint     SerialPortPrint
  #define MmioHexDump   SerialPortMmioHexDump
#endif

//
// Functions
//

VOID
EFIAPI
InitSdMmc (
  IN  CONST VOID*  Fdt
  )
{
  EFI_STATUS          Status;
  InfoPrint ("Initializing SD/MMC controller\r\n");
  Status = AlteraSdMmcPeiInit ();
  ASSERT_PLATFORM_INIT(!EFI_ERROR(Status));
  InfoPrint ("Done Initializing SD/MMC controller\r\n");
}


VOID
EFIAPI
HexDumpFile (
  IN  CHAR8*           TextFileFilename
  )
{
  FAT32_FILE           MyFile;
  EFI_STATUS           Status;
  // Open a file by filename
  MyFile.FileName = TextFileFilename;
  OpenFileInRootDirectory (&MyFile);
  if (MyFile.Found == TRUE)
  {
    while (MyFile.EndOfFile == FALSE)
    {
      Status = ReadFileNextData (&MyFile);
      SerialPortMmioHexDumpEx((UINTN)MyFile.FileData, MyFile.LastReadDataSize/4, MyFile.NextFilePos - MyFile.LastReadDataSize);
    }
  } else {
    SerialPortPrint ("File not found %a\r\n", TextFileFilename);
  }
}


EFI_STATUS
EFIAPI
LoadFileToMemory (
  IN  CHAR8*           TextFileFilename,
  IN  UINTN            DestinationMemoryBase,
  OUT UINT32*          pFileSize
  )
{
  EFI_STATUS           Status;
  FAT32_FILE           MyFile;
  UINT8*               Destination;
  UINTN                BytesReadCounter;
  UINTN                Percentage;

  Destination = (UINT8*) DestinationMemoryBase;
  BytesReadCounter = 0;
  Percentage = 0;

  // Open a file by filename
  MyFile.FileName = TextFileFilename;
  OpenFileInRootDirectory (&MyFile);
  if (MyFile.Found == TRUE)
  {
    ProgressPrint ("Copying %a from Flash to RAM at 0x%08x\r\n", TextFileFilename, (UINT32) DestinationMemoryBase);
    while (MyFile.EndOfFile == FALSE)
    {
      // Read a block of data
      Status = ReadFileNextData (&MyFile);

      // Copy to memory
      CopyMem (Destination, MyFile.FileData, MyFile.LastReadDataSize);

      // Dump data
      // SerialPortMmioHexDumpEx((UINTN)Destination, MyFile.LastReadDataSize/4, MyFile.NextFilePos - MyFile.LastReadDataSize);

      // Point to new destination for next loop
      Destination += MyFile.LastReadDataSize;

      // Count progress
      BytesReadCounter += MyFile.LastReadDataSize;

      // Update once every 10%
      if (((BytesReadCounter * 100 / MyFile.FileSize) - Percentage) >= 10)
      {
        Percentage = (BytesReadCounter * 100 / MyFile.FileSize);
        InfoPrint ("\r%2d%% ", Percentage);
      }
    }
    ProgressPrint ("\rDone.\r\n");
    // Return with filesize
    if (pFileSize != NULL)
      *pFileSize = MyFile.FileSize;
    return EFI_SUCCESS;
  } else {
    SerialPortPrint ("File not found %a\r\n", TextFileFilename);
    return EFI_NOT_FOUND;
  }
}


EFI_STATUS
EFIAPI
LoadBootImageFile (
  IN  CHAR8*           TextFileFilename,
  OUT UINTN*           pLoadAddr,
  OUT UINTN*           pEntryPoint,
  OUT UINT32*          pFileSize
  )
{
  EFI_STATUS           Status;
  FAT32_FILE           MyFile;
  UINT8*               Destination;
  UINTN                BytesReadCounter;
  UINTN                Percentage;
  MKIMG_HEADER*        ImgHdrPtr;
  UINTN                SkipFirstNBytes;
  BOOLEAN              IsFirstBlockOfData;

  Destination = (UINT8*) *pLoadAddr;
  BytesReadCounter = 0;
  Percentage = 0;
  SkipFirstNBytes = 0;
  IsFirstBlockOfData = TRUE;

  // Open a file by filename
  MyFile.FileName = TextFileFilename;
  OpenFileInRootDirectory (&MyFile);

  if (MyFile.Found == TRUE)
  {
    while (MyFile.EndOfFile == FALSE)
    {
      // Read a block of data
      Status = ReadFileNextData (&MyFile);
      if (EFI_ERROR(Status)) return EFI_DEVICE_ERROR;

      // First block of data?
      if (IsFirstBlockOfData == TRUE)
      {
        // Check if the file contain MKIMAGE header
        ImgHdrPtr = (MKIMG_HEADER*)(MyFile.FileData);
        Status = ValidateMkimageHeader(ImgHdrPtr);
        if (Status == EFI_SUCCESS) {
          // Boot Image have mkimage header, skip copying the header
          SkipFirstNBytes = sizeof(MKIMG_HEADER);

          // LoadAddr are based on MKIMAGE header
          Destination  = (UINT8*) ImgHdrPtr->LoadAddr;

          // Pass the information back to caller of this function
          *pLoadAddr   = ImgHdrPtr->LoadAddr;
          *pEntryPoint = ImgHdrPtr->EntryPoint;
        }
        ProgressPrint ("Copying file %a to RAM at 0x%08x\r\n", MyFile.FileName, (UINT32) Destination);
      }

      // Copy to memory
      CopyMem (Destination, (MyFile.FileData + SkipFirstNBytes), (MyFile.LastReadDataSize - SkipFirstNBytes));
      // Point to new destination for next loop
      Destination += (MyFile.LastReadDataSize - SkipFirstNBytes);

      // Clear variables that is only use in first loop for handling MKIMAGE header
      SkipFirstNBytes = 0;
      IsFirstBlockOfData = FALSE;

      // Count progress
      BytesReadCounter += MyFile.LastReadDataSize;

      // Update once every 10%
      if (((BytesReadCounter * 100 / MyFile.FileSize) - Percentage) >= 10)
      {
        Percentage = (BytesReadCounter * 100 / MyFile.FileSize);
        InfoPrint ("\r%2d%% ", Percentage);
      }
    }
    ProgressPrint ("\rDone.\r\n");
    // Return with filesize
    if (pFileSize != NULL)
      *pFileSize = MyFile.FileSize;
    return EFI_SUCCESS;
  } else {
    SerialPortPrint ("File not found %a\r\n", TextFileFilename);
    return EFI_NOT_FOUND;
  }
}



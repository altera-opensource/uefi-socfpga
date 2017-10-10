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
#include <libfdt.h>
#include <Library/ArmGicLib.h>
#include <Library/ArmLib.h>
#include <Library/ArmCpuLib.h>
#include <Library/BaseMemoryLib.h>
#include <Library/DebugLib.h>
#include <Library/IoLib.h>
#include <Library/MemoryAllocationLib.h>
#include <Library/PcdLib.h>
#include <Library/PrintLib.h>
#include <Library/SerialPortPrintLib.h>
#include <Library/TimerLib.h>
#include <Chipset/ArmArchTimer.h>
#include <Chipset/ArmCortexA5x.h>

#include "AlteraSdMmc/AlteraSdMmcPei.h"
#include "Assert.h"
#include "SdMmc.h"
#include "Boot.h"
#include "MkimageHeader.h"
#include "NandLib.h"

#if (FixedPcdGet32(PcdDebugMsg_Boot) == 0)
  #define ProgressPrint(FormatString, ...)    /* do nothing */
  #define InfoPrint(FormatString, ...)        /* do nothing */
  #define MmioHexDump(BaseAddr, Data32Size)   /* do nothing */
#else
  #define ProgressPrint SerialPortPrint
  #define InfoPrint     SerialPortPrint
  #define MmioHexDump   SerialPortMmioHexDump
#endif
 

VOID
EFIAPI
LoadPei (
  IN  BOOT_SOURCE_TYPE  BootSourceType
  )
{
  EFI_STATUS               Status;
  MKIMG_HEADER             ImgHdr;
  UINT64                   FlashOffset;
  UINTN                    LoadAddr;
  UINTN                    EntryPoint;
  UINT32                   DataSize;

  DataSize = 0;

  // Open and Load Boot Image
  // Objective:
  // 1. Get Data location and Size of the BootImage
  // 2. Get Memory Load Address and CPU Entry Point of the BootImage
  // 3. Copy BootImage from Flash to Memory
  switch (BootSourceType)
  {
    case BOOT_SOURCE_SDMMC:
        // If the file contain an MKIMAGE header,
        // it will utilize the LoadAddr and EntryPoint value from the MKIMAGE header
        // otherwiser, value from the PCDs will be used
        LoadAddr = PcdGet64 (PcdFvBaseAddress);
		Status = LoadBootImageFile (
          (CHAR8*) PcdGetPtr (PcdFileName_PEI_ROM),
          &LoadAddr,
          &EntryPoint,
          &DataSize
          );
      
      // Check if BootImage loading failed?
      if (EFI_ERROR(Status)) {
        ASSERT_PLATFORM_INIT(0);
        // Halt the system
        EFI_DEADLOOP();
      }
      break;

    case BOOT_SOURCE_NAND:
    // case BOOT_SOURCE_QSPI:
      FlashOffset = PcdGet64 (PcdQspiOrNand_BOOTLOADER_PEIROM_ADDR);

      // Check if MKIMAGE header exist
      // if MKIMAGE header exist, image size and entry point will be based on MKIMAGE header
      // else value in PCDs will be used.
      //if (BootSourceType == BOOT_SOURCE_QSPI)
      //{
      //  // Read from QSPI
      //  Status = QspiRead ((VOID *) &ImgHdr, FlashOffset, sizeof(ImgHdr));
      //} else {
        // Read from NAND
        Status = NandRead ((VOID *) &ImgHdr, FlashOffset, sizeof(ImgHdr));
     // }
      // Flash Read Error Checking
      if (EFI_ERROR(Status)) {
        InfoPrint ("Flash Read Error!");
        ASSERT_PLATFORM_INIT(0);
        EFI_DEADLOOP();
      }

      Status = ValidateMkimageHeader(&ImgHdr);
      if (EFI_ERROR(Status)) {
        // Boot Image do not use mkimage header
        DataSize    = PcdGet32 (PcdFvSize);
      } else {
        // Boot Image have mkimage header
        FlashOffset = FlashOffset + sizeof(ImgHdr);
        DataSize    = ImgHdr.DataSize;
        LoadAddr    = ImgHdr.LoadAddr;
        EntryPoint  = ImgHdr.EntryPoint;
      }

      // Print message that we are going to read BootImage from QSPI or NAND flash
      ProgressPrint ("Copying PEI Image from Flash Offset 0x%08lx to Memory Address 0x%08lx where Image's size is 0x%08x bytes\r\n",
        (UINT64) FlashOffset,
        (UINT64) LoadAddr,
        (UINT32) DataSize);
      //if (BootSourceType == BOOT_SOURCE_QSPI)
      //{
      //  // Read from QSPI
      //  Status = QspiRead ((VOID *) LoadAddr, FlashOffset, DataSize);
      //} else {
        // Read from NAND
        Status = NandRead ((VOID *) LoadAddr, FlashOffset, DataSize);
    //  }
      // Flash Read Error Checking
      if (EFI_ERROR(Status)) {
        InfoPrint ("Flash Read Error!");
        ASSERT_PLATFORM_INIT(0);
        EFI_DEADLOOP();
      }
      break;

    case BOOT_SOURCE_RSVD:
    case BOOT_SOURCE_FPGA:
    default:
      // No Flash device.
      ASSERT_PLATFORM_INIT(0);
      EFI_DEADLOOP();
      break;
  }

}



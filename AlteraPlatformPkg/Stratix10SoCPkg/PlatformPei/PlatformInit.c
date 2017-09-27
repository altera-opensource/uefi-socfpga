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

#include "Assert.h"
#include "Boot.h"
#include "Board.h"
#include "NandLib.h"
#include "PitStopUtility.h"
#include "PlatformInit.h"
//#include "QspiLib.h"
#include "SdMmc.h"

#define InfoPrint  SerialPortPrint

//
// Functions
//
EFI_STATUS
EFIAPI
PeiStagePlatformInit (
  VOID
  )
{
  EFI_STATUS        Status;
  BOOT_SOURCE_TYPE  BootSourceType;

  Status = EFI_SUCCESS;

  BootSourceType = BOOT_SOURCE_SDMMC;

  // Detect Boot Source Type
  BootSourceType = GetBootSourceType ();

  // Flash Device initialization
  switch (BootSourceType)
  {
    case BOOT_SOURCE_NAND:
      // Init NAND
      NandInit ();
      break;
     //case BOOT_SOURCE_QSPI:
     // // Init QSPI
     // QspiInit ();
     // break;
    case BOOT_SOURCE_SDMMC:
      // Init SDMMC
      InitSdMmc ();
      break;
    case BOOT_SOURCE_RSVD:
    case BOOT_SOURCE_FPGA:
    default:
      // No Flash device.
      InfoPrint ("No Flash Device Available!\r\n");
      break;
  }

  BoardSpecificInitialization ();

  //
  // Enter Pit Stop utility ?
  //

  if (IsEnterPitStop() == TRUE)
  {
    PitStopCmdLine ();
  }

  //
  // Transfer Control to BootImage now, without going through UEFI DXE phase ?
  //
  if (PcdGet32 (PcdBoot_LOAD_UEFI_DXE_PHASE) != 1)
  {
    LoadBootImageAndTransferControl (
      BootSourceType,
      PcdGet32 (PcdBoot_LOAD_ZIMAGE_AT_PEI_PHASE),
      (CHAR8*) PcdGetPtr (PcdFileName_LINUX_DTB)
      );
  }

  return Status;
}




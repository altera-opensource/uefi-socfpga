/** @file
  Diagnostics Protocol implementation for the MMC DXE driver

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

  Copyright (c) 2011-2014, ARM Limited. All rights reserved.

  This program and the accompanying materials
  are licensed and made available under the terms and conditions of the BSD License
  which accompanies this distribution.  The full text of the license may be found at
  http://opensource.org/licenses/bsd-license.php

  THE PROGRAM IS DISTRIBUTED UNDER THE BSD 3 Clause LICENSE ON AN "AS IS" BASIS,
  WITHOUT WARRANTIES OR REPRESENTATIONS OF ANY KIND, EITHER EXPRESS OR IMPLIED.

**/

//
// Include files
//
#include <Uefi.h>
#include <IndustryStandard/Mbr.h>
#include <Library/AlteraSdMmcLib.h>
#include <Library/BaseMemoryLib.h>
#include <Library/DebugLib.h>
#include <Library/DevicePathLib.h>
#include <Library/IoLib.h>
#include <Library/PcdLib.h>
#include <Library/SerialPortPrintLib.h>
#include <Library/TimerLib.h>
#include "Diagnostics.h"
#include "Mmc.h"
#include "SdMmcHostProtocol.h"

#ifdef SDMMC_DIAGNOSTICS

#define MmioHexDumpEx   SerialPortMmioHexDumpEx
//
// Functions
//
VOID
GenerateRandomBuffer (
  VOID* Buffer,
  UINTN BufferSize
  )
{
  UINT64  i;
  UINT64* Buffer64 = (UINT64*)Buffer;

  for (i = 0; i < (BufferSize >> 3); i++) {
    *Buffer64 = i | (~i << 32);
    Buffer64++;
  }
}

BOOLEAN
CompareBuffer (
  VOID* BufferA,
  VOID* BufferB,
  UINTN BufferSize
  )
{
  UINTN i;
  UINT64* BufferA64 = (UINT64*)BufferA;
  UINT64* BufferB64 = (UINT64*)BufferB;

  for (i = 0; i < (BufferSize >> 3); i++) {
    if (*BufferA64 != *BufferB64) {
      SerialPortPrint ("CompareBuffer: Error at %i", i);
      SerialPortPrint ("(0x%lX) != (0x%lX)\n", *BufferA64, *BufferB64);
      return FALSE;
    }
    BufferA64++;
    BufferB64++;
  }
  return TRUE;
}

EFI_STATUS
MmcReadWriteDataTest (
  PEI_SDMMC_INSTANCE* MmcHostInstance,
  EFI_LBA             Lba
  )
{
  UINT8                       BackBufferArray[512];
  UINT8                       WriteBufferArray[512];
  UINT8                       ReadBufferArray[512];
  VOID*                       BackBuffer;
  VOID*                       WriteBuffer;
  VOID*                       ReadBuffer;
  EFI_STATUS                  Status;

  // Check if a Media is Present
  if (!MmcHostInstance->BlockIo.Media->MediaPresent) {
    SerialPortPrint ("ERROR: No Media Present\r\n");
    return EFI_NO_MEDIA;
  }

  if (MmcHostInstance->State != MmcTransferState) {
    SerialPortPrint ("ERROR: Not ready for Transfer state\r\n");
    return EFI_NOT_READY;
  }

  BackBuffer  = &BackBufferArray[0];
  WriteBuffer = &WriteBufferArray[0];
  ReadBuffer  = &ReadBufferArray[0];

  // Read (and save) buffer at a specific location
  SerialPortPrint ("Read (and save) buffer at a specific location\r\n");
  Status = MmcReadBlocks (&(MmcHostInstance->BlockIo), MmcHostInstance->BlockIo.Media->MediaId,Lba,MmcHostInstance->BlockIo.Media->BlockSize,BackBuffer);
  if (Status != EFI_SUCCESS) {
    SerialPortPrint ("ERROR: Fail to Read Block (1)\r\n");
    return Status;
  }
  MmioHexDumpEx((UINTN)BackBuffer, 512/4, (UINTN)Lba);

  // Write buffer at the same location
  SerialPortPrint ("Write buffer at the same location\r\n");
  GenerateRandomBuffer (WriteBuffer,MmcHostInstance->BlockIo.Media->BlockSize);
  Status = MmcWriteBlocks (&(MmcHostInstance->BlockIo), MmcHostInstance->BlockIo.Media->MediaId,Lba,MmcHostInstance->BlockIo.Media->BlockSize,WriteBuffer);
  if (Status != EFI_SUCCESS) {
    SerialPortPrint ("ERROR: Fail to Write Block (1)\r\n");
    return Status;
  }
  MmioHexDumpEx((UINTN)WriteBuffer, 512/4, (UINTN)Lba);

  // Read the buffer at the same location
  SerialPortPrint ("Read the buffer at the same location\r\n");
  Status = MmcReadBlocks (&(MmcHostInstance->BlockIo), MmcHostInstance->BlockIo.Media->MediaId,Lba,MmcHostInstance->BlockIo.Media->BlockSize,ReadBuffer);
  if (Status != EFI_SUCCESS) {
    SerialPortPrint ("ERROR: Fail to Read Block (2)\r\n");
    return Status;
  }
  MmioHexDumpEx((UINTN)ReadBuffer, 512/4, (UINTN)Lba);

  // Check that is conform
  SerialPortPrint ("Check that is conform\r\n");
  if (!CompareBuffer (ReadBuffer,WriteBuffer,MmcHostInstance->BlockIo.Media->BlockSize)) {
    SerialPortPrint ("ERROR: Fail to Read/Write Block (1)\r\n");
    return EFI_INVALID_PARAMETER;
  }

  // Restore content at the original location
  SerialPortPrint ("Restore content at the original location\r\n");
  Status = MmcWriteBlocks (&(MmcHostInstance->BlockIo), MmcHostInstance->BlockIo.Media->MediaId,Lba,MmcHostInstance->BlockIo.Media->BlockSize,BackBuffer);
  if (Status != EFI_SUCCESS) {
    SerialPortPrint ("ERROR: Fail to Write Block (2)\r\n");
    return Status;
  }

  // Read the restored content
  SerialPortPrint ("Read the restored content\r\n");
  Status = MmcReadBlocks (&(MmcHostInstance->BlockIo), MmcHostInstance->BlockIo.Media->MediaId,Lba,MmcHostInstance->BlockIo.Media->BlockSize,ReadBuffer);
  if (Status != EFI_SUCCESS) {
    SerialPortPrint ("ERROR: Fail to Read Block (3)\r\n");
    return Status;
  }
  MmioHexDumpEx((UINTN)ReadBuffer, 512/4, (UINTN)Lba);

  // Check the content is correct
  SerialPortPrint ("Check the content is correct\r\n");
  if (!CompareBuffer (ReadBuffer,BackBuffer,MmcHostInstance->BlockIo.Media->BlockSize)) {
    SerialPortPrint ("ERROR: Fail to Read/Write Block (2)\r\n");
    return EFI_INVALID_PARAMETER;
  }

  return EFI_SUCCESS;
}

EFI_STATUS
EFIAPI
PeiSdMmcRunDiagnostics (
  IN PEI_SDMMC_INSTANCE*  MmcHostInstance
  )
{
  EFI_STATUS              Status;

  if (MmcHostInstance->BlockIo.Media->LastBlock != 0) {

    // LBA=1 Size=BlockSize
    SerialPortPrint ("MMC Driver Diagnostics - Test: LBA Block 1\n");
    Status = MmcReadWriteDataTest (MmcHostInstance, 1);

    // LBA=LastBlock Size=BlockSize
    SerialPortPrint ("MMC Driver Diagnostics - Test: Last Block\n");
    Status = MmcReadWriteDataTest (MmcHostInstance, MmcHostInstance->BlockIo.Media->LastBlock);

  } else {
    Status = EFI_NO_MEDIA;
  }

  return Status;
}
#endif


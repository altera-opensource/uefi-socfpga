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
#include <Library/BaseMemoryLib.h>
#include <Library/DebugLib.h>
#include <Library/MemoryAllocationLib.h>
#include <Library/SerialPortPrintLib.h>
#include "MemoryTest.h"
#include "NandBlockIo.h"
#include "NandLib.h"

#define InfoPrint      SerialPortPrint
#define MmioHexDump   SerialPortMmioHexDump

EFI_STATUS
EFIAPI
NandReset (
  IN EFI_BLOCK_IO_PROTOCOL    *This,
  IN BOOLEAN                  ExtendedVerification
  )
{
  //This->Media->MediaPresent = FALSE;
  //NandHostInstance->BlockIo.Media->LastBlock    = 0;
  //NandHostInstance->BlockIo.Media->BlockSize    = 512;  // Should be zero but there is a bug in DiskIo
  //NandHostInstance->BlockIo.Media->ReadOnly     = FALSE;

  InfoPrint ("NandFlashInit\n");
  return NandInit();

}


EFI_STATUS
EFIAPI
NandReadBlocks (
  IN EFI_BLOCK_IO_PROTOCOL    *This,
  IN UINT32                   MediaId,
  IN EFI_LBA                  Lba,
  IN UINTN                    BufferSize,
  OUT VOID                    *Buffer
  )
{
  return NandRead (Buffer, (UINT32)Lba, (UINT32)BufferSize);
}


EFI_STATUS
EFIAPI
NandWriteBlocks (
  IN EFI_BLOCK_IO_PROTOCOL    *This,
  IN UINT32                   MediaId,
  IN EFI_LBA                  Lba,
  IN UINTN                    BufferSize,
  IN VOID                     *Buffer
  )
{
  return NandUpdate (Buffer, (UINT32)Lba, (UINT32)BufferSize);
}


EFI_STATUS
EFIAPI
NandFlushBlocks (
  IN EFI_BLOCK_IO_PROTOCOL  *This
  )
{
  return EFI_SUCCESS;
}


BOOLEAN
EFIAPI
DataCompare (
  IN CONST UINT8 * DataA,
  IN CONST UINT8 * DataB,
  IN UINT32 Num
  )
{
  UINTN i;
  for (i = 0; i < Num; ++i)
  {
    if (DataA[i] != DataB[i])
    {
      InfoPrint("Data mismatch: 0x%02x, 0x%02x, index: %d\n", DataA[i], DataB[i], i);
      return FALSE;
    }
  }
  return TRUE;
}


VOID
EFIAPI
NandBlockIoTest (
  VOID
  )
{
  EFI_BLOCK_IO_PROTOCOL  BlockIo;
  UINT8                  ReadBuffer[2048];
  UINT8                  WriteBuffer[2048];
  UINT8                  *WriteData;
  EFI_STATUS             Status;
  UINTN                  BufferSize = 2048;
  UINTN                  i;

  WriteData = WriteBuffer;
  // Generate Test Data
  for (i = 0; i < BufferSize; ++i)
  {
    WriteBuffer[i] = (UINT8)(i & 0xff);
  }

  InfoPrint ("NandReset\n");
  Status = NandReset (&BlockIo, 0);
   if (Status != EFI_SUCCESS) {
    return;
  }
  Status = NandWriteBlocks (&BlockIo, 0, 0x720000, BufferSize, WriteBuffer);
  if (Status != EFI_SUCCESS) {
    return;
  }
  Status = NandReadBlocks (&BlockIo, 0, 0x720000, BufferSize, ReadBuffer);
  if (Status != EFI_SUCCESS) {
    return;
  }
  InfoPrint ("Read Buffer is \n");
  MmioHexDump((UINTN)ReadBuffer, BufferSize/4);

  InfoPrint ("Write Buffer is \n");
  MmioHexDump((UINTN)WriteData, BufferSize/4);

  if (!DataCompare(ReadBuffer, WriteData, 2048)) {
      InfoPrint("ERROR: Mistmatch read/write data found.\n");
  }

}


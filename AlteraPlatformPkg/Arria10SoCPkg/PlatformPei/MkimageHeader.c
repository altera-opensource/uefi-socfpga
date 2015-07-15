/** @file

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

  Copyright (c) 2008 - 2010, Apple Inc. All rights reserved.<BR>
  Copyright (c) 2011 - 2014, ARM Ltd. All rights reserved.<BR>

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
#include <AlteraPlatform.h>
#include <Library/BaseMemoryLib.h>
#include <Library/DebugLib.h>
#include <Library/IoLib.h>
#include <Library/MemoryAllocationLib.h>
#include <Library/PcdLib.h>
#include <Library/SerialPortPrintLib.h>

#include "MkimageHeader.h"

#if (FixedPcdGet32(PcdDebugMsg_Rbf) == 0) // Borrow RBF's flag
  #define ProgressPrint(FormatString, ...)    /* do nothing */
  #define InfoPrint(FormatString, ...)        /* do nothing */
  #define MmioHexDumpEx(BaseAddr, Data32Size, PrintLineStartAddress)   /* do nothing */
#else
  #define ProgressPrint SerialPortPrint
  #define InfoPrint     SerialPortPrint
  #define MmioHexDumpEx SerialPortMmioHexDumpEx
#endif


EFI_STATUS
EFIAPI
ValidateMkimageHeader (
  IN OUT MKIMG_HEADER* MkImgHdrPtr
  )
{
  UINT8     Buffer[80];
  UINTN     j;
  EFI_TIME  Time;

  // Verify magic
  if (MKIMG_MAGIC != MkImgHdrPtr->Magic) {
    return EFI_DEVICE_ERROR;
  }

  // Swap endianess
  MkImgHdrPtr->HeaderCrc  = SWAP_UINT32(MkImgHdrPtr->HeaderCrc);
  MkImgHdrPtr->Timestamp  = SWAP_UINT32(MkImgHdrPtr->Timestamp);
  MkImgHdrPtr->DataSize   = SWAP_UINT32(MkImgHdrPtr->DataSize);
  MkImgHdrPtr->LoadAddr   = SWAP_UINT32(MkImgHdrPtr->LoadAddr);
  MkImgHdrPtr->EntryPoint = SWAP_UINT32(MkImgHdrPtr->EntryPoint);
  MkImgHdrPtr->DataCrc    = SWAP_UINT32(MkImgHdrPtr->DataCrc);

  //
  // Print Debug Info:
  //
  // Convert image name to ASCII NULL terminalted string
  for (j=0;j<32;j++)
  {
    if (MkImgHdrPtr->Name[j] != 0)
      Buffer[j] = MkImgHdrPtr->Name[j];
    else
      break;
  }
  Buffer[j] = 0;
  InfoPrint("MkImage Name : %a\r\n", Buffer);
  // Time is stored in Unix Epoch format
  EpochToEfiTime(MkImgHdrPtr->Timestamp, &Time);
  InfoPrint("Time Created : %t UTC\r\n", &Time);
  InfoPrint("Data Size    : %d Bytes\r\n", MkImgHdrPtr->DataSize);

  return EFI_SUCCESS;
}


/**
  Converts Epoch seconds (elapsed since 1970 JANUARY 01, 00:00:00 UTC) to EFI_TIME
  Reference from ArmPlatformPkg/Library/PL031RealTimeClockLib/PL031RealTimeClockLib.c
 **/
VOID
EpochToEfiTime (
  IN  UINTN     EpochSeconds,
  OUT EFI_TIME  *Time
  )
{
  UINTN         a;
  UINTN         b;
  UINTN         c;
  UINTN         d;
  UINTN         g;
  UINTN         j;
  UINTN         m;
  UINTN         y;
  UINTN         da;
  UINTN         db;
  UINTN         dc;
  UINTN         dg;
  UINTN         hh;
  UINTN         mm;
  UINTN         ss;
  UINTN         J;

  J  = (EpochSeconds / 86400) + 2440588;
  j  = J + 32044;
  g  = j / 146097;
  dg = j % 146097;
  c  = (((dg / 36524) + 1) * 3) / 4;
  dc = dg - (c * 36524);
  b  = dc / 1461;
  db = dc % 1461;
  a  = (((db / 365) + 1) * 3) / 4;
  da = db - (a * 365);
  y  = (g * 400) + (c * 100) + (b * 4) + a;
  m  = (((da * 5) + 308) / 153) - 2;
  d  = da - (((m + 4) * 153) / 5) + 122;

  Time->Year  = y - 4800 + ((m + 2) / 12);
  Time->Month = ((m + 2) % 12) + 1;
  Time->Day   = d + 1;

  ss = EpochSeconds % 60;
  a  = (EpochSeconds - ss) / 60;
  mm = a % 60;
  b = (a - mm) / 60;
  hh = b % 24;

  Time->Hour        = hh;
  Time->Minute      = mm;
  Time->Second      = ss;
  Time->Nanosecond  = 0;

}


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

  Copyright (c) 2011-2013, ARM Limited. All rights reserved.

  This program and the accompanying materials
  are licensed and made available under the terms and conditions of the BSD License
  which accompanies this distribution.  The full text of the license may be found at
  http://opensource.org/licenses/bsd-license.php

  THE PROGRAM IS DISTRIBUTED UNDER THE BSD 3 Clause LICENSE ON AN "AS IS" BASIS,
  WITHOUT WARRANTIES OR REPRESENTATIONS OF ANY KIND, EITHER EXPRESS OR IMPLIED.

**/

#include <Library/SerialPortPrintLib.h>
#include "Mmc.h"

#if (FixedPcdGet32(PcdDebugMsg_SdMmc) == 0)
//  #define ProgressPrint(FormatString, ...)    /* do nothing */
  #define ProgressPrint                       SerialPortPrint
  #define InfoPrint(FormatString, ...)        /* do nothing */
  #define DEBUG_SDMMC_STATE(s)                /* do nothing */
#else
  #define ProgressPrint SerialPortPrint
  #define InfoPrint     SerialPortPrint
#endif

//#if !defined(MDEPKG_NDEBUG_MSG)
CONST CHAR8* mStrUnit[] = { "100kbit/s", "1Mbit/s", "10Mbit/s", "100MBit/s",
                            "Unknown", "Unknown", "Unknown", "Unknown" };
CONST CHAR8* mStrValue[] = { "1.0", "1.2", "1.3", "1.5", "2.0", "2.5", "3.0", "3.5", "4.0", "4.5", "5.0",
                             "Unknown", "Unknown", "Unknown", "Unknown" };
//#endif


VOID
PrintCID (
  IN UINT32* Cid
  )
{
  InfoPrint ("\t- PrintCID\r\n");
  ProgressPrint (
    "\t- Manufacturer ID: 0x%02x\r\n"
    "\t- OEM ID: %c%c\r\n"
    "\t- Product name: %c%c%c%c%c\r\n"
    "\t- Product revision: %x.%x\r\n"
    "\t- Product serial number: 0x%06X%02X\r\n"
    "\t- Manufacturing date: %d/%d\r\n",
    (Cid[3] >> 24) & 0xFF,
    (Cid[3] >> 16) & 0xFF,  (Cid[3] >>  8) & 0xFF,
    (Cid[3]      ) & 0xFF,  (Cid[2] >> 24) & 0xFF, (Cid[2] >> 16) & 0xFF, (Cid[2] >> 8) & 0xFF, (Cid[2]) & 0xFF,
   ((Cid[1] >> 24)>> 0x04), (Cid[1] >> 24) & 0x0F,
     Cid[1]  & 0x00FFFFFF,  (Cid[0] >> 24) & 0xFF,
    (Cid[0] >> 8)  & 0x0F,  (Cid[0] >> 12) & 0xFF);
}


VOID
PrintCSD (
  IN UINT32* Csd
  )
{
  UINTN Value;

  if (((Csd[2] >> 30) & 0x3) == 0) {
    InfoPrint ("\t- PrintCSD Version 1.01-1.10/Version 2.00/Standard Capacity\r\n");
  } else if (((Csd[2] >> 30) & 0x3) == 1) {
    InfoPrint ("\t- PrintCSD Version 2.00/High Capacity\r\n");
  } else {
    InfoPrint ("\t- PrintCSD Version Higher than v3.3\r\n");
  }

  ProgressPrint (
    "\t- Supported card command class: 0x%X\r\n"
    "\t- Speed: %a %a\r\n",
    MMC_CSD_GET_CCC (Csd),
    mStrValue[(MMC_CSD_GET_TRANSPEED (Csd) >> 3) & 0xF],mStrUnit[MMC_CSD_GET_TRANSPEED (Csd) & 7]);

  InfoPrint (
    "\t- Maximum Read Data Block: %d\r\n"
    "\t- Maximum Write Data Block: %d\r\n",
    2 << (MMC_CSD_GET_READBLLEN (Csd)-1),
    2 << (MMC_CSD_GET_WRITEBLLEN (Csd)-1));

  if (!MMC_CSD_GET_FILEFORMATGRP (Csd)) {
    Value = MMC_CSD_GET_FILEFORMAT (Csd);
    if (Value == 0) {
      InfoPrint ("\t- Format (0): Hard disk-like file system with partition table\r\n");
    } else if (Value == 1) {
      InfoPrint ("\t- Format (1): DOS FAT (floppy-like) with boot sector only (no partition table)\r\n");
    } else if (Value == 2) {
      InfoPrint ("\t- Format (2): Universal File Format\r\n");
    } else {
      InfoPrint ("\t- Format (3): Others/Unknown\r\n");
    }
  } else {
    InfoPrint ("\t- Format: Reserved\r\n");
  }
}


VOID
PrintRCA (
  IN UINT32 Rca
  )
{
  InfoPrint ("\t- PrintRCA: 0x%X\r\n"
             "\t\t- Status: 0x%X\r\n"
             "\t\t- RCA: 0x%X\r\n",
             Rca,
             Rca & 0xFFFF,
            (Rca >> 16) & 0xFFFF);
}


VOID
PrintOCR (
  IN UINT32 Ocr
  )
{
  UINTN MinV;
  UINTN MaxV;
  UINTN Volts;
  UINTN Loop;

  MinV  = 36;  // 3.6
  MaxV  = 20;  // 2.0
  Volts = 20;  // 2.0

  // The MMC register bits [23:8] indicate the working range of the card
  for (Loop = 8; Loop < 24; Loop++) {
    if (Ocr & (1 << Loop)) {
      if (MinV > Volts) {
        MinV = Volts;
      }
      if (MaxV < Volts) {
        MaxV = Volts + 1;
      }
    }
    Volts++;
  }

  InfoPrint ("\t- PrintOCR Ocr (0x%X)\r\n",Ocr);
  ProgressPrint ("\t- Card operating voltage: %d.%d to %d.%d\r\n", MinV/10, MinV % 10, MaxV/10, MaxV % 10);
  if (((Ocr >> 29) & 3) == 0) {
    InfoPrint ("\t- AccessMode: Byte Mode\r\n");
  } else {
    InfoPrint ("\t- AccessMode: Block Mode (0x%X)\r\n", ((Ocr >> 29) & 3));
  }

  if (Ocr & MMC_OCR_POWERUP) {
    InfoPrint ("\t- PowerUp\r\n");
  } else {
    InfoPrint ("\t- Voltage Not Supported\r\n");
  }
}

VOID
PrintResponseR1 (
  IN  UINT32 Response
  )
{
  InfoPrint ( "\tResponse: 0x%X\r\n", Response);
  if (Response & MMC_R0_READY_FOR_DATA) {
    InfoPrint ( "\t\t- READY_FOR_DATA\r\n");
  }

  switch ((Response >> 9) & 0xF) {
  case 0:
    InfoPrint ( "\t\t- State: Idle\r\n");
    break;
  case 1:
    InfoPrint ( "\t\t- State: Ready\r\n");
    break;
  case 2:
    InfoPrint ( "\t\t- State: Ident\r\n");
    break;
  case 3:
    InfoPrint ( "\t\t- State: StandBy\r\n");
    break;
  case 4:
    InfoPrint ( "\t\t- State: Tran\r\n");
    break;
  case 5:
    InfoPrint ( "\t\t- State: Data\r\n");
    break;
  case 6:
    InfoPrint ( "\t\t- State: Rcv\r\n");
    break;
  case 7:
    InfoPrint ( "\t\t- State: Prg\r\n");
    break;
  case 8:
    InfoPrint ( "\t\t- State: Dis\r\n");
    break;
  default:
    InfoPrint ( "\t\t- State: Reserved\r\n");
    break;
  }
}


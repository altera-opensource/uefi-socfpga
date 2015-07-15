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
#include "Banner.h"

VOID
EFIAPI
DisplayFirmwareVersion (
  VOID
  )
{
  SerialPortPrint ("UEFI firmware version %s built at %a on %a\r\n",
                   (CHAR16*)PcdGetPtr(PcdFirmwareVersionString),
                   __TIME__, __DATE__);
}

VOID
EFIAPI
DisplayBanner (
  VOID
  )
{
  // Display Firmware Version
  DisplayFirmwareVersion ();

  // Display Platform Info
  SerialPortPrint ( "%s\r\n"
                   "Hard Processor System - Silicon ID 1: 0x%08x\r\n"
                   "Hard Processor: ARM(R) Cortex(R)-A9 MPCore\r\n"
                   "ARM(R) TrustZone(R) is ",
                   (CHAR16*)PcdGetPtr(PcdPlatformNameString),
                   MmioRead32 (ALT_SYSMGR_OFST + ALT_SYSMGR_SILICONID1_OFST));
  if (FixedPcdGetBool (PcdTrustzoneSupport)) {
    SerialPortPrint ("enabled\r\n");
  } else {
    SerialPortPrint ("disabled\r\n");
  }
}


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
#include "DeviceTree.h"
#include "SystemManager.h"

#if (FixedPcdGet32(PcdDebugMsg_SystemManager) == 0)
  #define ProgressPrint(FormatString, ...)    /* do nothing */
  #define InfoPrint(FormatString, ...)        /* do nothing */
#else
  #define ProgressPrint SerialPortPrint
  #define InfoPrint     SerialPortPrint
#endif


//
// Functions
//

VOID
EFIAPI
DisplaySystemManagerInfo (
  VOID
  )
{
  UINT32          Data32;

  // Display bootinfo Register
  Data32 = MmioRead32 (ALT_SYSMGR_OFST + ALT_SYSMGR_BOOT_OFST);
  InfoPrint ( "BootInfo reg: 0x%08x\r\n",
              Data32);
  if (ALT_SYSMGR_BOOT_BSEL_GET(Data32) == ALT_SYSMGR_BOOT_BSEL_E_RSVDX)
    InfoPrint ("\t BSEL = RSVD \r\n");
  if (ALT_SYSMGR_BOOT_BSEL_GET(Data32) == ALT_SYSMGR_BOOT_BSEL_E_FPGA)
    InfoPrint ("\t BSEL = FPGA \r\n");
  if (ALT_SYSMGR_BOOT_BSEL_GET(Data32) == ALT_SYSMGR_BOOT_BSEL_E_NAND_FLSH_1_8V)
    InfoPrint ("\t BSEL = NAND 1.8V \r\n");
  if (ALT_SYSMGR_BOOT_BSEL_GET(Data32) == ALT_SYSMGR_BOOT_BSEL_E_NAND_FLSH_3_0V)
    InfoPrint ("\t BSEL = NAND 3.0V \r\n");
  if (ALT_SYSMGR_BOOT_BSEL_GET(Data32) == ALT_SYSMGR_BOOT_BSEL_E_SD_MMC_EXTERNAL_TRANSCEIVER_1_8V)
    InfoPrint ("\t BSEL = SDMMC 1.8V \r\n");
  if (ALT_SYSMGR_BOOT_BSEL_GET(Data32) == ALT_SYSMGR_BOOT_BSEL_E_SD_MMC_INTERNAL_TRANSCEIVER_3_0V)
    InfoPrint ("\t BSEL = SDMMC_3.0V \r\n");
  if (ALT_SYSMGR_BOOT_BSEL_GET(Data32) == ALT_SYSMGR_BOOT_BSEL_E_QSPI_FLSH_1_8V)
    InfoPrint ("\t BSEL = QSPI 1.8V \r\n");
  if (ALT_SYSMGR_BOOT_BSEL_GET(Data32) == ALT_SYSMGR_BOOT_BSEL_E_QSPI_FLSH_3_0V)
    InfoPrint ("\t BSEL = QSPI 3.0V \r\n");

}

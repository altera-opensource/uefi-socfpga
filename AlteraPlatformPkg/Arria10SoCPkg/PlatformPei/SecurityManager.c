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
#include <Library/PrintLib.h>
#include <Library/SerialPortLib.h>
#include <Library/SerialPortPrintLib.h>
#include "DeviceTree.h"
#include "SecurityManager.h"

#if (FixedPcdGet32(PcdDebugMsg_SecurityManager) == 0)
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
DisplaySecurityManagerInfo (
  VOID
  )
{
  UINT32          Data32;

  // Char8Str is use to speed up Semihosting printing speed
  CHAR8   Char8Str[2048];
  CHAR8*  Char8Ptr = &Char8Str[0];

  // Display HPS_cursecstate
  Data32 = MmioRead32 (ALT_SEC_MGR_OFST + ALT_SEC_MGR_CURSECSTATE_OFST);
  Char8Ptr += AsciiSPrint (Char8Ptr, 1024,
              "Security Manager current state (HPS_cursecstate reg): 0x%08x\r\n",
              Data32);
  if (ALT_SEC_MGR_CURSECSTATE_STATE_AUTHEN_EN_GET(Data32) ==
      ALT_SEC_MGR_CURSECSTATE_STATE_AUTHEN_EN_E_AUTH_EN)
    Char8Ptr += AsciiSPrint (Char8Ptr, 1024, "\t AUTH_EN \r\n");
  else
    Char8Ptr += AsciiSPrint (Char8Ptr, 1024, "\t AUTH_DIS \r\n");
  if (ALT_SEC_MGR_CURSECSTATE_STATE_AES_EN_GET(Data32) ==
      ALT_SEC_MGR_CURSECSTATE_STATE_AES_EN_E_AES_EN)
    Char8Ptr += AsciiSPrint (Char8Ptr, 1024, "\t AES_EN  \r\n");
  else
    Char8Ptr += AsciiSPrint (Char8Ptr, 1024, "\t AES_DIS \r\n");
  if (ALT_SEC_MGR_CURSECSTATE_STATE_FPGA_BOOT_GET(Data32) ==
      ALT_SEC_MGR_CURSECSTATE_STATE_FPGA_BOOT_E_FPGA_BOOT_EN)
    Char8Ptr += AsciiSPrint (Char8Ptr, 1024, "\t FPGA_BOOT_EN  \r\n");
  else
    Char8Ptr += AsciiSPrint (Char8Ptr, 1024, "\t FPGA_BOOT_DIS \r\n");
  if (ALT_SEC_MGR_CURSECSTATE_STATE_CLK_GET(Data32) ==
      ALT_SEC_MGR_CURSECSTATE_STATE_CLK_E_IOSC_CLK)
    Char8Ptr += AsciiSPrint (Char8Ptr, 1024, "\t HPS_IOSC_CLK  \r\n");
  else
    Char8Ptr += AsciiSPrint (Char8Ptr, 1024, "\t HPS_EOSC1_CLK \r\n");
  if (ALT_SEC_MGR_CURSECSTATE_STATE_OC_BOOT_GET(Data32) ==
      ALT_SEC_MGR_CURSECSTATE_STATE_OC_BOOT_E_OCRAM_BOOT_DIS)
    Char8Ptr += AsciiSPrint (Char8Ptr, 1024, "\t OCRAM_BOOT_DIS \r\n");
  else
    Char8Ptr += AsciiSPrint (Char8Ptr, 1024, "\t OCRAM_BOOT_EN \r\n");
  if (ALT_SEC_MGR_CURSECSTATE_STATE_CLR_RAM_WARM_GET(Data32) ==
      ALT_SEC_MGR_CURSECSTATE_STATE_CLR_RAM_WARM_E_CLR_RAM_WARM_EN)
    Char8Ptr += AsciiSPrint (Char8Ptr, 1024, "\t CLR_RAM_WARM_DIS  \r\n");
  else
    Char8Ptr += AsciiSPrint (Char8Ptr, 1024, "\t CLR_RAM_WARM_DIS \r\n");
  if (ALT_SEC_MGR_CURSECSTATE_STATE_CLR_RAM_COLD_GET(Data32) ==
      ALT_SEC_MGR_CURSECSTATE_STATE_CLR_RAM_COLD_E_CLR_RAM_COLD_EN)
    Char8Ptr += AsciiSPrint (Char8Ptr, 1024, "\t CLR_RAM_COLD_EN  \r\n");
  else
    Char8Ptr += AsciiSPrint (Char8Ptr, 1024, "\t CLR_RAM_COLD_DIS \r\n");
  if (ALT_SEC_MGR_CURSECSTATE_STATE_CLR_RAM_ORDER_GET(Data32) ==
      ALT_SEC_MGR_CURSECSTATE_STATE_CLR_RAM_ORDER_E_CLR_RAM_ORDER_SER)
    Char8Ptr += AsciiSPrint (Char8Ptr, 1024, "\t CLR_RAM_ORDER_PAR  \r\n");
  else
    Char8Ptr += AsciiSPrint (Char8Ptr, 1024, "\t CLR_RAM_ORDER_SER \r\n");
  // state_debug
  if ((Data32 & BIT22) != 0)
    Char8Ptr += AsciiSPrint (Char8Ptr, 1024, "\t dbg_disable_access \r\n");
  if ((Data32 & BIT21) != 0)
    Char8Ptr += AsciiSPrint (Char8Ptr, 1024, "\t dbg_lock_JTAG \r\n");
  if ((Data32 & BIT20) != 0)
    Char8Ptr += AsciiSPrint (Char8Ptr, 1024, "\t dbg_lock_DAP \r\n");
  if ((Data32 & BIT19) != 0)
    Char8Ptr += AsciiSPrint (Char8Ptr, 1024, "\t dbg_lock_CPU0 \r\n");
  if ((Data32 & BIT18) != 0)
    Char8Ptr += AsciiSPrint (Char8Ptr, 1024, "\t dbg_lock_CPU1 \r\n");
  if ((Data32 & BIT17) != 0)
    Char8Ptr += AsciiSPrint (Char8Ptr, 1024, "\t dbg_lock_CS \r\n");
  if ((Data32 & BIT16) != 0)
    Char8Ptr += AsciiSPrint (Char8Ptr, 1024, "\t dbg_lock_FPGA \r\n");

  Char8Ptr += AsciiSPrint (Char8Ptr, 1024,
             "SEC_MGR hps_fusesec reg: 0x%08x\r\n"
             "SEC_MGR hps_fusesec2 reg: 0x%08x\r\n"
             "SEC_MGR fpga_fusesec reg: 0x%08x\r\n"
             "SEC_MGR HPS_Swoptset reg: 0x%08x\r\n"
             "SEC_MGR sec_fpgachk reg: 0x%08x\r\n"
             "SEC_MGR sec_hpschk reg: 0x%08x\r\n"
             "SEC_MGR swromcode reg: 0x%08x\r\n",
              MmioRead32 (ALT_SEC_MGR_OFST + ALT_SEC_MGR_FUSESEC_OFST),
              MmioRead32 (ALT_SEC_MGR_OFST + ALT_SEC_MGR_FUSESEC2_OFST),
              MmioRead32 (ALT_SEC_MGR_OFST + ALT_SEC_MGR_FPGA_FUSESEC_OFST),
              MmioRead32 (ALT_SEC_MGR_OFST + ALT_SEC_MGR_SWOPTSET_OFST),
              MmioRead32 (ALT_SEC_MGR_OFST + ALT_SEC_MGR_SEC_FPGACHK_OFST),
              MmioRead32 (ALT_SEC_MGR_OFST + ALT_SEC_MGR_SEC_HPSCHK_OFST),
              MmioRead32 (ALT_SEC_MGR_OFST + ALT_SEC_MGR_SEC_SWROMCODE_OFST));

  InfoPrint ("%a", Char8Str); Char8Ptr = &Char8Str[0];
}



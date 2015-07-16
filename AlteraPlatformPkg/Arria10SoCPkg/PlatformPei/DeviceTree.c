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
#include <libfdt.h>
#include <Library/BaseMemoryLib.h>
#include <Library/IoLib.h>
#include <Library/MemoryAllocationLib.h>
#include <Library/PrintLib.h>
#include <Library/SerialPortPrintLib.h>
#include "Assert.h"
#include "DeviceTree.h"
#include "DeviceTreeDefStr.h"

#if (FixedPcdGet32(PcdDebugMsg_FDT) == 0)
  #define InfoPrint(FormatString, ...)    /* do nothing */
#else
  #define InfoPrint SerialPortPrint
#endif

#define FDT_HEADER_MAGICNUM 0xedfe0dd0

//
// Functions
//

EFI_STATUS
EFIAPI
GetFlattenedDeviceTreePtr (
  OUT VOID**              PtrToDeviceTreePtr
  )
{
  UINT32        DeviceTreeBaseAddress;
  BOOLEAN       DtbFileValid;

  DeviceTreeBaseAddress = PcdGet32(PcdFvDtbBaseAddress);

  *PtrToDeviceTreePtr = (VOID*)DeviceTreeBaseAddress;

  // Assert if DTB header is not 0xedfe0dd0 in little endian (0xdoodfeed in big endian)
  DtbFileValid = (*(UINT32*)DeviceTreeBaseAddress == FDT_HEADER_MAGICNUM)? TRUE : FALSE;

  if (DtbFileValid == FALSE) {
    InfoPrint ("Invalid DTB file header.\r\n");
    return EFI_NOT_FOUND;
  }

  return EFI_SUCCESS;
}

VOID
EFIAPI
GetFdtPropertyValue (
  IN CONST VOID*           Fdt,
  IN       INT32           Node,
  IN       UINT32*         Cfg,
  IN       UINT32          CfgSize,
  IN       PROPERTY_NAME   CfgStr[]
  )
{
  CONST UINT32* Prop32Ptr;
  UINT32        ArraySize;
  UINT32        Index;
  UINT32        Val;
  UINT32        *Vcfg;

  // Char8Str is use to speed up Semihosting printing speed
  CHAR8   Char8Str[2048];
  CHAR8*  Char8Ptr = &Char8Str[0];

  ZeroMem (Cfg, sizeof *Cfg);
  ArraySize = CfgSize / sizeof (UINT32);

  for (Index = 0; Index < ArraySize; Index++) {
    Prop32Ptr = fdt_getprop(Fdt, Node, CfgStr[Index].PropName, NULL);
    if (Prop32Ptr == NULL) {
      Char8Ptr += AsciiSPrint (Char8Ptr, 1024, "FDT: %a not found\r\n", CfgStr[Index].PropName);
      ASSERT_PLATFORM_INIT(Prop32Ptr != NULL);
      continue;
    }
    Val = fdt32_to_cpu (ReadUnaligned32(Prop32Ptr));
    Vcfg = (UINT32 *)Cfg + Index;
    *Vcfg = Val;
    Char8Ptr += AsciiSPrint (Char8Ptr, 1024, "\t FDT: %a : 0x%x\n", CfgStr[Index].PropName,  Val);
  }

  InfoPrint ("%a", Char8Str);
}


EFI_STATUS
EFIAPI
GetClockSourceCfg (
  IN  CONST VOID*                  Fdt,
  OUT       CLOCK_SOURCE_CONFIG*   Cfg
  )
{
  INT32         Node, ChildNode;
  CONST UINT32* Prop32Ptr;
  UINT32        Val;

  Cfg->clk_freq_of_eosc1    = 0;
  Cfg->clk_freq_of_f2h_free = 0;
  Cfg->clk_freq_of_cb_intosc_ls = 0;

  Node = fdt_subnode_offset(Fdt, 0, NODE_clocks);
  if (Node == -FDT_ERR_NOTFOUND) {
    InfoPrint ("FDT: %a not found\r\n", NODE_clocks);
    return EFI_NOT_FOUND;
  }

  ChildNode = fdt_subnode_offset(Fdt, Node, NODE_clock_src_eosc1);
  if (ChildNode == -FDT_ERR_NOTFOUND) {
    InfoPrint ("FDT: %a not found\r\n", NODE_clock_src_eosc1);
    return EFI_NOT_FOUND;
  } else {
    Prop32Ptr = fdt_getprop(Fdt, ChildNode, PROP_clock_frequency, NULL);
    if (Prop32Ptr == NULL) {
      InfoPrint ("FDT: %a not found\r\n", PROP_clock_frequency);
      return EFI_NOT_FOUND;
    } else {
      Val = fdt32_to_cpu (ReadUnaligned32(Prop32Ptr));
      Cfg->clk_freq_of_eosc1 = Val;
    }
  }

  ChildNode = fdt_subnode_offset(Fdt, Node, NODE_clock_src_f2h);
  if (ChildNode == -FDT_ERR_NOTFOUND) {
    InfoPrint ("FDT: %a not found\r\n", NODE_clock_src_f2h);
    return EFI_NOT_FOUND;
  } else {
    Prop32Ptr = fdt_getprop(Fdt, ChildNode, PROP_clock_frequency, NULL);
    if (Prop32Ptr == NULL) {
      InfoPrint ("FDT: %a not found\r\n", PROP_clock_frequency);
      return EFI_NOT_FOUND;
    } else {
      Val = fdt32_to_cpu (ReadUnaligned32(Prop32Ptr));
      Cfg->clk_freq_of_f2h_free = Val;
    }
  }

  ChildNode = fdt_subnode_offset(Fdt, Node, NODE_clock_src_intosc);
  if (ChildNode == -FDT_ERR_NOTFOUND) {
    //It is OK, if altera_arria10_hps_cb_intosc_ls not found, since it is optional
    //InfoPrint ("FDT: %a not found\r\n", NODE_clock_src_intosc);
  } else {
    Prop32Ptr = fdt_getprop(Fdt, ChildNode, PROP_clock_frequency, NULL);
    if (Prop32Ptr == NULL) {
      InfoPrint ("FDT: %a not found\r\n", PROP_clock_frequency);
      return EFI_NOT_FOUND;
    } else {
      Val = fdt32_to_cpu (ReadUnaligned32(Prop32Ptr));
      Cfg->clk_freq_of_cb_intosc_ls = Val;
    }
  }

  InfoPrint ("\t FDT: eosc1        : %d Hz\n"
             "\t FDT: f2h_free     : %d Hz\n"
             "\t FDT: cb_intosc_ls : %d Hz\n",
             Cfg->clk_freq_of_eosc1,
             Cfg->clk_freq_of_f2h_free,
             Cfg->clk_freq_of_cb_intosc_ls);

  return EFI_SUCCESS;
}


EFI_STATUS
EFIAPI
GetClockManagerCfg (
  IN  CONST VOID*                  Fdt,
  OUT       CLOCK_MANAGER_CONFIG*  Cfg
  )
{
  INT32 Node, ChildNode;

  Node = fdt_node_offset_by_compatible(Fdt, -1, COMP_clock_manager);
  if (Node == -FDT_ERR_NOTFOUND) {
    InfoPrint ("FDT: %a not found\r\n", COMP_clock_manager);
    return EFI_NOT_FOUND;
  }

  ChildNode = fdt_subnode_offset(Fdt, Node, NODE_mainpll);
  if (ChildNode == -FDT_ERR_NOTFOUND) {
    InfoPrint ("FDT: %a not found\r\n", NODE_mainpll);
    return EFI_NOT_FOUND;
  }
  GetFdtPropertyValue (Fdt, ChildNode,
                      (UINT32 *)&Cfg->mainpll,
                         sizeof (Cfg->mainpll),
                          ClockManagerMainPllCfgStr);

  ChildNode = fdt_subnode_offset(Fdt, Node, NODE_perpll);
  if (ChildNode == -FDT_ERR_NOTFOUND) {
    InfoPrint ("FDT: %a not found\r\n", NODE_perpll);
    return EFI_NOT_FOUND;
  }
  GetFdtPropertyValue (Fdt, ChildNode,
                       (UINT32 *)&Cfg->perpll,
                          sizeof (Cfg->perpll),
                           ClockManagerPerPllCfgStr);

  ChildNode = fdt_subnode_offset(Fdt, Node, NODE_alteragrp);
  if (ChildNode == -FDT_ERR_NOTFOUND) {
    InfoPrint ("FDT: %a not found\r\n", NODE_alteragrp);
    return EFI_NOT_FOUND;
  }
  GetFdtPropertyValue (Fdt, ChildNode,
                       (UINT32 *)&Cfg->alteragrp,
                          sizeof (Cfg->alteragrp),
                           ClockManagerAlteraGrpCfgStr);

  return EFI_SUCCESS;
}


VOID
EFIAPI
ConfigPinMux (
  IN  CONST VOID*   Fdt,
  IN  CONST CHAR8*  SubNodeName
  )
{
  INT32             Node, ChildNode;
  CONST UINT32*     Prop32Ptr;
  UINT32            BaseAddress, Offset, Value, Temp;
  UINT32            Index =0;

  Node = fdt_node_offset_by_compatible(Fdt, -1, COMP_pinmux);
  if (Node == -FDT_ERR_NOTFOUND) {
    InfoPrint ("FDT: %a not found\r\n", COMP_pinmux);
    ASSERT_PLATFORM_INIT(Node >= 0);
    return;
  }
  ChildNode = fdt_subnode_offset(Fdt, Node, SubNodeName);
  if (ChildNode == -FDT_ERR_NOTFOUND) {
    InfoPrint ("FDT: %a not found\r\n", SubNodeName);
    ASSERT_PLATFORM_INIT(ChildNode >= 0);
    return;
  }

  // get reg based address
  Prop32Ptr = fdt_getprop (Fdt, ChildNode, PROP_pinmux_baseaddress, NULL);
  if (Prop32Ptr == NULL) {
    InfoPrint ("FDT: %a not found\r\n", PROP_pinmux_baseaddress);
    ASSERT_PLATFORM_INIT(Prop32Ptr != NULL);
    return;
  }

  BaseAddress = fdt32_to_cpu (ReadUnaligned32(Prop32Ptr));

  // get pin mux offset n value
  Prop32Ptr = fdt_getprop (Fdt, ChildNode, PROP_pinmux_offsetandvalue, NULL);
  if (Prop32Ptr == NULL) {
    InfoPrint ("FDT: %a not found\r\n", PROP_pinmux_offsetandvalue);
    ASSERT_PLATFORM_INIT(Prop32Ptr != NULL);
    return;
  }

  Temp = 0;
  Offset = fdt32_to_cpu (ReadUnaligned32(Prop32Ptr + Index++));
  Value = fdt32_to_cpu (ReadUnaligned32(Prop32Ptr + Index++));

  while (Offset >= Temp) {
    MmioWrite32 (BaseAddress + Offset, Value);
    Temp  = Offset;
    Offset = fdt32_to_cpu (ReadUnaligned32(Prop32Ptr + Index++));
    Value = fdt32_to_cpu (ReadUnaligned32(Prop32Ptr + Index++));
  }

}

VOID
EFIAPI
GetFdtFirewallValue (
  IN CONST VOID*           Fdt,
  IN       INT32           Node,
  IN       UINT32*         Cfg,
  IN       UINT32          CfgSize,
  IN       PROPERTY_NAME   CfgStr[]
  )
{
  CONST UINT32*        Prop32Ptr;
  UINT32               ArraySize;
  UINT32               Index;
  FIREWALL_PROP* Vcfg;

  ZeroMem (Cfg, sizeof *Cfg);
  ArraySize = CfgSize / sizeof(FIREWALL_PROP);

  for (Index = 0; Index < ArraySize; Index++) {
    Vcfg = (FIREWALL_PROP *)Cfg + Index;
    Vcfg->enable = 0;
    Prop32Ptr = fdt_getprop(Fdt, Node, CfgStr[Index].PropName, NULL);
    if (Prop32Ptr == NULL) {
      //InfoPrint ("\t FDT: %a disabled\r\n", CfgStr[Index].PropName);
      continue;
    }
    Vcfg->enable = 1;
    Vcfg->base  = fdt32_to_cpu (ReadUnaligned32(Prop32Ptr + 0));
    Vcfg->limit = fdt32_to_cpu (ReadUnaligned32(Prop32Ptr + 1));
  }

}

VOID
EFIAPI
GetFirewallCfg (
  IN  CONST VOID*                  Fdt,
  OUT       FIREWALL_CONFIG*       Cfg
  )
{
  INT32         Node;
  INT32         ChildNode;

  Node = fdt_node_offset_by_compatible(Fdt, -1, COMP_firewall);
  if (Node == -FDT_ERR_NOTFOUND) {
    InfoPrint ("FDT: %a not found\r\n", COMP_firewall);
    ASSERT_PLATFORM_INIT(Node >= 0);
    return;
  }

  ChildNode = fdt_subnode_offset(Fdt, Node, NODE_firewall);
  if (ChildNode == -FDT_ERR_NOTFOUND) {
    InfoPrint ("FDT: %a not found\r\n", NODE_firewall);
    ASSERT_PLATFORM_INIT(ChildNode >= 0);
    return;
  }

  GetFdtFirewallValue (Fdt, ChildNode,
                           (UINT32 *)Cfg,
                            sizeof (FIREWALL_CONFIG),
                            FirewallCfgStr);

}

VOID
EFIAPI
GetRbfFileCfg (
  IN  CONST VOID*                  Fdt,
  OUT       RBF_FILE_CONFIG*       Cfg
  )
{
  INT32        Node;
  CHAR8*       RbfFileName;

  // Point to "Chosen"
  Node = fdt_subnode_offset(Fdt, 0, NODE_chosen);
  if (Node == -FDT_ERR_NOTFOUND) {
    InfoPrint ("FDT: %a not found\r\n", NODE_chosen);
    ASSERT_PLATFORM_INIT(Node >= 0);
    return;
  }

  // Point to the first filename
  RbfFileName = (CHAR8*)fdt_getprop(Fdt, Node, NODE_cff_file, NULL);
  if ((RbfFileName == NULL) ||
     (*RbfFileName == 0)) {
    InfoPrint ("FDT: %a not found\r\n", NODE_cff_file);
    ASSERT_PLATFORM_INIT(RbfFileName != NULL);
    return;
  }

  // There can be more than one parts of RBF files
  Cfg->NumOfRbfFileParts = 0;
  while(*RbfFileName != 0)
  {
    Cfg->RBF_FileName[Cfg->NumOfRbfFileParts] = RbfFileName;
    InfoPrint ("FDT: RBF file %d = %a\n", Cfg->NumOfRbfFileParts + 1, Cfg->RBF_FileName[Cfg->NumOfRbfFileParts]);
    Cfg->NumOfRbfFileParts += 1;
    ASSERT_PLATFORM_INIT(Cfg->NumOfRbfFileParts <= MAX_NumOfRbfFileParts);
    // Find next RBF file parts if any
    // Point to the next NULL terminated ASCII string
    while(*RbfFileName++ != 0);
  }

}

BOOLEAN
EFIAPI
IsSkipFpgaConfig (
  IN  CONST VOID*   Fdt
  )
{
  INT32        Node;

  // Point to "Chosen"
  Node = fdt_subnode_offset(Fdt, 0, NODE_chosen);
  if (Node == -FDT_ERR_NOTFOUND) {
    InfoPrint ("FDT: %a not found\r\n", NODE_chosen);
    ASSERT_PLATFORM_INIT(Node >= 0);
    return FALSE;
  }

  if (fdt_getprop(Fdt, Node, NODE_ext_fpga_config, NULL) != NULL) {
    InfoPrint ("FDT: external fpga configuration is enabled\r\n");
	return TRUE;
  }

  return FALSE;

}



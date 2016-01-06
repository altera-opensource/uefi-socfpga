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

#include <AlteraPlatform.h>
#include <Library/ArmLib.h>
#include <Library/BaseMemoryLib.h>
#include <Library/DebugLib.h>
#include <Library/IoLib.h>
#include <Library/MemoryAllocationLib.h>
#include <Library/PcdLib.h>
#include <Library/SerialPortPrintLib.h>
#include <Library/TimerLib.h>
#include "Assert.h"
#include "MemoryController.h"
#include "Mmu.h"

#if (FixedPcdGet32(PcdDebugMsg_MemoryController) == 0)
  #define ProgressPrint(FormatString, ...)    /* do nothing */
  #define InfoPrint(FormatString, ...)        /* do nothing */
#else
  #define ProgressPrint SerialPortPrint
  #define InfoPrint     SerialPortPrint
#endif

// The total number of descriptors, including the final "end-of-table" descriptor.
#define MAX_VIRTUAL_MEMORY_MAP_DESCRIPTORS 16
ARM_MEMORY_REGION_DESCRIPTOR  gVirtualMemoryTable[MAX_VIRTUAL_MEMORY_MAP_DESCRIPTORS];

// DDR attributes
#define DDR_ATTRIBUTES_CACHED           ARM_MEMORY_REGION_ATTRIBUTE_WRITE_BACK
#define DDR_ATTRIBUTES_UNCACHED         ARM_MEMORY_REGION_ATTRIBUTE_UNCACHED_UNBUFFERED

/**
  Return the Virtual Memory Map of your platform

  This Virtual Memory Map is used to initialize the MMU for DXE Phase.

  @param[out]   VirtualMemoryMap    Array of ARM_MEMORY_REGION_DESCRIPTOR describing a Physical-to-
                                    Virtual Memory mapping. This array must be ended by a zero-filled
                                    entry

**/
VOID
EFIAPI
ArmPlatformGetVirtualMemoryMap (
  IN ARM_MEMORY_REGION_DESCRIPTOR** VirtualMemoryMap
  )
{
  ARM_MEMORY_REGION_DESCRIPTOR  *VirtualMemoryTable;
  ARM_MEMORY_REGION_ATTRIBUTES  CacheAttributes;
  UINTN                         i, Index = 0;

  VirtualMemoryTable = &gVirtualMemoryTable[0];

  if (FeaturePcdGet(PcdCacheEnable) == TRUE) {
    CacheAttributes = DDR_ATTRIBUTES_CACHED;
  } else {
    CacheAttributes = DDR_ATTRIBUTES_UNCACHED;
  }

  // Start create the Virtual Memory Map table
  // Our goal is to a simple 1:1 mapping where virtual==physical address

  // Check if OCRAM Remap to offset zero then it is OCRAM and not DRAM
  if (PcdGet32(PcdRemapOnChipRamTo1stOneMB) == 1) {
    // From Offset 0x00000000 to the upper bound address range of the OCRAM component
    VirtualMemoryTable[Index].PhysicalBase = 0;
    VirtualMemoryTable[Index].VirtualBase  = VirtualMemoryTable[Index].PhysicalBase;
    VirtualMemoryTable[Index].Length       = ((UINTN) ALT_OCRAM_UB_ADDR - ALT_OCRAM_OFST + 1) - (UINTN) VirtualMemoryTable[Index].PhysicalBase;
    VirtualMemoryTable[Index++].Attributes = CacheAttributes;

    // Unused OCRAM Remap Window hole from ALT_OCRAM_UB_ADDR until 0x000FFFFF
    Index = Index + 1;
    VirtualMemoryTable[Index].PhysicalBase = ((UINTN) ALT_OCRAM_UB_ADDR - ALT_OCRAM_OFST + 1);
    VirtualMemoryTable[Index].VirtualBase  = VirtualMemoryTable[Index].PhysicalBase;
    VirtualMemoryTable[Index].Length       = 0x100000 - (UINTN) VirtualMemoryTable[Index].PhysicalBase;
    VirtualMemoryTable[Index++].Attributes = ARM_MEMORY_REGION_ATTRIBUTE_UNCACHED_UNBUFFERED;
  }

  // DDR SDRAM
  VirtualMemoryTable[Index].PhysicalBase = GetMpuWindowDramBaseAddr();
  VirtualMemoryTable[Index].VirtualBase  = VirtualMemoryTable[Index].PhysicalBase;
  VirtualMemoryTable[Index].Length       = GetMpuWindowDramSize();
  VirtualMemoryTable[Index++].Attributes = CacheAttributes;

  // Unused DDR SDRAM Window
  VirtualMemoryTable[Index].PhysicalBase = GetMpuWindowDramBaseAddr() + GetMpuWindowDramSize();
  VirtualMemoryTable[Index].VirtualBase  = VirtualMemoryTable[Index].PhysicalBase;
  VirtualMemoryTable[Index].Length       = ALT_FPGA_BRIDGE_H2F128_OFST - (UINTN) VirtualMemoryTable[Index].PhysicalBase;
  VirtualMemoryTable[Index++].Attributes = ARM_MEMORY_REGION_ATTRIBUTE_UNCACHED_UNBUFFERED;

  // FPGA Bridge
  VirtualMemoryTable[Index].PhysicalBase = ALT_FPGA_BRIDGE_H2F128_OFST;
  VirtualMemoryTable[Index].VirtualBase  = VirtualMemoryTable[Index].PhysicalBase;
  VirtualMemoryTable[Index].Length       = ((UINTN) ALT_FPGA_BRIDGE_H2F128_UB_ADDR + 1) - (UINTN) VirtualMemoryTable[Index].PhysicalBase;
  VirtualMemoryTable[Index++].Attributes = ARM_MEMORY_REGION_ATTRIBUTE_DEVICE;

  // STM Module
  VirtualMemoryTable[Index].PhysicalBase = 0xFC000000;
  VirtualMemoryTable[Index].VirtualBase  = VirtualMemoryTable[Index].PhysicalBase;
  VirtualMemoryTable[Index].Length       = 0xFF000000 - (UINTN) VirtualMemoryTable[Index].PhysicalBase;
  VirtualMemoryTable[Index++].Attributes = ARM_MEMORY_REGION_ATTRIBUTE_DEVICE;

  // DAP Moudle
  VirtualMemoryTable[Index].PhysicalBase = 0xFF000000;
  VirtualMemoryTable[Index].VirtualBase  = VirtualMemoryTable[Index].PhysicalBase;
  VirtualMemoryTable[Index].Length       = ALT_FPGA_BRIDGE_LWH2F_OFST - (UINTN) VirtualMemoryTable[Index].PhysicalBase;
  VirtualMemoryTable[Index++].Attributes = ARM_MEMORY_REGION_ATTRIBUTE_DEVICE;

  // Light Weight FPGA Bridge
  VirtualMemoryTable[Index].PhysicalBase = ALT_FPGA_BRIDGE_LWH2F_OFST;
  VirtualMemoryTable[Index].VirtualBase  = VirtualMemoryTable[Index].PhysicalBase;
  VirtualMemoryTable[Index].Length       = ((UINTN) ALT_FPGA_BRIDGE_LWH2F_UB_ADDR + 1) - (UINTN) VirtualMemoryTable[Index].PhysicalBase;
  VirtualMemoryTable[Index++].Attributes = ARM_MEMORY_REGION_ATTRIBUTE_DEVICE;

  // Undefined
  VirtualMemoryTable[Index].PhysicalBase = (UINTN) ALT_FPGA_BRIDGE_LWH2F_UB_ADDR + 1;
  VirtualMemoryTable[Index].VirtualBase  = VirtualMemoryTable[Index].PhysicalBase;
  VirtualMemoryTable[Index].Length       = 0XFF800000 - (UINTN) VirtualMemoryTable[Index].PhysicalBase;
  VirtualMemoryTable[Index++].Attributes = ARM_MEMORY_REGION_ATTRIBUTE_UNCACHED_UNBUFFERED;

  // Peripherals Region
  VirtualMemoryTable[Index].PhysicalBase = 0XFF800000;
  VirtualMemoryTable[Index].VirtualBase  = VirtualMemoryTable[Index].PhysicalBase;
  VirtualMemoryTable[Index].Length       = 0x800000;
  VirtualMemoryTable[Index++].Attributes = ARM_MEMORY_REGION_ATTRIBUTE_DEVICE;

  // End of Table
  VirtualMemoryTable[Index].PhysicalBase = 0;
  VirtualMemoryTable[Index].VirtualBase  = 0;
  VirtualMemoryTable[Index].Length       = 0;
  VirtualMemoryTable[Index++].Attributes   = (ARM_MEMORY_REGION_ATTRIBUTES)0;

  ASSERT_PLATFORM_INIT((Index) <= MAX_VIRTUAL_MEMORY_MAP_DESCRIPTORS);

  *VirtualMemoryMap = VirtualMemoryTable;

  // Print VirtualMemoryTable for easier debug

  InfoPrint ("MMU 1:1 Virtual:Physical Mapping Table:\r\n", (UINT32) VirtualMemoryTable);
  for (i = 0; i < Index - 1; i++)
  {
    InfoPrint (
      "    Entry[%02d] = 0x%08Lx - 0x%08Lx : ATTR = 0x%02x\r\n",
        i,
       (UINT64) VirtualMemoryTable[i].PhysicalBase,
       (UINT64) VirtualMemoryTable[i].PhysicalBase + VirtualMemoryTable[i].Length - 1,
       (UINT32) VirtualMemoryTable[i].Attributes
       );
  }


}


VOID
EFIAPI
InitMmu (
  VOID
  )
{
  ARM_MEMORY_REGION_DESCRIPTOR  *MemoryTable;
  VOID                          *TranslationTableBase;
  UINTN                         TranslationTableSize;
  RETURN_STATUS                 Status;

  // Construct a Virtual Memory Map for this platform
  ArmPlatformGetVirtualMemoryMap (&MemoryTable);

  // Configure the MMU
  Status = ArmConfigureMmu (MemoryTable, &TranslationTableBase, &TranslationTableSize);
  if (EFI_ERROR (Status)) {
    InfoPrint ("Error: Failed to enable MMU\n");
    ASSERT_PLATFORM_INIT(0);
  }

  InfoPrint (
    "MMU TTBR0 Address   = 0x%08Lx - 0x%08Lx\n",
    (UINT64)(UINTN)TranslationTableBase,
    (UINT64)(UINTN)TranslationTableBase + TranslationTableSize - 1
    );
}


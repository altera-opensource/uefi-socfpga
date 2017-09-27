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

#define DRAM_BASE                0x0
#define DRAM_SIZE                0x40000000

#define FPGA_SLAVES_BASE         0x80000000
#define FPGA_SLAVES_SIZE         0x60000000

#define PERIPHERAL_BASE          0xF7000000
#define PERIPHERAL_SIZE          0x08E00000

#define OCRAM_BASE               0xFFE00000
#define OCRAM_SIZE               0x00100000

#define GIC_BASE                 0xFFFC0000
#define GIC_SIZE                 0x00008000

#define MEM64_BASE               0x0100000000
#define MEM64_SIZE               0x1F00000000

#define DEVICE64_BASE            0x2000000000
#define DEVICE64_SIZE            0x0100000000
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

   // DDR SDRAM
  VirtualMemoryTable[Index].PhysicalBase = DRAM_BASE;
  VirtualMemoryTable[Index].VirtualBase  = VirtualMemoryTable[Index].PhysicalBase;
  VirtualMemoryTable[Index].Length      = DRAM_SIZE;
  VirtualMemoryTable[Index++].Attributes = CacheAttributes;

  // FPGA
  VirtualMemoryTable[Index].PhysicalBase = FPGA_SLAVES_BASE;
  VirtualMemoryTable[Index].VirtualBase  = VirtualMemoryTable[Index].PhysicalBase;
  VirtualMemoryTable[Index].Length       = FPGA_SLAVES_SIZE;
  VirtualMemoryTable[Index++].Attributes = ARM_MEMORY_REGION_ATTRIBUTE_DEVICE;

  // DEVICE 142MB
  VirtualMemoryTable[Index].PhysicalBase = PERIPHERAL_BASE;
  VirtualMemoryTable[Index].VirtualBase  = VirtualMemoryTable[Index].PhysicalBase;
  VirtualMemoryTable[Index].Length       = PERIPHERAL_SIZE;
  VirtualMemoryTable[Index++].Attributes = ARM_MEMORY_REGION_ATTRIBUTE_DEVICE;

  // OCRAM 1MB but available 256KB
  VirtualMemoryTable[Index].PhysicalBase = OCRAM_BASE;
  VirtualMemoryTable[Index].VirtualBase  = VirtualMemoryTable[Index].PhysicalBase;
  VirtualMemoryTable[Index].Length       = OCRAM_SIZE;
  VirtualMemoryTable[Index++].Attributes = CacheAttributes;

   // DEVICE 32KB
  VirtualMemoryTable[Index].PhysicalBase = GIC_BASE;
  VirtualMemoryTable[Index].VirtualBase  = VirtualMemoryTable[Index].PhysicalBase;
  VirtualMemoryTable[Index].Length       = GIC_SIZE;
  VirtualMemoryTable[Index++].Attributes = ARM_MEMORY_REGION_ATTRIBUTE_DEVICE;

  // MEM 124GB
  VirtualMemoryTable[Index].PhysicalBase = MEM64_BASE;
  VirtualMemoryTable[Index].VirtualBase  = VirtualMemoryTable[Index].PhysicalBase;
  VirtualMemoryTable[Index].Length       = MEM64_SIZE;
  VirtualMemoryTable[Index++].Attributes = CacheAttributes;

   // DEVICE 4GB
  VirtualMemoryTable[Index].PhysicalBase = DEVICE64_BASE;
  VirtualMemoryTable[Index].VirtualBase  = VirtualMemoryTable[Index].PhysicalBase;
  VirtualMemoryTable[Index].Length       = DEVICE64_SIZE;
  VirtualMemoryTable[Index++].Attributes = ARM_MEMORY_REGION_ATTRIBUTE_DEVICE;

  // End of Table
  VirtualMemoryTable[Index].PhysicalBase = 0;
  VirtualMemoryTable[Index].VirtualBase  = 0;
  VirtualMemoryTable[Index].Length       = 0;
  VirtualMemoryTable[Index++].Attributes   = (ARM_MEMORY_REGION_ATTRIBUTES)0;

  //ASSERT_PLATFORM_INIT((Index) <= MAX_VIRTUAL_MEMORY_MAP_DESCRIPTORS);

  *VirtualMemoryMap = VirtualMemoryTable;

  // Print VirtualMemoryTable for easier debug

  InfoPrint ("MMU 1:1 Virtual:Physical Mapping Table:\r\n", (UINT32)(UINTN) VirtualMemoryTable);
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
 InfoPrint ("Init MMU\r\n");
  // Construct a Virtual Memory Map for this platform
  ArmPlatformGetVirtualMemoryMap (&MemoryTable);

  // Configure the MMU
  Status = ArmConfigureMmu (MemoryTable, &TranslationTableBase, &TranslationTableSize);
  if (EFI_ERROR (Status)) {
    InfoPrint ("Error: Failed to enable MMU with Status = 0x%x\n", Status);
    //ASSERT_PLATFORM_INIT(0);
  }

  InfoPrint (
    "MMU TTBR0 Address   = 0x%08Lx - 0x%08Lx\n",
    (UINT64)(UINTN)TranslationTableBase,
    (UINT64)(UINTN)TranslationTableBase + TranslationTableSize - 1
    );
}


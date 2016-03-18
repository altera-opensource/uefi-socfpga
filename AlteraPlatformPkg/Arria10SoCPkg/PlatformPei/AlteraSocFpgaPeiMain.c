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
#include <Ppi/ArmMpCoreInfo.h>
#include <Ppi/GuidedSectionExtraction.h>

#include <Guid/ArmGlobalVariableHob.h>
#include <Guid/LzmaDecompress.h>
#include <Guid/MemoryTypeInformation.h>

#include <AlteraPlatform.h>
#include <PiPei.h>
#include <Library/ArmLib.h>
#include <Library/BaseMemoryLib.h>
#include <Library/DebugAgentLib.h>
#include <Library/DebugLib.h>
#include <Library/HobLib.h>
#include <Library/IoLib.h>
#include <Library/MemoryAllocationLib.h>
#include <Library/PcdLib.h>
#include <Library/PeCoffGetEntryPointLib.h>
#include <Library/PerformanceLib.h>
#include <Library/PrePiHobListPointerLib.h>
#include <Library/PrePiLib.h>
#include <Library/PrintLib.h>
#include <Library/SerialPortLib.h>
#include <Library/SerialPortPrintLib.h>
#include <Library/TimerLib.h>

#include "AlteraSocFpgaPeiMain.h"
#include "Assert.h"
#include "Banner.h"
#include "Boot.h"
#include "LzmaDecompress.h"
#include "MemoryController.h"
#include "Mmu.h"
#include "PlatformInit.h"
#include "ResetManager.h"
#include "SdMmc.h"
#include "SecurityManager.h"
#include "SystemManager.h"

#define EFI_DXE_CORE_GUID \
{ \
  0xD6A2CB7F, 0x6A18, 0x4e2f, {0xB4, 0x3B, 0x99, 0x20, 0xa7, 0x33, 0x70, 0x0a} \
}


typedef
VOID
(EFIAPI *DXE_CORE_ENTRY_POINT) (
  IN  VOID *HobStart
  );

VOID
EFIAPI
AlteraSocFpgaPeiMainEntry (
  IN CONST EFI_SEC_PEI_HAND_OFF        *SecCoreData,
  IN CONST EFI_PEI_PPI_DESCRIPTOR      *PpiList,
  IN VOID                              *Data
  )
{
  // Initialize the Timer Library
  TimerConstructor ();

  // Init Hob and also record time of PEI start
  InitializeHOBs (SecCoreData, PpiList, GetPerformanceCounter());

  // Initialize Memory Serial Log
  MemorySerialLogInit ();

  // Print some useful information to semihosting console if enabled
  PrintModuleEntryPointAndMemoryMapInfoToMemorySerialLogOrSemihostingConsoleIfEnabled (SecCoreData);

  // Initialize Platform
  PeiStagePlatformInit();

  // External Memory should be up, build the System Memory Hobs
  BuildSystemMemoryHOBs (
    GetMpuWindowDramBaseAddr(),
    GetMpuWindowDramSize()
    );

  // Boot UEFI DXE phase
  BootCompressedDxeFv (SecCoreData);    // Try LZMA compressed DXE FV
  BootUnCompressedDxeFv (SecCoreData);  // If it fail try RAW DXE FV

  // Should not reach here
  ASSERT_PLATFORM_INIT(0);
}


VOID
EFIAPI
MemorySerialLogInit (
  VOID
  )
{
  // At this point, UART is not up yet, so the primary purpose of calling
  // this DisplayFirmwareVersion function is to give a header to MemorySerialLogWrite
  DisplayFirmwareVersion ();

  // Display System Manager Info
  DisplaySystemManagerInfo ();

  // Display Reset Manager Info
  DisplayResetManagerInfo ();

  // Display Security Manager Info
  DisplaySecurityManagerInfo ();
}


VOID
EFIAPI
PrintModuleEntryPointAndMemoryMapInfoToMemorySerialLogOrSemihostingConsoleIfEnabled (
  IN CONST EFI_SEC_PEI_HAND_OFF        *SecCoreData
  )
{
  UINT32 ArmPlatformSecEntry;
  UINT32 ArmPlatformPrePeiCoreEntry;
  UINT32 AlteraSocFpgaPeiMainEntry;
  UINT32 SecFvBegin;
  UINT32 SecFvEnd;
  UINT32 PeiFvBegin;
  UINT32 PeiFvEnd;
  UINT32 DtbFvBegin;
  UINT32 DtbFvEnd;
  UINT32 MkpFvBegin;
  UINT32 MkpFvEnd;
  UINT32 MemLogBegin;
  UINT32 MemLogEnd;
  UINT32 TotalPeiMemBegin;
  UINT32 TotalPeiMemEnd;
  UINT32 PpiListBegin;
  UINT32 PpiListEnd;
  UINT32 PeiHeapBegin;
  UINT32 PeiHeapEnd;
  UINT32 PeiStackBegin;
  UINT32 PeiStackEnd;
  UINT32 GlobalVariablePtrBegin;
  UINT32 GlobalVariablePtrEnd;
  UINT32 SecStackBegin;
  UINT32 SecStackEnd;
  UINT32 MonModeStackBegin;
  UINT32 MonModeStackEnd;

  // Use OCRAM base as SEC entry
  ArmPlatformSecEntry = ALT_OCRAM_OFST;
  // Decode the entry offset from BL jump opcode
  ArmPlatformPrePeiCoreEntry = (UINTN)PcdGet64(PcdFvBaseAddress) + (((*((UINT32 *)(UINTN)PcdGet64(PcdFvBaseAddress)) & 0x00FFFFFF) + 2) * 4);
  // Decode the PEI MAIN entry value patched into this location by GenFv tool's UpdateArmResetVectorIfNeeded function
  AlteraSocFpgaPeiMainEntry = (*((UINT32 *)(UINTN)(PcdGet64(PcdFvBaseAddress) + 4)) & 0xFFFFFFFE);

  SecFvBegin = ALT_OCRAM_OFST;
  SecFvEnd = (UINT32)SecCoreData->BootFirmwareVolumeBase - 1;
  PeiFvBegin = (UINT32)SecCoreData->BootFirmwareVolumeBase;
  PeiFvEnd = (UINT32)SecCoreData->BootFirmwareVolumeBase + (UINT32)SecCoreData->BootFirmwareVolumeSize - 1;
  DtbFvBegin = PcdGet32(PcdFvDtbBaseAddress);
  DtbFvEnd = PcdGet32(PcdFvDtbBaseAddress) + PcdGet32(PcdDtbFvSize) - 1;
  MkpFvBegin = PcdGet32(PcdFvDtbBaseAddress) + PcdGet32(PcdDtbFvSize);
  MkpFvEnd = PcdGet32(PcdFvDtbBaseAddress) + PcdGet32(PcdDtbFvSize) + 4 - 1;
  MemLogBegin = (UINT32)PcdGet64 (PcdMemorySerialLogBase);
  MemLogEnd = (UINT32)PcdGet64 (PcdMemorySerialLogBase) + (UINT32)PcdGet64 (PcdMemorySerialLogSize) - 1;

  TotalPeiMemBegin = (UINTN)PcdGet64 (PcdCPUCoresStackBase);
  TotalPeiMemEnd = (UINTN)PcdGet64 (PcdCPUCoresStackBase) + PcdGet32 (PcdCPUCorePrimaryStackSize) - 1;
  PpiListBegin = (UINTN)PcdGet64 (PcdCPUCoresStackBase);
  PpiListEnd = (UINTN)PcdGet64 (PcdCPUCoresStackBase) + (UINT32)SecCoreData->TemporaryRamBase - (UINTN)PcdGet64 (PcdCPUCoresStackBase) - 1;
  PeiHeapBegin = (UINT32)SecCoreData->PeiTemporaryRamBase;
  PeiHeapEnd = (UINT32)SecCoreData->PeiTemporaryRamBase + (UINT32)SecCoreData->PeiTemporaryRamSize - 1;
  PeiStackBegin = (UINT32)SecCoreData->StackBase;
  PeiStackEnd = (UINT32)SecCoreData->StackBase + (UINT32)SecCoreData->StackSize - 1;
  GlobalVariablePtrBegin = (UINTN)PcdGet64 (PcdCPUCoresStackBase) + PcdGet32 (PcdCPUCorePrimaryStackSize) - PcdGet32 (PcdPeiGlobalVariableSize);
  GlobalVariablePtrEnd = (UINTN)PcdGet64 (PcdCPUCoresStackBase) +  PcdGet32 (PcdCPUCorePrimaryStackSize) - 1;
  SecStackBegin = PcdGet32 (PcdCPUCoresSecStackBase);
  SecStackEnd = PcdGet32 (PcdCPUCoresSecStackBase) + PcdGet32 (PcdCPUCoreSecPrimaryStackSize) - 1;
  MonModeStackBegin = PcdGet32 (PcdCPUCoresSecMonStackBase);
  MonModeStackEnd = PcdGet32 (PcdCPUCoresSecMonStackBase) + PcdGet32 (PcdCPUCoreSecMonStackSize) - 1;

  // Visible only on Memory Serial Log or Semihosting console when enabled

  // Print UEFI Entry Points Map
  SerialPortPrint ("UEFI Entry Points Map: \r\n"
                   "  ArmPlatformSec        Entry = 0x%08x \r\n"
                   "  ArmPlatformPrePeiCore Entry = 0x%08x \r\n"
                   "  AlteraSocFpgaPeiMain  Entry = 0x%08x \r\n",
                    ArmPlatformSecEntry,
                    ArmPlatformPrePeiCoreEntry,
                    AlteraSocFpgaPeiMainEntry);


  // Print OCRAM Memory Map
  SerialPortPrint ("OCRAM Memory Map : \r\n"
                   "SEC Firmware Volume  = 0x%08x - 0x%08x\r\n"
                   "PEI Firmware Volume  = 0x%08x - 0x%08x\r\n"
                   "DTB Firmware Volume  = 0x%08x - 0x%08x\r\n"
                   "MkpImage CheckSum    = 0x%08x - 0x%08x\r\n"
                   "Memory Serial Log    = 0x%08x - 0x%08x \r\n",
                    SecFvBegin,  SecFvEnd,
                    PeiFvBegin,  PeiFvEnd,
                    DtbFvBegin,  DtbFvEnd,
                    MkpFvBegin,  MkpFvEnd,
                   MemLogBegin, MemLogEnd);

  // Print Memory Map for PEI Heap and Stack structure
  SerialPortPrint ("PEI Phase Memory Map: (0x%08x - 0x%08x)\r\n"
                   "  Ppi List           = 0x%08x - 0x%08x\r\n"
                   "  PEI Heap           = 0x%08x - 0x%08x\r\n"
                   "  PEI Stack          = 0x%08x - 0x%08x\r\n"
                   "  GlobalVariablePtr  = 0x%08x - 0x%08x\r\n"
                   "SEC Phase Stack      = 0x%08x - 0x%08x\r\n"
                   "Monitor Mode Stack   = 0x%08x - 0x%08x\r\n",
                    TotalPeiMemBegin, TotalPeiMemEnd,
                    PpiListBegin, PpiListEnd,
                    PeiHeapBegin, PeiHeapEnd,
                    PeiStackBegin,PeiStackEnd,
                    GlobalVariablePtrBegin, GlobalVariablePtrEnd,
                    SecStackBegin,SecStackEnd,
                    MonModeStackBegin, MonModeStackEnd);
}


VOID
EFIAPI
InitializeHOBs (
  IN CONST EFI_SEC_PEI_HAND_OFF    *SecCoreData,
  IN CONST EFI_PEI_PPI_DESCRIPTOR  *PpiList,
  IN CONST UINT64                   StartTimeStamp
  )
{
  EFI_STATUS                    Status;
  EFI_HOB_HANDOFF_INFO_TABLE*   HobList;
  ARM_MP_CORE_INFO_PPI*         ArmMpCoreInfoPpi;
  UINTN                         ArmCoreCount;
  ARM_CORE_INFO*                ArmCoreInfoTable;

  // Construct a Hob header at SecCoreData->PeiTemporaryRamBase
  HobList = HobConstructor (
    (VOID*)SecCoreData->PeiTemporaryRamBase,  // The lowest address location of temp memory
    SecCoreData->PeiTemporaryRamSize,         // The size of temp memory
    (VOID*)SecCoreData->PeiTemporaryRamBase,  // The lowest address for use by the HOB producer phase
    (VOID*)SecCoreData->StackBase             // The highest address is StackBase for use by the HOB producer phase
    );

  // Save the head of HobList so that we can retrieve later using PrePeiGetHobList
  PrePeiSetHobList (HobList);

  // Now, the HOB List has been initialized, we can register performance information
  PERF_START (NULL, "PEI", NULL, StartTimeStamp);

  // Put PEI stack base and size information into Hob
  BuildStackHob ((UINTN)SecCoreData->StackBase, SecCoreData->StackSize);

  // Put PeiGlobalVariable pointer base and size information into Hob
  BuildGlobalVariableHob ((UINTN)PcdGet64 (PcdCPUCoresStackBase) +
                          PcdGet32 (PcdCPUCorePrimaryStackSize) -
                          PcdGet32 (PcdPeiGlobalVariableSize),
                          PcdGet32 (PcdPeiGlobalVariableSize));

  // Put the CPU memory and io spaces sizes into Hob
  BuildCpuHob (PcdGet8 (PcdPrePiCpuMemorySize), PcdGet8 (PcdPrePiCpuIoSize));

  // Put the CPU MP Info table into Hob
  if (ArmIsMpCore ()) {
    // Only MP Core platform need to produce gArmMpCoreInfoPpiGuid
    Status = GetPlatformPpi (SecCoreData, PpiList, &gArmMpCoreInfoPpiGuid, (VOID**)&ArmMpCoreInfoPpi);

    // On MP Core Platform we must implement the ARM MP Core Info PPI (gArmMpCoreInfoPpiGuid)
    ASSERT_EFI_ERROR (Status);

    // Build the MP Core Info Table
    ArmCoreCount = 0;
    Status = ArmMpCoreInfoPpi->GetMpCoreInfo (&ArmCoreCount, &ArmCoreInfoTable);
    if (!EFI_ERROR(Status) && (ArmCoreCount > 0)) {
      // Build MPCore Info HOB
      BuildGuidDataHob (&gArmMpCoreInfoGuid, ArmCoreInfoTable, sizeof (ARM_CORE_INFO) * ArmCoreCount);
    }
  }

  // Put the EFI_BOOT_MODE information into hob
  SetBootMode (PlatformPeiGetBootMode ());

  // Build HOBs to pass up our version of stuff the DXE Core needs to save space
  BuildPeCoffLoaderHob ();
  BuildExtractSectionHob (
    &gLzmaCustomDecompressGuid,
    LzmaGuidedSectionGetInfo,
    LzmaGuidedSectionExtraction
    );

}


EFI_STATUS
EFIAPI
GetPlatformPpi (
IN CONST EFI_SEC_PEI_HAND_OFF      *SecCoreData,
  IN CONST EFI_PEI_PPI_DESCRIPTOR  *PpiList,
  IN  EFI_GUID                     *PpiGuid,
  OUT VOID                         **Ppi
  )
{
  UINTN                   PpiListSize;
  UINTN                   PpiListCount;
  UINTN                   Index;

  PpiListSize = (UINT32)SecCoreData->TemporaryRamBase -  (UINTN)PcdGet64 (PcdCPUCoresStackBase);
  PpiListCount = PpiListSize / sizeof(EFI_PEI_PPI_DESCRIPTOR);
  for (Index = 0; Index < PpiListCount; Index++, PpiList++) {
    if (CompareGuid (PpiList->Guid, PpiGuid) == TRUE) {
      *Ppi = PpiList->Ppi;
      return EFI_SUCCESS;
    }
  }

  return EFI_NOT_FOUND;
}


VOID
EFIAPI
BuildGlobalVariableHob (
  IN EFI_PHYSICAL_ADDRESS         GlobalVariableBase,
  IN UINT32                       GlobalVariableSize
  )
{
  ARM_HOB_GLOBAL_VARIABLE  *Hob;

  Hob = CreateHob (EFI_HOB_TYPE_GUID_EXTENSION, sizeof (ARM_HOB_GLOBAL_VARIABLE));
  ASSERT_PLATFORM_INIT(Hob != NULL);

  CopyGuid (&(Hob->Header.Name), &gArmGlobalVariableGuid);
  Hob->GlobalVariableBase = GlobalVariableBase;
  Hob->GlobalVariableSize = GlobalVariableSize;
}


VOID
BuildMemoryTypeInfoHob (
  VOID
  )
{
  EFI_MEMORY_TYPE_INFORMATION   Info[10];

  Info[0].Type          = EfiACPIReclaimMemory;
  Info[0].NumberOfPages = PcdGet32 (PcdMemoryTypeEfiACPIReclaimMemory);
  Info[1].Type          = EfiACPIMemoryNVS;
  Info[1].NumberOfPages = PcdGet32 (PcdMemoryTypeEfiACPIMemoryNVS);
  Info[2].Type          = EfiReservedMemoryType;
  Info[2].NumberOfPages = PcdGet32 (PcdMemoryTypeEfiReservedMemoryType);
  Info[3].Type          = EfiRuntimeServicesData;
  Info[3].NumberOfPages = PcdGet32 (PcdMemoryTypeEfiRuntimeServicesData);
  Info[4].Type          = EfiRuntimeServicesCode;
  Info[4].NumberOfPages = PcdGet32 (PcdMemoryTypeEfiRuntimeServicesCode);
  Info[5].Type          = EfiBootServicesCode;
  Info[5].NumberOfPages = PcdGet32 (PcdMemoryTypeEfiBootServicesCode);
  Info[6].Type          = EfiBootServicesData;
  Info[6].NumberOfPages = PcdGet32 (PcdMemoryTypeEfiBootServicesData);
  Info[7].Type          = EfiLoaderCode;
  Info[7].NumberOfPages = PcdGet32 (PcdMemoryTypeEfiLoaderCode);
  Info[8].Type          = EfiLoaderData;
  Info[8].NumberOfPages = PcdGet32 (PcdMemoryTypeEfiLoaderData);

  // Terminator for the list
  Info[9].Type          = EfiMaxMemoryType;
  Info[9].NumberOfPages = 0;

  BuildGuidDataHob (&gEfiMemoryTypeInformationGuid, &Info, sizeof (Info));
}


EFI_STATUS
EFIAPI
BuildSystemMemoryHOBs (
  IN EFI_PHYSICAL_ADDRESS     DramMemoryBase,
  IN UINT64                   DramMemorySize
  )
{
  EFI_RESOURCE_ATTRIBUTE_TYPE ResourceAttributes;

  // Declared the DRAM base, size and attributes into HOB
  ResourceAttributes = (
      EFI_RESOURCE_ATTRIBUTE_PRESENT |
      EFI_RESOURCE_ATTRIBUTE_INITIALIZED |
      EFI_RESOURCE_ATTRIBUTE_UNCACHEABLE |
      EFI_RESOURCE_ATTRIBUTE_WRITE_COMBINEABLE |
      EFI_RESOURCE_ATTRIBUTE_WRITE_THROUGH_CACHEABLE |
      EFI_RESOURCE_ATTRIBUTE_WRITE_BACK_CACHEABLE |
      EFI_RESOURCE_ATTRIBUTE_TESTED
  );
  BuildResourceDescriptorHob (
      EFI_RESOURCE_SYSTEM_MEMORY,
      ResourceAttributes,
      DramMemoryBase,
      DramMemorySize
  );

  // Build MemoryTypeInformationHob to prevent UEFI memory map fragmentation.
  BuildMemoryTypeInfoHob ();

  return EFI_SUCCESS;
}


VOID
EFIAPI
BootUnCompressedDxeFv (
  IN CONST EFI_SEC_PEI_HAND_OFF  *SecCoreData
  )
{
  UINT32                       DxeFileSize;
  UINTN                        DxeFvBase;
  EFI_RESOURCE_ATTRIBUTE_TYPE  ResourceAttributes;

  // Load DXE FV to DRAM
  DxeFvBase = (UINTN)PcdGet64(PcdFdBaseAddress); // Defined in .FDF file
  LoadDxeImageToRam (DxeFvBase, &DxeFileSize);
  // Assert if this is not a DXE FV file matching this PEI
  ASSERT_PLATFORM_INIT(DxeFileSize == PcdGet32(PcdDxeFvSize));

  // Put DXE FV base and size information into Hob
  BuildFvHob (DxeFvBase, DxeFileSize);

  // Reserved the memory space occupied by the firmware volume
  // Mark the chunk of DRAM contains the DXE FV with non-present attribute
  ResourceAttributes = (
      EFI_RESOURCE_ATTRIBUTE_PRESENT |
      EFI_RESOURCE_ATTRIBUTE_INITIALIZED |
      EFI_RESOURCE_ATTRIBUTE_UNCACHEABLE |
      EFI_RESOURCE_ATTRIBUTE_WRITE_COMBINEABLE |
      EFI_RESOURCE_ATTRIBUTE_WRITE_THROUGH_CACHEABLE |
      EFI_RESOURCE_ATTRIBUTE_WRITE_BACK_CACHEABLE |
      EFI_RESOURCE_ATTRIBUTE_TESTED
  );
  BuildResourceDescriptorHob (
    EFI_RESOURCE_SYSTEM_MEMORY,
    ResourceAttributes & ~EFI_RESOURCE_ATTRIBUTE_PRESENT,
    DxeFvBase,
    DxeFileSize);

  // Find and load main entry point of the DXE Core
  EnterDxeCoreEntryPoint (SecCoreData, DxeFvBase, DxeFileSize);

}


VOID
EFIAPI
BootCompressedDxeFv (
  IN CONST EFI_SEC_PEI_HAND_OFF  *SecCoreData
  )
{
  EFI_STATUS                   Status;
  UINT32                       DxeFileSize;
  UINTN                        DxeDecompressedFvBase;
  UINTN                        DxeCompressedFvBase;
  EFI_COMMON_SECTION_HEADER*   Section;
  VOID*                        OutputBuffer;
  UINT32                       OutputBufferSize;
  VOID*                        ScratchBuffer;
  UINT32                       ScratchBufferSize;
  UINT16                       SectionAttribute;
  UINT32                       AuthenticationStatus;
  EFI_RESOURCE_ATTRIBUTE_TYPE  ResourceAttributes;

  // Load Compress DXE FV to DRAM
  // PcdFdBaseAddress is defined in .FDF file
  DxeDecompressedFvBase = (UINTN)PcdGet64(PcdFdBaseAddress);
  // Compressed FV is load 16 MB after decompression destination offset
  DxeCompressedFvBase = DxeDecompressedFvBase + 0x1000000;
  // LZMA Scratch Buffer
  ScratchBuffer = (VOID*)(UINTN)(DxeCompressedFvBase + 0x1000000);
  // LZMA Output Buffer
  OutputBuffer = (VOID*)(UINTN)(DxeDecompressedFvBase - sizeof(EFI_COMMON_SECTION_HEADER2));
  // Load the Compressed FV
  LoadDxeImageToRam (DxeCompressedFvBase, &DxeFileSize);
  // Assert if this is not a DXE FV file
  ASSERT_PLATFORM_INIT(DxeFileSize == PcdGet32(PcdDxeFvSize));
  // Verify is a compressed DXE FV
  Section = (EFI_COMMON_SECTION_HEADER*)(DxeCompressedFvBase + 0x60); // FVH header size (0x48) + FFS header size (0x18)
  Status = LzmaGuidedSectionGetInfo (Section, &OutputBufferSize, &ScratchBufferSize, &SectionAttribute);
  if (EFI_ERROR(Status)) {
    // Not a LZMA compressed DXE FV
    return;
  }
  // Decompress the FV
  LzmaGuidedSectionExtraction (Section, &OutputBuffer, ScratchBuffer, &AuthenticationStatus);

  // Put decompressed DXE FV base and size information into Hob
  BuildFvHob (DxeDecompressedFvBase, OutputBufferSize);

  // Reserved the memory space occupied by the firmware volume
  // Mark the chunk of DRAM contains the DXE FV with non-present attribute
  ResourceAttributes = (
      EFI_RESOURCE_ATTRIBUTE_PRESENT |
      EFI_RESOURCE_ATTRIBUTE_INITIALIZED |
      EFI_RESOURCE_ATTRIBUTE_UNCACHEABLE |
      EFI_RESOURCE_ATTRIBUTE_WRITE_COMBINEABLE |
      EFI_RESOURCE_ATTRIBUTE_WRITE_THROUGH_CACHEABLE |
      EFI_RESOURCE_ATTRIBUTE_WRITE_BACK_CACHEABLE |
      EFI_RESOURCE_ATTRIBUTE_TESTED
  );
  BuildResourceDescriptorHob (
    EFI_RESOURCE_SYSTEM_MEMORY,
    ResourceAttributes & ~EFI_RESOURCE_ATTRIBUTE_PRESENT,
    DxeDecompressedFvBase,
    OutputBufferSize);

  SerialPortPrint (
    "DXE FV compressed   = 0x%08Lx - 0x%08Lx\n"
    "DXE FV decompressed = 0x%08Lx - 0x%08Lx\n",
    (UINT64) DxeCompressedFvBase,
    (UINT64) DxeCompressedFvBase + DxeFileSize - 1,
    (UINT64) DxeDecompressedFvBase,
    (UINT64) DxeDecompressedFvBase + OutputBufferSize - 1
    );

  // Find and load main entry point of the DXE Core
  EnterDxeCoreEntryPoint (SecCoreData, DxeDecompressedFvBase, OutputBufferSize);

}


VOID
EFIAPI
EnterDxeCoreEntryPoint (
  IN CONST EFI_SEC_PEI_HAND_OFF  *SecCoreData,
  IN UINTN                       DxeFvBase,
  IN UINTN                       DxeFvSize
  )
{
  EFI_PHYSICAL_ADDRESS              DxeCoreAddress;
  UINT32                            PECoffBase;
  EFI_FV_FILE_INFO                  DxeCoreFileInfo;
  EFI_PHYSICAL_ADDRESS              DxeCoreEntryPoint;
  EFI_FIRMWARE_VOLUME_HEADER       *FwVolHeader;
  EFI_FIRMWARE_VOLUME_EXT_HEADER   *FwVolExtHeader;
  EFI_FFS_FILE_HEADER              *FfsFileHeader;
  EFI_COMMON_SECTION_HEADER        *Section;
  UINT32                            FileSize;
  VOID                             *BaseOfStack;
  VOID                             *TopOfStack;
  EFI_HOB_HANDOFF_INFO_TABLE       *Hob;
  UINT32                            PeiHeapBegin;
  UINT32                            PeiHeapEnd;
  UINT32                            CopyOfPeiHeapBegin;
  UINT32                            CopyOfPeiHeapEnd;
  VOID*                             SourceBuffer;
  VOID*                             DestBuffer;
  //
  // Find the DXE Entry point
  //

  // Convert the handle of FV to FV header for memory-mapped firmware volume
  FwVolHeader = (EFI_FIRMWARE_VOLUME_HEADER *) DxeFvBase;

  // If FileHeader is not specified (NULL) or FileName is not NULL,
  // start with the first file in the firmware volume.  Otherwise,
  // start from the FileHeader.
  if (FwVolHeader->ExtHeaderOffset != 0) {
    // FFS is after FwVolHeader size (0x48) + FwVolExtHeader size + Padding size
    // Searching for files starts on an 8 byte aligned boundary after the end of the Extended Header if it exists.
    FwVolExtHeader = (EFI_FIRMWARE_VOLUME_EXT_HEADER *) ((UINT8 *) FwVolHeader + FwVolHeader->ExtHeaderOffset);
    FfsFileHeader = (EFI_FFS_FILE_HEADER *) ((UINT8 *) FwVolExtHeader + FwVolExtHeader->ExtHeaderSize);
    FfsFileHeader = (EFI_FFS_FILE_HEADER *) ALIGN_POINTER (FfsFileHeader, 8);
  } else {
    // FFS is after FwVolHeader size (0x48)
    FfsFileHeader = (EFI_FFS_FILE_HEADER *)((UINT8 *) FwVolHeader + FwVolHeader->HeaderLength);
  }

  if (IS_FFS_FILE2 (FfsFileHeader)) {
    Section = (EFI_COMMON_SECTION_HEADER *) ((UINT8 *) FfsFileHeader + sizeof (EFI_FFS_FILE_HEADER2));
    FileSize = FFS_FILE2_SIZE (FfsFileHeader) - sizeof (EFI_FFS_FILE_HEADER2);
  } else {
    Section = (EFI_COMMON_SECTION_HEADER *) ((UINT8 *) FfsFileHeader + sizeof (EFI_FFS_FILE_HEADER));
    FileSize = FFS_FILE_SIZE (FfsFileHeader) - sizeof (EFI_FFS_FILE_HEADER);
  }

  // DxeCoreAddress start after FVH header size + FFS header size + Section header
  DxeCoreAddress = (UINTN)((UINT8 *) Section + sizeof (EFI_COMMON_SECTION_HEADER));
  PECoffBase = DxeCoreAddress;
  CopyGuid (&DxeCoreFileInfo.FileName, (EFI_GUID*)&(FfsFileHeader->Name)); // EFI_GUID offset

  //SerialPortPrint ("FwVolHeader     : %08x\r\n", FwVolHeader);
  //SerialPortPrint ("FvLength        : %ld\r\n",  FwVolHeader->FvLength);
  //SerialPortPrint ("FvSignature     : %08x\r\n", FwVolHeader->Signature);
  //SerialPortPrint ("ExtHeaderOffset : %04x\r\n", FwVolHeader->ExtHeaderOffset);
  //SerialPortPrint ("FfsFileHeader   : %08x\r\n", FfsFileHeader);
  //SerialPortPrint ("DxeCoreFileSize : %d\r\n",   FileSize);
  //SerialPortPrint ("Section         : %08x\r\n", Section);
  //SerialPortPrint ("PECoffBase      : %08x\r\n", PECoffBase);
  //SerialPortPrint ("DxeCore GUID    : %g\r\n", &DxeCoreFileInfo.FileName);

  // Make sure it is really a EFI_SECTION_PE32 executable
  if ((*((UINT32*)(PECoffBase)) == 0x00005A4D) && (*((UINT32*)(PECoffBase + 0x80)) == 0x00004550))
  {
    // Found "MZ" and "PE" signature, prepare to boot DXE phase
    DxeCoreEntryPoint = (*((UINT32*)(PECoffBase + 0x80 + 0x34)) + *((UINT32*)(PECoffBase + 0x80 + 0x28)));

    // Add HOB for the DXE Core
    BuildModuleHob (
      &DxeCoreFileInfo.FileName,
      DxeCoreAddress,
      ALIGN_VALUE (FileSize, EFI_PAGE_SIZE),
      DxeCoreEntryPoint
      );

    // Allocate 128KB of Stack for DXE phase
    #define STACK_SIZE 0x20000

    // Compute the top of the stack we were allocated.
    BaseOfStack = (VOID*)(UINTN)(DxeFvBase + ALIGN_VALUE (DxeFvSize, EFI_PAGE_SIZE));
    TopOfStack = (VOID *) ((UINTN) BaseOfStack + EFI_SIZE_TO_PAGES (STACK_SIZE) * EFI_PAGE_SIZE - CPU_STACK_ALIGNMENT);
    TopOfStack = ALIGN_POINTER (TopOfStack, CPU_STACK_ALIGNMENT);

    // Update stack HOB to reflect the real stack info passed to DxeCore.
    UpdateStackHob ((EFI_PHYSICAL_ADDRESS)(UINTN) BaseOfStack, STACK_SIZE);

    SerialPortPrint (
      "DXE Stack           = 0x%08Lx - 0x%08Lx\n",
      (UINT64)(UINTN)BaseOfStack,
      (UINT64)(UINTN)TopOfStack + 7
      );

    // Enable the MMU because the UEFI DXE phase's ArmPkg\Drivers\CpuDxe
    // required MMU to be enabled and filed with section translations
    InitMmu ();

    // Update PHIT HOB to reflect the DRAM memory range instead of OCRAM
    Hob = GetHobList ();
    Hob->EfiMemoryTop        = (UINTN)GetMpuWindowDramBaseAddr() + GetMpuWindowDramSize();
    Hob->EfiMemoryBottom     = (UINTN)GetMpuWindowDramBaseAddr();
    Hob->EfiFreeMemoryTop    = Hob->EfiMemoryTop;

    // Make a copy of PEI Hob to DRAM memory range for DXE Core HOB relocation code to work correctly
    // because DXE Core Gcd.c CoreInitializeGcdServices will Relocate HOB List using
    // the calculation formula "PhitHob->EfiFreeMemoryBottom - (*HobStart)"
    // which will have the wrong hob size calculated if EfiFreeMemoryBottom is not within the SDRAM range.
    PeiHeapBegin = (UINT32)SecCoreData->PeiTemporaryRamBase;
    PeiHeapEnd = (UINT32)SecCoreData->PeiTemporaryRamBase + (UINT32)SecCoreData->PeiTemporaryRamSize - 1;
    CopyOfPeiHeapBegin = ALIGN_VALUE ((UINTN)TopOfStack + CPU_STACK_ALIGNMENT, EFI_PAGE_SIZE);
    CopyOfPeiHeapEnd = CopyOfPeiHeapBegin + (UINT32)SecCoreData->PeiTemporaryRamSize - 1;
    Hob->EfiFreeMemoryBottom = CopyOfPeiHeapEnd + 1;
    DestBuffer = (VOID*)(UINTN)(CopyOfPeiHeapBegin);
    SourceBuffer = (VOID*)(UINTN)(PeiHeapBegin);
    CopyMem (DestBuffer, SourceBuffer, (UINT32)SecCoreData->PeiTemporaryRamSize);

    //SerialPortPrint ("PrePeiGetHobList    = 0x%08x\n", (UINT32) PrePeiGetHobList());
    //SerialPortPrint ("PeiHeapBegin        = 0x%08x\n", (UINT32) PeiHeapBegin);
    //SerialPortPrint ("PeiHeapEnd          = 0x%08x\n", (UINT32) PeiHeapEnd);
    //SerialPortPrint ("CopyOfPeiHeapBegin  = 0x%08x\n", (UINT32) CopyOfPeiHeapBegin);
    //SerialPortPrint ("CopyOfPeiHeapEnd    = 0x%08x\n", (UINT32) CopyOfPeiHeapEnd);
    //SerialPortPrint ("EfiMemoryBottom     = 0x%08x\n", (UINT32) Hob->EfiMemoryBottom);
    //SerialPortPrint ("EfiMemoryTop        = 0x%08x\n", (UINT32) Hob->EfiMemoryTop);
    //SerialPortPrint ("EfiFreeMemoryBottom = 0x%08x\n", (UINT32) Hob->EfiFreeMemoryBottom);
    //SerialPortPrint ("EfiFreeMemoryTop    = 0x%08x\n", (UINT32) Hob->EfiFreeMemoryTop);

    // Jump to DxeCoreEntryPoint after switch Stack pointer from OCRAM to DRAM
    SerialPortPrint ("DXE Core entry at 0x%08Lx\n", DxeCoreEntryPoint);
    SwitchStack (
      (SWITCH_STACK_ENTRY_POINT)(UINTN)DxeCoreEntryPoint,
      (VOID*)(UINTN)CopyOfPeiHeapBegin,
      NULL,
      TopOfStack
      );
    // Function will not / should not return here
    // Note: If for some reason not going to use SwitchStack, then use jump only as follow:
    //       ((DXE_CORE_ENTRY_POINT)(UINTN)DxeCoreEntryPoint) (PrePeiGetHobList());

  }

  // Should not reach here
  ASSERT_PLATFORM_INIT(0);
  // Dead loop
  EFI_DEADLOOP();
}


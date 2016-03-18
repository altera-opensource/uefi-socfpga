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

  Copyright (c) 2011-2015, ARM Limited. All rights reserved.

  This program and the accompanying materials
  are licensed and made available under the terms and conditions of the BSD License
  which accompanies this distribution.  The full text of the license may be found at
  http://opensource.org/licenses/bsd-license.php

  THE PROGRAM IS DISTRIBUTED UNDER THE BSD 3 CLAUSE LICENSE ON AN "AS IS" BASIS,
  WITHOUT WARRANTIES OR REPRESENTATIONS OF ANY KIND, EITHER EXPRESS OR IMPLIED.

**/


#include <PiDxe.h>
#include <Library/ArmLib.h>
#include <Library/HobLib.h>

#include <Guid/ArmMpCoreInfo.h>

#include "LinuxLoader.h"

#define ALIGN(x, a)     (((x) + ((a) - 1)) & ~((a) - 1))
#define PALIGN(p, a)    ((void *)(ALIGN ((unsigned long)(p), (a))))
#define GET_CELL(p)     (p += 4, *((const UINT32 *)(p-4)))

STATIC
UINTN
cpu_to_fdtn (UINTN x) {
  if (sizeof (UINTN) == sizeof (UINT32)) {
    return cpu_to_fdt32 (x);
  } else {
    return cpu_to_fdt64 (x);
  }
}

typedef struct {
  UINTN   Base;
  UINTN   Size;
} FDT_REGION;

/**
** Relocate the FDT blob to a more appropriate location for the Linux kernel.
** This function will allocate memory for the relocated FDT blob.
**
** @retval EFI_SUCCESS on success.
** @retval EFI_OUT_OF_RESOURCES or EFI_INVALID_PARAMETER on failure.
*/
STATIC
EFI_STATUS
RelocateFdt (
  EFI_PHYSICAL_ADDRESS   SystemMemoryBase,
  EFI_PHYSICAL_ADDRESS   OriginalFdt,
  UINTN                  OriginalFdtSize,
  EFI_PHYSICAL_ADDRESS   *RelocatedFdt,
  UINTN                  *RelocatedFdtSize,
  EFI_PHYSICAL_ADDRESS   *RelocatedFdtAlloc
  )
{
  EFI_STATUS            Status;
  INTN                  Error;
  UINT64                FdtAlignment;

  *RelocatedFdtSize = OriginalFdtSize + FDT_ADDITIONAL_ENTRIES_SIZE;

  // If FDT load address needs to be aligned, allocate more space.
  FdtAlignment = PcdGet32 (PcdArmLinuxFdtAlignment);
  if (FdtAlignment != 0) {
    *RelocatedFdtSize += FdtAlignment;
  }

  // Try below a watermark address.
  Status = EFI_NOT_FOUND;
  if (PcdGet32 (PcdArmLinuxFdtMaxOffset) != 0) {
    *RelocatedFdt = LINUX_FDT_RELOCATED_OFFSET;
    Status = gBS->AllocatePages (AllocateMaxAddress, EfiBootServicesData,
                    EFI_SIZE_TO_PAGES (*RelocatedFdtSize), RelocatedFdt);
    if (EFI_ERROR (Status)) {
      DEBUG ((EFI_D_WARN, "Warning: Failed to load FDT below address 0x%lX (%r). Will try again at a random address anywhere.\n", *RelocatedFdt, Status));
    }
  }

  // Try anywhere there is available space.
  if (EFI_ERROR (Status)) {
    Status = gBS->AllocatePages (AllocateAnyPages, EfiBootServicesData,
                    EFI_SIZE_TO_PAGES (*RelocatedFdtSize), RelocatedFdt);
    if (EFI_ERROR (Status)) {
      ASSERT_EFI_ERROR (Status);
      return EFI_OUT_OF_RESOURCES;
    } else {
      DEBUG ((EFI_D_WARN, "WARNING: Loaded FDT at random address 0x%lX.\nWARNING: There is a risk of accidental overwriting by other code/data.\n", *RelocatedFdt));
    }
  }

  *RelocatedFdtAlloc = *RelocatedFdt;
  if (FdtAlignment != 0) {
    *RelocatedFdt = ALIGN (*RelocatedFdt, FdtAlignment);
  }
  *RelocatedFdt = LINUX_FDT_RELOCATED_OFFSET;
  // Load the Original FDT tree into the new region
  Error = fdt_open_into ((VOID*)(UINTN) OriginalFdt,
            (VOID*)(UINTN)(*RelocatedFdt), *RelocatedFdtSize);
  if (Error) {
    DEBUG ((EFI_D_ERROR, "fdt_open_into(): %a\n", fdt_strerror (Error)));
    gBS->FreePages (*RelocatedFdtAlloc, EFI_SIZE_TO_PAGES (*RelocatedFdtSize));
    return EFI_INVALID_PARAMETER;
  }

  return EFI_SUCCESS;
}

EFI_STATUS
PrepareFdt (
  IN     EFI_PHYSICAL_ADDRESS SystemMemoryBase,
  IN     CONST CHAR8*         CommandLineArguments,
  IN     EFI_PHYSICAL_ADDRESS InitrdImage,
  IN     UINTN                InitrdImageSize,
  IN OUT EFI_PHYSICAL_ADDRESS *FdtBlobBase,
  IN OUT UINTN                *FdtBlobSize
  )
{
  EFI_STATUS            Status;
  EFI_PHYSICAL_ADDRESS  NewFdtBlobBase;
  EFI_PHYSICAL_ADDRESS  NewFdtBlobAllocation;
  UINTN                 NewFdtBlobSize;
  VOID*                 fdt;
  INTN                  err;
  INTN                  node;
  INTN                  cpu_node;
  INT32                 lenp;
  CONST VOID*           BootArg;
  CONST VOID*           Method;
  EFI_PHYSICAL_ADDRESS  InitrdImageStart;
  EFI_PHYSICAL_ADDRESS  InitrdImageEnd;
  FDT_REGION            Region;
  UINTN                 Index;
  CHAR8                 Name[10];
  LIST_ENTRY            ResourceList;
  SYSTEM_MEMORY_RESOURCE  *Resource;
  ARM_PROCESSOR_TABLE   *ArmProcessorTable;
  ARM_CORE_INFO         *ArmCoreInfoTable;
  UINT32                MpId;
  UINT32                ClusterId;
  UINT32                CoreId;
  UINT64                CpuReleaseAddr;
 // UINTN                 MemoryMapSize;
 // EFI_MEMORY_DESCRIPTOR *MemoryMap;
 // EFI_MEMORY_DESCRIPTOR *MemoryMapPtr;
  //UINTN                 MapKey;
 // UINTN                 DescriptorSize;
 // UINT32                DescriptorVersion;
 // UINTN                 Pages;
  UINTN                 OriginalFdtSize;
  BOOLEAN               CpusNodeExist;
  UINTN                 CoreMpId;

  NewFdtBlobAllocation = 0;

  //
  // Sanity checks on the original FDT blob.
  //
  err = fdt_check_header ((VOID*)(UINTN)(*FdtBlobBase));
  if (err != 0) {
    Print (L"ERROR: Device Tree header not valid (err:%d)\n", err);
    return EFI_INVALID_PARAMETER;
  }

  // The original FDT blob might have been loaded partially.
  // Check that it is not the case.
  OriginalFdtSize = (UINTN)fdt_totalsize ((VOID*)(UINTN)(*FdtBlobBase));
  if (OriginalFdtSize > *FdtBlobSize) {
    Print (L"ERROR: Incomplete FDT. Only %d/%d bytes have been loaded.\n",
           *FdtBlobSize, OriginalFdtSize);
    return EFI_INVALID_PARAMETER;
  }

  //
  // Relocate the FDT to its final location.
  //
  Status = RelocateFdt (SystemMemoryBase, *FdtBlobBase, OriginalFdtSize,
             &NewFdtBlobBase, &NewFdtBlobSize, &NewFdtBlobAllocation);
  if (EFI_ERROR (Status)) {
    goto FAIL_RELOCATE_FDT;
  }

  fdt = (VOID*)(UINTN)NewFdtBlobBase;

  node = fdt_subnode_offset (fdt, 0, "chosen");
  if (node < 0) {
    // The 'chosen' node does not exist, create it
    node = fdt_add_subnode (fdt, 0, "chosen");
    if (node < 0) {
      DEBUG ((EFI_D_ERROR, "Error on finding 'chosen' node\n"));
      Status = EFI_INVALID_PARAMETER;
      goto FAIL_COMPLETE_FDT;
    }
  }

  DEBUG_CODE_BEGIN ();
    BootArg = fdt_getprop (fdt, node, "bootargs", &lenp);
    if (BootArg != NULL) {
      DEBUG ((EFI_D_ERROR, "BootArg: %a\n", BootArg));
    }
  DEBUG_CODE_END ();

  //
  // Set Linux CmdLine
  //
  if ((CommandLineArguments != NULL) && (AsciiStrLen (CommandLineArguments) > 0)) {
    err = fdt_setprop (fdt, node, "bootargs", CommandLineArguments, AsciiStrSize (CommandLineArguments));
    DEBUG ((EFI_D_ERROR, "Command line BootArg: %a\n", CommandLineArguments));
	if (err) {
      DEBUG ((EFI_D_ERROR, "Fail to set new 'bootarg' (err:%d)\n", err));
    }
  }

  //
  // Set Linux Initrd
  //
  if (InitrdImageSize != 0) {
    InitrdImageStart = cpu_to_fdt64 (InitrdImage);
    err = fdt_setprop (fdt, node, "linux,initrd-start", &InitrdImageStart, sizeof (EFI_PHYSICAL_ADDRESS));
    if (err) {
      DEBUG ((EFI_D_ERROR, "Fail to set new 'linux,initrd-start' (err:%d)\n", err));
    }
    InitrdImageEnd = cpu_to_fdt64 (InitrdImage + InitrdImageSize);
    err = fdt_setprop (fdt, node, "linux,initrd-end", &InitrdImageEnd, sizeof (EFI_PHYSICAL_ADDRESS));
    if (err) {
      DEBUG ((EFI_D_ERROR, "Fail to set new 'linux,initrd-start' (err:%d)\n", err));
    }
  }

  //
  // Set Physical memory setup if does not exist
  //
  GetSystemMemoryResources (&ResourceList);
  Resource = (SYSTEM_MEMORY_RESOURCE*)ResourceList.ForwardLink;

  Region.Base = cpu_to_fdtn ((UINTN)Resource->PhysicalStart);
  Region.Size = cpu_to_fdtn ((UINTN)Resource->ResourceLength);

  DEBUG ((EFI_D_ERROR, "PhysicalStart : 0x%08x\r\n", (UINT32)Resource->PhysicalStart));
  DEBUG ((EFI_D_ERROR, "ResourceLength: 0x%08x\r\n", (UINT32)Resource->ResourceLength));


  node = fdt_subnode_offset (fdt, 0, "memory");
  if (node < 0) {
    // The 'memory' node does not exist, create it
    node = fdt_add_subnode (fdt, 0, "memory");
    if (node >= 0) {
      fdt_setprop_string (fdt, node, "name", "memory");
      fdt_setprop_string (fdt, node, "device_type", "memory");


      err = fdt_setprop (fdt, node, "reg", &Region, sizeof (Region));
	  if (err) {
        DEBUG ((EFI_D_ERROR, "Fail to set new 'memory region' (err:%d)\n", err));
      }
    }
  } else {
     err = fdt_setprop (fdt, node, "reg", &Region, sizeof (Region));
     if (err) {
      DEBUG ((EFI_D_ERROR, "Fail to set new 'memory region' (err:%d)\n", err));
     }
  }

  //
  // Setup Arm Mpcore Info if it is a multi-core or multi-cluster platforms.
  //
  // For 'cpus' and 'cpu' device tree nodes bindings, refer to this file
  // in the kernel documentation:
  // Documentation/devicetree/bindings/arm/cpus.txt
  //
  for (Index = 0; Index < gST->NumberOfTableEntries; Index++) {
    // Check for correct GUID type
    if (CompareGuid (&gArmMpCoreInfoGuid, &(gST->ConfigurationTable[Index].VendorGuid))) {
      MpId = ArmReadMpidr ();
      ClusterId = GET_CLUSTER_ID (MpId);
      CoreId    = GET_CORE_ID (MpId);

      node = fdt_subnode_offset (fdt, 0, "cpus");
      if (node < 0) {
        // Create the /cpus node
        node = fdt_add_subnode (fdt, 0, "cpus");
        fdt_setprop_string (fdt, node, "name", "cpus");
        fdt_setprop_cell (fdt, node, "#address-cells", sizeof (UINTN) / 4);
        fdt_setprop_cell (fdt, node, "#size-cells", 0);
        CpusNodeExist = FALSE;
      } else {
        CpusNodeExist = TRUE;
      }

      // Get pointer to ARM processor table
      ArmProcessorTable = (ARM_PROCESSOR_TABLE *)gST->ConfigurationTable[Index].VendorTable;
      ArmCoreInfoTable = ArmProcessorTable->ArmCpus;

      for (Index = 0; Index < ArmProcessorTable->NumberOfEntries; Index++) {
        CoreMpId = (UINTN) GET_MPID (ArmCoreInfoTable[Index].ClusterId,
                             ArmCoreInfoTable[Index].CoreId);
        AsciiSPrint (Name, 10, "cpu@%x", CoreMpId);

        // If the 'cpus' node did not exist then create all the 'cpu' nodes.
        // In case 'cpus' node is provided in the original FDT then we do not add
        // any 'cpu' node.
        if (!CpusNodeExist) {
          cpu_node = fdt_add_subnode (fdt, node, Name);
          if (cpu_node < 0) {
            DEBUG ((EFI_D_ERROR, "Error on creating '%s' node\n", Name));
            Status = EFI_INVALID_PARAMETER;
            goto FAIL_COMPLETE_FDT;
          }

          fdt_setprop_string (fdt, cpu_node, "device_type", "cpu");

          CoreMpId = cpu_to_fdtn (CoreMpId);
          fdt_setprop (fdt, cpu_node, "reg", &CoreMpId, sizeof (CoreMpId));
        } else {
          cpu_node = fdt_subnode_offset (fdt, node, Name);
        }

        if (cpu_node >= 0) {
          Method = fdt_getprop (fdt, cpu_node, "enable-method", &lenp);
          // We only care when 'enable-method' == 'spin-table'. If the enable-method is not defined
          // or defined as 'psci' then we ignore its properties.
          if ((Method != NULL) && (AsciiStrCmp ((CHAR8 *)Method, "spin-table") == 0)) {
            // There are two cases;
            //  - UEFI firmware parked the secondary cores and/or UEFI firmware is aware of the CPU
            //    release addresses (PcdArmLinuxSpinTable == TRUE)
            //  - the parking of the secondary cores has been managed before starting UEFI and/or UEFI
            //    does not anything about the CPU release addresses - in this case we do nothing
            if (FeaturePcdGet (PcdArmLinuxSpinTable)) {
              CpuReleaseAddr = cpu_to_fdt64 (ArmCoreInfoTable[Index].MailboxSetAddress);
              fdt_setprop (fdt, cpu_node, "cpu-release-addr", &CpuReleaseAddr, sizeof (CpuReleaseAddr));

              // If it is not the primary core than the cpu should be disabled
              if (((ArmCoreInfoTable[Index].ClusterId != ClusterId) || (ArmCoreInfoTable[Index].CoreId != CoreId))) {
                fdt_setprop_string (fdt, cpu_node, "status", "disabled");
              }
            }
          }
        }
      }
      break;
    }
  }

  // If we succeeded to generate the new Device Tree then free the old Device Tree
  gBS->FreePages (*FdtBlobBase, EFI_SIZE_TO_PAGES (*FdtBlobSize));

  // Update the real size of the Device Tree
  fdt_pack ((VOID*)(UINTN)(NewFdtBlobBase));

  *FdtBlobBase = NewFdtBlobBase;
  *FdtBlobSize = (UINTN)fdt_totalsize ((VOID*)(UINTN)(NewFdtBlobBase));
  return EFI_SUCCESS;

FAIL_COMPLETE_FDT:
  gBS->FreePages (NewFdtBlobAllocation, EFI_SIZE_TO_PAGES (NewFdtBlobSize));

FAIL_RELOCATE_FDT:
  *FdtBlobSize = (UINTN)fdt_totalsize ((VOID*)(UINTN)(*FdtBlobBase));
  // Return success even if we failed to update the FDT blob.
  // The original one is still valid.
  return EFI_SUCCESS;
}

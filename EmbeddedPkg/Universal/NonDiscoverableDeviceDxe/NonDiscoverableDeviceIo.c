/** @file

  Copyright (c) 2008 - 2009, Apple Inc. All rights reserved.<BR>
  Copyright (c) 2016-2018, Linaro, Ltd. All rights reserved.<BR>

  This program and the accompanying materials
  are licensed and made available under the terms and conditions of the BSD License
  which accompanies this distribution.  The full text of the license may be found at
  http://opensource.org/licenses/bsd-license.php

  THE PROGRAM IS DISTRIBUTED UNDER THE BSD LICENSE ON AN "AS IS" BASIS,
  WITHOUT WARRANTIES OR REPRESENTATIONS OF ANY KIND, EITHER EXPRESS OR IMPLIED.

**/

#include <PiDxe.h>

#include <Library/BaseMemoryLib.h>
#include <Library/DxeServicesTableLib.h>
#include <Library/MemoryAllocationLib.h>
#include <Library/UefiBootServicesTableLib.h>
#include <Library/UefiLib.h>

#include <Protocol/IoMmu.h>
#include <Protocol/NonDiscoverableDevice.h>

#include "NonDiscoverableDeviceIo.h"

typedef struct {
  EFI_PHYSICAL_ADDRESS            AllocAddress;
  VOID                            *HostAddress;
  EFI_IO_OPERATION_TYPE           Operation;
  UINTN                           NumberOfBytes;
} NON_DISCOVERABLE_IO_DEVICE_MAP_INFO;

/**
  Enable a driver to access controller registers in the memory or I/O space.

  @param  Width         Signifies the width of the memory or I/O operations.
  @param  Count         The number of memory or I/O operations to perform.
  @param  DstStride     The stride of the destination buffer.
  @param  Dst           For read operations, the destination buffer to store
                        the results. For write operations, the destination
                        buffer to write data to.
  @param  SrcStride     The stride of the source buffer.
  @param  Src           For read operations, the source buffer to read data
                        from. For write operations, the source buffer to write
                        data from.

  @retval EFI_SUCCESS            The data was read from or written to the
                                 controller.
  @retval EFI_INVALID_PARAMETER  One or more parameters are invalid.

**/
STATIC
EFI_STATUS
EFIAPI
IoMemRW (
  IN     EFI_IO_WIDTH                  Width,
  IN     UINTN                         Count,
  IN     UINTN                         DstStride,
  IN     VOID                          *Dst,
  IN     UINTN                         SrcStride,
     OUT CONST VOID                    *Src
  )
{
  volatile UINT8             *Dst8;
  volatile UINT16            *Dst16;
  volatile UINT32            *Dst32;
  volatile CONST UINT8       *Src8;
  volatile CONST UINT16      *Src16;
  volatile CONST UINT32      *Src32;

  //
  // Loop for each iteration and move the data
  //
  switch (Width & 0x3) {
  case IO_UINT8:
    Dst8 = (UINT8 *)Dst;
    Src8 = (UINT8 *)Src;
    for (;Count > 0; Count--, Dst8 += DstStride, Src8 += SrcStride) {
      *Dst8 = *Src8;
    }
    break;
  case IO_UINT16:
    Dst16 = (UINT16 *)Dst;
    Src16 = (UINT16 *)Src;
    for (;Count > 0; Count--, Dst16 += DstStride, Src16 += SrcStride) {
      *Dst16 = *Src16;
    }
    break;
  case IO_UINT32:
    Dst32 = (UINT32 *)Dst;
    Src32 = (UINT32 *)Src;
    for (;Count > 0; Count--, Dst32 += DstStride, Src32 += SrcStride) {
      *Dst32 = *Src32;
    }
    break;
  default:
    return EFI_INVALID_PARAMETER;
  }

  return EFI_SUCCESS;
}

/**
  Enable a driver to access controller registers in the memory or I/O space.

  @param  This                  A pointer to the EFI_DEVICE_IO_PROTOCOL
                                instance.
  @param  Width                 Signifies the width of the memory or I/O
                                operations.
  @param  Offset                The offset to start the memory or I/O operation.
  @param  Count                 The number of memory or I/O operations to
                                perform.
  @param  Buffer                For read operations, the destination buffer to
                                store the results. For write operations, the
                                source buffer to write data from.

  @retval EFI_SUCCESS           The data was read from or written to the
                                controller.
  @retval EFI_UNSUPPORTED       The address range specified by Offset, Width,
                                and Count is not valid.
  @retval EFI_INVALID_PARAMETER One or more parameters are invalid.

**/
STATIC
EFI_STATUS
EFIAPI
IoMemRead (
  IN     EFI_DEVICE_IO_PROTOCOL        *This,
  IN     EFI_IO_WIDTH                  Width,
  IN     UINT64                        Offset,
  IN     UINTN                         Count,
  IN OUT VOID                          *Buffer
  )
{
  NON_DISCOVERABLE_IO_DEVICE           *Dev;
  UINTN                                AlignMask;
  UINT64                               Address;
  EFI_ACPI_ADDRESS_SPACE_DESCRIPTOR    *Desc;

  if (Buffer == NULL) {
    return EFI_INVALID_PARAMETER;
  }

  Dev = NON_DISCOVERABLE_IO_DEVICE_FROM_IO(This);

  Desc = Dev->Device->Resources;
  Address = Desc->AddrRangeMin + Offset;

  if (Address + (Count << (Width & 0x3)) > Desc->AddrRangeMax) {
    return EFI_UNSUPPORTED;
  }

  AlignMask = (1 << (Width & 0x03)) - 1;
  if ((UINTN)Address & AlignMask) {
    return EFI_INVALID_PARAMETER;
  }

  switch (Width) {
  case IO_UINT8:
  case IO_UINT16:
  case IO_UINT32:
  case IO_UINT64:
    return IoMemRW (Width, Count, 1, Buffer, 1, (VOID *)Address);
  default:
    break;
  }
  return EFI_INVALID_PARAMETER;
}

/**
  Enable a driver to access controller registers in the memory or I/O space.

  @param  This                  A pointer to the EFI_DEVICE_IO_PROTOCOL
                                instance.
  @param  Width                 Signifies the width of the memory or I/O
                                operations.
  @param  Offset                The offset to start the memory or I/O operation.
  @param  Count                 The number of memory or I/O operations to
                                perform.
  @param  Buffer                For read operations, the destination buffer to
                                store the results. For write operations, the
                                source buffer to write data from.

  @retval EFI_SUCCESS           The data was read from or written to the
                                controller.
  @retval EFI_UNSUPPORTED       The address range specified by Offset, Width,
                                and Count is not valid.
  @retval EFI_INVALID_PARAMETER One or more parameters are invalid.

**/
STATIC
EFI_STATUS
EFIAPI
IoMemWrite (
  IN     EFI_DEVICE_IO_PROTOCOL        *This,
  IN     EFI_IO_WIDTH                  Width,
  IN     UINT64                        Offset,
  IN     UINTN                         Count,
  IN OUT VOID                          *Buffer
  )
{
  NON_DISCOVERABLE_IO_DEVICE           *Dev;
  UINTN                                AlignMask;
  UINT64                               Address;
  EFI_ACPI_ADDRESS_SPACE_DESCRIPTOR    *Desc;

  if (Buffer == NULL) {
    return EFI_INVALID_PARAMETER;
  }

  Dev = NON_DISCOVERABLE_IO_DEVICE_FROM_IO(This);

  Desc = Dev->Device->Resources;

  Address = Desc->AddrRangeMin + Offset;
  if (Address + (Count << (Width & 0x3)) > Desc->AddrRangeMax) {
    return EFI_UNSUPPORTED;
  }

  AlignMask = (1 << (Width & 0x03)) - 1;
  if ((UINTN)Address & AlignMask) {
    return EFI_INVALID_PARAMETER;
  }

  switch (Width) {
  case IO_UINT8:
  case IO_UINT16:
  case IO_UINT32:
  case IO_UINT64:
    return IoMemRW (Width, Count, 1, (VOID *)Address, 1, Buffer);
  default:
    break;
  }
  return EFI_INVALID_PARAMETER;
}

/**
  Enable a driver to access controller registers in the memory or I/O space.

  @param  This                  A pointer to the EFI_DEVICE_IO_PROTOCOL
                                instance.
  @param  Width                 Signifies the width of the memory or I/O
                                operations.
  @param  Offset                The offset to start the memory or I/O operation.
  @param  Count                 The number of memory or I/O operations to
                                perform.
  @param  Buffer                For read operations, the destination buffer
                                to store the results. For write operations,
                                the source buffer to write data from.

**/
STATIC
EFI_STATUS
EFIAPI
IoIoRead (
  IN     EFI_DEVICE_IO_PROTOCOL       *This,
  IN     EFI_IO_WIDTH                 Width,
  IN     UINT64                       Offset,
  IN     UINTN                        Count,
  IN OUT VOID                         *Buffer
  )
{
  ASSERT (FALSE);
  return EFI_UNSUPPORTED;
}

/**
  Enable a driver to access controller registers in the memory or I/O space.

  @param  This                  A pointer to the EFI_DEVICE_IO_PROTOCOL
                                instance.
  @param  Width                 Signifies the width of the memory or I/O
                                operations.
  @param  Offset                The offset to start the memory or I/O operation.
  @param  Count                 The number of memory or I/O operations to
                                perform.
  @param  Buffer                For read operations, the destination buffer to
                                store the results. For write operations, the
                                source buffer to write data from.

**/
STATIC
EFI_STATUS
EFIAPI
IoIoWrite (
  IN     EFI_DEVICE_IO_PROTOCOL       *This,
  IN     EFI_IO_WIDTH                 Width,
  IN     UINT64                       Address,
  IN     UINTN                        Count,
  IN OUT VOID                         *Buffer
  )
{
  ASSERT (FALSE);
  return EFI_UNSUPPORTED;
}

/**
  Provides the controller-specific addresses needed to access system memory.

  @param  This                  A pointer to the EFI_DEVICE_IO_PROTOCOL
                                instance.
  @param  Operation             Indicates if the bus master is going to read
                                or write to system memory.
  @param  HostAddress           The system memory address to map to the
                                controller.
  @param  NumberOfBytes         On input the number of bytes to map. On output
                                the number of bytes that were mapped.
  @param  DeviceAddress         The resulting map address for the bus master
                                controller to use to access the hosts
                                HostAddress.
  @param  Mapping               A resulting value to pass to Unmap().

  @retval EFI_SUCCESS           The range was mapped for the returned
                                NumberOfBytes.
  @retval EFI_UNSUPPORTED       The HostAddress cannot be mapped as a common
                                buffer.
  @retval EFI_INVALID_PARAMETER One or more parameters are invalid.
  @retval EFI_OUT_OF_RESOURCES  The request could not be completed due to a
                                lack of resources.
  @retval EFI_DEVICE_ERROR      The system hardware could not map the requested
                                address.

**/
STATIC
EFI_STATUS
EFIAPI
CoherentIoMap (
  IN     EFI_DEVICE_IO_PROTOCOL        *This,
  IN     EFI_IO_OPERATION_TYPE         Operation,
  IN     EFI_PHYSICAL_ADDRESS          *HostAddress,
  IN OUT UINTN                         *NumberOfBytes,
     OUT EFI_PHYSICAL_ADDRESS          *DeviceAddress,
     OUT VOID                          **Mapping
  )
{
  EFI_STATUS                           Status;
  NON_DISCOVERABLE_IO_DEVICE_MAP_INFO  *MapInfo;

  //
  // If HostAddress exceeds 4 GB, and this device does not support 64-bit DMA
  // addressing, we need to allocate a bounce buffer and copy over the data.
  //
  if ((EFI_PHYSICAL_ADDRESS)(UINTN)HostAddress + *NumberOfBytes > SIZE_4GB) {
    //
    // Bounce buffering is not possible for consistent mappings
    //
    if (Operation == EfiBusMasterCommonBuffer) {
      return EFI_UNSUPPORTED;
    }

    MapInfo = AllocatePool (sizeof *MapInfo);
    if (MapInfo == NULL) {
      return EFI_OUT_OF_RESOURCES;
    }

    MapInfo->AllocAddress = MAX_UINT32;
    MapInfo->HostAddress = HostAddress;
    MapInfo->Operation = Operation;
    MapInfo->NumberOfBytes = *NumberOfBytes;

    Status = gBS->AllocatePages (
                    AllocateMaxAddress,
                    EfiBootServicesData,
                    EFI_SIZE_TO_PAGES (MapInfo->NumberOfBytes),
                    &MapInfo->AllocAddress
                    );
    if (EFI_ERROR (Status)) {
      //
      // If we fail here, it is likely because the system has no memory below
      // 4 GB to begin with. There is not much we can do about that other than
      // fail the map request.
      //
      FreePool (MapInfo);
      return EFI_DEVICE_ERROR;
    }
    if (Operation == EfiBusMasterRead) {
      gBS->CopyMem (
             (VOID *)(UINTN)MapInfo->AllocAddress,
             HostAddress,
             *NumberOfBytes
             );
    }
    *DeviceAddress = MapInfo->AllocAddress;
    *Mapping = MapInfo;
  } else {
    *DeviceAddress = (EFI_PHYSICAL_ADDRESS)(UINTN)HostAddress;
    *Mapping = NULL;
  }
  return EFI_SUCCESS;
}

/**
  Completes the Map() operation and releases any corresponding resources.

  @param  This                  A pointer to the EFI_DEVICE_IO_PROTOCOL
                                instance.
  @param  Mapping               The mapping value returned from Map().

  @retval EFI_SUCCESS           The range was unmapped.

**/
STATIC
EFI_STATUS
EFIAPI
CoherentIoUnmap (
  IN EFI_DEVICE_IO_PROTOCOL            *This,
  IN VOID                              *Mapping
  )
{
  NON_DISCOVERABLE_IO_DEVICE_MAP_INFO  *MapInfo;

  MapInfo = Mapping;
  if (MapInfo != NULL) {
    if (MapInfo->Operation == EfiBusMasterWrite) {
      gBS->CopyMem (
             MapInfo->HostAddress,
             (VOID *)(UINTN)MapInfo->AllocAddress,
             MapInfo->NumberOfBytes
             );
    }
    gBS->FreePages (
           MapInfo->AllocAddress,
           EFI_SIZE_TO_PAGES (MapInfo->NumberOfBytes)
           );
    FreePool (MapInfo);
  }
  return EFI_SUCCESS;
}

/**
  Allocates pages.

  @param  This                  A pointer to the EFI_PCI_IO_PROTOCOL instance.
  @param  Type                  This parameter is not used and must be ignored.
  @param  MemoryType            The type of memory to allocate,
                                EfiBootServicesData or EfiRuntimeServicesData.
  @param  Pages                 The number of pages to allocate.
  @param  HostAddress           A pointer to store the base system memory
                                address of the allocated range.
  @param  Attributes            The requested bit mask of attributes for the
                                allocated range.

  @retval EFI_SUCCESS           The requested memory pages were allocated.
  @retval EFI_UNSUPPORTED       Attributes is unsupported. The only legal
                                attribute bits are MEMORY_WRITE_COMBINE and
                                MEMORY_CACHED.
  @retval EFI_INVALID_PARAMETER One or more parameters are invalid.
  @retval EFI_OUT_OF_RESOURCES  The memory pages could not be allocated.

**/
STATIC
EFI_STATUS
EFIAPI
CoherentIoAllocateBuffer (
  IN     EFI_DEVICE_IO_PROTOCOL        *This,
  IN     EFI_ALLOCATE_TYPE             Type,
  IN     EFI_MEMORY_TYPE               MemoryType,
  IN     UINTN                         Pages,
  IN OUT EFI_PHYSICAL_ADDRESS          *HostAddress
  )
{
  EFI_STATUS                           Status;
  NON_DISCOVERABLE_IO_DEVICE           *Dev;
  EFI_ALLOCATE_TYPE                    AllocType;
  EFI_PHYSICAL_ADDRESS                 AllocAddress;

  //
  // Allocate below 4 GB if the dual address cycle attribute has not
  // been set. If the system has no memory available below 4 GB, there
  // is little we can do except propagate the error.
  //
  Dev = NON_DISCOVERABLE_IO_DEVICE_FROM_IO(This);
  if ((Dev->Attributes & EDKII_IOMMU_ATTRIBUTE_DUAL_ADDRESS_CYCLE) == 0) {
    AllocAddress = MAX_UINT32;
    AllocType = AllocateMaxAddress;
  } else {
    AllocType = AllocateAnyPages;
  }

  Status = gBS->AllocatePages (AllocType, MemoryType, Pages, &AllocAddress);
  if (!EFI_ERROR (Status)) {
    *HostAddress = AllocAddress;
  }
  return Status;
}

/**
  Frees memory that was allocated in function CoherentIoAllocateBuffer ().

  @param  This                  A pointer to the EFI_DEVICE_IO_PROTOCOL
                                instance.
  @param  Pages                 The number of pages to free.
  @param  HostAddress           The base system memory address of the
                                allocated range.

  @retval EFI_SUCCESS           The requested memory pages were freed.

**/
STATIC
EFI_STATUS
EFIAPI
CoherentIoFreeBuffer (
  IN EFI_DEVICE_IO_PROTOCOL            *This,
  IN UINTN                             Pages,
  IN EFI_PHYSICAL_ADDRESS              HostAddress
  )
{
  FreePages ((VOID *)HostAddress, Pages);
  return EFI_SUCCESS;
}

/**
  Frees memory that was allocated in function NonCoherentIoAllocateBuffer ().

  @param  This                  A pointer to the EFI_DEVICE_IO_PROTOCOL
                                instance.
  @param  Pages                 The number of pages to free.
  @param  HostAddress           The base system memory address of the allocated
                                range.

  @retval EFI_SUCCESS           The requested memory pages were freed.
  @retval others                The operation contain some errors.

**/
STATIC
EFI_STATUS
EFIAPI
NonCoherentIoFreeBuffer (
  IN EFI_DEVICE_IO_PROTOCOL            *This,
  IN UINTN                             Pages,
  IN EFI_PHYSICAL_ADDRESS              HostAddress
  )
{
  NON_DISCOVERABLE_IO_DEVICE                    *Dev;
  LIST_ENTRY                                    *Entry;
  EFI_STATUS                                    Status;
  NON_DISCOVERABLE_DEVICE_UNCACHED_ALLOCATION   *Alloc;
  BOOLEAN                                       Found;

  Dev = NON_DISCOVERABLE_IO_DEVICE_FROM_IO(This);

  Found = FALSE;
  Alloc = NULL;

  //
  // Find the uncached allocation list entry associated
  // with this allocation
  //
  for (Entry = Dev->UncachedAllocationList.ForwardLink;
       Entry != &Dev->UncachedAllocationList;
       Entry = Entry->ForwardLink) {

    Alloc = BASE_CR (Entry, NON_DISCOVERABLE_DEVICE_UNCACHED_ALLOCATION, List);
    if (Alloc->HostAddress == (VOID *)HostAddress && Alloc->NumPages == Pages) {
      //
      // We are freeing the exact allocation we were given
      // before by AllocateBuffer()
      //
      Found = TRUE;
      break;
    }
  }

  if (!Found) {
    ASSERT_EFI_ERROR (EFI_NOT_FOUND);
    return EFI_NOT_FOUND;
  }

  RemoveEntryList (&Alloc->List);

  Status = gDS->SetMemorySpaceAttributes (
                  (EFI_PHYSICAL_ADDRESS)(UINTN)HostAddress,
                  EFI_PAGES_TO_SIZE (Pages),
                  Alloc->Attributes);
  if (EFI_ERROR (Status)) {
    goto FreeAlloc;
  }

  //
  // If we fail to restore the original attributes, it is better to leak the
  // memory than to return it to the heap
  //
  FreePages ((VOID *)HostAddress, Pages);

FreeAlloc:
  FreePool (Alloc);
  return Status;
}

/**
  Allocates pages.

  @param  This                  A pointer to the EFI_DEVICE_IO_PROTOCOL
                                instance.
  @param  Type                  This parameter is not used and must be ignored.
  @param  MemoryType            The type of memory to allocate,
                                EfiBootServicesData or EfiRuntimeServicesData.
  @param  Pages                 The number of pages to allocate.
  @param  HostAddress           A pointer to store the base system memory
                                address of the allocated range.
  @param  Attributes            The requested bit mask of attributes for the
                                allocated range.

  @retval EFI_SUCCESS           The requested memory pages were allocated.
  @retval EFI_UNSUPPORTED       Attributes is unsupported. The only legal
                                attribute bits are MEMORY_WRITE_COMBINE and
                                MEMORY_CACHED.
  @retval EFI_INVALID_PARAMETER One or more parameters are invalid.
  @retval EFI_OUT_OF_RESOURCES  The memory pages could not be allocated.

**/
STATIC
EFI_STATUS
EFIAPI
NonCoherentIoAllocateBuffer (
  IN     EFI_DEVICE_IO_PROTOCOL        *This,
  IN     EFI_ALLOCATE_TYPE             Type,
  IN     EFI_MEMORY_TYPE               MemoryType,
  IN     UINTN                         Pages,
  IN OUT EFI_PHYSICAL_ADDRESS          *HostAddress
  )
{
  NON_DISCOVERABLE_IO_DEVICE                  *Dev;
  EFI_GCD_MEMORY_SPACE_DESCRIPTOR             GcdDescriptor;
  EFI_STATUS                                  Status;
  UINT64                                      MemType;
  NON_DISCOVERABLE_DEVICE_UNCACHED_ALLOCATION *Alloc;
  EFI_PHYSICAL_ADDRESS                        AllocAddress;

  Dev = NON_DISCOVERABLE_IO_DEVICE_FROM_IO(This);

  Status = CoherentIoAllocateBuffer (
             This,
             Type,
             MemoryType,
             Pages,
             &AllocAddress
             );
  if (EFI_ERROR (Status)) {
    return Status;
  }

  Status = gDS->GetMemorySpaceDescriptor (
                  (EFI_PHYSICAL_ADDRESS)(UINTN)AllocAddress,
                  &GcdDescriptor
                  );
  if (EFI_ERROR (Status)) {
    goto FreeBuffer;
  }

  if ((GcdDescriptor.Capabilities & (EFI_MEMORY_WC | EFI_MEMORY_UC)) == 0) {
    Status = EFI_UNSUPPORTED;
    goto FreeBuffer;
  }

  //
  // Set the preferred memory attributes
  //
  if ((GcdDescriptor.Capabilities & EFI_MEMORY_UC) == 0) {
    //
    // Use write combining if it was requested, or if it is the only
    // type supported by the region.
    //
    MemType = EFI_MEMORY_WC;
  } else {
    MemType = EFI_MEMORY_UC;
  }

  Alloc = AllocatePool (sizeof *Alloc);
  if (Alloc == NULL) {
    goto FreeBuffer;
  }

  Alloc->HostAddress = (VOID *)AllocAddress;
  Alloc->NumPages = Pages;
  Alloc->Attributes = GcdDescriptor.Attributes;

  //
  // Record this allocation in the linked list, so we
  // can restore the memory space attributes later
  //
  InsertHeadList (&Dev->UncachedAllocationList, &Alloc->List);

  Status = gDS->SetMemorySpaceAttributes (
                  (EFI_PHYSICAL_ADDRESS)(UINTN)AllocAddress,
                  EFI_PAGES_TO_SIZE (Pages),
                  MemType
                  );
  if (EFI_ERROR (Status)) {
    goto RemoveList;
  }

  Status = mCpu->FlushDataCache (
                   mCpu,
                   (EFI_PHYSICAL_ADDRESS)(UINTN)AllocAddress,
                   EFI_PAGES_TO_SIZE (Pages),
                   EfiCpuFlushTypeInvalidate);
  if (EFI_ERROR (Status)) {
    goto RemoveList;
  }

  *HostAddress = AllocAddress;

  return EFI_SUCCESS;

RemoveList:
  RemoveEntryList (&Alloc->List);
  FreePool (Alloc);

FreeBuffer:
  CoherentIoFreeBuffer (This, Pages, AllocAddress);
  return Status;
}

/**
  Provides the controller-specific addresses needed to access system memory.

  @param  This                  A pointer to the EFI_DEVICE_IO_PROTOCOL
                                instance.
  @param  Operation             Indicates if the bus master is going to read or
                                write to system memory.
  @param  HostAddress           The system memory address to map to the
                                controller.
  @param  NumberOfBytes         On input the number of bytes to map. On output
                                the number of bytes that were mapped.
  @param  DeviceAddress         The resulting map address for the bus master
                                controller to use to access the hosts
                                HostAddress.
  @param  Mapping               A resulting value to pass to Unmap().

  @retval EFI_SUCCESS           The range was mapped for the returned
                                NumberOfBytes.
  @retval EFI_UNSUPPORTED       The HostAddress cannot be mapped as a common
                                buffer.
  @retval EFI_INVALID_PARAMETER One or more parameters are invalid.
  @retval EFI_OUT_OF_RESOURCES  The request could not be completed due to a
                                lack of resources.
  @retval EFI_DEVICE_ERROR      The system hardware could not map the requested
                                address.

**/
STATIC
EFI_STATUS
EFIAPI
NonCoherentIoMap (
  IN     EFI_DEVICE_IO_PROTOCOL        *This,
  IN     EFI_IO_OPERATION_TYPE         Operation,
  IN     EFI_PHYSICAL_ADDRESS          *HostAddress,
  IN OUT UINTN                         *NumberOfBytes,
     OUT EFI_PHYSICAL_ADDRESS          *DeviceAddress,
     OUT VOID                          **Mapping
  )
{
  NON_DISCOVERABLE_IO_DEVICE           *Dev;
  EFI_STATUS                           Status;
  NON_DISCOVERABLE_IO_DEVICE_MAP_INFO  *MapInfo;
  UINTN                                AlignMask;
  EFI_PHYSICAL_ADDRESS                 AllocAddress;
  EFI_GCD_MEMORY_SPACE_DESCRIPTOR      GcdDescriptor;
  BOOLEAN                              Bounce;

  MapInfo = AllocatePool (sizeof *MapInfo);
  if (MapInfo == NULL) {
    return EFI_OUT_OF_RESOURCES;
  }

  MapInfo->HostAddress = HostAddress;
  MapInfo->Operation = Operation;
  MapInfo->NumberOfBytes = *NumberOfBytes;

  Dev = NON_DISCOVERABLE_IO_DEVICE_FROM_IO(This);

  //
  // If this device does not support 64-bit DMA addressing, we need to allocate
  // a bounce buffer and copy over the data in case HostAddress >= 4 GB.
  //
  Bounce = ((Dev->Attributes & EDKII_IOMMU_ATTRIBUTE_DUAL_ADDRESS_CYCLE) == 0 &&
            (EFI_PHYSICAL_ADDRESS)(UINTN)HostAddress + *NumberOfBytes > SIZE_4GB);

  if (!Bounce) {
    switch (Operation) {
    case EfiBusMasterRead:
    case EfiBusMasterWrite:
      //
      // For streaming DMA, it is sufficient if the buffer is aligned to
      // the CPUs DMA buffer alignment.
      //
      AlignMask = mCpu->DmaBufferAlignment - 1;
      if ((((UINTN) HostAddress | *NumberOfBytes) & AlignMask) == 0) {
        break;
      }
      // fall through

    case EfiBusMasterCommonBuffer:
      //
      // Check whether the host address refers to an uncached mapping.
      //
      Status = gDS->GetMemorySpaceDescriptor (
                      (EFI_PHYSICAL_ADDRESS)(UINTN)HostAddress,
                      &GcdDescriptor
                      );
      if (EFI_ERROR (Status) ||
          (GcdDescriptor.Attributes & (EFI_MEMORY_WB|EFI_MEMORY_WT)) != 0) {
        Bounce = TRUE;
      }
      break;

    default:
      ASSERT (FALSE);
    }
  }

  if (Bounce) {
    if (Operation == EfiBusMasterCommonBuffer) {
      Status = EFI_DEVICE_ERROR;
      goto FreeMapInfo;
    }

    Status = NonCoherentIoAllocateBuffer (
               This,
               AllocateAnyPages,
               EfiBootServicesData,
               EFI_SIZE_TO_PAGES (MapInfo->NumberOfBytes),
               &AllocAddress);
    if (EFI_ERROR (Status)) {
      goto FreeMapInfo;
    }
    MapInfo->AllocAddress = AllocAddress;
    if (Operation == EfiBusMasterRead) {
      gBS->CopyMem ((VOID *)AllocAddress, HostAddress, *NumberOfBytes);
    }
    *DeviceAddress = MapInfo->AllocAddress;
  } else {
    MapInfo->AllocAddress = 0;
    *DeviceAddress = (EFI_PHYSICAL_ADDRESS)(UINTN)HostAddress;

    //
    // We are not using a bounce buffer: the mapping is sufficiently
    // aligned to allow us to simply flush the caches. Note that cleaning
    // the caches is necessary for both data directions:
    // - for bus master read, we want the latest data to be present
    //   in main memory
    // - for bus master write, we don't want any stale dirty cachelines that
    //   may be written back unexpectedly, and clobber the data written to
    //   main memory by the device.
    //
    mCpu->FlushDataCache (
            mCpu,
            (EFI_PHYSICAL_ADDRESS)(UINTN)HostAddress,
            *NumberOfBytes,
            EfiCpuFlushTypeWriteBack
            );
  }

  *Mapping = MapInfo;
  return EFI_SUCCESS;

FreeMapInfo:
  FreePool (MapInfo);

  return Status;
}

/**
  Completes the Map() operation and releases any corresponding resources.

  @param  This                  A pointer to the EFI_DEVICE_IO_PROTOCOL
                                instance.
  @param  Mapping               The mapping value returned from Map().

  @retval EFI_SUCCESS           The range was unmapped.

**/
STATIC
EFI_STATUS
EFIAPI
NonCoherentIoUnmap (
  IN  EFI_DEVICE_IO_PROTOCOL           *This,
  IN  VOID                             *Mapping
  )
{
  NON_DISCOVERABLE_IO_DEVICE_MAP_INFO  *MapInfo;

  if (Mapping == NULL) {
    return EFI_DEVICE_ERROR;
  }

  MapInfo = Mapping;
  if (MapInfo->AllocAddress != 0) {
    //
    // We are using a bounce buffer: copy back the data if necessary,
    // and free the buffer.
    //
    if (MapInfo->Operation == EfiBusMasterWrite) {
      mCpu->FlushDataCache (
              mCpu,
              (EFI_PHYSICAL_ADDRESS)(UINTN)MapInfo->HostAddress,
              MapInfo->NumberOfBytes,
              EfiCpuFlushTypeInvalidate
              );
      gBS->CopyMem (
             MapInfo->HostAddress,
             (VOID *)(UINTN)MapInfo->AllocAddress,
             MapInfo->NumberOfBytes
             );
    }
    NonCoherentIoFreeBuffer (
      This,
      EFI_SIZE_TO_PAGES (MapInfo->NumberOfBytes),
      MapInfo->AllocAddress
      );
  } else {
    //
    // We are *not* using a bounce buffer: if this is a bus master write,
    // we have to invalidate the caches so the CPU will see the uncached
    // data written by the device.
    //
    if (MapInfo->Operation == EfiBusMasterWrite) {
      mCpu->FlushDataCache (
              mCpu,
              (EFI_PHYSICAL_ADDRESS)(UINTN)MapInfo->HostAddress,
              MapInfo->NumberOfBytes,
              EfiCpuFlushTypeInvalidate
              );
    }
  }
  FreePool (MapInfo);
  return EFI_SUCCESS;
}

STATIC CONST EFI_DEVICE_IO_PROTOCOL IoTemplate =
{
  { IoMemRead, IoMemWrite },
  { IoIoRead,  IoIoWrite },
  { 0, 0 },
  CoherentIoMap,
  0,
  CoherentIoUnmap,
  CoherentIoAllocateBuffer,
  0,
  CoherentIoFreeBuffer,
};

/**
  Initialize DevIo Protocol.

  @param  Dev      Point to NON_DISCOVERABLE_IO_DEVICE instance.

**/
VOID
InitializeIoProtocol (
  NON_DISCOVERABLE_IO_DEVICE           *Dev
  )
{
  InitializeListHead (&Dev->UncachedAllocationList);

  //
  // Copy protocol structure
  //
  CopyMem(&Dev->Io, &IoTemplate, sizeof (IoTemplate));

  if (Dev->Device->DmaType == NonDiscoverableDeviceDmaTypeNonCoherent) {
    Dev->Io.AllocateBuffer  = NonCoherentIoAllocateBuffer;
    Dev->Io.FreeBuffer      = NonCoherentIoFreeBuffer;
    Dev->Io.Map             = NonCoherentIoMap;
    Dev->Io.Unmap           = NonCoherentIoUnmap;
  } else {
    Dev->Io.AllocateBuffer  = CoherentIoAllocateBuffer;
    Dev->Io.FreeBuffer      = CoherentIoFreeBuffer;
    Dev->Io.Map             = CoherentIoMap;
    Dev->Io.Unmap           = CoherentIoUnmap;
  }
}

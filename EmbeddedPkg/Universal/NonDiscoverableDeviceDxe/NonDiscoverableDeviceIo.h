/** @file

  Copyright (C) 2016-2018, Linaro Ltd. All rights reserved.<BR>

  This program and the accompanying materials are licensed and made available
  under the terms and conditions of the BSD License which accompanies this
  distribution. The full text of the license may be found at
  http://opensource.org/licenses/bsd-license.php

  THE PROGRAM IS DISTRIBUTED UNDER THE BSD LICENSE ON AN "AS IS" BASIS, WITHOUT
  WARRANTIES OR REPRESENTATIONS OF ANY KIND, EITHER EXPRESS OR IMPLIED.

**/

#ifndef __NON_DISCOVERABLE_DEVICE_IO_H__
#define __NON_DISCOVERABLE_DEVICE_IO_H__

#include <Library/DebugLib.h>

#include <Protocol/ComponentName.h>
#include <Protocol/ComponentName2.h>
#include <Protocol/Cpu.h>
#include <Protocol/DeviceIo.h>
#include <Protocol/NonDiscoverableDevice.h>

#define NON_DISCOVERABLE_IO_DEVICE_SIG SIGNATURE_32 ('N', 'D', 'I', 'D')

#define NON_DISCOVERABLE_IO_DEVICE_FROM_IO(IoPointer)      \
        CR (IoPointer, NON_DISCOVERABLE_IO_DEVICE, Io,     \
            NON_DISCOVERABLE_IO_DEVICE_SIG)

extern EFI_CPU_ARCH_PROTOCOL      *mCpu;

typedef struct {
  //
  // The linked-list next pointer
  //
  LIST_ENTRY          List;
  //
  // The address of the uncached allocation
  //
  VOID                *HostAddress;
  //
  // The number of pages in the allocation
  //
  UINTN               NumPages;
  //
  // The attributes of the allocation
  //
  UINT64              Attributes;
} NON_DISCOVERABLE_DEVICE_UNCACHED_ALLOCATION;

typedef struct {
  UINT32                    Signature;
  //
  // The bound non-discoverable device protocol instance
  //
  NON_DISCOVERABLE_DEVICE   *Device;
  //
  // The exposed I/O protocol instance.
  //
  EFI_DEVICE_IO_PROTOCOL    Io;
  //
  // The I/O attributes for this device
  //
  UINT64                    Attributes;
  //
  // Whether this device has been enabled
  //
  BOOLEAN                   Enabled;
  //
  // Linked list to keep track of uncached allocations performed
  // on behalf of this device
  //
  LIST_ENTRY                UncachedAllocationList;
} NON_DISCOVERABLE_IO_DEVICE;

/**
  Initialize Io Protocol.

  @param  Device      Point to NON_DISCOVERABLE_IO_DEVICE instance.

**/
VOID
InitializeIoProtocol (
  NON_DISCOVERABLE_IO_DEVICE     *Device
  );

extern EFI_COMPONENT_NAME_PROTOCOL gComponentName;
extern EFI_COMPONENT_NAME2_PROTOCOL gComponentName2;

#endif /* __NON_DISCOVERABLE_DEVICE_IO_H__ */

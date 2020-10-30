/** @file

  Copyright (C) 2016-2018, Linaro Ltd. All rights reserved.<BR>

  This program and the accompanying materials are licensed and made available
  under the terms and conditions of the BSD License which accompanies this
  distribution. The full text of the license may be found at
  http://opensource.org/licenses/bsd-license.php

  THE PROGRAM IS DISTRIBUTED UNDER THE BSD LICENSE ON AN "AS IS" BASIS, WITHOUT
  WARRANTIES OR REPRESENTATIONS OF ANY KIND, EITHER EXPRESS OR IMPLIED.

**/

#include <Library/BaseMemoryLib.h>
#include <Library/DebugLib.h>
#include <Library/MemoryAllocationLib.h>
#include <Library/UefiBootServicesTableLib.h>
#include <Library/UefiLib.h>

#include <Protocol/DriverBinding.h>

#include "NonDiscoverableDeviceIo.h"


EFI_CPU_ARCH_PROTOCOL      *mCpu;

//
// We only support the following device types
//
STATIC
CONST EFI_GUID * CONST
SupportedNonDiscoverableDevices[] = {
  &gEdkiiNonDiscoverableSdhciDeviceGuid,
  &gEdkiiNonDiscoverableUfsDeviceGuid,
};

//
// Probe, start and stop functions of this driver, called by the DXE core for
// specific devices.
//
// The following specifications document these interfaces:
// - Driver Writer's Guide for UEFI 2.3.1 v1.01, 9 Driver Binding Protocol
// - UEFI Spec 2.3.1 + Errata C, 10.1 EFI Driver Binding Protocol
//
// The implementation follows:
// - Driver Writer's Guide for UEFI 2.3.1 v1.01
//   - 5.1.3.4 OpenProtocol() and CloseProtocol()
// - UEFI Spec 2.3.1 + Errata C
//   -  6.3 Protocol Handler Services
//

/**
  Supported function of Driver Binding protocol for this driver.
  Test to see if this driver supports ControllerHandle.

  @param This                   Protocol instance pointer.
  @param DeviceHandle           Handle of device to test.
  @param RemainingDevicePath    A pointer to the device path.
                                it should be ignored by device driver.

  @retval EFI_SUCCESS           This driver supports this device.
  @retval other                 This driver does not support this device.

**/
STATIC
EFI_STATUS
EFIAPI
NonDiscoverableIoDeviceSupported (
  IN EFI_DRIVER_BINDING_PROTOCOL *This,
  IN EFI_HANDLE                  DeviceHandle,
  IN EFI_DEVICE_PATH_PROTOCOL    *RemainingDevicePath
  )
{
  NON_DISCOVERABLE_DEVICE             *Device;
  EFI_STATUS                          Status;
  INTN                                Idx;

  Status = gBS->OpenProtocol (DeviceHandle,
                  &gEdkiiNonDiscoverableDeviceProtocolGuid, (VOID **)&Device,
                  This->DriverBindingHandle, DeviceHandle,
                  EFI_OPEN_PROTOCOL_BY_DRIVER);
  if (EFI_ERROR (Status)) {
    return Status;
  }

  Status = EFI_UNSUPPORTED;
  for (Idx = 0; Idx < ARRAY_SIZE (SupportedNonDiscoverableDevices); Idx++) {
    if (CompareGuid (Device->Type, SupportedNonDiscoverableDevices [Idx])) {
      Status = EFI_SUCCESS;
      break;
    }
  }

  if (EFI_ERROR (Status)) {
    goto CloseProtocol;
  }

CloseProtocol:
  gBS->CloseProtocol (DeviceHandle, &gEdkiiNonDiscoverableDeviceProtocolGuid,
         This->DriverBindingHandle, DeviceHandle);

  return Status;
}

/**
  This routine is called right after the .Supported() called and
  Start this driver on ControllerHandle.

  @param This                   Protocol instance pointer.
  @param DeviceHandle           Handle of device to bind driver to.
  @param RemainingDevicePath    A pointer to the device path.
                                it should be ignored by device driver.

  @retval EFI_SUCCESS           This driver is added to this device.
  @retval other                 Some error occurs when binding this driver to this device.

**/
STATIC
EFI_STATUS
EFIAPI
NonDiscoverableIoDeviceStart (
  IN EFI_DRIVER_BINDING_PROTOCOL *This,
  IN EFI_HANDLE                  DeviceHandle,
  IN EFI_DEVICE_PATH_PROTOCOL    *RemainingDevicePath
  )
{
  EFI_STATUS                     Status;
  NON_DISCOVERABLE_IO_DEVICE     *Dev;

  Dev = AllocateZeroPool (sizeof *Dev);
  if (Dev == NULL) {
    return EFI_OUT_OF_RESOURCES;
  }

  Status = gBS->OpenProtocol (DeviceHandle,
                  &gEdkiiNonDiscoverableDeviceProtocolGuid,
                  (VOID **)&Dev->Device, This->DriverBindingHandle,
                  DeviceHandle, EFI_OPEN_PROTOCOL_BY_DRIVER);
  if (EFI_ERROR (Status)) {
    goto FreeDev;
  }

  Dev->Signature = NON_DISCOVERABLE_IO_DEVICE_SIG;

  InitializeIoProtocol (Dev);

  Status = gBS->InstallProtocolInterface (
                  &DeviceHandle,
                  &gEmbeddedNonDiscoverableIoProtocolGuid,
                  EFI_NATIVE_INTERFACE,
                  &Dev->Io
                  );
  if (EFI_ERROR (Status)) {
    goto CloseProtocol;
  }
  return EFI_SUCCESS;

CloseProtocol:
  gBS->CloseProtocol (DeviceHandle, &gEdkiiNonDiscoverableDeviceProtocolGuid,
         This->DriverBindingHandle, DeviceHandle);
FreeDev:
  FreePool (Dev);

  return Status;
}

/**
  Stop this driver on ControllerHandle.

  @param This               Protocol instance pointer.
  @param DeviceHandle       Handle of device to stop driver on.
  @param NumberOfChildren   Not used.
  @param ChildHandleBuffer  Not used.

  @retval EFI_SUCCESS   This driver is removed from this device.
  @retval other         Some error occurs when removing this driver from this
                        device.

**/
STATIC
EFI_STATUS
EFIAPI
NonDiscoverableIoDeviceStop (
  IN EFI_DRIVER_BINDING_PROTOCOL *This,
  IN EFI_HANDLE                  DeviceHandle,
  IN UINTN                       NumberOfChildren,
  IN EFI_HANDLE                  *ChildHandleBuffer
  )
{

  gBS->CloseProtocol (DeviceHandle, &gEdkiiNonDiscoverableDeviceProtocolGuid,
         This->DriverBindingHandle, DeviceHandle);

  return EFI_SUCCESS;
}


//
// The static object that groups the Supported() (ie. probe), Start() and
// Stop() functions of the driver together. Refer to UEFI Spec 2.3.1 + Errata
// C, 10.1 EFI Driver Binding Protocol.
//
STATIC EFI_DRIVER_BINDING_PROTOCOL gDriverBinding = {
  &NonDiscoverableIoDeviceSupported,
  &NonDiscoverableIoDeviceStart,
  &NonDiscoverableIoDeviceStop,
  0x10, // Version, must be in [0x10 .. 0xFFFFFFEF] for IHV-developed drivers
  NULL,
  NULL
};

/**
  Entry point of this driver.

  @param  ImageHandle     Image handle this driver.
  @param  SystemTable     Pointer to the System Table.

  @retval EFI_SUCCESS     The entry point is executed successfully.
  @retval other           Some error occurred when executing this entry point.

**/
EFI_STATUS
EFIAPI
NonDiscoverableDeviceDxeEntryPoint (
  IN EFI_HANDLE       ImageHandle,
  IN EFI_SYSTEM_TABLE *SystemTable
  )
{
  EFI_STATUS      Status;

  Status = gBS->LocateProtocol (&gEfiCpuArchProtocolGuid, NULL, (VOID **)&mCpu);
  ASSERT_EFI_ERROR(Status);

  return EfiLibInstallDriverBindingComponentName2 (
           ImageHandle,
           SystemTable,
           &gDriverBinding,
           ImageHandle,
           &gComponentName,
           &gComponentName2
           );
}
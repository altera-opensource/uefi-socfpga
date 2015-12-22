/** @file
  Altera Platform DXE Init functions

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

  Copyright (c) 2013-2015, ARM Limited. All rights reserved.

  This program and the accompanying materials
  are licensed and made available under the terms and conditions of the BSD License
  which accompanies this distribution.  The full text of the license may be found at
  http://opensource.org/licenses/bsd-license.php

  THE PROGRAM IS DISTRIBUTED UNDER THE BSD 3 Clause LICENSE ON AN "AS IS" BASIS,
  WITHOUT WARRANTIES OR REPRESENTATIONS OF ANY KIND, EITHER EXPRESS OR IMPLIED.

**/

#include <Uefi.h>
#include <Guid/GlobalVariable.h>
#include <Library/ArmLib.h>
#include <Library/ArmShellCmdLib.h>
#include <Library/BaseLib.h>
#include <Library/BaseMemoryLib.h>
#include <Library/DebugLib.h>
#include <Library/DevicePathLib.h>
#include <Library/DxeServicesTableLib.h>
#include <Library/IoLib.h>
#include <Library/MemoryAllocationLib.h>
#include <Library/PrintLib.h>
#include <Library/UefiBootServicesTableLib.h>
#include <Library/UefiLib.h>
#include <Library/UefiRuntimeServicesTableLib.h>
#include <Protocol/DevicePathFromText.h>


EFI_STATUS
EFIAPI
SetDefaultBootEntries (
  VOID
  );

EFI_STATUS
EFIAPI
AlteraPlatformDxeEntryPoint (
  IN EFI_HANDLE         ImageHandle,
  IN EFI_SYSTEM_TABLE   *SystemTable
  )
{
  EFI_STATUS            Status;

  // Install dynamic Shell command "runaxf"
  // to allow user to type runaxf command under UEFI Shell to load .axf ELF binaries.
  // Example: fs0:\> runaxf file.axf
  Status = ShellDynCmdRunAxfInstall (ImageHandle);
  if (EFI_ERROR (Status)) {
    DEBUG ((EFI_D_ERROR, "ArmPlatformDxe: Failed to install ShellDynCmdRunAxf\n"));
  }

  // Create default Boot Entries to be consume by BDS driver later
  Status = SetDefaultBootEntries ();
  if (EFI_ERROR (Status)) {
    return Status;
  }

  return Status;
}


/**
 * Build and Set UEFI Variable Boot####
 *
 * @param BootVariableName       Name of the UEFI Variable
 * @param Attributes             'Attributes' for the Boot#### variable as per UEFI spec
 * @param BootDescription        Description of the Boot#### variable
 * @param DevicePath             EFI Device Path of the EFI Application to boot
 * @param OptionalData           Parameters to pass to the EFI application
 * @param OptionalDataSize       Size of the parameters to pass to the EFI application
 *
 * @return EFI_OUT_OF_RESOURCES  A memory allocation failed
 * @return                       Return value of RT.SetVariable
 */
STATIC
EFI_STATUS
BootOptionCreate (
  IN  CHAR16                    BootVariableName[9],
  IN  UINT32                    Attributes,
  IN  CHAR16*                   BootDescription,
  IN  EFI_DEVICE_PATH_PROTOCOL* DevicePath,
  IN  UINT8*                    OptionalData,
  IN  UINTN                     OptionalDataSize
  )
{
  UINTN                         VariableSize;
  UINT8                         *Variable;
  UINT8                         *VariablePtr;
  UINTN                         FilePathListLength;
  UINTN                         BootDescriptionSize;

  FilePathListLength  = GetDevicePathSize (DevicePath);
  BootDescriptionSize = StrSize (BootDescription);

  // Each Boot#### variable is built as follow:
  //   UINT32                   Attributes
  //   UINT16                   FilePathListLength
  //   CHAR16*                  Description
  //   EFI_DEVICE_PATH_PROTOCOL FilePathList[]
  //   UINT8                    OptionalData[]
  VariableSize = sizeof (UINT32) + sizeof (UINT16) +
      BootDescriptionSize + FilePathListLength + OptionalDataSize;
  Variable = AllocateZeroPool (VariableSize);
  if (Variable == NULL) {
    return EFI_OUT_OF_RESOURCES;
  }

  // 'Attributes' field
  *(UINT32*)Variable = Attributes;
  // 'FilePathListLength' field
  VariablePtr = Variable + sizeof (UINT32);
  *(UINT16*)VariablePtr = FilePathListLength;
  // 'Description' field
  VariablePtr += sizeof (UINT16);
  CopyMem (VariablePtr, BootDescription, BootDescriptionSize);
  // 'FilePathList' field
  VariablePtr += BootDescriptionSize;
  CopyMem (VariablePtr, DevicePath, FilePathListLength);
  // 'OptionalData' field
  VariablePtr += FilePathListLength;
  CopyMem (VariablePtr, OptionalData, OptionalDataSize);

  return gRT->SetVariable (
      BootVariableName,
      &gEfiGlobalVariableGuid,
      EFI_VARIABLE_NON_VOLATILE | EFI_VARIABLE_BOOTSERVICE_ACCESS | EFI_VARIABLE_RUNTIME_ACCESS,
      VariableSize, Variable
      );
}


EFI_STATUS
EFIAPI
SetDefaultBootEntries (
  VOID
  )
{
  CHAR16*          DefaultBootDescription;
  CHAR16*          DefaultBootArgument;
  UINTN            DefaultBootArgumentSize;
  EFI_STATUS       Status;
  UINTN            Size;
  EFI_DEVICE_PATH* BootDevicePath;
  UINT16           BootOrder[1];
  EFI_DEVICE_PATH_FROM_TEXT_PROTOCOL  *EfiDevicePathFromTextProtocol;

  BootDevicePath       = NULL;

  //
  // Because the driver has a dependency on gEfiVariable(Write)ArchProtocolGuid
  // (see [Depex] section of the INF file), we know we can safely access the
  // UEFI Variable at that stage.
  //
  Size = 0;
  Status = gRT->GetVariable (L"BootOrder", &gEfiGlobalVariableGuid, NULL, &Size, NULL);
  if (Status != EFI_NOT_FOUND) {
    return EFI_SUCCESS;
  }

  Status = gBS->LocateProtocol (
                  &gEfiDevicePathFromTextProtocolGuid,
                  NULL,
                  (VOID **)&EfiDevicePathFromTextProtocol
                  );
  if (EFI_ERROR (Status)) {
    //
    // You must provide an implementation of DevicePathFromTextProtocol
    // in your firmware (eg: DevicePathDxe)
    //
    DEBUG ((EFI_D_ERROR, "Error: Require DevicePathFromTextProtocol\n"));
    return Status;
  }
  //
  // Convert the text string in PcdDefaultBootDevicePath to DevicePath
  //
  BootDevicePath = EfiDevicePathFromTextProtocol->ConvertTextToDevicePath (
                     (CHAR16*)PcdGetPtr (PcdDefaultBootDevicePath)
                     );
  if (BootDevicePath == NULL) {
    DEBUG ((EFI_D_ERROR, "Error: Cannot find DevicePath as defined in PcdDefaultBootDevicePath\n"));
    return EFI_UNSUPPORTED;
  }

  DefaultBootDescription = (CHAR16*)PcdGetPtr (PcdDefaultBootDescription);
  DefaultBootArgument = (CHAR16*)PcdGetPtr (PcdDefaultBootArgument);
  DefaultBootArgumentSize = StrSize (DefaultBootArgument);

  //
  // Create Boot0001 environment variable
  //
  Status = BootOptionCreate (
             L"Boot0001",
             LOAD_OPTION_ACTIVE | LOAD_OPTION_CATEGORY_BOOT,
             DefaultBootDescription,
             BootDevicePath,
             (UINT8*)DefaultBootArgument, DefaultBootArgumentSize
             );
  if (EFI_ERROR (Status)) {
    ASSERT_EFI_ERROR (Status);
    goto Error;
  }

  //
  // Add the new Boot Index to the list
  //
  BootOrder[0] = 1; // Boot0001
  Status = gRT->SetVariable (
                  L"BootOrder",
                  &gEfiGlobalVariableGuid,
                  EFI_VARIABLE_NON_VOLATILE       |
                  EFI_VARIABLE_BOOTSERVICE_ACCESS |
                  EFI_VARIABLE_RUNTIME_ACCESS,
                  sizeof (BootOrder),
                  BootOrder
                  );

Error:
  if (BootDevicePath != NULL) {
    FreePool (BootDevicePath);
  }

  if (EFI_ERROR (Status)) {
    DEBUG ((
      EFI_D_ERROR,
      "ArmPlatformDxe - The setting of the default boot entries failed - %r\n",
      Status
      ));
  }

  return Status;
}


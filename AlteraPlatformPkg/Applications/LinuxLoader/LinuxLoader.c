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


#include <Library/UefiApplicationEntryPoint.h>
#include <Library/BaseMemoryLib.h>

#include <Protocol/DevicePathFromText.h>

#include "LinuxLoader.h"

/**
  The user Entry Point for Application. The user code starts with this function
  as the real entry point for the application.

  @param[in]  ImageHandle  The firmware allocated handle for the EFI image.
  @param[in]  SystemTable  A pointer to the EFI System Table.

  @retval  EFI_SUCCESS            The entry point was executed successfully.
  @retval  EFI_NOT_FOUND          Protocol not found.
  @retval  EFI_NOT_FOUND          Path to the Linux kernel not found.
  @retval  EFI_ABORTED            The initialisation of the Shell Library failed.
  @retval  EFI_INVALID_PARAMETER  At least one parameter is not valid or there is a
                                  conflict between two parameters.
  @retval  EFI_OUT_OF_RESOURCES   A memory allocation failed.

**/
EFI_STATUS
EFIAPI
LinuxLoaderEntryPoint (
  IN EFI_HANDLE        ImageHandle,
  IN EFI_SYSTEM_TABLE  *SystemTable
  )
{
  EFI_STATUS                          Status;
  EFI_DEVICE_PATH_FROM_TEXT_PROTOCOL  *EfiDevicePathFromTextProtocol;
  EFI_SHELL_PARAMETERS_PROTOCOL       *ShellParameters;
  CHAR16                              *KernelPath;
  CHAR16                              *FdtPath;
  CHAR16                              *InitrdPath;
  CHAR16                              *KernelTextDevicePath;
  CHAR16                              *FdtTextDevicePath;
  CHAR16                              *InitrdTextDevicePath;
  CHAR16                              *LinuxCommandLine;
  UINTN                               AtagMachineType;
  EFI_DEVICE_PATH                     *KernelDevicePath;
  EFI_DEVICE_PATH                     *FdtDevicePath;
  EFI_DEVICE_PATH                     *InitrdDevicePath;
  CHAR8                               *AsciiLinuxCommandLine;
  EFI_PHYSICAL_ADDRESS                SystemMemoryBase;

  Status = gBS->LocateProtocol (
                  &gEfiDevicePathFromTextProtocolGuid,
                  NULL,
                  (VOID **)&EfiDevicePathFromTextProtocol
                  );
  if (EFI_ERROR (Status)) {
    return EFI_NOT_FOUND;
  }

  //
  // Register the strings for the user interface in the HII Database.
  // This shows the way to the multi-language support, even if
  // only the English language is actually supported. The strings to register
  // are stored in the "LinuxLoaderStrings[]" array. This array is
  // built by the building process from the "*.uni" file associated to
  // the present application (cf. LinuxLoader.inf). Examine the Build
  // folder of the application and you will find the array defined in the
  // LinuxLoaderStrDefs.h file.
  //
  mLinuxLoaderHiiHandle = HiiAddPackages (
                             &mLinuxLoaderHiiGuid,
                             ImageHandle,
                             LinuxLoaderStrings,
                             NULL
                             );
  if (mLinuxLoaderHiiHandle == NULL) {
    return EFI_NOT_FOUND;
  }

  Status = gBS->HandleProtocol (
                  ImageHandle,
                  &gEfiShellParametersProtocolGuid,
                  (VOID**)&ShellParameters
                  );

  KernelDevicePath      = NULL;
  FdtDevicePath         = NULL;
  InitrdDevicePath      = NULL;
  AsciiLinuxCommandLine = NULL;

  //
  // Call the proper function to handle the command line
  // depending on whether the application has been called
  // from the Shell or not.
  //

  if (!EFI_ERROR (Status)) {
    KernelTextDevicePath = NULL;
    FdtTextDevicePath    = NULL;
    InitrdTextDevicePath = NULL;

    Status = ProcessShellParameters (
               &KernelPath, &FdtPath, &InitrdPath, &LinuxCommandLine, &AtagMachineType
               );
    if (EFI_ERROR (Status)) {
      goto Error;
    }

    KernelDevicePath = gEfiShellProtocol->GetDevicePathFromFilePath (KernelPath);
    if (KernelDevicePath != NULL) {
      FreePool (KernelPath);
    } else {
      KernelTextDevicePath = KernelPath;
    }

    if (FdtPath != NULL) {
      FdtDevicePath = gEfiShellProtocol->GetDevicePathFromFilePath (FdtPath);
      if (FdtDevicePath != NULL) {
        FreePool (FdtPath);
      } else {
        FdtTextDevicePath = FdtPath;
      }
    }

    if (InitrdPath != NULL) {
      InitrdDevicePath = gEfiShellProtocol->GetDevicePathFromFilePath (InitrdPath);
      if (InitrdDevicePath != NULL) {
        FreePool (InitrdPath);
      } else {
        InitrdTextDevicePath = InitrdPath;
      }
    }

  } else {
    Status = ProcessAppCommandLine (
               &KernelTextDevicePath, &FdtTextDevicePath,
               &InitrdTextDevicePath, &LinuxCommandLine, &AtagMachineType
               );
    if (EFI_ERROR (Status)) {
      goto Error;
    }
  }

  Status = EFI_INVALID_PARAMETER;
  if (KernelTextDevicePath != NULL) {
    KernelDevicePath = EfiDevicePathFromTextProtocol->ConvertTextToDevicePath (
                                                        KernelTextDevicePath
                                                        );
    if (KernelDevicePath == NULL) {
      goto Error;
    }
  }
  if (FdtTextDevicePath != NULL) {
    FdtDevicePath = EfiDevicePathFromTextProtocol->ConvertTextToDevicePath (
                                                     FdtTextDevicePath
                                                     );
    if (FdtDevicePath == NULL) {
      goto Error;
    }
  }
  if (InitrdTextDevicePath != NULL) {
    InitrdDevicePath = EfiDevicePathFromTextProtocol->ConvertTextToDevicePath (
                                                        InitrdTextDevicePath
                                                        );
    if (InitrdDevicePath == NULL) {
      goto Error;
    }
  }

  if (LinuxCommandLine != NULL) {
    AsciiLinuxCommandLine = AllocatePool ((StrLen (LinuxCommandLine) + 1) * sizeof (CHAR8));
    if (AsciiLinuxCommandLine == NULL) {
      Status = EFI_OUT_OF_RESOURCES;
      goto Error;
    }
    UnicodeStrToAsciiStr (LinuxCommandLine, AsciiLinuxCommandLine);
  }

  //
  // Assume base of System Memory is always zero
  //
  SystemMemoryBase = 0;

  if (AtagMachineType != ARM_FDT_MACHINE_TYPE) {
    Status = BootLinuxAtag (SystemMemoryBase, KernelDevicePath, InitrdDevicePath, AsciiLinuxCommandLine, AtagMachineType);
  } else {
    Status = BootLinuxFdt (SystemMemoryBase, KernelDevicePath, InitrdDevicePath, FdtDevicePath, AsciiLinuxCommandLine);
  }

Error:
  if (KernelTextDevicePath != NULL) {
    FreePool (KernelTextDevicePath);
  }
  if (FdtTextDevicePath != NULL) {
    FreePool (FdtTextDevicePath);
  }
  if (InitrdTextDevicePath != NULL) {
    FreePool (InitrdTextDevicePath);
  }
  if (LinuxCommandLine != NULL) {
    FreePool (LinuxCommandLine);
  }
  if (KernelDevicePath != NULL) {
    FreePool (KernelDevicePath);
  }
  if (FdtDevicePath != NULL) {
    FreePool (FdtDevicePath);
  }
  if (InitrdDevicePath != NULL) {
    FreePool (InitrdDevicePath);
  }
  if (AsciiLinuxCommandLine != NULL) {
    FreePool (AsciiLinuxCommandLine);
  }

  if (EFI_ERROR (Status)) {
    PrintHii (NULL, STRING_TOKEN (STR_ERROR), Status);
  }

  HiiRemovePackages (mLinuxLoaderHiiHandle);

  return Status;
}

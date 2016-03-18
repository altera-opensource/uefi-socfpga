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

#ifndef _ALTERA_SOC_FPGA_PEI_MAIN_H_
#define _ALTERA_SOC_FPGA_PEI_MAIN_H_

VOID
EFIAPI
AlteraSocFpgaPeiMainEntry (
  IN CONST EFI_SEC_PEI_HAND_OFF        *SecCoreData,
  IN CONST EFI_PEI_PPI_DESCRIPTOR      *PpiList,
  IN VOID                              *Data
  );

VOID
EFIAPI
MemorySerialLogInit (
  VOID
  );

VOID
EFIAPI
PrintModuleEntryPointAndMemoryMapInfoToMemorySerialLogOrSemihostingConsoleIfEnabled (
  IN CONST EFI_SEC_PEI_HAND_OFF        *SecCoreData
  );

VOID
EFIAPI
InitializeHOBs (
  IN CONST EFI_SEC_PEI_HAND_OFF    *SecCoreData,
  IN CONST EFI_PEI_PPI_DESCRIPTOR  *PpiList,
  IN CONST UINT64                   StartTimeStamp
  );

EFI_STATUS
EFIAPI
GetPlatformPpi (
IN CONST EFI_SEC_PEI_HAND_OFF      *SecCoreData,
  IN CONST EFI_PEI_PPI_DESCRIPTOR  *PpiList,
  IN  EFI_GUID                     *PpiGuid,
  OUT VOID                         **Ppi
  );

VOID
EFIAPI
BuildGlobalVariableHob (
  IN EFI_PHYSICAL_ADDRESS         GlobalVariableBase,
  IN UINT32                       GlobalVariableSize
  );

EFI_STATUS
EFIAPI
BuildSystemMemoryHOBs (
  IN EFI_PHYSICAL_ADDRESS     DramMemoryBase,
  IN UINT64                   DramMemorySize
  );

VOID
EFIAPI
BootUnCompressedDxeFv (
  IN CONST EFI_SEC_PEI_HAND_OFF  *SecCoreData
  );

VOID
EFIAPI
BootCompressedDxeFv (
  IN CONST EFI_SEC_PEI_HAND_OFF  *SecCoreData
  );

VOID
EFIAPI
EnterDxeCoreEntryPoint (
  IN CONST EFI_SEC_PEI_HAND_OFF  *SecCoreData,
  IN UINTN                       DxeFvBase,
  IN UINTN                       DxeFvSize
  );

// List of Library Constructor in use
// This is declare here to workaround linker issue

RETURN_STATUS
EFIAPI
TimerConstructor (
  VOID
  );

EFI_STATUS
EFIAPI
ExtractGuidedSectionLibConstructor (
  VOID
  );

EFI_STATUS
EFIAPI
LzmaDecompressLibConstructor (
  VOID
  );

#endif /* _PREPI_H_ */

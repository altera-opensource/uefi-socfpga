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

#ifndef __CLOCKMANAGER_H__
#define __CLOCKMANAGER_H__

#include "DeviceTree.h"

// ==================================================================
// Functions Definition
// ==================================================================

//
// Public Functions
//
VOID
EFIAPI
ConfigureClockManager (
  IN  CONST VOID*  Fdt
  );

VOID
EFIAPI
DisplayClockManagerInfo (
  VOID
  );

VOID
EFIAPI
DisplayClockFrequencyInfo (
  VOID
  );

//
// Private Functions
//

VOID
EFIAPI
SetClockManagerCfg (
  IN CLOCK_MANAGER_CONFIG*  Cfg
  );

VOID
EFIAPI
SaveClockManagerCfg (
  IN  CONST VOID*  Fdt
  );

UINT32
EFIAPI
Get_l4_sp_ClockFrequencyInMhz (
  VOID
  );

BOOLEAN
EFIAPI
IsPllRampRequired(
  IN UINTN MainOrPeripheral
  );

UINT32
EFIAPI
GetSafePllNumerator (
  IN UINTN  MainOrPeripheral,
  IN UINT32 SafeHz
  );

VOID
EFIAPI
PllRampMain(
  IN UINT32 PllRampMainHz
  );

VOID
EFIAPI
PllRampPeripheral(
  IN UINT32 PllRampMainHz
  );

UINT32
EFIAPI
GetMainVcoClockFrequencyInHz(
  VOID
  );

UINT32
EFIAPI
GetPeripheralVcoClockFrequencyInHz(
  VOID
  );

UINT32
EFIAPI
GetMpuClockFrequencyInHz(
  VOID
  );

UINT32
EFIAPI
GetNocClockFrequencyInHz(
  VOID
  );

VOID
EFIAPI
WaitPllLocked (
  VOID
  );

UINT32
EFIAPI
GetMainVcoClockSource(
  VOID
  );

UINT32
EFIAPI
GetPeripheralVcoClockSource(
  VOID
  );


#endif

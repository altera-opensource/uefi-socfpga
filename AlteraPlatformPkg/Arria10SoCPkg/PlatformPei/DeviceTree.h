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

#ifndef __DEVICE_TREE_H__
#define __DEVICE_TREE_H__

#include "DeviceTreeDefCfg.h"

// ==================================================================
// Functions Definition
// ==================================================================

EFI_STATUS
EFIAPI
GetFlattenedDeviceTreePtr (
  OUT VOID**              PtrToDeviceTreePtr
  );

VOID
EFIAPI
GetFdtPropertyValue (
  IN CONST VOID*           Fdt,
  IN       INT32           Node,
  IN       UINT32*         Cfg,
  IN       UINT32          CfgSize,
  IN       PROPERTY_NAME   CfgStr[]
  );

VOID
EFIAPI
ConfigPinMux (
  IN  CONST VOID*   Fdt,
  IN  CONST CHAR8*  SubNodeName
  );

VOID
EFIAPI
GetFirewallCfg (
  IN  CONST VOID*                  Fdt,
  OUT       FIREWALL_CONFIG*       Cfg
  );

VOID
EFIAPI
GetRbfFileCfg (
  IN  CONST VOID*                  Fdt,
  OUT       RBF_FILE_CONFIG*       Cfg
  );

EFI_STATUS
EFIAPI
GetClockSourceCfg (
  IN  CONST VOID*                  Fdt,
  OUT       CLOCK_SOURCE_CONFIG*   Cfg
  );

EFI_STATUS
EFIAPI
GetClockManagerCfg (
  IN  CONST VOID*                  Fdt,
  OUT       CLOCK_MANAGER_CONFIG*  Cfg
  );

BOOLEAN
EFIAPI
IsSkipFpgaConfig (
  IN  CONST VOID*   Fdt
  );
#endif


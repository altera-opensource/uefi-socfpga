/** @file
  ArmPlatformSecLib for Arria 10 SoC

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

  Copyright (c) 2011-2014, ARM Limited. All rights reserved.

  This program and the accompanying materials
  are licensed and made available under the terms and conditions of the BSD License
  which accompanies this distribution.  The full text of the license may be found at
  http://opensource.org/licenses/bsd-license.php

  THE PROGRAM IS DISTRIBUTED UNDER THE BSD 3 Clause LICENSE ON AN "AS IS" BASIS,
  WITHOUT WARRANTIES OR REPRESENTATIONS OF ANY KIND, EITHER EXPRESS OR IMPLIED.

**/

#include <Library/ArmPlatformLib.h>
#include <Library/DebugLib.h>
#include <Library/PcdLib.h>
#include <AlteraPlatform.h>
#include <Drivers/PL310L2Cache.h>
#define L2X0_AUXCTRL_FULL_LINE_OF_ZERO  BIT0

/**
  Initialize the Secure peripherals and memory regions

  If Trustzone is supported by your platform then this function makes the required initialization
  of the secure peripherals and memory regions.

**/
VOID
ArmPlatformSecTrustzoneInit (
  IN  UINTN                     MpId
  )
{
  // Secondary cores might have to set the Secure SGIs into the GICD_IGROUPR0
  if (!ArmPlatformIsPrimaryCore (MpId)) {
    return;
  }

  ASSERT(FALSE);
}

/**
  Initialize controllers that must setup at the early stage

  Some peripherals must be initialized in Secure World.
  For example, some L2x0 requires to be initialized in Secure World

**/
RETURN_STATUS
ArmPlatformSecInitialize (
  IN  UINTN                     MpId
  )
{
  UINTN   L2x0Base;
  UINT32  L2x0TagLatencies;
  UINT32  L2x0DataLatencies;
  UINT32  L2x0AuxOrMask;
  UINT32  L2x0AuxAndMask;
  BOOLEAN CacheEnabled;

  // If it is not the primary core then there is nothing to do
  if (!ArmPlatformIsPrimaryCore (MpId)) {
    return RETURN_SUCCESS;
  }

  // The L2x0 controller must be intialize in Secure World
  L2x0Base = ARM_MPUL2_OFST;
  L2x0TagLatencies   = PL310_TAG_LATENCIES(L2x0_LATENCY_1_CYCLE,L2x0_LATENCY_1_CYCLE,L2x0_LATENCY_1_CYCLE);
  L2x0DataLatencies = PL310_DATA_LATENCIES(L2x0_LATENCY_1_CYCLE,L2x0_LATENCY_2_CYCLES,L2x0_LATENCY_1_CYCLE);
  L2x0AuxOrMask = L2x0_AUXCTRL_DPREFETCH | L2x0_AUXCTRL_IPREFETCH | L2x0_AUXCTRL_EARLY_BRESP | L2X0_AUXCTRL_FULL_LINE_OF_ZERO;
  L2x0AuxAndMask = ~L2X0_AUXCTRL_NSAC;
  CacheEnabled = FALSE;

  L2x0CacheInit(
      L2x0Base,
      L2x0TagLatencies,
      L2x0DataLatencies,
      L2x0AuxOrMask,
      L2x0AuxAndMask,
      FALSE);

  return RETURN_SUCCESS;
}

/**
  Call before jumping to Normal World

  This function allows the firmware platform to do extra actions before
  jumping to the Normal World

**/
VOID
ArmPlatformSecExtraAction (
  IN  UINTN         MpId,
  OUT UINTN*        JumpAddress
  )
{
  *JumpAddress = PcdGet64 (PcdFvBaseAddress);
}


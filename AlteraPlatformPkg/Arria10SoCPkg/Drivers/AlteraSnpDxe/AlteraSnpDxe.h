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

  Copyright (c) 2012-2014, ARM Limited. All rights reserved.

  This program and the accompanying materials
  are licensed and made available under the terms and conditions of the BSD License
  which accompanies this distribution.  The full text of the license may be found at
  http://opensource.org/licenses/bsd-license.php

  THE PROGRAM IS DISTRIBUTED UNDER THE BSD 3 Clause LICENSE ON AN "AS IS" BASIS,
  WITHOUT WARRANTIES OR REPRESENTATIONS OF ANY KIND, EITHER EXPRESS OR IMPLIED.

**/

#ifndef __ALTERA_SNP_DXE_H__
#define __ALTERA_SNP_DXE_H__

#include <Uefi.h>
#include <Uefi/UefiSpec.h>
#include <Base.h>

// Protocols used by this driver
#include <Protocol/SimpleNetwork.h>
#include <Protocol/ComponentName2.h>
#include <Protocol/PxeBaseCode.h>
#include <Protocol/DevicePath.h>

// Libraries used by this driver
#include <Library/UefiLib.h>
#include <Library/DebugLib.h>
#include <Library/UefiBootServicesTableLib.h>
#include <Library/MemoryAllocationLib.h>
#include <Library/IoLib.h>
#include <Library/PcdLib.h>
#include <Library/NetLib.h>
#include <Library/DevicePathLib.h>

#include <AlteraPlatform.h>

#include "PhyDxeUtil.h"
#include "EmacDxeUtil.h"

/*------------------------------------------------------------------------------
  Information Structure
------------------------------------------------------------------------------*/

typedef struct {
  // Driver signature
  UINT32            Signature;
  EFI_HANDLE        ControllerHandle;

  // EFI SNP protocol instances
  EFI_SIMPLE_NETWORK_PROTOCOL Snp;
  EFI_SIMPLE_NETWORK_MODE SnpMode;

  // EFI Snp statistics instance
  EFI_NETWORK_STATISTICS Stats;

  EMAC_DRIVER  MacDriver;
  PHY_DRIVER   PhyDriver;

} EFI_SIMPLE_NETWORK_DRIVER;


#define SNP_DRIVER_SIGNATURE                    SIGNATURE_32('A', 'S', 'N', 'P')
#define INSTANCE_FROM_SNP_THIS(a)               CR(a, EFI_SIMPLE_NETWORK_DRIVER, Snp, SNP_DRIVER_SIGNATURE)


/*---------------------------------------------------------------------------------------------------------------------

  UEFI-Compliant functions for EFI_SIMPLE_NETWORK_PROTOCOL

  Refer to the Simple Network Protocol section (21.1) in the UEFI 2.3.1 Specification for related definitions

---------------------------------------------------------------------------------------------------------------------*/

EFI_STATUS
EFIAPI
SnpStart (
  IN        EFI_SIMPLE_NETWORK_PROTOCOL* Snp
  );

EFI_STATUS
EFIAPI
SnpStop (
  IN        EFI_SIMPLE_NETWORK_PROTOCOL* Snp
  );

EFI_STATUS
EFIAPI
SnpInitialize (
  IN EFI_SIMPLE_NETWORK_PROTOCOL *Snp,
  IN UINTN                       ExtraRxBufferSize OPTIONAL,
  IN UINTN                       ExtraTxBufferSize OPTIONAL
  );

EFI_STATUS
EFIAPI
SnpReset (
  IN EFI_SIMPLE_NETWORK_PROTOCOL *Snp,
  IN BOOLEAN                     ExtendedVerification
  );

EFI_STATUS
EFIAPI
SnpShutdown (
  IN        EFI_SIMPLE_NETWORK_PROTOCOL* Snp
  );

EFI_STATUS
EFIAPI
SnpReceiveFilters (
  IN  EFI_SIMPLE_NETWORK_PROTOCOL  *Snp,
  IN  UINT32                       Enable,
  IN  UINT32                       Disable,
  IN  BOOLEAN                      ResetMCastFilter,
  IN  UINTN                        MCastFilterCnt  OPTIONAL,
  IN  EFI_MAC_ADDRESS              *MCastFilter  OPTIONAL
  );

EFI_STATUS
EFIAPI
SnpStationAddress (
  IN        EFI_SIMPLE_NETWORK_PROTOCOL *Snp,
  IN        BOOLEAN                      Reset,
  IN        EFI_MAC_ADDRESS             *NewMac
);

EFI_STATUS
EFIAPI
SnpStatistics (
  IN        EFI_SIMPLE_NETWORK_PROTOCOL *Snp,
  IN        BOOLEAN                      Reset,
  IN  OUT   UINTN                        *StatSize,
      OUT   EFI_NETWORK_STATISTICS       *Statistics
  );

EFI_STATUS
EFIAPI
SnpMcastIptoMac (
  IN        EFI_SIMPLE_NETWORK_PROTOCOL  *Snp,
  IN        BOOLEAN                      IsIpv6,
  IN        EFI_IP_ADDRESS               *Ip,
      OUT   EFI_MAC_ADDRESS              *McastMac
  );

EFI_STATUS
EFIAPI
SnpNvData (
  IN EFI_SIMPLE_NETWORK_PROTOCOL *Snp,
  IN BOOLEAN                     ReadWrite,
  IN UINTN                       Offset,
  IN UINTN                       BufferSize,
  IN OUT VOID                    *Buffer
  );

EFI_STATUS
EFIAPI
SnpGetStatus (
  IN  EFI_SIMPLE_NETWORK_PROTOCOL *Snp,
  OUT UINT32                      *IrqStat  OPTIONAL,
  OUT VOID                        **TxBuff  OPTIONAL
  );

EFI_STATUS
EFIAPI
SnpTransmit (
  IN  EFI_SIMPLE_NETWORK_PROTOCOL  *Snp,
  IN  UINTN                        HdrSize,
  IN  UINTN                        BuffSize,
  IN  VOID*                        Data,
  IN  EFI_MAC_ADDRESS              *SrcAddr  OPTIONAL,
  IN  EFI_MAC_ADDRESS              *DstAddr  OPTIONAL,
  IN  UINT16                       *Protocol OPTIONAL
  );

EFI_STATUS
EFIAPI
SnpReceive (
  IN        EFI_SIMPLE_NETWORK_PROTOCOL *Snp,
      OUT   UINTN                       *HdrSize      OPTIONAL,
  IN  OUT   UINTN                       *BuffSize,
      OUT   VOID                        *Data,
      OUT   EFI_MAC_ADDRESS             *SrcAddr      OPTIONAL,
      OUT   EFI_MAC_ADDRESS             *DstAddr      OPTIONAL,
      OUT   UINT16                      *Protocol     OPTIONAL
  );


/*---------------------------------------------------------------------------------------------------------------------

  UEFI-Compliant functions for EFI_COMPONENT_NAME2_PROTOCOL

  Refer to the Component Name Protocol section (10.5) in the UEFI 2.3.1 Specification for related definitions

---------------------------------------------------------------------------------------------------------------------*/

EFI_STATUS
EFIAPI
SnpGetDriverName (
  IN        EFI_COMPONENT_NAME2_PROTOCOL *Snp,
  IN        CHAR8 *Lang,
      OUT   CHAR16 **DriverName
  );

EFI_STATUS
EFIAPI
SnpGetControllerName (
  IN        EFI_COMPONENT_NAME2_PROTOCOL *Cnp,
  IN        EFI_HANDLE ControllerHandle,
  IN        EFI_HANDLE ChildHandle            OPTIONAL,
  IN        CHAR8 *Lang,
      OUT   CHAR16 **ControllerName
  );

#endif // __ALTERA_SNP_DXE_H__

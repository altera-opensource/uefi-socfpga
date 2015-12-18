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

  Copyright (c) 2008 - 2009, Apple Inc. All rights reserved.
  Copyright (c) 2011 - 2014, ARM Limited. All rights reserved.

  This program and the accompanying materials
  are licensed and made available under the terms and conditions of the BSD License
  which accompanies this distribution.  The full text of the license may be found at
  http://opensource.org/licenses/bsd-license.php

  THE PROGRAM IS DISTRIBUTED UNDER THE BSD 3 Clause LICENSE ON AN "AS IS" BASIS,
  WITHOUT WARRANTIES OR REPRESENTATIONS OF ANY KIND, EITHER EXPRESS OR IMPLIED.

**/

#ifndef __EMAC_DXE_UTIL_H__
#define __EMAC_DXE_UTIL_H__

#include <AlteraPlatform.h>
#include <libfdt.h>
#include <Library/BaseMemoryLib.h>
#include <Library/UefiLib.h>
#include <Library/DebugLib.h>
#include <Library/IoLib.h>
#include <Library/MemoryAllocationLib.h>
#include <Library/PcdLib.h>
#include <Library/PrintLib.h>
#include <Library/TimerLib.h>
#include <Library/NetLib.h>

// Protocols used by this driver
#include <Protocol/SimpleNetwork.h>
#include <Protocol/ComponentName2.h>
#include <Protocol/PxeBaseCode.h>
#include <Protocol/DevicePath.h>


// Most common CRC32 Polynomial for little endian machines
#define CRC_POLYNOMIAL         0xEDB88320
#define HASH_TABLE_REG(n)      0x500 + (0x4 * n)
#define RX_MAX_PACKET          1600

#define CONFIG_ETH_BUFSIZE     2048
#define CONFIG_TX_DESCR_NUM    10
#define CONFIG_RX_DESCR_NUM    10
#define TX_TOTAL_BUFSIZE       (CONFIG_ETH_BUFSIZE * CONFIG_TX_DESCR_NUM)
#define RX_TOTAL_BUFSIZE       (CONFIG_ETH_BUFSIZE * CONFIG_RX_DESCR_NUM)

// DMA status error bit
#define RX_DMA_WRITE_DATA_TRANSFER_ERROR       0x0
#define TX_DMA_READ_DATA_TRANSFER_ERROR        0x3
#define RX_DMA_DESCRIPTOR_WRITE_ACCESS_ERROR   0x4
#define TX_DMA_DESCRIPTOR_WRITE_ACCESS_ERROR   0x5
#define RX_DMA_DESCRIPTOR_READ_ACCESS_ERROR    0x6
#define TX_DMA_DESCRIPTOR_READ_ACCESS_ERROR    0x7

// tx descriptor
#define TDES0_OWN           BIT31
#define TDES0_TXINT         BIT30
#define TDES0_TXLAST        BIT29
#define TDES0_TXFIRST       BIT28
#define TDES0_TXCRCDIS      BIT27
#define TDES0_TXRINGEND     BIT21
#define TDES0_TXCHAIN       BIT20

#define TDES1_SIZE1MASK     (0x1FFF << 0)
#define TDES1_SIZE1SHFT     (0)
#define TDES1_SIZE2MASK     (0x1FFF << 16)
#define TDES1_SIZE2SHFT     (16)

// rx descriptor
#define RDES0_FL_MASK       0x3fff
#define RDES0_FL_SHIFT      16
#define RDES1_CHAINED       BIT14

#define RDES0_CE            BIT1
#define RDES0_DBE           BIT2
#define RDES0_RE            BIT3
#define RDES0_RWT           BIT4
#define RDES0_LC            BIT6
#define RDES0_GF            BIT7
#define RDES0_OE            BIT11
#define RDES0_LE            BIT12
#define RDES0_SAF           BIT13
#define RDES0_DE            BIT14
#define RDES0_ES            BIT15
#define RDES0_AFM           BIT30
#define RDES0_OWN           BIT31


// emac config phy interface setting
#define PHY_INTERFACE_MODE_GMII      0
#define PHY_INTERFACE_MODE_MII       1
#define PHY_INTERFACE_MODE_RGMII     2
#define PHY_INTERFACE_MODE_RMII      3

typedef struct {
  UINT32 Tdes0;
  UINT32 Tdes1;
  VOID   *Addr;
  VOID   *AddrNext;
} DESIGNWARE_HW_DESCRIPTOR;

typedef struct {
  DESIGNWARE_HW_DESCRIPTOR TxdescRing[CONFIG_TX_DESCR_NUM];
  DESIGNWARE_HW_DESCRIPTOR RxdescRing[CONFIG_RX_DESCR_NUM];
  CHAR8                    TxBuffer[TX_TOTAL_BUFSIZE];
  CHAR8                    RxBuffer[RX_TOTAL_BUFSIZE];
  UINT32                   TxCurrentDescriptorNum;
  UINT32                   TxNextDescriptorNum;
  UINT32                   RxCurrentDescriptorNum;
  UINT32                   RxNextDescriptorNum;
} EMAC_DRIVER;

VOID
EFIAPI
EmacChooseInterface (
  IN UINT32 MacInterface
  );

VOID
EFIAPI
EmacSetMacAddress (
  IN EFI_MAC_ADDRESS  *MacAddress
  );

VOID
EFIAPI
EmacReadMacAddress (
  OUT EFI_MAC_ADDRESS *MacAddress
  );

VOID
EFIAPI
AlteraHpsInitialization (
  VOID
  );

VOID
EFIAPI
EmacConfigPhyInterface (
  IN UINT32 PhyMode
  );

EFI_STATUS
EFIAPI
EmacDxeInitialization (
  IN EMAC_DRIVER *EmacDriver
  );

EFI_STATUS
EFIAPI
EmacDmaInit (
  IN EMAC_DRIVER *EmacDriver
  );

EFI_STATUS
EFIAPI
EmacSetupTxdesc (
  IN EMAC_DRIVER *EmacDriver
 );

EFI_STATUS
EFIAPI
EmacSetupRxdesc (
  IN EMAC_DRIVER *EmacDriver
  );

VOID
EFIAPI
EmacStartTransmission (
  VOID
  );

EFI_STATUS
EFIAPI
EmacRxFilters (
  IN        UINT32           ReceiveFilterSetting,
  IN        BOOLEAN          Reset,
  IN        UINTN            NumMfilter             OPTIONAL,
  IN        EFI_MAC_ADDRESS *Mfilter                OPTIONAL
  );

UINT32
EFIAPI
GenEtherCrc32 (
  IN    EFI_MAC_ADDRESS *Mac,
  IN    UINT32 AddrLen
  );

UINT8
EFIAPI
BitReverse (
  UINT8 X
  );

VOID
EFIAPI
EmacStopTxRx (
  VOID
  );

EFI_STATUS
EFIAPI
EmacDmaStart (
  VOID
  );

VOID
EFIAPI
EmacGetDmaStatus (
  OUT   UINT32 *IrqStat  OPTIONAL
  );

VOID
EFIAPI
EmacGetStatistic (
  IN EFI_NETWORK_STATISTICS* Stats
  );

VOID
EFIAPI
EmacConfigAdjust (
  IN UINT32 Speed,
  IN UINT32 Duplex
  );

#endif // __EMAC_DXE_UTIL_H__

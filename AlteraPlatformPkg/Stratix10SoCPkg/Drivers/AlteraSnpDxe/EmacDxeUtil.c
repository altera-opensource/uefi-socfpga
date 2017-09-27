/** @file
  Portions of the code modified by Altera to support SoC devices are licensed as follows:
  Copyright (c) 2016, Intel Corporation. All rights reserved.

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

  Copyright (c) 2012 - 2014, ARM Limited. All rights reserved.
  Copyright (c) 2004 - 2010, Intel Corporation. All rights reserved.

  This program and the accompanying materials
  are licensed and made available under the terms and conditions of the BSD License
  which accompanies this distribution.  The full text of the license may be found at
  http://opensource.org/licenses/bsd-license.php

  THE PROGRAM IS DISTRIBUTED UNDER THE BSD 3 Clause LICENSE ON AN "AS IS" BASIS,
  WITHOUT WARRANTIES OR REPRESENTATIONS OF ANY KIND, EITHER EXPRESS OR IMPLIED.

**/

#include "EmacDxeUtil.h"
#include "PhyDxeUtil.h"

#include <Library/SerialPortPrintLib.h>
#if (FixedPcdGet32(PcdDebugMsg_EmacSnpDxe) == 0)
  #define ProgressPrint                       SerialPortPrint
  #define InfoPrint(FormatString, ...)        /* do nothing */
#else
  #define ProgressPrint SerialPortPrint
  #define InfoPrint     SerialPortPrint
#endif

//
// The HPS provides 3 EMAC
// Lookup table for EMAC selected by PcdDefaultMacInterface
//
STATIC CONST UINT32 mLookUpTable_ALT_EMAC_OFST[] = {
  ALT_EMAC0_OFST,
  ALT_EMAC1_OFST,
  ALT_EMAC2_OFST
};

UINT32 mALT_EMAC_OFST;


VOID
EFIAPI
EmacChooseInterface (
  IN UINT32 MacInterface
  )
{
  InfoPrint ("SNP:MAC: %a (%d)\r\n", __FUNCTION__, MacInterface);

  mALT_EMAC_OFST                              = mLookUpTable_ALT_EMAC_OFST[MacInterface];
}


VOID
EFIAPI
EmacSetMacAddress (
  IN EFI_MAC_ADDRESS  *MacAddress
  )
{
  InfoPrint ("SNP:MAC: %a ()\r\n", __FUNCTION__);

  // Note: This MAC_ADDR0 registers programming sequence cannot be swap:
  // Must program HIGH Offset first before LOW Offset
  // because synchronization is triggered when MAC Address0 Low Register are written.
  MmioWrite32 (mALT_EMAC_OFST +ALT_EMAC_GMACGRP_MAC_ADDRESS0_HIGH_OFST,
               (UINT32)(MacAddress->Addr[4] & 0xFF) |
                      ((MacAddress->Addr[5] & 0xFF) << 8)
                    );
  // MacAddress->Addr[0,1,2] is the 3 bytes OUI
  MmioWrite32 (mALT_EMAC_OFST + ALT_EMAC_GMACGRP_MAC_ADDRESS0_LOW_OFST,
                       (MacAddress->Addr[0] & 0xFF) |
                      ((MacAddress->Addr[1] & 0xFF) << 8) |
                      ((MacAddress->Addr[2] & 0xFF) << 16) |
                      ((MacAddress->Addr[3] & 0xFF) << 24)
                    );

  InfoPrint ("SNP:MAC: gmacgrp_mac_address0_low  = 0x%08X \r\n", MmioRead32 (mALT_EMAC_OFST + ALT_EMAC_GMACGRP_MAC_ADDRESS0_LOW_OFST));
  InfoPrint ("SNP:MAC: gmacgrp_mac_address0_high = 0x%08X \r\n", MmioRead32 (mALT_EMAC_OFST +ALT_EMAC_GMACGRP_MAC_ADDRESS0_HIGH_OFST));
}


VOID
EFIAPI
EmacReadMacAddress (
  OUT EFI_MAC_ADDRESS *MacAddress
  )
{
  UINT32          MacAddrHighValue;
  UINT32          MacAddrLowValue;

  InfoPrint ("SNP:MAC: %a ()\r\n", __FUNCTION__);

  // Read the Mac Addr high register
  MacAddrHighValue = (MmioRead32 (mALT_EMAC_OFST +ALT_EMAC_GMACGRP_MAC_ADDRESS0_HIGH_OFST) & 0xFFFF);
  // Read the Mac Addr low register
  MacAddrLowValue = MmioRead32 (mALT_EMAC_OFST + ALT_EMAC_GMACGRP_MAC_ADDRESS0_LOW_OFST);

  SetMem (MacAddress, sizeof(*MacAddress), 0);
  MacAddress->Addr[0] = (MacAddrLowValue & 0xFF);
  MacAddress->Addr[1] = (MacAddrLowValue & 0xFF00) >> 8;
  MacAddress->Addr[2] = (MacAddrLowValue & 0xFF0000) >> 16;
  MacAddress->Addr[3] = (MacAddrLowValue & 0xFF000000) >> 24;
  MacAddress->Addr[4] = (MacAddrHighValue & 0xFF);
  MacAddress->Addr[5] = (MacAddrHighValue & 0xFF00) >> 8;

  InfoPrint ("SNP:MAC: MAC Address = %02X:%02X:%02X:%02X:%02X:%02X\r\n",
    MacAddress->Addr[0], MacAddress->Addr[1], MacAddress->Addr[2],
    MacAddress->Addr[3], MacAddress->Addr[4], MacAddress->Addr[5]
  );

}

EFI_STATUS
EFIAPI
EmacDxeInitialization (
  IN EMAC_DRIVER* EmacDriver
  )
{
  InfoPrint ("SNP:MAC: %a ()\r\n", __FUNCTION__);

  // Init EMAC DMA
  EmacDmaInit (EmacDriver);

  return EFI_SUCCESS;
}


EFI_STATUS
EFIAPI
EmacDmaInit (
  IN   EMAC_DRIVER      *EmacDriver
  )
{
  UINT32 DmaConf;
  UINT32 DmaOpmode;
  UINT32 InterruptEnable;

  InfoPrint ("SNP:MAC: %a ()\r\n", __FUNCTION__);

  // This section provides the instructions for initializing the DMA registers in the proper sequence. This
  // initialization sequence can be done after the EMAC interface initialization has been completed. Perform
  // the following steps to initialize the DMA:
  // 1. Provide a software reset to reset all of the EMAC internal registers and logic. (DMA Register 0 (Bus
  // Mode Register) – bit 0).

  MmioOr32 (mALT_EMAC_OFST +
             ALT_EMAC_DMAGRP_BUS_MODE_OFST,
             ALT_EMAC_DMAGRP_BUS_MODE_SWR_SET_MSK);

  // 2. Wait for the completion of the reset process (poll bit 0 of the DMA Register 0 (Bus Mode Register),
  // which is only cleared after the reset operation is completed).
  while (ALT_EMAC_DMAGRP_BUS_MODE_SWR_GET (MmioRead32 (mALT_EMAC_OFST + ALT_EMAC_DMAGRP_BUS_MODE_OFST)));

  // 3. Poll the bits of Register 11 (AHB or AXI Status) to confirm that all previously initiated (before
  // software reset) or ongoing transactions are complete.
  // Note: If the application cannot poll the register after soft reset (because of performance reasons), then
  // it is recommended that you continue with the next steps and check this register again (as
  // mentioned in 12 on page 1-72) before triggering the DMA operations.†

  // 4. Program the following fields to initialize the Bus Mode Register by setting values in DMA Register 0
  // (Bus Mode Register):
  // • Mixed Burst and AAL
  // • Fixed burst or undefined burst
  // • Burst length values and burst mode values
  // • Descriptor Length (only valid if Ring Mode is used)
  // • TX and RX DMA Arbitration scheme
  // 5. Program the interface options in Register 10 (AXI Bus Mode Register). If fixed burst-length is enabled,
  // then select the maximum burst-length possible on the bus (bits[7:1]).†

  DmaConf = ALT_EMAC_DMAGRP_BUS_MODE_FB_SET_MSK | ALT_EMAC_DMAGRP_BUS_MODE_PBL_SET_MSK | ALT_EMAC_DMAGRP_BUS_MODE_PR_SET_MSK;
  MmioOr32 (mALT_EMAC_OFST +
            ALT_EMAC_DMAGRP_BUS_MODE_OFST,
            DmaConf);

  // 6. Create a proper descriptor chain for transmit and receive. In addition, ensure that the receive descriptors
  // are owned by DMA (bit 31 of descriptor should be set). When OSF mode is used, at least two
  // descriptors are required.
  // 7. Make sure that your software creates three or more different transmit or receive descriptors in the
  // chain before reusing any of the descriptors.
  // 8. Initialize receive and transmit descriptor list address with the base address of the transmit and receive
  // descriptor (Register 3 (Receive Descriptor List Address Register) and Register 4 (Transmit Descriptor
  // List Address Register) respectively).

  EmacSetupTxdesc (EmacDriver);
  EmacSetupRxdesc (EmacDriver);

  // 9. Program the following fields to initialize the mode of operation in Register 6 (Operation Mode
  // Register):
  // • Receive and Transmit Store And Forward†
  // • Receive and Transmit Threshold Control (RTC and TTC)†
  // • Hardware Flow Control enable†
  // • Flow Control Activation and De-activation thresholds for MTL Receive and Transmit FIFO buffers
  // (RFA and RFD)†
  // • Error frame and undersized good frame forwarding enable†
  // • OSF Mode†

  DmaOpmode = ALT_EMAC_DMAGRP_OPERATION_MODE_FTF_SET_MSK | ALT_EMAC_DMAGRP_OPERATION_MODE_TSF_SET_MSK;
  MmioOr32 (mALT_EMAC_OFST +
            ALT_EMAC_DMAGRP_OPERATION_MODE_OFST,
            DmaOpmode);
  //while (ALT_EMAC_DMAGRP_OPERATION_MODE_FTF_GET (MmioRead32 (mALT_EMAC_OFST + ALT_EMAC_DMAGRP_OPERATION_MODE_OFST)));
 
// 10.Clear the interrupt requests, by writing to those bits of the status register (interrupt bits only) that are
  // set. For example, by writing 1 into bit 16, the normal interrupt summary clears this bit (DMA Register 5 (Status Register)).
  MmioOr32 (mALT_EMAC_OFST +
            ALT_EMAC_DMAGRP_STATUS_OFST,
            0x1FFFF);

  // 11.Enable the interrupts by programming Register 7 (Interrupt Enable Register).
  InterruptEnable = ALT_EMAC_DMAGRP_INTERRUPT_ENABLE_TIE_SET_MSK |
                    ALT_EMAC_DMAGRP_INTERRUPT_ENABLE_RIE_SET_MSK |
                    ALT_EMAC_DMAGRP_INTERRUPT_ENABLE_NIE_SET_MSK |
                    ALT_EMAC_DMAGRP_INTERRUPT_ENABLE_AIE_SET_MSK |
                    ALT_EMAC_DMAGRP_INTERRUPT_ENABLE_FBE_SET_MSK |
                    ALT_EMAC_DMAGRP_INTERRUPT_ENABLE_UNE_SET_MSK |
                    ALT_EMAC_DMAGRP_INTERRUPT_ENABLE_TSE_SET_MSK |
                    ALT_EMAC_DMAGRP_INTERRUPT_ENABLE_TUE_SET_MSK |
                    ALT_EMAC_DMAGRP_INTERRUPT_ENABLE_TJE_SET_MSK |
                    ALT_EMAC_DMAGRP_INTERRUPT_ENABLE_OVE_SET_MSK |
                    ALT_EMAC_DMAGRP_INTERRUPT_ENABLE_RUE_SET_MSK |
                    ALT_EMAC_DMAGRP_INTERRUPT_ENABLE_RSE_SET_MSK |
                    ALT_EMAC_DMAGRP_INTERRUPT_ENABLE_RWE_SET_MSK |
                    ALT_EMAC_DMAGRP_INTERRUPT_ENABLE_ETE_SET_MSK |
                    ALT_EMAC_DMAGRP_INTERRUPT_ENABLE_ERE_SET_MSK;
   MmioWrite32 (mALT_EMAC_OFST +
                ALT_EMAC_DMAGRP_INTERRUPT_ENABLE_OFST,
                InterruptEnable);

  // 12.Read Register 11 (AHB or AXI Status) to confirm that all previous transactions are complete.†
  // Note: If any previous transaction is still in progress when you read the Register 11 (AHB or AXI
  // Status), then it is strongly recommended to check the slave components addressed by the
  // master interface.
  if (MmioRead32 (mALT_EMAC_OFST + ALT_EMAC_DMAGRP_AHB_OR_AXI_STATUS_OFST) != 0) {
    InfoPrint("SNP:MAC: Error! Previous AXI transaction is still in progress\r\n");
    //check the slave components addressed by the master interface
    return EFI_DEVICE_ERROR;
  }

  // 13.Start the receive and transmit DMA by setting SR (bit 1) and ST (bit 13) of the control register (DMA
  // Register 6 (Operation Mode Register).
  DmaOpmode = ALT_EMAC_DMAGRP_OPERATION_MODE_ST_SET_MSK | ALT_EMAC_DMAGRP_OPERATION_MODE_SR_SET_MSK;
  MmioOr32 (mALT_EMAC_OFST +
            ALT_EMAC_DMAGRP_OPERATION_MODE_OFST,
            DmaOpmode);

  return EFI_SUCCESS;
}


EFI_STATUS
EFIAPI
EmacSetupTxdesc (
  IN EMAC_DRIVER *EmacDriver
 )
{
  INTN Index;
  DESIGNWARE_HW_DESCRIPTOR *TxDescriptor;

  //InfoPrint ("SNP:MAC: %a ()\r\n", __FUNCTION__);

  for (Index = 0; Index < CONFIG_TX_DESCR_NUM; Index++) {
    TxDescriptor = &EmacDriver->TxdescRing[Index];
    TxDescriptor->Addr = (UINT32)(UINTN) &EmacDriver->TxBuffer[Index * CONFIG_ETH_BUFSIZE];
    TxDescriptor->AddrNext = (UINT32)(UINTN) &EmacDriver->TxdescRing[Index + 1];
    TxDescriptor->Tdes0 = TDES0_TXCHAIN;
    TxDescriptor->Tdes1 = 0;
  }

  // Correcting the last pointer of the chain
  TxDescriptor->AddrNext = (UINT32)(UINTN) &EmacDriver->TxdescRing[0];

  // Write the address of tx descriptor list
  MmioWrite32(mALT_EMAC_OFST +
              ALT_EMAC_DMAGRP_TRANSMIT_DESCRIPTOR_LIST_ADDRESS_OFST,
              (UINT32)(UINTN) &EmacDriver->TxdescRing[0]);

  // Initialize the descriptor number
  EmacDriver->TxCurrentDescriptorNum = 0;
  EmacDriver->TxNextDescriptorNum = 0;
  return EFI_SUCCESS;
}


EFI_STATUS
EFIAPI
EmacSetupRxdesc (
  IN EMAC_DRIVER *EmacDriver
  )
{
  INTN                      Index;
  DESIGNWARE_HW_DESCRIPTOR *RxDescriptor;

  //InfoPrint ("SNP:MAC: %a ()\r\n", __FUNCTION__);

  for (Index = 0; Index < CONFIG_RX_DESCR_NUM; Index++) {
    RxDescriptor = &EmacDriver->RxdescRing[Index];
    RxDescriptor->Addr = (UINT32)(UINTN) &EmacDriver->RxBuffer[Index * CONFIG_ETH_BUFSIZE];
    RxDescriptor->AddrNext = (UINT32)(UINTN) &EmacDriver->RxdescRing[Index + 1];
    RxDescriptor->Tdes0 = RDES0_OWN;
    RxDescriptor->Tdes1 = RDES1_CHAINED | RX_MAX_PACKET;
  }

  // Correcting the last pointer of the chain
  RxDescriptor->AddrNext = (UINT32)(UINTN) &EmacDriver->RxdescRing[0];

  // Write the address of tx descriptor list
  MmioWrite32(mALT_EMAC_OFST +
              ALT_EMAC_DMAGRP_RECEIVE_DESCRIPTOR_LIST_ADDRESS_OFST,
              (UINT32)(UINTN) &EmacDriver->RxdescRing[0]);

  // Initialize the descriptor number
  EmacDriver->RxCurrentDescriptorNum = 0;
  EmacDriver->RxNextDescriptorNum = 0;
  return EFI_SUCCESS;
}


VOID
EFIAPI
EmacStartTransmission (
  VOID
  )
{
  InfoPrint ("SNP:MAC: %a ()\r\n", __FUNCTION__);

  // Enable the receiver and transmitter
  MmioOr32 (mALT_EMAC_OFST +
            ALT_EMAC_GMACGRP_MAC_CONFIGURATION_OFST,
            ALT_EMAC_GMACGRP_MAC_CONFIGURATION_RE_SET_MSK |
            ALT_EMAC_GMACGRP_MAC_CONFIGURATION_TE_SET_MSK
            );
}


EFI_STATUS
EFIAPI
EmacRxFilters (
  IN        UINT32 ReceiveFilterSetting,
  IN        BOOLEAN Reset,
  IN        UINTN NumMfilter          OPTIONAL,
  IN        EFI_MAC_ADDRESS *Mfilter  OPTIONAL
  )
 {
  UINT32 MacFilter;
  UINT32 Crc;
  UINT32 Count;
  UINT32 HashReg;
  UINT32 HashBit;
  UINT32 Reg;
  UINT32 Val;

  //InfoPrint ("SNP:MAC: %a ()\r\n", __FUNCTION__);

  // If reset then clear the filter registers
  if (Reset) {
    for (Count = 0; Count < NumMfilter; Count++)
    {
      MmioWrite32 (mALT_EMAC_OFST + HASH_TABLE_REG(Count), 0x00000000);
    }
  }

  // Set MacFilter to the reset value of the  ALT_EMAC_GMACGRP_MAC_FRAME_FILTER register.
  MacFilter =  ALT_EMAC_GMACGRP_MAC_FRAME_FILTER_RESET;

  if (ReceiveFilterSetting & EFI_SIMPLE_NETWORK_RECEIVE_UNICAST) {
    //InfoPrint("SNP:MAC: Enable Unicast Frame Reception\n");
  }

  if (ReceiveFilterSetting & EFI_SIMPLE_NETWORK_RECEIVE_MULTICAST) {

    //InfoPrint("SNP:MAC: Enable Hash Multicast Frame Reception\n");
    MacFilter |=  ALT_EMAC_GMACGRP_MAC_FRAME_FILTER_HMC_SET_MSK;

    // Set the hash tables
    if ((NumMfilter > 0) && (!Reset)) {
      // Go through each filter address and set appropriate bits on hash table
      for (Count = 0; Count < NumMfilter; Count++) {
        // Generate a 32-bit CRC
        Crc = GenEtherCrc32 (&Mfilter[Count], 6);
        // reserve CRC + take upper 8 bit = take lower 8 bit and reverse it
        Val = BitReverse(Crc & 0xff);
        // The most significant bits determines the register to be used (Hash Table Register X),
        // and the least significant five bits determine the bit within the register.
        // For example, a hash value of 8b'10111111 selects Bit 31 of the Hash Table Register 5.
        HashReg = (Val >> 5);
        HashBit = (Val & 31);

        Reg = MmioRead32(mALT_EMAC_OFST + HASH_TABLE_REG(HashReg));
        // set 1 to HashBit of HashReg
        // for example, set 1 to bit 31 to Reg 5 as in above example
        Reg |= (1 << HashBit);
        MmioWrite32(mALT_EMAC_OFST + HASH_TABLE_REG(HashReg), Reg);
      }
    }
  }

  if ((ReceiveFilterSetting & EFI_SIMPLE_NETWORK_RECEIVE_BROADCAST) == 0) {
    MacFilter |=  ALT_EMAC_GMACGRP_MAC_FRAME_FILTER_DBF_SET_MSK;
  } else if (ReceiveFilterSetting & EFI_SIMPLE_NETWORK_RECEIVE_BROADCAST) {
    //InfoPrint("SNP:MAC: Enable Broadcast Frame Reception\n");
  }

  if (ReceiveFilterSetting & EFI_SIMPLE_NETWORK_RECEIVE_PROMISCUOUS) {
    MacFilter |=  ALT_EMAC_GMACGRP_MAC_FRAME_FILTER_PR_SET_MSK;
    //InfoPrint("SNP:MAC: Enable Promiscuous Mode\n");
  }

  if (ReceiveFilterSetting & EFI_SIMPLE_NETWORK_RECEIVE_PROMISCUOUS_MULTICAST) {
    MacFilter |= ( ALT_EMAC_GMACGRP_MAC_FRAME_FILTER_PM_SET_MSK);
    //InfoPrint("SNP:MAC: Enable All Multicast Mode\n");
  }

  // Set MacFilter to EMAC register
  MmioWrite32 (mALT_EMAC_OFST +  ALT_EMAC_GMACGRP_MAC_FRAME_FILTER_OFST, MacFilter);
  return EFI_SUCCESS;
}


UINT32
EFIAPI
GenEtherCrc32 (
  IN    EFI_MAC_ADDRESS *Mac,
  IN    UINT32 AddrLen
  )
{
  INT32  Iter;
  UINT32 Remainder;
  UINT8  *Ptr;

  Iter = 0;
  Remainder = 0xFFFFFFFF;    // 0xFFFFFFFF is standard seed for Ethernet

  // Convert Mac Address to array of bytes
  Ptr = (UINT8*)Mac;

  // Generate the Crc bit-by-bit (LSB first)
  while (AddrLen--) {
    Remainder ^= *Ptr++;
    for (Iter = 0;Iter < 8;Iter++) {
      // Check if exponent is set
      if (Remainder & 1) {
        Remainder = (Remainder >> 1) ^ CRC_POLYNOMIAL;
      } else {
        Remainder = (Remainder >> 1) ^ 0;
      }
    }
  }

  return (~Remainder);
}


STATIC CONST UINT8 NibbleTab[] = {
    /* 0x0 0000 -> 0000 */  0x0,
    /* 0x1 0001 -> 1000 */  0x8,
    /* 0x2 0010 -> 0100 */  0x4,
    /* 0x3 0011 -> 1100 */  0xc,
    /* 0x4 0100 -> 0010 */  0x2,
    /* 0x5 0101 -> 1010 */  0xa,
    /* 0x6 0110 -> 0110 */  0x6,
    /* 0x7 0111 -> 1110 */  0xe,
    /* 0x8 1000 -> 0001 */  0x1,
    /* 0x9 1001 -> 1001 */  0x9,
    /* 0xa 1010 -> 0101 */  0x5,
    /* 0xb 1011 -> 1101 */  0xd,
    /* 0xc 1100 -> 0011 */  0x3,
    /* 0xd 1101 -> 1011 */  0xb,
    /* 0xe 1110 -> 0111 */  0x7,
    /* 0xf 1111 -> 1111 */  0xf
};

UINT8
EFIAPI
BitReverse (
  UINT8 X
  )
{
  return (NibbleTab[X & 0xf] << 4) | NibbleTab[X >> 4];
}


VOID
EFIAPI
EmacStopTxRx (
  VOID
  )
{
  InfoPrint ("SNP:MAC: %a ()\r\n", __FUNCTION__);

  // Stop DMA TX
  MmioAnd32 (mALT_EMAC_OFST +
            ALT_EMAC_DMAGRP_OPERATION_MODE_OFST,
            ALT_EMAC_DMAGRP_OPERATION_MODE_ST_CLR_MSK);

  // Flush TX
  MmioOr32 (mALT_EMAC_OFST +
            ALT_EMAC_DMAGRP_OPERATION_MODE_OFST,
            ALT_EMAC_DMAGRP_OPERATION_MODE_FTF_SET_MSK);

  // Stop transmitters
  MmioAnd32 (mALT_EMAC_OFST +
            ALT_EMAC_GMACGRP_MAC_CONFIGURATION_OFST,
            ALT_EMAC_GMACGRP_MAC_CONFIGURATION_RE_CLR_MSK &
            ALT_EMAC_GMACGRP_MAC_CONFIGURATION_TE_CLR_MSK);

  // Stop DMA RX
  MmioAnd32 (mALT_EMAC_OFST +
            ALT_EMAC_DMAGRP_OPERATION_MODE_OFST,
            ALT_EMAC_DMAGRP_OPERATION_MODE_SR_CLR_MSK);

}


EFI_STATUS
EFIAPI
EmacDmaStart (
  VOID
  )
{
  //InfoPrint ("SNP:MAC: %a ()\r\n", __FUNCTION__);

  // Start the transmission
  MmioWrite32(mALT_EMAC_OFST +
              ALT_EMAC_DMAGRP_TRANSMIT_POLL_DEMAND_OFST,
              0x1);
    return EFI_SUCCESS;
}


VOID
EFIAPI
EmacGetDmaStatus (
  OUT   UINT32 *IrqStat  OPTIONAL
  )
{
  UINT32  DmaStatus;
  UINT32  ErrorBit;
  UINT32  Mask = 0;

  //InfoPrint ("SNP:MAC: %a ()\r\n", __FUNCTION__);

  DmaStatus = MmioRead32 (mALT_EMAC_OFST +
                           ALT_EMAC_DMAGRP_STATUS_OFST);
  if (DmaStatus & ALT_EMAC_DMAGRP_STATUS_NIS_SET_MSK) {
    Mask |= ALT_EMAC_DMAGRP_STATUS_NIS_SET_MSK;
    // Rx interrupt
    if (DmaStatus & ALT_EMAC_DMAGRP_STATUS_RI_SET_MSK) {
      *IrqStat |= EFI_SIMPLE_NETWORK_RECEIVE_INTERRUPT;
      Mask |= ALT_EMAC_DMAGRP_STATUS_RI_SET_MSK;
    } else {
      *IrqStat &= ~EFI_SIMPLE_NETWORK_RECEIVE_INTERRUPT;
    }
    // Tx interrupt
    if (DmaStatus & ALT_EMAC_DMAGRP_STATUS_TI_SET_MSK) {
      *IrqStat |= EFI_SIMPLE_NETWORK_TRANSMIT_INTERRUPT;
      Mask |= ALT_EMAC_DMAGRP_STATUS_TI_SET_MSK;
    } else {
      *IrqStat &= ~EFI_SIMPLE_NETWORK_TRANSMIT_INTERRUPT;
    }
    // Tx Buffer
    if (DmaStatus & ALT_EMAC_DMAGRP_STATUS_TU_SET_MSK){
      Mask |= ALT_EMAC_DMAGRP_STATUS_TU_SET_MSK;
      }
    // Early receive interrupt
    if (DmaStatus & ALT_EMAC_DMAGRP_STATUS_ERI_SET_MSK) {
      Mask |= ALT_EMAC_DMAGRP_STATUS_ERI_SET_MSK;
    }
  }
  if (DmaStatus & ALT_EMAC_DMAGRP_STATUS_AIS_SET_MSK) {
    Mask |= ALT_EMAC_DMAGRP_STATUS_AIS_SET_MSK;
    // Transmit process stop
    if (DmaStatus & ALT_EMAC_DMAGRP_STATUS_TPS_SET_MSK) {
      InfoPrint("SNP:MAC: Transmit process stop\n");
      Mask |= ALT_EMAC_DMAGRP_STATUS_TPS_SET_MSK;
    }
    // Transmit jabber timeout
    if (DmaStatus & ALT_EMAC_DMAGRP_STATUS_TJT_SET_MSK) {
      InfoPrint("SNP:MAC: Transmit jabber timeout\n");
      Mask |= ALT_EMAC_DMAGRP_STATUS_TJT_SET_MSK;
    }
    // Receive FIFO overflow
    if (DmaStatus & ALT_EMAC_DMAGRP_STATUS_OVF_SET_MSK) {
      InfoPrint("SNP:MAC: Receive FIFO overflow\n");
      Mask |= ALT_EMAC_DMAGRP_STATUS_OVF_SET_MSK;
    }
    // Transmit FIFO underflow
    if (DmaStatus & ALT_EMAC_DMAGRP_STATUS_UNF_SET_MSK) {
      InfoPrint("SNP:MAC: Receive FIFO underflow\n");
      Mask |= ALT_EMAC_DMAGRP_STATUS_UNF_SET_MSK;
    }
    // Receive buffer unavailable
    if (DmaStatus & ALT_EMAC_DMAGRP_STATUS_RU_SET_MSK) {
      //InfoPrint("SNP:MAC: Receive buffer unavailable\n");
      Mask |= ALT_EMAC_DMAGRP_STATUS_RU_SET_MSK;
    }

    // Receive process stop
    if (DmaStatus & ALT_EMAC_DMAGRP_STATUS_RPS_SET_MSK) {
      InfoPrint("SNP:MAC: Receive process stop\n");
      Mask |= ALT_EMAC_DMAGRP_STATUS_RPS_SET_MSK;
    }
    // Receive watchdog timeout
    if (DmaStatus & ALT_EMAC_DMAGRP_STATUS_RWT_SET_MSK) {
      InfoPrint("SNP:MAC: Receive watchdog timeout\n");
      Mask |= ALT_EMAC_DMAGRP_STATUS_RWT_SET_MSK;
    }
    // Early transmit interrupt
    if (DmaStatus & ALT_EMAC_DMAGRP_STATUS_ETI_SET_MSK) {
      //InfoPrint("SNP:MAC: Early transmit interrupt\n");
      Mask |= ALT_EMAC_DMAGRP_STATUS_ETI_SET_MSK;
    }
    // Fatal bus error
    if (DmaStatus & ALT_EMAC_DMAGRP_STATUS_FBI_SET_MSK) {
      InfoPrint("SNP:MAC: Fatal bus error:\n");
      Mask |= ALT_EMAC_DMAGRP_STATUS_FBI_SET_MSK;

      ErrorBit = ALT_EMAC_DMAGRP_STATUS_EB_GET (DmaStatus);
      switch (ErrorBit) {
        case RX_DMA_WRITE_DATA_TRANSFER_ERROR:
          InfoPrint("SNP:MAC: Rx Dma write data transfer error\n");
          break;
        case TX_DMA_READ_DATA_TRANSFER_ERROR:
          InfoPrint("SNP:MAC: Tx Dma read data transfer error\n");
          break;
        case RX_DMA_DESCRIPTOR_WRITE_ACCESS_ERROR:
          InfoPrint("SNP:MAC: Rx Dma descriptor write access error\n");
          break;
        case RX_DMA_DESCRIPTOR_READ_ACCESS_ERROR:
          InfoPrint("SNP:MAC: Rx Dma descriptor read access error\n");
          break;
        case TX_DMA_DESCRIPTOR_WRITE_ACCESS_ERROR:
          InfoPrint("SNP:MAC: Tx Dma descriptor write access error\n");
          break;
        case TX_DMA_DESCRIPTOR_READ_ACCESS_ERROR:
          InfoPrint("SNP:MAC: Tx Dma descriptor read access error\n");
          break;
        default:
          InfoPrint("SNP:MAC: Undefined error\n");
          break;
      }
    }
  }
  MmioOr32 (mALT_EMAC_OFST +
            ALT_EMAC_DMAGRP_STATUS_OFST,
            Mask);
}


VOID
EFIAPI
EmacGetStatistic (
  OUT EFI_NETWORK_STATISTICS* Statistic
  )
{
  EFI_NETWORK_STATISTICS* Stats;

  InfoPrint ("SNP:MAC: %a ()\r\n", __FUNCTION__);

  // Allocate Resources
  Stats = AllocateZeroPool (sizeof (EFI_NETWORK_STATISTICS));
  if (Stats == NULL) {
    return;
  }

  Stats->RxTotalFrames     = MmioRead32 (mALT_EMAC_OFST + ALT_EMAC_GMACGRP_RXFRAMECOUNT_GB_OFST);
  Stats->RxUndersizeFrames = MmioRead32 (mALT_EMAC_OFST + ALT_EMAC_GMACGRP_RXUNDERSIZE_G_OFST);
  Stats->RxOversizeFrames  = MmioRead32 (mALT_EMAC_OFST + ALT_EMAC_GMACGRP_RXOVERSIZE_G_OFST);
  Stats->RxUnicastFrames   = MmioRead32 (mALT_EMAC_OFST + ALT_EMAC_GMACGRP_RXUNICASTFRAMES_G_OFST);
  Stats->RxBroadcastFrames = MmioRead32 (mALT_EMAC_OFST + ALT_EMAC_GMACGRP_RXBROADCASTFRAMES_G_OFST);
  Stats->RxMulticastFrames = MmioRead32 (mALT_EMAC_OFST + ALT_EMAC_GMACGRP_RXMULTICASTFRAMES_G_OFST);
  Stats->RxCrcErrorFrames  = MmioRead32 (mALT_EMAC_OFST + ALT_EMAC_GMACGRP_RXCRCERROR_OFST);
  Stats->RxTotalBytes      = MmioRead32 (mALT_EMAC_OFST + ALT_EMAC_GMACGRP_RXOCTETCOUNT_GB_OFST);
  Stats->RxGoodFrames      = Stats->RxUnicastFrames + Stats->RxBroadcastFrames + Stats->RxMulticastFrames;

  Stats->TxTotalFrames     = MmioRead32 (mALT_EMAC_OFST + ALT_EMAC_GMACGRP_TXFRAMECOUNT_GB_OFST);
  Stats->TxGoodFrames      = MmioRead32 (mALT_EMAC_OFST + ALT_EMAC_GMACGRP_TXFRAMECOUNT_G_OFST);
  Stats->TxOversizeFrames  = MmioRead32 (mALT_EMAC_OFST + ALT_EMAC_GMACGRP_TXOVERSIZE_G_OFST);
  Stats->TxUnicastFrames   = MmioRead32 (mALT_EMAC_OFST + ALT_EMAC_GMACGRP_TXUNICASTFRAMES_GB_OFST);
  Stats->TxBroadcastFrames = MmioRead32 (mALT_EMAC_OFST + ALT_EMAC_GMACGRP_TXBROADCASTFRAMES_G_OFST);
  Stats->TxMulticastFrames = MmioRead32 (mALT_EMAC_OFST + ALT_EMAC_GMACGRP_TXMULTICASTFRAMES_G_OFST);
  Stats->TxTotalBytes      = MmioRead32 (mALT_EMAC_OFST + ALT_EMAC_GMACGRP_TXOCTETCOUNT_GB_OFST);
  Stats->Collisions        = MmioRead32 (mALT_EMAC_OFST + ALT_EMAC_GMACGRP_TXLATECOL_OFST) +
                             MmioRead32 (mALT_EMAC_OFST + ALT_EMAC_GMACGRP_TXEXESSCOL_OFST);

  // Fill in the statistics
  CopyMem(Statistic, Stats, sizeof(EFI_NETWORK_STATISTICS));
}


VOID
EFIAPI
EmacConfigAdjust (
  IN UINT32 Speed,
  IN UINT32 Duplex
  )
{
  UINT32 Config;

  //InfoPrint ("SNP:MAC: %a ()\r\n", __FUNCTION__);

  Config =0;
  if (Speed != SPEED_1000)
   Config |= ALT_EMAC_GMACGRP_MAC_CONFIGURATION_PS_SET_MSK;

  if (Speed == SPEED_100)
    Config |= ALT_EMAC_GMACGRP_MAC_CONFIGURATION_FES_SET_MSK;

  if (Duplex == DUPLEX_FULL)
    Config |= ALT_EMAC_GMACGRP_MAC_CONFIGURATION_DM_SET_MSK;

  MmioOr32 (mALT_EMAC_OFST +
            ALT_EMAC_GMACGRP_MAC_CONFIGURATION_OFST,
            ALT_EMAC_GMACGRP_MAC_CONFIGURATION_BE_SET_MSK |
            ALT_EMAC_GMACGRP_MAC_CONFIGURATION_DO_SET_MSK |
            Config);

}

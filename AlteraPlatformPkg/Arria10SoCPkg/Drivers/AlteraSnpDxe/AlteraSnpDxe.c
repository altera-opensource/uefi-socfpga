/** @file
  Altera SNP DXE driver

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

  Copyright (c) 2012 - 2014, ARM Limited. All rights reserved.
  Copyright (c) 2004 - 2010, Intel Corporation. All rights reserved.

  This program and the accompanying materials
  are licensed and made available under the terms and conditions of the BSD License
  which accompanies this distribution.  The full text of the license may be found at
  http://opensource.org/licenses/bsd-license.php

  THE PROGRAM IS DISTRIBUTED UNDER THE BSD 3 Clause LICENSE ON AN "AS IS" BASIS,
  WITHOUT WARRANTIES OR REPRESENTATIONS OF ANY KIND, EITHER EXPRESS OR IMPLIED.

**/

#include "AlteraSnpDxe.h"
#include "EmacDxeUtil.h"
#include "PhyDxeUtil.h"
#include <Library/UncachedMemoryAllocationLib.h>

#include <Library/SerialPortPrintLib.h>
#if (FixedPcdGet32(PcdDebugMsg_EmacSnpDxe) == 0)
  #define ProgressPrint                       SerialPortPrint
  #define InfoPrint(FormatString, ...)        /* do nothing */
#else
  #define ProgressPrint SerialPortPrint
  #define InfoPrint     SerialPortPrint
#endif

typedef struct {
  MAC_ADDR_DEVICE_PATH      MacAddrDP;
  EFI_DEVICE_PATH_PROTOCOL  End;
} EFI_SIMPLE_NETWORK_DEVICE_PATH;

EFI_SIMPLE_NETWORK_DEVICE_PATH PathTemplate =  {
  {
    {
      MESSAGING_DEVICE_PATH, MSG_MAC_ADDR_DP,
      { (UINT8) (sizeof(MAC_ADDR_DEVICE_PATH)), (UINT8) ((sizeof(MAC_ADDR_DEVICE_PATH)) >> 8) }
    },
    { { 0 } },
    0
  },
  {
    END_DEVICE_PATH_TYPE,
    END_ENTIRE_DEVICE_PATH_SUBTYPE,
    { sizeof(EFI_DEVICE_PATH_PROTOCOL), 0 }
  }
};


/**
  The SNP driver entry point.

  This is the declaration of an EFI image entry point. This entry point is
  the same for UEFI Applications, UEFI OS Loaders, and UEFI Drivers including
  both device drivers and bus drivers.

  @param  ImageHandle           The firmware allocated handle for the UEFI image.
  @param  SystemTable           A pointer to the EFI System Table.

  @retval EFI_SUCCESS           The operation completed successfully.
  @retval EFI_OUT_OF_RESOURCES  The request could not be completed due to a lack of resources.

**/
EFI_STATUS
AlteraSnpDxeEntry (
  IN EFI_HANDLE        ImageHandle,
  IN EFI_SYSTEM_TABLE  *SystemTable
  )
{
  EFI_STATUS                       Status;
  EFI_SIMPLE_NETWORK_DRIVER       *Driver;
  EFI_SIMPLE_NETWORK_PROTOCOL     *Snp;
  EFI_SIMPLE_NETWORK_MODE         *SnpMode;
  EFI_SIMPLE_NETWORK_DEVICE_PATH  *DevicePath;
  EFI_HANDLE                       ControllerHandle;
  UINT64                           DefaultMacAddress;
  EFI_MAC_ADDRESS                 *SwapMacAddressPtr;
  UINT32                           MacIf;

  InfoPrint ("SNP:DXE: %a ()\r\n", __FUNCTION__);

  Status = EFI_SUCCESS;
  // Allocate Resources
  Driver = UncachedAllocatePages (EFI_SIZE_TO_PAGES (sizeof (EFI_SIMPLE_NETWORK_DRIVER)));
  if (Driver == NULL) {
    return EFI_OUT_OF_RESOURCES;
  }
  // Initialized signature (used by INSTANCE_FROM_SNP_THIS macro)
  Driver->Signature = SNP_DRIVER_SIGNATURE;

  DevicePath = (EFI_SIMPLE_NETWORK_DEVICE_PATH*)AllocateCopyPool (sizeof (EFI_SIMPLE_NETWORK_DEVICE_PATH), &PathTemplate);
  if (DevicePath == NULL) {
    return EFI_OUT_OF_RESOURCES;
  }

  // Select the Mac Interface based on PCD (Emac0, Emac1 or Emac2)
  MacIf = FixedPcdGet32 (PcdDefaultMacInterface);
  ASSERT(MacIf < 3);
  EmacChooseInterface (MacIf);

  // Initialize pointers
  Snp = &(Driver->Snp);
  SnpMode = &(Driver->SnpMode);
  Snp->Mode = SnpMode;

  // Assign fields and func pointers
  Snp->Revision = EFI_SIMPLE_NETWORK_PROTOCOL_REVISION;
  Snp->WaitForPacket = NULL;
  Snp->Initialize = SnpInitialize;
  Snp->Start = SnpStart;
  Snp->Stop = SnpStop;
  Snp->Reset = SnpReset;
  Snp->Shutdown = SnpShutdown;
  Snp->ReceiveFilters = SnpReceiveFilters;
  Snp->StationAddress = SnpStationAddress;
  Snp->Statistics = SnpStatistics;
  Snp->MCastIpToMac = SnpMcastIptoMac;
  Snp->NvData = SnpNvData;
  Snp->GetStatus = SnpGetStatus;
  Snp->Transmit = SnpTransmit;
  Snp->Receive = SnpReceive;

  // Start completing simple network mode structure
  SnpMode->State = EfiSimpleNetworkStopped;
  SnpMode->HwAddressSize = NET_ETHER_ADDR_LEN; // HW address is 6 bytes
  SnpMode->MediaHeaderSize = sizeof(ETHER_HEAD); // Not sure of this
  SnpMode->MaxPacketSize = EFI_PAGE_SIZE; // Preamble + SOF + Ether Frame (with VLAN tag +4bytes)
  SnpMode->NvRamSize = 0;           // No NVRAM with this device
  SnpMode->NvRamAccessSize = 0; // No NVRAM with this device

  // Update network mode information
  SnpMode->ReceiveFilterMask = EFI_SIMPLE_NETWORK_RECEIVE_UNICAST     |
                               EFI_SIMPLE_NETWORK_RECEIVE_MULTICAST   |
                               EFI_SIMPLE_NETWORK_RECEIVE_BROADCAST   |
                               EFI_SIMPLE_NETWORK_RECEIVE_PROMISCUOUS |
                               EFI_SIMPLE_NETWORK_RECEIVE_PROMISCUOUS_MULTICAST;

  // We do not intend to receive anything for the time being.
  SnpMode->ReceiveFilterSetting = 0;

  // EMAC has 64bit hash table, can filter 64 MCast MAC Addresses
  SnpMode->MaxMCastFilterCount = MAX_MCAST_FILTER_CNT;
  SnpMode->MCastFilterCount = 0;
  ZeroMem (&SnpMode->MCastFilter, MAX_MCAST_FILTER_CNT * sizeof(EFI_MAC_ADDRESS));

  // Set the interface type (1: Ethernet or 6: IEEE 802 Networks)
  SnpMode->IfType = NET_IFTYPE_ETHERNET;

  // Mac address is changeable as it is loaded from erasable memory
  SnpMode->MacAddressChangeable = TRUE;

  // Can only transmit one packet at a time
  SnpMode->MultipleTxSupported = FALSE;

  // MediaPresent checks for cable connection and partner link
  SnpMode->MediaPresentSupported = TRUE;
  SnpMode->MediaPresent = FALSE;

  // Set broadcast address
  SetMem (&SnpMode->BroadcastAddress, sizeof (EFI_MAC_ADDRESS), 0xFF);

  // Set current address
  DefaultMacAddress = FixedPcdGet64 (PcdDefaultMacAddress);
  // Swap PCD human readable form to correct endianess
  SwapMacAddressPtr = (EFI_MAC_ADDRESS *) &DefaultMacAddress;
  SnpMode->CurrentAddress.Addr[0] = SwapMacAddressPtr->Addr[5];
  SnpMode->CurrentAddress.Addr[1] = SwapMacAddressPtr->Addr[4];
  SnpMode->CurrentAddress.Addr[2] = SwapMacAddressPtr->Addr[3];
  SnpMode->CurrentAddress.Addr[3] = SwapMacAddressPtr->Addr[2];
  SnpMode->CurrentAddress.Addr[4] = SwapMacAddressPtr->Addr[1];
  SnpMode->CurrentAddress.Addr[5] = SwapMacAddressPtr->Addr[0];

  // Assign fields for device path
  CopyMem (&DevicePath->MacAddrDP.MacAddress, &Snp->Mode->CurrentAddress, NET_ETHER_ADDR_LEN);
  DevicePath->MacAddrDP.IfType = Snp->Mode->IfType;

  // Initialise the protocol
  ControllerHandle = NULL;
  Status = gBS->InstallMultipleProtocolInterfaces (
                  &ControllerHandle,
                  &gEfiSimpleNetworkProtocolGuid, Snp,
                  &gEfiDevicePathProtocolGuid, DevicePath,
                  NULL
                  );
  // Say what the status of loading the protocol structure is
  if (EFI_ERROR(Status)) {
    FreePool (Driver);
  } else {
    Driver->ControllerHandle = ControllerHandle;
  }

  return Status;
}


/**
  Change the state of a network interface from "stopped" to "started."

  This function starts a network interface. If the network interface successfully
  starts, then EFI_SUCCESS will be returned.

  @param  Snp                    A pointer to the EFI_SIMPLE_NETWORK_PROTOCOL instance.

  @retval EFI_SUCCESS            The network interface was started.
  @retval EFI_ALREADY_STARTED    The network interface is already in the started state.
  @retval EFI_INVALID_PARAMETER  This parameter was NULL or did not point to a valid
                                 EFI_SIMPLE_NETWORK_PROTOCOL structure.
  @retval EFI_DEVICE_ERROR       The command could not be sent to the network interface.
  @retval EFI_UNSUPPORTED        This function is not supported by the network interface.

**/
EFI_STATUS
EFIAPI
SnpStart (
  IN  EFI_SIMPLE_NETWORK_PROTOCOL  *Snp
 )
{
  InfoPrint ("SNP:DXE: %a ()\r\n", __FUNCTION__);

  // Check Snp instance
  if (Snp == NULL) {
    return EFI_INVALID_PARAMETER;
  }

  // Check state
  if ((Snp->Mode->State == EfiSimpleNetworkStarted)    ||
      (Snp->Mode->State == EfiSimpleNetworkInitialized)  ) {
    return EFI_ALREADY_STARTED;
  }

  // Change state
  Snp->Mode->State = EfiSimpleNetworkStarted;
  return EFI_SUCCESS;
}


/**
  Changes the state of a network interface from "started" to "stopped."

  This function stops a network interface. This call is only valid if the network
  interface is in the started state. If the network interface was successfully
  stopped, then EFI_SUCCESS will be returned.

  @param  Snp                     A pointer to the EFI_SIMPLE_NETWORK_PROTOCOL
                                  instance.


  @retval EFI_SUCCESS             The network interface was stopped.
  @retval EFI_NOT_STARTED         The network interface has not been started.
  @retval EFI_INVALID_PARAMETER   This parameter was NULL or did not point to a
                                  valid EFI_SIMPLE_NETWORK_PROTOCOL structure.
  @retval EFI_DEVICE_ERROR        The command could not be sent to the network
                                  interface.
  @retval EFI_UNSUPPORTED         This function is not supported by the network
                                  interface.

**/
EFI_STATUS
EFIAPI
SnpStop (
  IN        EFI_SIMPLE_NETWORK_PROTOCOL* Snp
  )
{
  InfoPrint ("SNP:DXE: %a ()\r\n", __FUNCTION__);

  // Check Snp Instance
  if (Snp == NULL) {
    return EFI_INVALID_PARAMETER;
  }

  // Check state of the driver
  if (Snp->Mode->State == EfiSimpleNetworkStopped) {
    return EFI_NOT_STARTED;
  }

  // Stop the Tx and Rx
  EmacStopTxRx ();
  // Change the state
  switch (Snp->Mode->State) {
    case EfiSimpleNetworkStarted:
    case EfiSimpleNetworkInitialized:
      Snp->Mode->State = EfiSimpleNetworkStopped;
      break;
    default:
      return EFI_DEVICE_ERROR;
  }

  // Put the device into a power saving mode ?
  return EFI_SUCCESS;
}


/**
  Resets a network adapter and allocates the transmit and receive buffers
  required by the network interface; optionally, also requests allocation of
  additional transmit and receive buffers.

  This function allocates the transmit and receive buffers required by the network
  interface. If this allocation fails, then EFI_OUT_OF_RESOURCES is returned.
  If the allocation succeeds and the network interface is successfully initialized,
  then EFI_SUCCESS will be returned.

  @param Snp                A pointer to the EFI_SIMPLE_NETWORK_PROTOCOL instance.

  @param ExtraRxBufferSize  The size, in bytes, of the extra receive buffer space
                            that the driver should allocate for the network interface.
                            Some network interfaces will not be able to use the
                            extra buffer, and the caller will not know if it is
                            actually being used.
  @param ExtraTxBufferSize  The size, in bytes, of the extra transmit buffer space
                            that the driver should allocate for the network interface.
                            Some network interfaces will not be able to use the
                            extra buffer, and the caller will not know if it is
                            actually being used.

  @retval EFI_SUCCESS           The network interface was initialized.
  @retval EFI_NOT_STARTED       The network interface has not been started.
  @retval EFI_OUT_OF_RESOURCES  There was not enough memory for the transmit and
                                receive buffers.
  @retval EFI_INVALID_PARAMETER This parameter was NULL or did not point to a valid
                                EFI_SIMPLE_NETWORK_PROTOCOL structure.
  @retval EFI_DEVICE_ERROR      The command could not be sent to the network interface.
  @retval EFI_UNSUPPORTED       The increased buffer size feature is not supported.

**/
EFI_STATUS
EFIAPI
SnpInitialize (
  IN EFI_SIMPLE_NETWORK_PROTOCOL *Snp,
  IN UINTN                       ExtraRxBufferSize OPTIONAL,
  IN UINTN                       ExtraTxBufferSize OPTIONAL
  )
{
  EFI_STATUS                  Status;
  EFI_SIMPLE_NETWORK_DRIVER  *Driver;

  InfoPrint ("SNP:DXE: %a ()\r\n", __FUNCTION__);

  // Check Snp Instance
  if (Snp == NULL) {
    return EFI_INVALID_PARAMETER;
  }

  Driver = INSTANCE_FROM_SNP_THIS (Snp);
  // First check that driver has not already been initialized
  if (Snp->Mode->State == EfiSimpleNetworkInitialized) {
    return EFI_SUCCESS;
  } else
  if (Snp->Mode->State == EfiSimpleNetworkStopped) {
    return EFI_NOT_STARTED;
  }

  // Init HPS
  AlteraHpsInitialization ();

  // Init PHY
  Status = PhyDxeInitialization (&Driver->PhyDriver);
  if (EFI_ERROR (Status)) {
    return EFI_DEVICE_ERROR;
  }

  // Init EMAC
  Status = EmacDxeInitialization (&Driver->MacDriver);
  if (EFI_ERROR (Status)) {
    return EFI_DEVICE_ERROR;
  }

  // Set MAC Address
  EmacSetMacAddress (&Snp->Mode->CurrentAddress);
  EmacReadMacAddress(&Snp->Mode->CurrentAddress);

  // Init Link
  ProgressPrint ("SNP:DXE: Auto-Negotiating Ethernet PHY Link ...\r\n");
  Status = PhyLinkAdjustEmacConfig (&Driver->PhyDriver);
  if (EFI_ERROR(Status)) {
    ProgressPrint ("SNP:DXE: Link is Down - Network Cable is not plugged in?\r\n");
    return EFI_DEVICE_ERROR;
  }

  EmacStartTransmission ();

  // Declare the driver as initialized
  Snp->Mode->State = EfiSimpleNetworkInitialized;

  return EFI_SUCCESS;
}


/**
  Resets a network adapter and reinitializes it with the parameters that were
  provided in the previous call to Initialize().

  This function resets a network adapter and reinitializes it with the parameters
  that were provided in the previous call to Initialize(). The transmit and
  receive queues are emptied and all pending interrupts are cleared.
  Receive filters, the station address, the statistics, and the multicast-IP-to-HW
  MAC addresses are not reset by this call. If the network interface was
  successfully reset, then EFI_SUCCESS will be returned. If the driver has not
  been initialized, EFI_DEVICE_ERROR will be returned.

  @param Snp                  A pointer to the EFI_SIMPLE_NETWORK_PROTOCOL instance.
  @param ExtendedVerification Indicates that the driver may perform a more
                              exhaustive verification operation of the device
                              during reset.

  @retval EFI_SUCCESS           The network interface was reset.
  @retval EFI_NOT_STARTED       The network interface has not been started.
  @retval EFI_INVALID_PARAMETER One or more of the parameters has an unsupported value.
  @retval EFI_DEVICE_ERROR      The command could not be sent to the network interface.
  @retval EFI_UNSUPPORTED       This function is not supported by the network interface.

**/
EFI_STATUS
EFIAPI
SnpReset (
  IN EFI_SIMPLE_NETWORK_PROTOCOL *Snp,
  IN BOOLEAN                     ExtendedVerification
  )
{
  EFI_STATUS                  Status;
  EFI_SIMPLE_NETWORK_DRIVER  *Driver;

  Driver = INSTANCE_FROM_SNP_THIS (Snp);

  InfoPrint ("SNP:DXE: %a ()\r\n", __FUNCTION__);

  // Check Snp Instance
  if (Snp == NULL) {
    return EFI_INVALID_PARAMETER;
  }

  // First check that driver has not already been initialized
  if (Snp->Mode->State == EfiSimpleNetworkStarted) {
    return EFI_DEVICE_ERROR;
  } else if (Snp->Mode->State == EfiSimpleNetworkStopped) {
    return EFI_NOT_STARTED;
  }

  // Initiate a PHY reset
  Status = PhySoftReset (&Driver->PhyDriver);
  if (EFI_ERROR (Status)) {
    Snp->Mode->State = EfiSimpleNetworkStopped;
    return EFI_NOT_STARTED;
  }

  return EFI_SUCCESS;
}


/**
  Resets a network adapter and leaves it in a state that is safe for another
  driver to initialize.

  This function releases the memory buffers assigned in the Initialize() call.
  Pending transmits and receives are lost, and interrupts are cleared and disabled.
  After this call, only the Initialize() and Stop() calls may be used. If the
  network interface was successfully shutdown, then EFI_SUCCESS will be returned.
  If the driver has not been initialized, EFI_DEVICE_ERROR will be returned.

  @param  This  A pointer to the EFI_SIMPLE_NETWORK_PROTOCOL instance.

  @retval EFI_SUCCESS           The network interface was shutdown.
  @retval EFI_NOT_STARTED       The network interface has not been started.
  @retval EFI_INVALID_PARAMETER This parameter was NULL or did not point to a valid
                                EFI_SIMPLE_NETWORK_PROTOCOL structure.
  @retval EFI_DEVICE_ERROR      The command could not be sent to the network interface.

**/
EFI_STATUS
EFIAPI
SnpShutdown (
  IN        EFI_SIMPLE_NETWORK_PROTOCOL* Snp
  )
{

  InfoPrint ("SNP:DXE: %a ()\r\n", __FUNCTION__);

  // Check Snp Instance
  if (Snp == NULL) {
    return EFI_INVALID_PARAMETER;
  }

  // First check that driver has not already been initialized
  if (Snp->Mode->State == EfiSimpleNetworkStarted) {
    return EFI_DEVICE_ERROR;
  } else if (Snp->Mode->State == EfiSimpleNetworkStopped) {
    return EFI_NOT_STARTED;
  }

  EmacStopTxRx ();

  Snp->Mode->State = EfiSimpleNetworkStopped;

  return EFI_SUCCESS;
}


/**
  Manages the multicast receive filters of a network interface.

  This function is used enable and disable the hardware and software receive
  filters for the underlying network device.
  The receive filter change is broken down into three steps:
  * The filter mask bits that are set (ON) in the Enable parameter are added to
    the current receive filter settings.
  * The filter mask bits that are set (ON) in the Disable parameter are subtracted
    from the updated receive filter settings.
  * If the resulting receive filter setting is not supported by the hardware a
    more liberal setting is selected.
  If the same bits are set in the Enable and Disable parameters, then the bits
  in the Disable parameter takes precedence.
  If the ResetMCastFilter parameter is TRUE, then the multicast address list
  filter is disabled (irregardless of what other multicast bits are set in the
  Enable and Disable parameters). The SNP->Mode->MCastFilterCount field is set
  to zero. The Snp->Mode->MCastFilter contents are undefined.
  After enabling or disabling receive filter settings, software should verify
  the new settings by checking the Snp->Mode->ReceiveFilterSettings,
  Snp->Mode->MCastFilterCount and Snp->Mode->MCastFilter fields.
  Note: Some network drivers and/or devices will automatically promote receive
    filter settings if the requested setting can not be honored. For example, if
    a request for four multicast addresses is made and the underlying hardware
    only supports two multicast addresses the driver might set the promiscuous
    or promiscuous multicast receive filters instead. The receiving software is
    responsible for discarding any extra packets that get through the hardware
    receive filters.
    Note: Note: To disable all receive filter hardware, the network driver must
      be Shutdown() and Stopped(). Calling ReceiveFilters() with Disable set to
      Snp->Mode->ReceiveFilterSettings will make it so no more packets are
      returned by the Receive() function, but the receive hardware may still be
      moving packets into system memory before inspecting and discarding them.
      Unexpected system errors, reboots and hangs can occur if an OS is loaded
      and the network devices are not Shutdown() and Stopped().
  If ResetMCastFilter is TRUE, then the multicast receive filter list on the
  network interface will be reset to the default multicast receive filter list.
  If ResetMCastFilter is FALSE, and this network interface allows the multicast
  receive filter list to be modified, then the MCastFilterCnt and MCastFilter
  are used to update the current multicast receive filter list. The modified
  receive filter list settings can be found in the MCastFilter field of
  EFI_SIMPLE_NETWORK_MODE. If the network interface does not allow the multicast
  receive filter list to be modified, then EFI_INVALID_PARAMETER will be returned.
  If the driver has not been initialized, EFI_DEVICE_ERROR will be returned.
  If the receive filter mask and multicast receive filter list have been
  successfully updated on the network interface, EFI_SUCCESS will be returned.

  @param Snp              A pointer to the EFI_SIMPLE_NETWORK_PROTOCOL instance.
  @param Enable           A bit mask of receive filters to enable on the network
                          interface.
  @param Disable          A bit mask of receive filters to disable on the network
                          interface. For backward compatibility with EFI 1.1
                          platforms, the EFI_SIMPLE_NETWORK_RECEIVE_MULTICAST bit
                          must be set when the ResetMCastFilter parameter is TRUE.
  @param ResetMCastFilter Set to TRUE to reset the contents of the multicast
                          receive filters on the network interface to their
                          default values.
  @param MCastFilterCnt   Number of multicast HW MAC addresses in the new MCastFilter
                          list. This value must be less than or equal to the
                          MCastFilterCnt field of EFI_SIMPLE_NETWORK_MODE.
                          This field is optional if ResetMCastFilter is TRUE.
  @param MCastFilter      A pointer to a list of new multicast receive filter HW
                          MAC addresses. This list will replace any existing
                          multicast HW MAC address list. This field is optional
                          if ResetMCastFilter is TRUE.

  @retval EFI_SUCCESS            The multicast receive filter list was updated.
  @retval EFI_NOT_STARTED        The network interface has not been started.
  @retval EFI_INVALID_PARAMETER  One or more of the following conditions is TRUE:
                                 * This is NULL
                                 * There are bits set in Enable that are not set
                                   in Snp->Mode->ReceiveFilterMask
                                 * There are bits set in Disable that are not set
                                   in Snp->Mode->ReceiveFilterMask
                                 * Multicast is being enabled (the
                                   EFI_SIMPLE_NETWORK_RECEIVE_MULTICAST bit is
                                   set in Enable, it is not set in Disable, and
                                   ResetMCastFilter is FALSE) and MCastFilterCount
                                   is zero
                                 * Multicast is being enabled and MCastFilterCount
                                   is greater than Snp->Mode->MaxMCastFilterCount
                                 * Multicast is being enabled and MCastFilter is NULL
                                 * Multicast is being enabled and one or more of
                                   the addresses in the MCastFilter list are not
                                   valid multicast MAC addresses
  @retval EFI_DEVICE_ERROR       One or more of the following conditions is TRUE:
                                 * The network interface has been started but has
                                   not been initialized
                                 * An unexpected error was returned by the
                                   underlying network driver or device
  @retval EFI_UNSUPPORTED        This function is not supported by the network
                                 interface.

**/
EFI_STATUS
EFIAPI
SnpReceiveFilters (
  IN  EFI_SIMPLE_NETWORK_PROTOCOL  *Snp,
  IN  UINT32                       Enable,
  IN  UINT32                       Disable,
  IN  BOOLEAN                      ResetMCastFilter,
  IN  UINTN                        MCastFilterCnt  OPTIONAL,
  IN  EFI_MAC_ADDRESS              *MCastFilter  OPTIONAL
  )
{
  EFI_SIMPLE_NETWORK_MODE    *Mode;
  UINT32                      ReceiveFilterSetting;
  EFI_SIMPLE_NETWORK_DRIVER  *Driver;

  Driver = INSTANCE_FROM_SNP_THIS (Snp);

  //InfoPrint ("SNP:DXE: %a ()\r\n", __FUNCTION__);

  // Check Snp Instance
  if (Snp == NULL) {
    return EFI_INVALID_PARAMETER;
  }
  Mode = Snp->Mode;

  // Check that driver was started and initialised
  if (Mode->State == EfiSimpleNetworkStarted) {
    return EFI_DEVICE_ERROR;
  } else if (Mode->State == EfiSimpleNetworkStopped) {
    return EFI_NOT_STARTED;
  }

   if ((Enable  & (~Mode->ReceiveFilterMask)) ||
      (Disable & (~Mode->ReceiveFilterMask))    ) {
    return EFI_INVALID_PARAMETER;
  }

  ReceiveFilterSetting = (Mode->ReceiveFilterSetting | Enable) & (~Disable);

  EmacRxFilters (ReceiveFilterSetting, ResetMCastFilter, MCastFilterCnt, MCastFilter);

  return EFI_SUCCESS;
}


/**
  Modifies or resets the current station address, if supported.

  This function modifies or resets the current station address of a network
  interface, if supported. If Reset is TRUE, then the current station address is
  set to the network interface's permanent address. If Reset is FALSE, and the
  network interface allows its station address to be modified, then the current
  station address is changed to the address specified by New. If the network
  interface does not allow its station address to be modified, then
  EFI_INVALID_PARAMETER will be returned. If the station address is successfully
  updated on the network interface, EFI_SUCCESS will be returned. If the driver
  has not been initialized, EFI_DEVICE_ERROR will be returned.

  @param Snp      A pointer to the EFI_SIMPLE_NETWORK_PROTOCOL instance.
  @param Reset    Flag used to reset the station address to the network interface's
                  permanent address.
  @param NewMac   New station address to be used for the network interface.


  @retval EFI_SUCCESS           The network interface's station address was updated.
  @retval EFI_NOT_STARTED       The Simple Network Protocol interface has not been
                                started by calling Start().
  @retval EFI_INVALID_PARAMETER The New station address was not accepted by the NIC.
  @retval EFI_INVALID_PARAMETER Reset is FALSE and New is NULL.
  @retval EFI_DEVICE_ERROR      The Simple Network Protocol interface has not
                                been initialized by calling Initialize().
  @retval EFI_DEVICE_ERROR      An error occurred attempting to set the new
                                station address.
  @retval EFI_UNSUPPORTED       The NIC does not support changing the network
                                interface's station address.

**/
EFI_STATUS
EFIAPI
SnpStationAddress (
  IN        EFI_SIMPLE_NETWORK_PROTOCOL *Snp,
  IN        BOOLEAN                      Reset,
  IN        EFI_MAC_ADDRESS             *NewMac
)
{
  return EFI_UNSUPPORTED;
}



/**
  Resets or collects the statistics on a network interface.

  This function resets or collects the statistics on a network interface. If the
  size of the statistics table specified by StatisticsSize is not big enough for
  all the statistics that are collected by the network interface, then a partial
  buffer of statistics is returned in StatisticsTable, StatisticsSize is set to
  the size required to collect all the available statistics, and
  EFI_BUFFER_TOO_SMALL is returned.
  If StatisticsSize is big enough for all the statistics, then StatisticsTable
  will be filled, StatisticsSize will be set to the size of the returned
  StatisticsTable structure, and EFI_SUCCESS is returned.
  If the driver has not been initialized, EFI_DEVICE_ERROR will be returned.
  If Reset is FALSE, and both StatisticsSize and StatisticsTable are NULL, then
  no operations will be performed, and EFI_SUCCESS will be returned.
  If Reset is TRUE, then all of the supported statistics counters on this network
  interface will be reset to zero.

  @param Snp             A pointer to the EFI_SIMPLE_NETWORK_PROTOCOL instance.
  @param Reset           Set to TRUE to reset the statistics for the network interface.
  @param StatSize        On input the size, in bytes, of StatisticsTable. On output
                         the size, in bytes, of the resulting table of statistics.
  @param Statistics      A pointer to the EFI_NETWORK_STATISTICS structure that
                         contains the statistics. Type EFI_NETWORK_STATISTICS is
                         defined in "Related Definitions" below.

  @retval EFI_SUCCESS           The requested operation succeeded.
  @retval EFI_NOT_STARTED       The Simple Network Protocol interface has not been
                                started by calling Start().
  @retval EFI_BUFFER_TOO_SMALL  StatisticsSize is not NULL and StatisticsTable is
                                NULL. The current buffer size that is needed to
                                hold all the statistics is returned in StatisticsSize.
  @retval EFI_BUFFER_TOO_SMALL  StatisticsSize is not NULL and StatisticsTable is
                                not NULL. The current buffer size that is needed
                                to hold all the statistics is returned in
                                StatisticsSize. A partial set of statistics is
                                returned in StatisticsTable.
  @retval EFI_INVALID_PARAMETER StatisticsSize is NULL and StatisticsTable is not
                                NULL.
  @retval EFI_DEVICE_ERROR      The Simple Network Protocol interface has not
                                been initialized by calling Initialize().
  @retval EFI_DEVICE_ERROR      An error was encountered collecting statistics
                                from the NIC.
  @retval EFI_UNSUPPORTED       The NIC does not support collecting statistics
                                from the network interface.

**/
EFI_STATUS
EFIAPI
SnpStatistics (
  IN        EFI_SIMPLE_NETWORK_PROTOCOL *Snp,
  IN        BOOLEAN                      Reset,
  IN  OUT   UINTN                        *StatSize,
      OUT   EFI_NETWORK_STATISTICS       *Statistics
  )
{
  EFI_SIMPLE_NETWORK_DRIVER  *Driver;

  Driver = INSTANCE_FROM_SNP_THIS (Snp);

  InfoPrint ("SNP:DXE: %a ()\r\n", __FUNCTION__);

  // Check Snp instance
  if (Snp == NULL) {
    return EFI_INVALID_PARAMETER;
  }

  // Check that driver was started and initialised
  if (Snp->Mode->State == EfiSimpleNetworkStarted) {
    return EFI_DEVICE_ERROR;
  } else if (Snp->Mode->State == EfiSimpleNetworkStopped) {
    return EFI_NOT_STARTED;
  }

  // Check pointless condition
  if ((!Reset) && (StatSize == NULL) && (Statistics == NULL)) {
    return EFI_SUCCESS;
  }

  // Check the parameters
  if ((StatSize == NULL) && (Statistics != NULL)) {
    return EFI_INVALID_PARAMETER;
  }

  // Do a reset if required
  if (Reset) {
    ZeroMem (&Driver->Stats, sizeof(EFI_NETWORK_STATISTICS));
  }

  // Check buffer size
  if (*StatSize < sizeof(EFI_NETWORK_STATISTICS)) {
    *StatSize = sizeof(EFI_NETWORK_STATISTICS);
    return EFI_BUFFER_TOO_SMALL;
  }
  // read statistic counters
  EmacGetStatistic (&Driver->Stats);

  // Fill in the statistics
  CopyMem(&Statistics, &Driver->Stats, sizeof(EFI_NETWORK_STATISTICS));

  return EFI_SUCCESS;
}


/**
  Converts a multicast IP address to a multicast HW MAC address.

  This function converts a multicast IP address to a multicast HW MAC address
  for all packet transactions. If the mapping is accepted, then EFI_SUCCESS will
  be returned.

  @param Snp         A pointer to the EFI_SIMPLE_NETWORK_PROTOCOL instance.
  @param IsIpv6      Set to TRUE if the multicast IP address is IPv6 [RFC 2460].
                     Set to FALSE if the multicast IP address is IPv4 [RFC 791].
  @param Ip          The multicast IP address that is to be converted to a multicast
                     HW MAC address.
  @param McastMac    The multicast HW MAC address that is to be generated from IP.

  @retval EFI_SUCCESS           The multicast IP address was mapped to the
                                multicast HW MAC address.
  @retval EFI_NOT_STARTED       The Simple Network Protocol interface has not
                                been started by calling Start().
  @retval EFI_INVALID_PARAMETER IP is NULL.
  @retval EFI_INVALID_PARAMETER MAC is NULL.
  @retval EFI_INVALID_PARAMETER IP does not point to a valid IPv4 or IPv6
                                multicast address.
  @retval EFI_DEVICE_ERROR      The Simple Network Protocol interface has not
                                been initialized by calling Initialize().
  @retval EFI_UNSUPPORTED       IPv6 is TRUE and the implementation does not
                                support IPv6 multicast to MAC address conversion.

**/
EFI_STATUS
EFIAPI
SnpMcastIptoMac (
  IN        EFI_SIMPLE_NETWORK_PROTOCOL  *Snp,
  IN        BOOLEAN                      IsIpv6,
  IN        EFI_IP_ADDRESS               *Ip,
      OUT   EFI_MAC_ADDRESS              *McastMac
  )
{

  InfoPrint ("SNP:DXE: %a ()\r\n", __FUNCTION__);

  // Check Snp instance
  if (Snp == NULL) {
    return EFI_INVALID_PARAMETER;
  }

  // Check that driver was started and initialised
  if (Snp->Mode->State == EfiSimpleNetworkStarted) {
    return EFI_DEVICE_ERROR;
  } else if (Snp->Mode->State == EfiSimpleNetworkStopped) {
    return EFI_NOT_STARTED;
  }

  // Check parameters
  if ((McastMac == NULL) || (Ip == NULL)) {
    return EFI_INVALID_PARAMETER;
  }

  // Make sure MAC address is empty
  ZeroMem (McastMac, sizeof(EFI_MAC_ADDRESS));

  // If we need ipv4 address
  if (!IsIpv6) {
    // Most significant 25 bits of a multicast HW address are set.
    // 01-00-5E is the IPv4 Ethernet Multicast Address (see RFC 1112)
    McastMac->Addr[0] = 0x01;
    McastMac->Addr[1] = 0x00;
    McastMac->Addr[2] = 0x5E;

    // Lower 23 bits from ipv4 address
    McastMac->Addr[3] = (Ip->v4.Addr[1] & 0x7F); // Clear the most significant bit (25th bit of MAC must be 0)
    McastMac->Addr[4] = Ip->v4.Addr[2];
    McastMac->Addr[5] = Ip->v4.Addr[3];
  } else {
    // Most significant 16 bits of multicast v6 HW address are set
    // 33-33 is the IPv6 Ethernet Multicast Address (see RFC 2464)
    McastMac->Addr[0] = 0x33;
    McastMac->Addr[1] = 0x33;

    // lower four octets are taken from ipv6 address
    McastMac->Addr[2] = Ip->v6.Addr[8];
    McastMac->Addr[3] = Ip->v6.Addr[9];
    McastMac->Addr[4] = Ip->v6.Addr[10];
    McastMac->Addr[5] = Ip->v6.Addr[11];
  }

  return EFI_SUCCESS;
}


/**
  Performs read and write operations on the NVRAM device attached to a network
  interface.

  This function performs read and write operations on the NVRAM device attached
  to a network interface. If ReadWrite is TRUE, a read operation is performed.
  If ReadWrite is FALSE, a write operation is performed. Offset specifies the
  byte offset at which to start either operation. Offset must be a multiple of
  NvRamAccessSize , and it must have a value between zero and NvRamSize.
  BufferSize specifies the length of the read or write operation. BufferSize must
  also be a multiple of NvRamAccessSize, and Offset + BufferSize must not exceed
  NvRamSize.
  If any of the above conditions is not met, then EFI_INVALID_PARAMETER will be
  returned.
  If all the conditions are met and the operation is "read," the NVRAM device
  attached to the network interface will be read into Buffer and EFI_SUCCESS
  will be returned. If this is a write operation, the contents of Buffer will be
  used to update the contents of the NVRAM device attached to the network
  interface and EFI_SUCCESS will be returned.

  It does the basic checking on the input parameters and retrieves snp structure
  and then calls the read_nvdata() call which does the actual reading

  @param Snp        A pointer to the EFI_SIMPLE_NETWORK_PROTOCOL instance.
  @param ReadWrite  TRUE for read operations, FALSE for write operations.
  @param Offset     Byte offset in the NVRAM device at which to start the read or
                    write operation. This must be a multiple of NvRamAccessSize
                    and less than NvRamSize. (See EFI_SIMPLE_NETWORK_MODE)
  @param BufferSize The number of bytes to read or write from the NVRAM device.
                    This must also be a multiple of NvramAccessSize.
  @param Buffer     A pointer to the data buffer.

  @retval EFI_SUCCESS           The NVRAM access was performed.
  @retval EFI_NOT_STARTED       The network interface has not been started.
  @retval EFI_INVALID_PARAMETER One or more of the following conditions is TRUE:
                                * The This parameter is NULL
                                * The This parameter does not point to a valid
                                  EFI_SIMPLE_NETWORK_PROTOCOL  structure
                                * The Offset parameter is not a multiple of
                                  EFI_SIMPLE_NETWORK_MODE.NvRamAccessSize
                                * The Offset parameter is not less than
                                  EFI_SIMPLE_NETWORK_MODE.NvRamSize
                                * The BufferSize parameter is not a multiple of
                                  EFI_SIMPLE_NETWORK_MODE.NvRamAccessSize
                                * The Buffer parameter is NULL
  @retval EFI_DEVICE_ERROR      The command could not be sent to the network
                                interface.
  @retval EFI_UNSUPPORTED       This function is not supported by the network
                                interface.

**/
EFI_STATUS
EFIAPI
SnpNvData (
  IN EFI_SIMPLE_NETWORK_PROTOCOL *Snp,
  IN BOOLEAN                     ReadWrite,
  IN UINTN                       Offset,
  IN UINTN                       BufferSize,
  IN OUT VOID                    *Buffer
  )
{
  InfoPrint ("SNP:DXE: %a ()\r\n", __FUNCTION__);

  return EFI_UNSUPPORTED;
}


/**
  Reads the current interrupt status and recycled transmit buffer status from a
  network interface.

  This function gets the current interrupt and recycled transmit buffer status
  from the network interface. The interrupt status is returned as a bit mask in
  InterruptStatus. If InterruptStatus is NULL, the interrupt status will not be
  read. If TxBuf is not NULL, a recycled transmit buffer address will be retrieved.
  If a recycled transmit buffer address is returned in TxBuf, then the buffer has
  been successfully transmitted, and the status for that buffer is cleared. If
  the status of the network interface is successfully collected, EFI_SUCCESS
  will be returned. If the driver has not been initialized, EFI_DEVICE_ERROR will
  be returned.

  @param Snp             A pointer to the EFI_SIMPLE_NETWORK_PROTOCOL instance.
  @param IrqStat         A pointer to the bit mask of the currently active
                         interrupts (see "Related Definitions"). If this is NULL,
                         the interrupt status will not be read from the device.
                         If this is not NULL, the interrupt status will be read
                         from the device. When the interrupt status is read, it
                         will also be cleared. Clearing the transmit interrupt does
                         not empty the recycled transmit buffer array.
  @param TxBuff          Recycled transmit buffer address. The network interface
                         will not transmit if its internal recycled transmit
                         buffer array is full. Reading the transmit buffer does
                         not clear the transmit interrupt. If this is NULL, then
                         the transmit buffer status will not be read. If there
                         are no transmit buffers to recycle and TxBuf is not NULL,
                         TxBuf will be set to NULL.

  @retval EFI_SUCCESS           The status of the network interface was retrieved.
  @retval EFI_NOT_STARTED       The network interface has not been started.
  @retval EFI_INVALID_PARAMETER This parameter was NULL or did not point to a valid
                                EFI_SIMPLE_NETWORK_PROTOCOL structure.
  @retval EFI_DEVICE_ERROR      The command could not be sent to the network
                                interface.

**/
EFI_STATUS
EFIAPI
SnpGetStatus (
  IN    EFI_SIMPLE_NETWORK_PROTOCOL *Snp,
  OUT   UINT32                      *IrqStat  OPTIONAL,
  OUT   VOID                        **TxBuff  OPTIONAL
  )
{
  EFI_STATUS                 Status;
  EFI_SIMPLE_NETWORK_DRIVER *Driver;
  UINT32                     TxDescNum;
  DESIGNWARE_HW_DESCRIPTOR  *TxDescriptor;

  //InfoPrint ("SNP:DXE: %a ()\r\n", __FUNCTION__);

  Driver = INSTANCE_FROM_SNP_THIS (Snp);

  // Check preliminaries
  if (Snp == NULL) {
    return EFI_INVALID_PARAMETER;
  }
  if (Snp->Mode->State != EfiSimpleNetworkInitialized) {
    return EFI_NOT_STARTED;
  }

  // Update the media status
  Status = PhyLinkAdjustEmacConfig (&Driver->PhyDriver);
  if (EFI_ERROR(Status)) {
    Snp->Mode->MediaPresent = FALSE;
  } else {
    Snp->Mode->MediaPresent = TRUE;
  }

  // TxBuff

  TxDescNum = Driver->MacDriver.TxCurrentDescriptorNum;
  TxDescriptor = &Driver->MacDriver.TxdescRing[TxDescNum];

  *TxBuff = TxDescriptor;

  // Check DMA Irq status
  EmacGetDmaStatus (IrqStat);

  return EFI_SUCCESS;
}


/**
  Places a packet in the transmit queue of a network interface.

  This function places the packet specified by Header and Buffer on the transmit
  queue. If HeaderSize is nonzero and HeaderSize is not equal to
  This->Mode->MediaHeaderSize, then EFI_INVALID_PARAMETER will be returned. If
  BufferSize is less than This->Mode->MediaHeaderSize, then EFI_BUFFER_TOO_SMALL
  will be returned. If Buffer is NULL, then EFI_INVALID_PARAMETER will be
  returned. If HeaderSize is nonzero and DestAddr or Protocol is NULL, then
  EFI_INVALID_PARAMETER will be returned. If the transmit engine of the network
  interface is busy, then EFI_NOT_READY will be returned. If this packet can be
  accepted by the transmit engine of the network interface, the packet contents
  specified by Buffer will be placed on the transmit queue of the network
  interface, and EFI_SUCCESS will be returned. GetStatus() can be used to
  determine when the packet has actually been transmitted. The contents of the
  Buffer must not be modified until the packet has actually been transmitted.
  The Transmit() function performs nonblocking I/O. A caller who wants to perform
  blocking I/O, should call Transmit(), and then GetStatus() until the
  transmitted buffer shows up in the recycled transmit buffer.
  If the driver has not been initialized, EFI_DEVICE_ERROR will be returned.

  @param Snp        A pointer to the EFI_SIMPLE_NETWORK_PROTOCOL instance.
  @param HdrSize    The size, in bytes, of the media header to be filled in by the
                    Transmit() function. If HeaderSize is nonzero, then it must
                    be equal to This->Mode->MediaHeaderSize and the DestAddr and
                    Protocol parameters must not be NULL.
  @param BuffSize   The size, in bytes, of the entire packet (media header and
                    data) to be transmitted through the network interface.
  @param Data       A pointer to the packet (media header followed by data) to be
                    transmitted. This parameter cannot be NULL. If HeaderSize is
                    zero, then the media header in Buffer must already be filled
                    in by the caller. If HeaderSize is nonzero, then the media
                    header will be filled in by the Transmit() function.
  @param SrcAddr    The source HW MAC address. If HeaderSize is zero, then this
                    parameter is ignored. If HeaderSize is nonzero and SrcAddr
                    is NULL, then This->Mode->CurrentAddress is used for the
                    source HW MAC address.
  @param DstAddr    The destination HW MAC address. If HeaderSize is zero, then
                    this parameter is ignored.
  @param Protocol   The type of header to build. If HeaderSize is zero, then this
                    parameter is ignored. See RFC 1700, section "Ether Types,"
                    for examples.

  @retval EFI_SUCCESS           The packet was placed on the transmit queue.
  @retval EFI_NOT_STARTED       The network interface has not been started.
  @retval EFI_NOT_READY         The network interface is too busy to accept this
                                transmit request.
  @retval EFI_BUFFER_TOO_SMALL  The BufferSize parameter is too small.
  @retval EFI_INVALID_PARAMETER One or more of the parameters has an unsupported
                                value.
  @retval EFI_DEVICE_ERROR      The command could not be sent to the network interface.
  @retval EFI_UNSUPPORTED       This function is not supported by the network interface.

**/
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
  )
{
  EFI_SIMPLE_NETWORK_DRIVER *Driver;
  UINT32                     DescNum;
  DESIGNWARE_HW_DESCRIPTOR  *TxDescriptor;
  UINT8*                     EthernetPacket;

  //InfoPrint ("SNP:DXE: %a ()\r\n", __FUNCTION__);

  EthernetPacket = Data;

  Driver = INSTANCE_FROM_SNP_THIS (Snp);

  // Check preliminaries
  if ((Snp == NULL) || (Data == NULL)) {
    return EFI_INVALID_PARAMETER;
  }
  if (Snp->Mode->State != EfiSimpleNetworkInitialized) {
    return EFI_NOT_STARTED;
  }


  Driver->MacDriver.TxCurrentDescriptorNum = Driver->MacDriver.TxNextDescriptorNum;
  DescNum = Driver->MacDriver.TxCurrentDescriptorNum;

  TxDescriptor = &Driver->MacDriver.TxdescRing[DescNum];

  // Ensure header is correct size if non-zero
  if (HdrSize) {
    if (HdrSize != Snp->Mode->MediaHeaderSize) {
      return EFI_INVALID_PARAMETER;
    }

    if ((DstAddr == NULL) || (Protocol == NULL)) {
      return EFI_INVALID_PARAMETER;
    }
  }

  if (HdrSize) {
    EthernetPacket[0] = DstAddr->Addr[0];
    EthernetPacket[1] = DstAddr->Addr[1];
    EthernetPacket[2] = DstAddr->Addr[2];
    EthernetPacket[3] = DstAddr->Addr[3];
    EthernetPacket[4] = DstAddr->Addr[4];
    EthernetPacket[5] = DstAddr->Addr[5];

    EthernetPacket[6] = SrcAddr->Addr[0];
    EthernetPacket[7] = SrcAddr->Addr[1];
    EthernetPacket[8] = SrcAddr->Addr[2];
    EthernetPacket[9] = SrcAddr->Addr[3];
    EthernetPacket[10] = SrcAddr->Addr[4];
    EthernetPacket[11] = SrcAddr->Addr[5];

    EthernetPacket[13] = *Protocol & 0xFF;
    EthernetPacket[12] = (*Protocol & 0xFF00) >> 8;

    /*
    InfoPrint ("SNP:MAC: S Dst  = %02X:%02X:%02X:%02X:%02X:%02X\r\n",
      EthernetPacket[0],
      EthernetPacket[1],
      EthernetPacket[2],
      EthernetPacket[3],
      EthernetPacket[4],
      EthernetPacket[5]
    );

    InfoPrint ("SNP:MAC: S Src  = %02X:%02X:%02X:%02X:%02X:%02X\r\n",
      EthernetPacket[6],
      EthernetPacket[7],
      EthernetPacket[8],
      EthernetPacket[9],
      EthernetPacket[10],
      EthernetPacket[11]
    );
    */
  }

  //InfoPrint ("SNP:DXE:\nTransmit Data:\r\n");
  //SerialPortMmioHexDump ((UINTN)EthernetPacket, BuffSize /4 + 1);

  CopyMem (TxDescriptor->Addr, EthernetPacket, BuffSize);

  TxDescriptor->Tdes1 = (BuffSize << TDES1_SIZE1SHFT) &
                         TDES1_SIZE1MASK;
  TxDescriptor->Tdes0 |= (TDES0_TXFIRST |
                          TDES0_TXLAST |
                          TDES0_OWN);

  // increase descriptor number
  DescNum++;
  if (DescNum >= CONFIG_TX_DESCR_NUM)
    DescNum = 0;

  Driver->MacDriver.TxNextDescriptorNum = DescNum;

  // Start the transmission
  EmacDmaStart ();

  return EFI_SUCCESS;
}


/**
  Receives a packet from a network interface.

  This function retrieves one packet from the receive queue of a network interface.
  If there are no packets on the receive queue, then EFI_NOT_READY will be
  returned. If there is a packet on the receive queue, and the size of the packet
  is smaller than BufferSize, then the contents of the packet will be placed in
  Buffer, and BufferSize will be updated with the actual size of the packet.
  In addition, if SrcAddr, DestAddr, and Protocol are not NULL, then these values
  will be extracted from the media header and returned. EFI_SUCCESS will be
  returned if a packet was successfully received.
  If BufferSize is smaller than the received packet, then the size of the receive
  packet will be placed in BufferSize and EFI_BUFFER_TOO_SMALL will be returned.
  If the driver has not been initialized, EFI_DEVICE_ERROR will be returned.

  @param Snp        A pointer to the EFI_SIMPLE_NETWORK_PROTOCOL instance.
  @param HdrSize    The size, in bytes, of the media header received on the network
                    interface. If this parameter is NULL, then the media header size
                    will not be returned.
  @param BuffSize   On entry, the size, in bytes, of Buffer. On exit, the size, in
                    bytes, of the packet that was received on the network interface.
  @param Data       A pointer to the data buffer to receive both the media
                    header and the data.
  @param SrcAddr    The source HW MAC address. If this parameter is NULL, the HW
                    MAC source address will not be extracted from the media header.
  @param DstAddr   The destination HW MAC address. If this parameter is NULL,
                    the HW MAC destination address will not be extracted from
                    the media header.
  @param Protocol   The media header type. If this parameter is NULL, then the
                    protocol will not be extracted from the media header. See
                    RFC 1700 section "Ether Types" for examples.

  @retval EFI_SUCCESS           The received data was stored in Buffer, and
                                BufferSize has been updated to the number of
                                bytes received.
  @retval EFI_NOT_STARTED       The network interface has not been started.
  @retval EFI_NOT_READY         No packets have been received on the network interface.
  @retval EFI_BUFFER_TOO_SMALL  BufferSize is too small for the received packets.
                                BufferSize has been updated to the required size.
  @retval EFI_INVALID_PARAMETER One or more of the following conditions is TRUE:
                                * The This parameter is NULL
                                * The This parameter does not point to a valid
                                  EFI_SIMPLE_NETWORK_PROTOCOL structure.
                                * The BufferSize parameter is NULL
                                * The Buffer parameter is NULL
  @retval EFI_DEVICE_ERROR      The command could not be sent to the network interface.

**/
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
  )
{
  EFI_SIMPLE_NETWORK_DRIVER  *Driver;
  EFI_MAC_ADDRESS Dst;
  EFI_MAC_ADDRESS Src;
  UINT32          Length, DesciptorStatus;
  UINT8*          RawData;
  UINT32 DescNum;
  DESIGNWARE_HW_DESCRIPTOR *RxDescriptor;

  // Check preliminaries
  if ((Snp == NULL) || (Data == NULL)) {
    return EFI_INVALID_PARAMETER;
  }

  if (Snp->Mode->State != EfiSimpleNetworkInitialized) {
    return EFI_NOT_STARTED;
  }

  Driver = INSTANCE_FROM_SNP_THIS (Snp);
  Driver->MacDriver.RxCurrentDescriptorNum = Driver->MacDriver.RxNextDescriptorNum;
  DescNum = Driver->MacDriver.RxCurrentDescriptorNum;
  RxDescriptor = &Driver->MacDriver.RxdescRing[DescNum];

  RawData = (UINT8*) Data;
  //Driver->Stats.RxTotalFrames += 1;

  DesciptorStatus = RxDescriptor->Tdes0;

  if (DesciptorStatus & ((UINT32)RDES0_OWN)) {
    return EFI_NOT_READY;
  }

  /*
  if (RawData[0] != 0xFF) // Ignore Broadcast frame
  {
    InfoPrint ("SNP:MAC: R Dst  = %02X:%02X:%02X:%02X:%02X:%02X\r\n",
      RawData[0],
      RawData[1],
      RawData[2],
      RawData[3],
      RawData[4],
      RawData[5]
    );

    InfoPrint ("SNP:MAC: R Src  = %02X:%02X:%02X:%02X:%02X:%02X\r\n",
      RawData[6],
      RawData[7],
      RawData[8],
      RawData[9],
      RawData[10],
      RawData[11]
    );
  }
  */

  if (DesciptorStatus & RDES0_SAF) {
    InfoPrint ("SNP:DXE: Rx Descritpor Status Error: Source Address Filter Fail\n");
    return EFI_DEVICE_ERROR;
  }

  if (DesciptorStatus & RDES0_AFM) {
    InfoPrint ("SNP:DXE: Rx Descritpor Status Error: Destination Address Filter Fail\n");
    return EFI_DEVICE_ERROR;
  }

  if (DesciptorStatus & RDES0_ES) {
    // check for errors
    if (DesciptorStatus & RDES0_RE) {
      InfoPrint ("SNP:DXE: Rx Descritpor Status Error: Receive Error\n");
    }
    if (DesciptorStatus & RDES0_DE) {
      InfoPrint ("SNP:DXE: Rx Descritpor Status Error: Receive Error\n");
    }
    if (DesciptorStatus & RDES0_RWT) {
      InfoPrint ("SNP:DXE: Rx Descritpor Status Error: Watchdog Timeout\n");
    }
    if (DesciptorStatus & RDES0_LC) {
      InfoPrint ("SNP:DXE: Rx Descritpor Status Error: Late Collision\n");
    }
    if (DesciptorStatus & RDES0_GF) {
      InfoPrint ("SNP:DXE: Rx Descritpor Status Error: Giant Frame\n");
    }
    if (DesciptorStatus & RDES0_OE) {
      InfoPrint ("SNP:DXE: Rx Descritpor Status Error: Overflow Error\n");
    }
    if (DesciptorStatus & RDES0_LE) {
      InfoPrint ("SNP:DXE: Rx Descritpor Status Error:Length Error\n");
    }
    if (DesciptorStatus & RDES0_DBE) {
      InfoPrint ("SNP:DXE: Rx Descritpor Status Error: Dribble Bit Error\n");
    }

    // check descriptor error status
    if (DesciptorStatus & RDES0_CE) {
      InfoPrint ("SNP:DXE: Rx Descritpor Status Error: CRC Error\n");
    }
    return EFI_DEVICE_ERROR;
  }

  Length = (DesciptorStatus >> RDES0_FL_SHIFT) & RDES0_FL_MASK;
  if (!Length) {
    InfoPrint ("SNP:DXE: Error: Invalid Frame Packet length \r\n");
    return EFI_NOT_READY;
  }
  // Check buffer size
  if (*BuffSize < Length) {
    InfoPrint ("SNP:DXE: Error: Buffer size is too small\n");
    return EFI_BUFFER_TOO_SMALL;
  }
  *BuffSize = Length;

  //InfoPrint ("SNP:DXE: %a ()\r\n", __FUNCTION__);

  if (HdrSize != NULL)
    *HdrSize = Snp->Mode->MediaHeaderSize;

  CopyMem (RawData, RxDescriptor->Addr, *BuffSize);

  if (DstAddr != NULL) {
    Dst.Addr[0] = RawData[0];
    Dst.Addr[1] = RawData[1];
    Dst.Addr[2] = RawData[2];
    Dst.Addr[3] = RawData[3];
    Dst.Addr[4] = RawData[4];
    Dst.Addr[5] = RawData[5];
    CopyMem (DstAddr, &Dst, NET_ETHER_ADDR_LEN);
  }

  // Get the source address
  if (SrcAddr != NULL) {
    Src.Addr[0] = RawData[6];
    Src.Addr[1] = RawData[7];
    Src.Addr[2] = RawData[8];
    Src.Addr[3] = RawData[9];
    Src.Addr[4] = RawData[10];
    Src.Addr[5] = RawData[11];
    CopyMem (SrcAddr,&Src, NET_ETHER_ADDR_LEN);
  }

  // Get the protocol
  if (Protocol != NULL) {
   *Protocol = NTOHS (RawData[12] | (RawData[13] >> 8) | (RawData[14] >> 16) | (RawData[15] >> 24));
  }

  RxDescriptor->Tdes0 |= (UINT32)RDES0_OWN;

  DescNum++;

  if (DescNum >= CONFIG_RX_DESCR_NUM)
    DescNum = 0;

  Driver->MacDriver.RxNextDescriptorNum = DescNum;

  return EFI_SUCCESS;
}


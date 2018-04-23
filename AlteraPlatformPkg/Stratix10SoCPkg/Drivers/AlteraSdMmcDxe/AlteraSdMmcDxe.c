/** @file
  Altera HPS SD/MMC controller Implementation of EFI_MMC_HOST_PROTOCOL

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

  Copyright (c) 2008 - 2009, Apple Inc. All rights reserved.
  Copyright (c) 2011 - 2014, ARM Limited. All rights reserved.

  This program and the accompanying materials
  are licensed and made available under the terms and conditions of the BSD License
  which accompanies this distribution.  The full text of the license may be found at
  http://opensource.org/licenses/bsd-license.php

  THE PROGRAM IS DISTRIBUTED UNDER THE BSD 3 Clause LICENSE ON AN "AS IS" BASIS,
  WITHOUT WARRANTIES OR REPRESENTATIONS OF ANY KIND, EITHER EXPRESS OR IMPLIED.

**/

#include "AlteraSdMmcDxe.h"
#include <Library/PcdLib.h>
#include <Library/ArmLib.h>

BOOLEAN
SdMmcIsCardPresent (
  IN EFI_MMC_HOST_PROTOCOL     *This
  )
{
  //DEBUG ((EFI_D_BLKIO, "SDMMC: %a ()\n", __FUNCTION__));
  return (IsCardPresent());
}

BOOLEAN
SdMmcIsReadOnly (
  IN EFI_MMC_HOST_PROTOCOL     *This
  )
{
  DEBUG ((EFI_D_BLKIO, "SDMMC: %a ()\n", __FUNCTION__));
  return (IsWriteProtected());
}

EFI_STATUS
SdMmcBuildDevicePath (
  IN EFI_MMC_HOST_PROTOCOL      *This,
  IN EFI_DEVICE_PATH_PROTOCOL   **DevicePath
  )
{
  EFI_DEVICE_PATH_PROTOCOL    *NewDevicePathNode;

  DEBUG ((EFI_D_BLKIO, "SDMMC: %a ()\n", __FUNCTION__));

  NewDevicePathNode = CreateDeviceNode (HARDWARE_DEVICE_PATH, HW_VENDOR_DP, sizeof (VENDOR_DEVICE_PATH));
  CopyGuid (& ((VENDOR_DEVICE_PATH*)NewDevicePathNode)->Guid, &mThisDriverDevicePathGuid);

  *DevicePath = NewDevicePathNode;
  return EFI_SUCCESS;
}


EFI_STATUS
SdMmcNotifyState (
  IN  EFI_MMC_HOST_PROTOCOL     *This,
  IN MMC_STATE                  State
  )
{
  EFI_STATUS    Status;

  switch (State) {
  case MmcInvalidState:
    ASSERT (0);
    break;
  case MmcHwInitializationState:
    DEBUG_SDMMC_STATE("Initialization");
    Status = InitializeSdMmc ();
    if (EFI_ERROR(Status)) {
      DEBUG ((EFI_D_ERROR, "SD/MMC Init Error!\n"));
      return Status;
    }
    break;
  case MmcIdleState:
    DEBUG_SDMMC_STATE("Idle");
    break;
  case MmcReadyState:
    DEBUG_SDMMC_STATE("Ready");
    break;
  case MmcIdentificationState:
    DEBUG_SDMMC_STATE("Identification");
    break;
  case MmcStandByState:
    DEBUG_SDMMC_STATE("StandBy");
	InitAfterEnumerateCardStack ();
    break;
  case MmcTransferState:
    //DEBUG_SDMMC_STATE("Transfer");
    // Change the card to faster Data Bus Mode
    ChangeDataBusMode ();
    break;
  case MmcSendingDataState:
    DEBUG_SDMMC_STATE("Sending Data");
    break;
  case MmcReceiveDataState:
    DEBUG_SDMMC_STATE("Receive Data");
    break;
  case MmcProgrammingState:
    //DEBUG_SDMMC_STATE("Programming");
    break;
  case MmcDisconnectState:
    DEBUG_SDMMC_STATE("Disconnect");
  default:
    ASSERT (0);
    break;
  }
  return EFI_SUCCESS;
}


EFI_STATUS
SdMmcSendCommand (
  IN EFI_MMC_HOST_PROTOCOL     *This,
  IN MMC_CMD                    MmcCmd,
  IN UINT32                     Argument
  )
{
  return SendCommand (MmcCmd, Argument);
}


EFI_STATUS
SdMmcReceiveResponse (
  IN EFI_MMC_HOST_PROTOCOL     *This,
  IN MMC_RESPONSE_TYPE          Type,
  IN UINT32*                    Buffer
  )
{
  if (Buffer == NULL) {
    return EFI_INVALID_PARAMETER;
  }
  return ReceiveCommandResponse(Type, Buffer);
}


EFI_STATUS
SdMmcReadBlockData (
  IN EFI_MMC_HOST_PROTOCOL     *This,
  IN EFI_LBA                    Lba,
  IN UINTN                      Length,
  IN UINT32*                    Buffer
  )
{
  EFI_TPL       Tpl;
  EFI_STATUS    Status;

  if (PcdGet32 (PcdSdmmcBlockUseInternalDMA) != 0)  {
    ArmCleanDataCache ();
    ArmDisableDataCache ();
    ArmInvalidateDataCache ();
  Tpl = gBS->RaiseTPL (TPL_HIGH_LEVEL);
    Status = ReadData(Length, Buffer);
    gBS->RestoreTPL (Tpl);

    ArmEnableDataCache ();
  } else {
  //  Tpl = gBS->RaiseTPL (TPL_HIGH_LEVEL);
    Status = ReadData(Length, Buffer);

//  gBS->RestoreTPL (Tpl);
  }
  return Status;
}

/**
  Writes a specified number of blocks to the device.

  It writes a specified number of blocks to the device.
  All blocks are written, or an error is returned.

  @param  This                   Indicates a pointer to the calling context.
  @param  Lba                    The starting logical block address to be written.
  @param  Length                 The size of the Buffer in bytes.
  @param  Buffer                 Pointer to the source buffer for the data.

  @retval EFI_SUCCESS            The data were written correctly to the device.
  @retval EFI_DEVICE_ERROR       The device reported an error while attempting to perform the write operation.

**/
EFI_STATUS
SdMmcWriteBlockData (
  IN EFI_MMC_HOST_PROTOCOL     *This,
  IN EFI_LBA                   Lba,
  IN UINTN                     Length,
  IN UINT32*                   Buffer
  )
{
  EFI_TPL Tpl;
  EFI_STATUS    Status;

  if (PcdGet32 (PcdSdmmcBlockUseInternalDMA) != 0)  {
    ArmCleanDataCache ();
    ArmDisableDataCache ();
    ArmInvalidateDataCache ();
  Tpl = gBS->RaiseTPL (TPL_HIGH_LEVEL);
    Status = WriteData(Length, Buffer);
    gBS->RestoreTPL (Tpl);

    ArmEnableDataCache ();
  } else {
    Tpl = gBS->RaiseTPL (TPL_HIGH_LEVEL);
    Status = WriteData(Length, Buffer);

  gBS->RestoreTPL (Tpl);
  }
  return Status;
}

EFI_MMC_HOST_PROTOCOL gSdMmcHost = {
  MMC_HOST_PROTOCOL_REVISION,
  SdMmcIsCardPresent,
  SdMmcIsReadOnly,
  SdMmcBuildDevicePath,
  SdMmcNotifyState,
  SdMmcSendCommand,
  SdMmcReceiveResponse,
  SdMmcReadBlockData,
  SdMmcWriteBlockData
};

EFI_STATUS
AlteraSdMmcDxeInitialize (
  IN EFI_HANDLE         ImageHandle,
  IN EFI_SYSTEM_TABLE   *SystemTable
  )
{
  EFI_STATUS    Status;
  EFI_HANDLE    Handle;

  Handle = NULL;

  DEBUG ((EFI_D_BLKIO, "SDMMC: %a ()\n", __FUNCTION__));

  //Publish EFI_MMC_HOST_PROTOCOL interfaces
  Status = gBS->InstallMultipleProtocolInterfaces (
                  &Handle,
                  &gEfiMmcHostProtocolGuid,
				  &gSdMmcHost,
                  NULL
                  );
  ASSERT_EFI_ERROR (Status);

  return Status;
}

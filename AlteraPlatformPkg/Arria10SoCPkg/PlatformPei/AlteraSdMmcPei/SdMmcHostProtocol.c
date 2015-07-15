/** @file
  Altera HPS SD/MMC controller Implementation of SDMMC_HOST_PROTOCOL

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

#include <Uefi.h>
#include <Library/AlteraSdMmcLib.h>
#include <Library/BaseMemoryLib.h>
#include <Library/DebugLib.h>
#include <Library/IoLib.h>
#include <Library/PcdLib.h>
#include <Library/SerialPortPrintLib.h>
#include <Library/TimerLib.h>
#include "SdMmcHostProtocol.h"

#if (FixedPcdGet32(PcdDebugMsg_SdMmc) == 0)
  #define ProgressPrint(FormatString, ...)    /* do nothing */
  #define InfoPrint(FormatString, ...)        /* do nothing */
  #define DEBUG_SDMMC_STATE(s)                /* do nothing */
#else
  #define ProgressPrint SerialPortPrint
  #define InfoPrint     SerialPortPrint
  #define DEBUG_SDMMC_STATE(s) SerialPortPrint ("SDMMC: %a ( %a ) \n", __FUNCTION__, s)
#endif

BOOLEAN
SdMmcIsCardPresent (
  IN EFI_MMC_HOST_PROTOCOL     *This
  )
{
  //InfoPrint ( "SDMMC: %a ()\n", __FUNCTION__);
  return (IsCardPresent());
}

BOOLEAN
SdMmcIsReadOnly (
  IN EFI_MMC_HOST_PROTOCOL     *This
  )
{
  //InfoPrint ( "SDMMC: %a ()\n", __FUNCTION__);
  return (IsWriteProtected());
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
      //InfoPrint ( "SD/MMC Init Error!\n");
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
//  InfoPrint ( "SDMMC SendCommand\t: 0x%08x \t 0x%08x\n", (UINT32)MmcCmd, (UINT32)Argument);
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
//  InfoPrint ( "SDMMC ReceiveCommand\t: 0x%08x \t 0x%08x\n", (UINT32)Type, (UINT32)Buffer);
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
  EFI_STATUS    Status;
  if (PcdGet32 (PcdSdmmcBlockReadUseInternalDMA) != 0)
  {
    Status = ReadDmaData(Length, Buffer);
  } else {
    Status = ReadFifoData(Length, Buffer);
  }
  return Status;
}


EFI_STATUS
SdMmcWriteBlockData (
  IN EFI_MMC_HOST_PROTOCOL     *This,
  IN EFI_LBA                   Lba,
  IN UINTN                     Length,
  IN UINT32*                   Buffer
  )
{
  EFI_STATUS    Status;
  Status = WriteFifoData(Length, Buffer);
  return Status;
}


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

  Copyright (c) 2011-2014, ARM Limited. All rights reserved.

  This program and the accompanying materials
  are licensed and made available under the terms and conditions of the BSD License
  which accompanies this distribution.  The full text of the license may be found at
  http://opensource.org/licenses/bsd-license.php

  THE PROGRAM IS DISTRIBUTED UNDER THE BSD 3 Clause LICENSE ON AN "AS IS" BASIS,
  WITHOUT WARRANTIES OR REPRESENTATIONS OF ANY KIND, EITHER EXPRESS OR IMPLIED.

**/

#include <Library/TimerLib.h>
#include <Library/SerialPortPrintLib.h>
#include "Mmc.h"

#if (FixedPcdGet32(PcdDebugMsg_SdMmc) == 0)
//  #define ProgressPrint(FormatString, ...)    /* do nothing */
  #define ProgressPrint                       SerialPortPrint
  #define InfoPrint(FormatString, ...)        /* do nothing */
#else
  #define ProgressPrint SerialPortPrint
  #define InfoPrint     SerialPortPrint
#endif

typedef union {
  UINT32 Raw;
  OCR    Ocr;
} OCR_RESPONSE;

#define MAX_RETRY_COUNT         25
#define CMD_RETRY_COUNT         20
#define RCA_SHIFT_OFFSET        16
#define EMMC_CARD_SIZE          512
#define EMMC_ECSD_SIZE_OFFSET   53

UINT32 mEmmcRcaCount = 0;

STATIC
EFI_STATUS
EFIAPI
EmmcIdentificationMode (
  IN PEI_SDMMC_INSTANCE    *MmcHostInstance,
  IN OCR_RESPONSE           Response
  )
{
  EFI_MMC_HOST_PROTOCOL *Host;
  EFI_BLOCK_IO_MEDIA    *Media;
  EFI_STATUS Status;
  UINT32     RCA;
  UINT32     ECSD[128];

  Host  = MmcHostInstance->MmcHost;
  Media = MmcHostInstance->BlockIo.Media;

  // Fetch card identity register
  Status = Host->SendCommand (Host, MMC_CMD2, 0);
  if (EFI_ERROR (Status)) {
    //InfoPrint ("EmmcIdentificationMode(): Failed to send CMD2, Status=%r.\r\n", Status);
    return Status;
  }

  Status = Host->ReceiveResponse (Host, MMC_RESPONSE_TYPE_R2, (UINT32 *)&(MmcHostInstance->CardInfo.CIDData));
  if (EFI_ERROR (Status)) {
    //InfoPrint ("EmmcIdentificationMode(): CID retrieval error, Status=%r.\r\n", Status);
    return Status;
  }

  // Assign a relative address value to the card
  MmcHostInstance->CardInfo.RCA = ++mEmmcRcaCount; // TODO: might need a more sophisticated way of doing this
  RCA = MmcHostInstance->CardInfo.RCA << RCA_SHIFT_OFFSET;
  Status = Host->SendCommand (Host, MMC_CMD3, RCA);
  if (EFI_ERROR (Status)) {
    //InfoPrint ("EmmcIdentificationMode(): RCA set error, Status=%r.\r\n", Status);
    return Status;
  }

  // Fetch card specific data
  Status = Host->SendCommand (Host, MMC_CMD9, RCA);
  if (EFI_ERROR (Status)) {
    //InfoPrint ("EmmcIdentificationMode(): Failed to send CMD9, Status=%r.\r\n", Status);
    return Status;
  }

  Status = Host->ReceiveResponse (Host, MMC_RESPONSE_TYPE_R2, (UINT32 *)&(MmcHostInstance->CardInfo.CSDData));
  if (EFI_ERROR (Status)) {
    //InfoPrint ("EmmcIdentificationMode(): CSD retrieval error, Status=%r.\r\n", Status);
    return Status;
  }

  // Select the card
  Status = Host->SendCommand (Host, MMC_CMD7, RCA);
  if (EFI_ERROR (Status)) {
    //InfoPrint ("EmmcIdentificationMode(): Card selection error, Status=%r.\r\n", Status);
  }

  // Fetch ECSD
  Status = Host->SendCommand (Host, MMC_CMD8, RCA);
  if (EFI_ERROR (Status)) {
    //InfoPrint ("EmmcIdentificationMode(): ECSD fetch error, Status=%r.\r\n", Status);
  }

  Status = Host->ReadBlockData (Host, 0, 512, ECSD);
  if (EFI_ERROR (Status)) {
    //InfoPrint ("EmmcIdentificationMode(): ECSD read error, Status=%r.\r\n", Status);
    return Status;
  }

  // Set up media
  Media->BlockSize = EMMC_CARD_SIZE; // 512-byte support is mandatory for eMMC cards
  Media->MediaId = MmcHostInstance->CardInfo.CIDData.PSN;
  Media->ReadOnly = MmcHostInstance->CardInfo.CSDData.PERM_WRITE_PROTECT;
  Media->LogicalBlocksPerPhysicalBlock = 1;
  Media->IoAlign = 4;
  // Compute last block using bits [215:212] of the ECSD
  Media->LastBlock = ECSD[EMMC_ECSD_SIZE_OFFSET] - 1; // eMMC isn't supposed to report this for
  // Cards <2GB in size, but the model does.

  // Setup card type
  MmcHostInstance->CardInfo.CardType = EMMC_CARD;
  return EFI_SUCCESS;
}

STATIC
EFI_STATUS
InitializeSdMmcDevice (
  IN  PEI_SDMMC_INSTANCE  *MmcHostInstance
  )
{
  UINT32        CmdArg;
  UINT32        Response[4];
  UINTN         BlockSize;
  UINTN         CardSize;
  UINTN         NumBlocks;
  EFI_STATUS    Status;
  EFI_MMC_HOST_PROTOCOL     *MmcHost;

  MmcHost = MmcHostInstance->MmcHost;

  // Send a command to get Card specific data
  CmdArg = MmcHostInstance->CardInfo.RCA << 16;
  Status = MmcHost->SendCommand (MmcHost, MMC_CMD9, CmdArg);
  if (EFI_ERROR (Status)) {
    //InfoPrint ("InitializeSdMmcDevice(MMC_CMD9): Error, Status=%r\r\n", Status);
    return Status;
  }

  // Read Response
  Status = MmcHost->ReceiveResponse (MmcHost, MMC_RESPONSE_TYPE_CSD, Response);
  if (EFI_ERROR (Status)) {
    //InfoPrint ("InitializeSdMmcDevice(): Failed to receive CSD, Status=%r\r\n", Status);
    return Status;
  }
  PrintCSD (Response);

  if (MmcHostInstance->CardInfo.CardType == SD_CARD_2_HIGH) {
    CardSize = HC_MMC_CSD_GET_DEVICESIZE (Response);
    NumBlocks = ((CardSize + 1) * 1024);
    BlockSize = 1 << MMC_CSD_GET_READBLLEN (Response);
  } else {
    CardSize = MMC_CSD_GET_DEVICESIZE (Response);
    NumBlocks = (CardSize + 1) * (1 << (MMC_CSD_GET_DEVICESIZEMULT (Response) + 2));
    BlockSize = 1 << MMC_CSD_GET_READBLLEN (Response);
  }

  // For >=2G card, BlockSize may be 1K, but the transfer size is 512 bytes.
  if (BlockSize > 512) {
    NumBlocks = MultU64x32 (NumBlocks, BlockSize / 512);
    BlockSize = 512;
  }

  MmcHostInstance->BlockIo.Media->LastBlock    = (NumBlocks - 1);
  MmcHostInstance->BlockIo.Media->BlockSize    = BlockSize;
  MmcHostInstance->BlockIo.Media->ReadOnly     = MmcHost->IsReadOnly (MmcHost);
  MmcHostInstance->BlockIo.Media->MediaPresent = TRUE;
  MmcHostInstance->BlockIo.Media->MediaId++;

  ProgressPrint ("\t- LastBlock: %d\r\n"
                 "\t- BlockSize: %d\r\n",
                 (UINT32) MmcHostInstance->BlockIo.Media->LastBlock,
                 MmcHostInstance->BlockIo.Media->BlockSize);

  CmdArg = MmcHostInstance->CardInfo.RCA << 16;
  Status = MmcHost->SendCommand (MmcHost, MMC_CMD7, CmdArg);
  if (EFI_ERROR (Status)) {
    //InfoPrint ("InitializeSdMmcDevice(MMC_CMD7): Error and Status = %r\r\n", Status);
    return Status;
  }

  return EFI_SUCCESS;
}

STATIC
EFI_STATUS
EFIAPI
MmcIdentificationMode (
  IN PEI_SDMMC_INSTANCE    *MmcHostInstance
  )
{
  EFI_STATUS              Status;
  UINT32                  Response[4];
  UINTN                   Timeout;
  UINTN                   CmdArg;
  BOOLEAN                 IsHCS;
  EFI_MMC_HOST_PROTOCOL   *MmcHost;
  OCR_RESPONSE            OcrResponse;

  MmcHost = MmcHostInstance->MmcHost;
  CmdArg = 0;
  IsHCS = FALSE;

  if (MmcHost == NULL) {
    return EFI_INVALID_PARAMETER;
  }

  // We can get into this function if we restart the identification mode
  if (MmcHostInstance->State == MmcHwInitializationState) {
    // Initialize the MMC Host HW
    Status = MmcNotifyState (MmcHostInstance, MmcHwInitializationState);
    if (EFI_ERROR (Status)) {
      //InfoPrint ("MmcIdentificationMode() : Error MmcHwInitializationState, Status=%r.\r\n", Status);
      return Status;
    }
  }

  Status = MmcHost->SendCommand (MmcHost, MMC_CMD0, 0);
  if (EFI_ERROR (Status)) {
    //InfoPrint ("MmcIdentificationMode(MMC_CMD0): Error, Status=%r.\r\n", Status);
    return Status;
  }
  Status = MmcNotifyState (MmcHostInstance, MmcIdleState);
  if (EFI_ERROR (Status)) {
    //InfoPrint ("MmcIdentificationMode() : Error MmcIdleState, Status=%r.\r\n", Status);
    return Status;
  }

  // Send CMD1 to get OCR (MMC)
  // This command only valid for MMC and eMMC
  Status = MmcHost->SendCommand (MmcHost, MMC_CMD1, EMMC_CMD1_CAPACITY_GREATER_THAN_2GB);
  if (Status == EFI_SUCCESS) {
    Status = MmcHost->ReceiveResponse (MmcHost, MMC_RESPONSE_TYPE_OCR, (UINT32 *)&OcrResponse);
    if (EFI_ERROR (Status)) {
      //InfoPrint ("MmcIdentificationMode() : Failed to receive OCR, Status=%r.\r\n", Status);
      return Status;
    }

    if (!OcrResponse.Ocr.PowerUp) {
      //InfoPrint ("MmcIdentificationMode(MMC_CMD1): Card initialisation failure, Status=%r.\r\n", Status);
      return EFI_DEVICE_ERROR;
    }
    OcrResponse.Ocr.PowerUp = 0;
    if (OcrResponse.Raw == EMMC_CMD1_CAPACITY_GREATER_THAN_2GB) {
      MmcHostInstance->CardInfo.OCRData.AccessMode = BIT1;
    }
    else {
      MmcHostInstance->CardInfo.OCRData.AccessMode = 0x0;
    }
    // Check whether MMC or eMMC
    if (OcrResponse.Raw == EMMC_CMD1_CAPACITY_GREATER_THAN_2GB ||
        OcrResponse.Raw == EMMC_CMD1_CAPACITY_LESS_THAN_2GB) {
      return EmmcIdentificationMode (MmcHostInstance, OcrResponse);
    }
  } else {
    InfoPrint ("Is not eMMC\r\n");
  }

  // Are we using SDIO ?
  Status = MmcHost->SendCommand (MmcHost, MMC_CMD5, 0);
  if (Status == EFI_SUCCESS) {
    InfoPrint ("MmcIdentificationMode(MMC_CMD5): Error - SDIO not supported, Status=%r.\r\n", Status);
    return EFI_UNSUPPORTED;
  } else {
    InfoPrint ("Is not SDIO\r\n");
  }

  // Check which kind of card we are using. Ver2.00 or later SD Memory Card (PL180 is SD v1.1)
  CmdArg = (0x0UL << 12 | BIT8 | 0xCEUL << 0);
  Status = MmcHost->SendCommand (MmcHost, MMC_CMD8, CmdArg);
  if (Status == EFI_SUCCESS) {
    //InfoPrint ("Card is SD2.0 => Supports high capacity\r\n");
    IsHCS = TRUE;
    Status = MmcHost->ReceiveResponse (MmcHost, MMC_RESPONSE_TYPE_R7, Response);
    if (EFI_ERROR (Status)) {
      //InfoPrint ("MmcIdentificationMode() : Failed to receive response to CMD8, Status=%r.\r\n", Status);
      return Status;
    }
    //PrintResponseR1 (Response[0]);
    // Check if it is valid response
    if (Response[0] != CmdArg) {
      InfoPrint ("The Card is not usable\r\n");
      return EFI_UNSUPPORTED;
    }
  } else {
    InfoPrint ("Is not SD2.0 Card\r\n");
  }

  // We need to wait for the MMC or SD card is ready => (gCardInfo.OCRData.PowerUp == 1)
  Timeout = MAX_RETRY_COUNT;
  while (Timeout > 0) {
    // SD Card or MMC Card ? CMD55 indicates to the card that the next command is an application specific command
    Status = MmcHost->SendCommand (MmcHost, MMC_CMD55, 0);
    if (Status == EFI_SUCCESS) {
      InfoPrint ("Detecting...card should be SD\r\n");
      if (IsHCS) {
        MmcHostInstance->CardInfo.CardType = SD_CARD_2;
      } else {
        MmcHostInstance->CardInfo.CardType = SD_CARD;
      }

      // Note: The first time CmdArg will be zero
      CmdArg = ((UINTN *) &(MmcHostInstance->CardInfo.OCRData))[0];
      if (IsHCS) {
        CmdArg |= BIT30;
      }
      Status = MmcHost->SendCommand (MmcHost, MMC_ACMD41, CmdArg);
      if (!EFI_ERROR (Status)) {
        Status = MmcHost->ReceiveResponse (MmcHost, MMC_RESPONSE_TYPE_OCR, Response);
        if (EFI_ERROR (Status)) {
          //InfoPrint ("MmcIdentificationMode() : Failed to receive OCR, Status=%r.\r\n", Status);
          return Status;
        }
        ((UINT32 *) &(MmcHostInstance->CardInfo.OCRData))[0] = Response[0];
      }
    } else if (IsHCS == FALSE) {
      if (Timeout == MAX_RETRY_COUNT) {
        // First time here in this loop
        InfoPrint ("Could be MMC Card?\r\n");
      } else {
        InfoPrint ("Detecting Card ...\r\n");
      }
      MmcHostInstance->CardInfo.CardType = MMC_CARD;

      Status = MmcHost->SendCommand (MmcHost, MMC_CMD1, 0x800000);
      if (!EFI_ERROR (Status)) {
        Status = MmcHost->ReceiveResponse (MmcHost, MMC_RESPONSE_TYPE_OCR, Response);
        if (EFI_ERROR (Status)) {
          InfoPrint ("MmcIdentificationMode() : Failed to receive OCR, Status=%r.\r\n", Status);
          return Status;
        }
        ((UINT32 *) &(MmcHostInstance->CardInfo.OCRData))[0] = Response[0];
      }
    }

    if (!EFI_ERROR (Status)) {
      if (!MmcHostInstance->CardInfo.OCRData.PowerUp) {
        InfoPrint ("Waiting for card to power up.\r\n");
        MicroSecondDelay (1000);
        Timeout--;
      } else {
        if ((MmcHostInstance->CardInfo.CardType == SD_CARD_2) && (MmcHostInstance->CardInfo.OCRData.AccessMode & BIT1)) {
          MmcHostInstance->CardInfo.CardType = SD_CARD_2_HIGH;
          ProgressPrint ("High capacity SD card detected\r\n");
        }
        // The MMC/SD card is ready. Continue the Identification Mode
        break;
      }
    } else {
      MicroSecondDelay (1);
      Timeout--;
    }
  }

  if (Timeout == 0) {
    ProgressPrint ("No SD/MMC card detected!\r\n");
    return EFI_NO_MEDIA;
  } else {
    PrintOCR (Response[0]);
  }

  Status = MmcNotifyState (MmcHostInstance, MmcReadyState);
  if (EFI_ERROR (Status)) {
    //InfoPrint ("MmcIdentificationMode() : Error MmcReadyState\r\n");
    return Status;
  }

  Status = MmcHost->SendCommand (MmcHost, MMC_CMD2, 0);
  if (EFI_ERROR (Status)) {
    //InfoPrint ("MmcIdentificationMode(MMC_CMD2): Error\r\n");
    return Status;
  }
  Status = MmcHost->ReceiveResponse (MmcHost, MMC_RESPONSE_TYPE_CID, Response);
  if (EFI_ERROR (Status)) {
    //InfoPrint ("MmcIdentificationMode() : Failed to receive CID, Status=%r.\r\n", Status);
    return Status;
  }

  PrintCID (Response);

  Status = MmcHost->NotifyState (MmcHost, MmcIdentificationState);
  if (EFI_ERROR (Status)) {
    //InfoPrint ("MmcIdentificationMode() : Error MmcIdentificationState\r\n");
    return Status;
  }

  //
  // Note, SD specifications say that "if the command execution causes a state change, it
  // will be visible to the host in the response to the next command"
  // The status returned for this CMD3 will be 2 - identification
  //
  CmdArg = 1;
  Status = MmcHost->SendCommand (MmcHost, MMC_CMD3, CmdArg);
  if (EFI_ERROR (Status)) {
    //InfoPrint ("MmcIdentificationMode(MMC_CMD3): Error\r\n");
    return Status;
  }

  Status = MmcHost->ReceiveResponse (MmcHost, MMC_RESPONSE_TYPE_RCA, Response);
  if (EFI_ERROR (Status)) {
    //InfoPrint ("MmcIdentificationMode() : Failed to receive RCA, Status=%r.\r\n", Status);
    return Status;
  }
  //PrintRCA (Response[0]);

  // For MMC card, RCA is assigned by CMD3 while CMD3 dumps the RCA for SD card
  if (MmcHostInstance->CardInfo.CardType != MMC_CARD) {
    MmcHostInstance->CardInfo.RCA = Response[0] >> 16;
  } else {
    MmcHostInstance->CardInfo.RCA = CmdArg;
  }
  Status = MmcNotifyState (MmcHostInstance, MmcStandByState);
  if (EFI_ERROR (Status)) {
    //InfoPrint ("MmcIdentificationMode() : Error MmcStandByState\r\n");
    return Status;
  }

  return EFI_SUCCESS;
}

EFI_STATUS
InitializeMmcDevice (
  IN  PEI_SDMMC_INSTANCE  *MmcHostInstance
  )
{
  EFI_STATUS              Status;
  EFI_MMC_HOST_PROTOCOL   *MmcHost;
  UINTN                   BlockCount;

  BlockCount = 1;
  MmcHost = MmcHostInstance->MmcHost;

  Status = MmcIdentificationMode (MmcHostInstance);
  if (EFI_ERROR (Status)) {
    //InfoPrint ("InitializeMmcDevice(): Error in Identification Mode, Status=%r\r\n", Status);
    return Status;
  }

  Status = MmcNotifyState (MmcHostInstance, MmcTransferState);
  if (EFI_ERROR (Status)) {
    //InfoPrint ("InitializeMmcDevice(): Error MmcTransferState, Status=%r\r\n", Status);
    return Status;
  }

  if (MmcHostInstance->CardInfo.CardType != EMMC_CARD) {
    Status = InitializeSdMmcDevice (MmcHostInstance);
    if (EFI_ERROR (Status)) {
      return Status;
    }
  }

  // Set Block Length
  Status = MmcHost->SendCommand (MmcHost, MMC_CMD16, MmcHostInstance->BlockIo.Media->BlockSize);
  if (EFI_ERROR (Status)) {
    //InfoPrint ("InitializeMmcDevice(MMC_CMD16): Error MmcHostInstance->BlockIo.Media->BlockSize: %d and Error = %r\r\n",
    //                    MmcHostInstance->BlockIo.Media->BlockSize, Status);
    return Status;
  }

  // Block Count (not used). Could return an error for SD card
  if (MmcHostInstance->CardInfo.CardType == MMC_CARD) {
    Status = MmcHost->SendCommand (MmcHost, MMC_CMD23, BlockCount);
    if (EFI_ERROR (Status)) {
      //InfoPrint ("InitializeMmcDevice(MMC_CMD23): Error, Status=%r\r\n", Status);
      return Status;
    }
  }

  return EFI_SUCCESS;
}


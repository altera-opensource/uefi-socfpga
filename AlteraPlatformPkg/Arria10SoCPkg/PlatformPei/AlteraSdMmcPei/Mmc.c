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

  Copyright (c) 2011-2013, ARM Limited. All rights reserved.

  This program and the accompanying materials
  are licensed and made available under the terms and conditions of the BSD License
  which accompanies this distribution.  The full text of the license may be found at
  http://opensource.org/licenses/bsd-license.php

  THE PROGRAM IS DISTRIBUTED UNDER THE BSD 3 Clause LICENSE ON AN "AS IS" BASIS,
  WITHOUT WARRANTIES OR REPRESENTATIONS OF ANY KIND, EITHER EXPRESS OR IMPLIED.

**/
#include <Uefi.h>
#include <Library/AlteraSdMmcLib.h>
#include <Library/BaseMemoryLib.h>
#include <Library/DebugLib.h>
#include <Library/DevicePathLib.h>
#include <Library/IoLib.h>
#include <Library/PcdLib.h>
#include <Library/SerialPortPrintLib.h>
#include "Mmc.h"
#include "SdMmcHostProtocol.h"


EFI_BLOCK_IO_MEDIA mMmcMediaTemplate = {
  SIGNATURE_32('p','m','m','o'),            // MediaId
  TRUE,                                     // RemovableMedia
  FALSE,                                    // MediaPresent
  FALSE,                                    // LogicalPartition
  FALSE,                                    // ReadOnly
  FALSE,                                    // WriteCaching
  512,                                      // BlockSize
  4,                                        // IoAlign
  0,                                        // Pad
  0                                         // LastBlock
};

EFI_MMC_HOST_PROTOCOL gSdMmcHost = {
  MMC_HOST_PROTOCOL_REVISION,
  SdMmcIsCardPresent,
  SdMmcIsReadOnly,
  NULL,
  SdMmcNotifyState,
  SdMmcSendCommand,
  SdMmcReceiveResponse,
  SdMmcReadBlockData,
  SdMmcWriteBlockData
};

PEI_SDMMC_INSTANCE  gMmcHostInstance;

PEI_SDMMC_INSTANCE* GetMmcHostInstance (
  VOID
  )
{
  PEI_SDMMC_INSTANCE*  MmcHostInstance;
  MmcHostInstance = &gMmcHostInstance;
  return MmcHostInstance;
}

PEI_SDMMC_INSTANCE* CreateMmcHostInstance (
  IN EFI_MMC_HOST_PROTOCOL* MmcHost
  )
{
  PEI_SDMMC_INSTANCE*  MmcHostInstance;
  MmcHostInstance = &gMmcHostInstance;
  MmcHostInstance->Signature           = PEI_SDMMC_INSTANCE_SIGNATURE;
  MmcHostInstance->State               = MmcHwInitializationState;
  MmcHostInstance->MmcHost             = MmcHost;
  MmcHostInstance->BlockIo.Revision    = EFI_BLOCK_IO_INTERFACE_REVISION;
  MmcHostInstance->BlockIo.Reset       = MmcReset;
  MmcHostInstance->BlockIo.ReadBlocks  = MmcReadBlocks;
  MmcHostInstance->BlockIo.WriteBlocks = MmcWriteBlocks;
  MmcHostInstance->BlockIo.FlushBlocks = MmcFlushBlocks;
  MmcHostInstance->BlockIo.Media       = &mMmcMediaTemplate;
  return MmcHostInstance;
}

VOID
EFIAPI
CheckCardsCallback (
  VOID
  )
{
  PEI_SDMMC_INSTANCE*  MmcHostInstance;
  EFI_STATUS           Status;

  MmcHostInstance = &gMmcHostInstance;
  ASSERT(MmcHostInstance != NULL);
  if (MmcHostInstance->MmcHost->IsCardPresent (MmcHostInstance->MmcHost) == !MmcHostInstance->Initialized)
  {
    MmcHostInstance->State = MmcHwInitializationState;
    MmcHostInstance->BlockIo.Media->MediaPresent = !MmcHostInstance->Initialized;
    MmcHostInstance->Initialized = !MmcHostInstance->Initialized;
    if (MmcHostInstance->BlockIo.Media->MediaPresent) {

      if (PcdGet32 (PcdSdmmcSweepAllDrvselAndSmplselValues) == 1)
      {
        // SD/MMC calibration helper
        // Sweep all possible values for drvsel and smplsel test if the card responds to CMD

	      UINTN drvsel, smplsel;
	      UINTN result[8][8];

	      for (drvsel = 0; drvsel <= 7; drvsel++)
	      {
	        for (smplsel = 0; smplsel <= 7; smplsel++)
	        {
            SerialPortPrint ("\r\nSweeping DrvSel = %d and SmplSel = %d\r\n", drvsel, smplsel);
            MmioAndThenOr32 (
              ALT_SYSMGR_OFST +
              ALT_SYSMGR_SDMMC_OFST,
              ALT_SYSMGR_SDMMC_SMPLSEL_CLR_MSK &
              ALT_SYSMGR_SDMMC_DRVSEL_CLR_MSK,
              ALT_SYSMGR_SDMMC_SMPLSEL_SET(smplsel) |
              ALT_SYSMGR_SDMMC_DRVSEL_SET (drvsel)
              );

            Status = InitializeMmcDevice (MmcHostInstance);

            if (Status == EFI_SUCCESS) {
	            result[drvsel][smplsel] = 1;
              SerialPortPrint ("PASSED!\r\n");
            } else {
              result[drvsel][smplsel] = 0;
              SerialPortPrint ("FAILED!\r\n");
            }
	        } // smpsel
	      } // drvsel

        // Generate Report
        SerialPortPrint ("\r\n"
                         "SD/MMC DrvSel and SmplSel Sweeping Completed!\r\n\r\n"
                         "DrvSel VS SmplSel Matrix Report:\r\n\r\n"
                         "              0<--smplsel-->7\r\n");
        SerialPortPrint ("");
	      for (drvsel = 0; drvsel <= 7; drvsel++)
	      {
          SerialPortPrint ("drvsel=0x%02x : ", drvsel);
	        for (smplsel = 0; smplsel <= 7; smplsel++)
	        {
            SerialPortPrint ("%d ", result[drvsel][smplsel]);
	        } // smpsel
          SerialPortPrint ("\r\n");
	      } // drvsel

        SerialPortPrint ("\r\nYour next steps:\r\n"
                         "1. Identify the largest rectangle with 1 that passed.\r\n"
                         "2. Choose drvsel/smplsel value from the middle of the rectangle.\r\n"
                         "3. Set PcdSdmmcDrvSel and PcdSdmmcSmplSel in project .DSC file.\r\n\r\n<HALT>");
        // This is intended to generate the Report only
        // Not allow to continue booting.
        EFI_DEADLOOP();

      } else {
        // Normal flow
        InitializeMmcDevice (MmcHostInstance);
      }

    }
  }
}

EFI_STATUS
EFIAPI
MmcDriverBindingStart (
  VOID
  )
{
  PEI_SDMMC_INSTANCE*  MmcHostInstance;
  MmcHostInstance = CreateMmcHostInstance(&gSdMmcHost);
  if (MmcHostInstance != NULL) {
    MmcHostInstance->Initialized = FALSE;
    CheckCardsCallback ();
  }
  return EFI_SUCCESS;
}

EFI_STATUS
EFIAPI
MmcPeiInitialize (
  VOID
  )
{
  EFI_STATUS  Status;
  Status = MmcDriverBindingStart ();
  return Status;
}


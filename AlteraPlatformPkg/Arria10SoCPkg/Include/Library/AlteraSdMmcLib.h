/** @file
  Altera HPS SD/MMC controller Lib header file

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

  Copyright (c) 2011, ARM Limited. All rights reserved.

  This program and the accompanying materials
  are licensed and made available under the terms and conditions of the BSD License
  which accompanies this distribution.  The full text of the license may be found at
  http://opensource.org/licenses/bsd-license.php

  THE PROGRAM IS DISTRIBUTED UNDER THE BSD 3 Clause LICENSE ON AN "AS IS" BASIS,
  WITHOUT WARRANTIES OR REPRESENTATIONS OF ANY KIND, EITHER EXPRESS OR IMPLIED.

**/

#ifndef __ALTERA_SDMMC_LIB_H
#define __ALTERA_SDMMC_LIB_H

#include <Uefi.h>
#include <Library/BaseMemoryLib.h>
#include <Library/DebugLib.h>
#include <Library/IoLib.h>
#include <Library/PcdLib.h>
#include <Library/TimerLib.h>
#include <Library/UefiLib.h>
#include <Protocol/MmcHost.h>
#include <AlteraPlatform.h>

//
// Constant
//
// DELAY
#define TIME_DELAY_IN_MICRO_SECOND_FOR_CARD_POWER_TO_RAMP_UP (1000)

// TIMEOUT
#define DATA_BUSY_TIMEOUT      (10000000)
#define SEND_CMD_WAIT_TIMEOUT  (25000000)
#define DATA_TIMEOUT_VALUE     (0xFFFFFF)
#define RESPONSE_TIMEOUT_VALUE (0xFF)

// CLKDIV VALUE
// Clock division is 2*n. For example,
// value of 0 means divide by 2*0 = 0 (no division, bypass),
// value of 1 means divide by 2*1 = 2,
// value of “ff” means divide by 2*255 = 510, and so on.
// SD card is typically set at 25 MHz (0x01)
// MMC card is typically set at 12.5 MHz (0x02)
// Full-speed SDIO is set to 25 MHz (0x01)
// Low-speed SDIO is set to 400 KHz (0x3E)
#define ALT_SDMMC_CLKDIV_VALUE_FOR_DATA_MODE  0x01  // 0x00 = 50Mhz, 0x01 = ~25Mhz, 0x02 = ~12.5Mhz

//#define USE_DATA_MODE_CLOCK_DURING_ID_MODE 1

#ifdef USE_DATA_MODE_CLOCK_DURING_ID_MODE
// Faster card detection
// Standard card detection method suggest detecting the card with starting clock rate of around ~400 Khz
// But detecting card using around 400 Khz is very slow
// If the board only support only certain latest technology card, why not start with faster clock speed.
#define ALT_SDMMC_CLKDIV_VALUE_FOR_ID_MODE    ALT_SDMMC_CLKDIV_VALUE_FOR_DATA_MODE
#else
// value of 1 means divide by 2*1 = 2, value of “ff” means divide by 2*255 = 510, and so on.
// cclk_out = cclk_in / (clk_divider0*2)
// cclk_in = sdmmc_clk / 4 (typically is 50 MHz)
// sdmmc_clk = PLL0.C6 (typically at 200 MHz)
#define ALT_SDMMC_CLKDIV_VALUE_FOR_ID_MODE    0x02  // 0x3E = ~400Khz, 0x10 = ~1.56 Mhz, 0x01 == ~25Mhz, 0x02 == ~12.5Mhz
#endif

// DEBUG
#define DEBUG_MmioRead32(base, data32) DEBUG ((EFI_D_BLKIO, "%a - MmioRead32 ( 0x%08X ) = 0x%08X \n", __FUNCTION__, base, data32));
#define DEBUG_MmioWrite32(base, data32) DEBUG ((EFI_D_BLKIO, "%a - MmioWrite32 ( 0x%08X, 0x%08X )\n", __FUNCTION__, base, data32));

// Command used to switch to 4-bit Mode
#define mACMD6             (MMC_INDX(6) | MMC_CMD_WAIT_RESPONSE | MMC_CMD_NO_CRC_RESPONSE)
#define mRCA_SHIFT_OFFSET  16

typedef struct {
  UINT32    des0;
  UINT32    des1;
  UINT32    des2;
  UINT32    des3;
} IDMA_DES;

#define IDMAC_OWN    0x80000000
#define IDMAC_ER     0x00000020
#define IDMAC_CH     0x00000010
#define IDMAC_FS     0x00000008
#define IDMAC_LD     0x00000004
#define IDMAC_DIC    0x00000002

//
// Public functions
//
BOOLEAN
EFIAPI
IsCardPresent (
  VOID
  );

BOOLEAN
EFIAPI
IsWriteProtected (
  VOID
  );

EFI_STATUS
EFIAPI
InitializeSdMmc (
  VOID
  );

EFI_STATUS
EFIAPI
InitAfterEnumerateCardStack(
  VOID
  );

EFI_STATUS
EFIAPI
SendCommand (
  IN MMC_CMD MmcCmd,
  IN UINT32  CmdArgument
  );

EFI_STATUS
EFIAPI
ReceiveCommandResponse(
  IN MMC_RESPONSE_TYPE   Type,
  IN UINT32*             Buffer
  );

//
// Private functions
// (intended to be call within this file only)
//

EFI_STATUS
EFIAPI
InitOnboardSdmmcVoltageRegulator(
  VOID
  );

EFI_STATUS
EFIAPI
EnablePowerToTheCard(
  VOID
  );

EFI_STATUS
EFIAPI
InitSdmmcInterrupt(
  VOID
  );

EFI_STATUS
EFIAPI
InitBeforeEnumerateCardStack(
  VOID
  );

EFI_STATUS
EFIAPI
InitCommonRegisters(
  VOID
  );

BOOLEAN
EFIAPI
IgnoreCommand (
  IN MMC_CMD MmcCmd
  );

EFI_STATUS
EFIAPI
WaitUntilCardDataNotBusy(
  VOID
  );

EFI_STATUS
EFIAPI
ChangeCardClockFrequency(
  IN UINT32 clkdiv
  );

EFI_STATUS
EFIAPI
ChangeDataBusMode (
  VOID
  );

EFI_STATUS
EFIAPI
ReadFifoData (
  IN  UINTN     Length,
  OUT UINT32*   Buffer
  );

EFI_STATUS
EFIAPI
WriteFifoData (
  IN UINTN      Length,
  IN UINT32*    Buffer
  );

EFI_STATUS
EFIAPI
InitInternalDMAC(
  VOID
  );

EFI_STATUS
EFIAPI
ReadDmaData (
  IN  UINTN     Length,
  OUT UINT32*   Buffer
  );

#endif


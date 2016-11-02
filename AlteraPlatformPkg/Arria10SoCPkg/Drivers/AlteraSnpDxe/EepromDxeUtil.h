/*****************************************************************************
 *
 * Copyright (c) 2016, Intel Corporation. All Rights Reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors
 * may be used to endorse or promote products derived from this software without
 * specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 *****************************************************************************/


#ifndef __EEPROM_DXE_UTIL_H__
#define __EEPROM_DXE_UTIL_H__


EFI_STATUS
EFIAPI
I2cInit (
  VOID
  );

EFI_STATUS
EFIAPI
PollingRxFifoNotEmpty (
  VOID
  );

EFI_STATUS
EFIAPI
PollingTxFifoNotEmpty (
  VOID
  );

EFI_STATUS
EFIAPI
ClearRxFifo (
  VOID
  );

EFI_STATUS
EFIAPI
SetAddress (
  IN UINT32 Address,
  IN UINT32 AddrLength
  );

EFI_STATUS
EFIAPI
PollingForBusBusy (
  VOID
  );

EFI_STATUS
EFIAPI
StopDataTransfer (
  VOID
  );

EFI_STATUS
EFIAPI
EepromReadByte (
  IN UINT16 AddrLength,
  IN UINT16 Address,
  IN UINT8 BufferLength,
  OUT UINT8  *Buffer
  );

EFI_STATUS
EFIAPI
EepromWriteByte (
  IN UINT8 AddrLength,
  IN UINT16 Address,
  IN UINT8 BufferLength,
  IN UINT8* Buffer
  );

EFI_STATUS
EFIAPI 
EepromMacRead (
  OUT EFI_MAC_ADDRESS  *MacAddress 
  );  
  
#endif // __EEPROM_DXE_UTIL_H__


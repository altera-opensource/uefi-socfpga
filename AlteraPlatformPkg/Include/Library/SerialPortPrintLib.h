/** @file

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

#ifndef __SERIALPORTPRINTLIB_H__
#define __SERIALPORTPRINTLIB_H__

// Max String Buffer Size
#define MAX_PRINT_SIZE 2560

// ==================================================================
// Functions Definition
// ==================================================================

//
// Public Functions
//
VOID
EFIAPI
SerialPortInit (
  VOID
  );

VOID
EFIAPI
SerialPortPrint (
  IN  CONST CHAR8  *FormatString,
  ...
  );

VOID
EFIAPI
SerialPortMmioHexDump(
  IN  UINTN  StartAddress,
  IN  UINTN  Data32Size
);

VOID
EFIAPI
SerialPortMmioHexDumpEx(
  IN  UINTN   StartAddress,
  IN  UINTN   Data32Size,
  IN  UINTN   PrintLineStartAddress
);

VOID
EFIAPI
PlatformInitAssertionPrint (
  IN CONST CHAR8  *FormatString,
  ...
  );

VOID
EFIAPI
MemorySerialLogWrite (
  IN UINT8     *Buffer,
  IN UINTN     NumberOfBytes
  );

#endif

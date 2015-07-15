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
#include <Uefi.h>
#include <Library/PcdLib.h>
#include <Library/PrintLib.h>
#include <Library/SerialPortLib.h>
#include <Library/SerialPortPrintLib.h>
#include <Library/IoLib.h>
#include <Library/SemihostLib.h>

VOID
EFIAPI
SerialPortInit (
  VOID
  )
{
  // Call the actual function that do the UART init from SerialPortLib
  SerialPortInitialize ();
}

VOID
EFIAPI
SerialPortPrint (
  IN  CONST CHAR8  *FormatString,
  ...
  )
{
  CHAR8   Buffer[MAX_PRINT_SIZE];
  VA_LIST Marker;
  UINTN   NumberOfPrinted;

  VA_START (Marker, FormatString);
  NumberOfPrinted = AsciiVSPrint (Buffer, sizeof(Buffer), FormatString, Marker);
  VA_END (Marker);

  MemorySerialLogWrite ((UINT8 *) Buffer, NumberOfPrinted);

  // Is Serial Port Ready ?
  #define R_UART_LCR  (3 * 4)
  if (PcdGet64 (PcdSerialRegisterBase) != 0) // Have serial port?
  {
    if ((MmioRead32 (PcdGet64 (PcdSerialRegisterBase) + R_UART_LCR) & 0x3F) == (PcdGet8 (PcdSerialLineControl) & 0x3F)) {
      SerialPortWrite ((UINT8 *) Buffer, NumberOfPrinted);
    }
  }

  // Is SemiHosting Supported by this firmware ?
  if (PcdGet32 (PcdEnableSemihosting) != 0)
  {
    // Print to SemiHosting App Console in DS-5
    SemihostWriteString (Buffer);
  }
}


VOID
EFIAPI
MemorySerialLogWrite (
  IN UINT8     *Buffer,
  IN UINTN     NumberOfBytes
  )
{
  UINTN  MemorySerialLogBase;
  UINTN  MemorySerialLogSize;
  STATIC UINTN  Offset = 0;
  UINT8* Data8Ptr;
  CONST  CHAR8 EndMarker[] = "\r\n<<<LOG TAIL>>>";
  UINTN  EndMarkerSize;
  UINTN  EndMarkerOffset;
  UINT8* EndMarkerBuffer;
  UINTN  LocalCopyOfNumberOfBytes;

  MemorySerialLogBase = (UINTN)PcdGet64 (PcdMemorySerialLogBase);
  MemorySerialLogSize = (UINTN)PcdGet64 (PcdMemorySerialLogSize);

  if (MemorySerialLogBase ==0) {
    return;
  }
  if (MemorySerialLogSize <= sizeof(EndMarker)) {
    return;
  }
  if (NumberOfBytes == 0) {
    return;
  }
  if (Buffer == NULL) {
    return;
  }

  //
  // Write byte to the log buffer.
  //
  LocalCopyOfNumberOfBytes = NumberOfBytes;
  while (LocalCopyOfNumberOfBytes != 0) {
    // Address Roll over?
    if (Offset >= MemorySerialLogSize)
    {
      Offset = 0;
    }
    Data8Ptr = (UINT8*)(MemorySerialLogBase + Offset);
    *Data8Ptr = *Buffer;
    LocalCopyOfNumberOfBytes--;
    Offset++;
    Buffer++;
  }

  // Write maker for end of log
  // Handy for reading the log after roll over
  EndMarkerSize = sizeof(EndMarker);
  EndMarkerOffset = Offset;
  EndMarkerBuffer = (UINT8*) &EndMarker[0];
  while (EndMarkerSize != 0) {
    // Address Roll over?
    if (EndMarkerOffset >= MemorySerialLogSize)
    {
      EndMarkerOffset = 0;
    }
    Data8Ptr = (UINT8*)(MemorySerialLogBase + EndMarkerOffset);
    //
    // Write byte to the mark end of log.
    //
    *Data8Ptr = *EndMarkerBuffer;
    EndMarkerSize--;
    EndMarkerOffset++;
    EndMarkerBuffer++;
  }
}

VOID
EFIAPI
SerialPortMmioHexDump(
  IN  UINTN   StartAddress,
  IN  UINTN   Data32Size
)
{
  SerialPortMmioHexDumpEx(StartAddress, Data32Size, StartAddress);
}

VOID
EFIAPI
SerialPortMmioHexDumpEx(
  IN  UINTN   StartAddress,
  IN  UINTN   Data32Size,
  IN  UINTN   PrintLineStartAddress
)
{
  UINTN   LineCount;
  UINTN   Remainder;
  UINTN   ByteCount;
  UINTN   ThisLine;
  UINTN   ThisByte;
  UINTN   ByteInThisLine;
  UINT32  Data32;
  UINT8*  Data8Ptr;
  CHAR8*  AsciiDumpStrPtr;
  CHAR8   AsciiDumpStr[20];
  CHAR8   HexPartStr[4][13];

  // Char8Str is use to speed up Semihosting printing speed
  CHAR8   Char8Str[1024];
  CHAR8*  Char8Ptr = &Char8Str[0];

  ByteCount = (Data32Size * 4);
  LineCount = ByteCount / 16;
  Remainder = ByteCount - (LineCount * 16);

  for (ThisLine = 0; ThisLine <= LineCount; ThisLine++)
  {
    if (ThisLine == LineCount)
    {
      ByteInThisLine = Remainder;
    } else {
      ByteInThisLine = 16;
    }

    if (ByteInThisLine == 0)
    {
      break;
    }

    // Init variables
    HexPartStr[0][0] = 0;
    HexPartStr[1][0] = 0;
    HexPartStr[2][0] = 0;
    HexPartStr[3][0] = 0;

    // Print Hex Dump for this line
    for (ThisByte = 0; ThisByte < ByteInThisLine; ThisByte += 4)
    {
      Data32 = *(UINT32*)(StartAddress + ThisByte + (ThisLine * 16));
      Data8Ptr = (UINT8*)(&Data32);
      AsciiSPrint (&(HexPartStr[ThisByte/4][0]), 13,
                      "%02X %02X %02X %02X ",
                      *(Data8Ptr + 0),
                      *(Data8Ptr + 1),
                      *(Data8Ptr + 2),
                      *(Data8Ptr + 3));
    }

    // Print ASCII printable Char for this line
    for (ThisByte = 0; ThisByte < ByteInThisLine; ThisByte += 4)
    {
      Data32 = *(UINT32*)(StartAddress + ThisByte + (ThisLine * 16));
      AsciiDumpStrPtr = (CHAR8*)(&Data32);
      for (ByteCount = 0; ByteCount < 4; ByteCount++)
      {
        if ((*(AsciiDumpStrPtr + ByteCount) >= 0x20) &&
            (*(AsciiDumpStrPtr + ByteCount) <= 0x7E)) {
          AsciiDumpStr[ThisByte + ByteCount] = *(AsciiDumpStrPtr + ByteCount);
        } else {
          AsciiDumpStr[ThisByte + ByteCount] = '.';
        }
        AsciiDumpStr[ThisByte + ByteCount + 1] = 0;
      }
    }

    // Print to output with new line
    Char8Ptr += AsciiSPrint (Char8Ptr, 1024,
      "%08X: %a%a%a%a\t%a\r\n",
       PrintLineStartAddress + (ThisLine * 16),
       HexPartStr[0],
       HexPartStr[1],
       HexPartStr[2],
       HexPartStr[3],
       AsciiDumpStr);

    // Flush the string buffer to serial port if it is going to be full
    if (((UINTN)Char8Ptr + 80) > ((UINTN)&Char8Str[0] + sizeof(Char8Str)))
    {
      SerialPortPrint ("%a", Char8Str); Char8Ptr = &Char8Str[0];
      Char8Str[0] = 0;
    }
  }

  // Print the remaining message in string buffer
  SerialPortPrint ("%a", Char8Str); Char8Ptr = &Char8Str[0];
}


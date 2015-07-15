/** @file
  Default exception handler

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

  Copyright (c) 2008 - 2010, Apple Inc. All rights reserved.<BR>
  Copyright (c) 2012, ARM Ltd. All rights reserved.<BR>

  This program and the accompanying materials
  are licensed and made available under the terms and conditions of the BSD License
  which accompanies this distribution.  The full text of the license may be found at
  http://opensource.org/licenses/bsd-license.php

  THE PROGRAM IS DISTRIBUTED UNDER THE BSD 3 CLAUSE LICENSE ON AN "AS IS" BASIS,
  WITHOUT WARRANTIES OR REPRESENTATIONS OF ANY KIND, EITHER EXPRESS OR IMPLIED.

**/

#include <Uefi.h>
#include <Library/BaseLib.h>
#include <Library/DebugLib.h>
#include <Library/PeCoffGetEntryPointLib.h>
#include <Library/PrintLib.h>
#include <Library/ArmDisassemblerLib.h>
#include <Library/SerialPortLib.h>
#include <Library/SerialPortPrintLib.h>
#include <Guid/DebugImageInfoTable.h>

#include <Protocol/DebugSupport.h>
#include <Library/DefaultExceptionHandlerLib.h>

EFI_DEBUG_IMAGE_INFO_TABLE_HEADER *gDebugImageTableHeader = NULL;

typedef struct {
  UINT32  BIT;
  CHAR8   Char;
} CPSR_CHAR;

CHAR8 *
GetImageName (
  IN  UINTN  FaultAddress,
  OUT UINTN  *ImageBase,
  OUT UINTN  *PeCoffSizeOfHeaders
  );

/**
  Convert the Current Program Status Register (CPSR) to a string. The string is
  a defacto standard in the ARM world.

  It is possible to add extra bits by adding them to CpsrChar array.

  @param  Cpsr         ARM CPSR register value
  @param  ReturnStr    32 byte string that contains string version of CPSR

**/
VOID
CpsrString (
  IN  UINT32  Cpsr,
  OUT CHAR8   *ReturnStr
  )
{
  UINTN     Index;
  CHAR8*    Str;
  CHAR8*    ModeStr;
  CPSR_CHAR CpsrChar[] = {
    { 31, 'n' },
    { 30, 'z' },
    { 29, 'c' },
    { 28, 'v' },

    { 9,  'e' },
    { 8,  'a' },
    { 7,  'i' },
    { 6,  'f' },
    { 5,  't' },
    { 0,  '?' }
  };

  Str = ReturnStr;

  for (Index = 0; CpsrChar[Index].BIT != 0; Index++, Str++) {
    *Str = CpsrChar[Index].Char;
    if ((Cpsr & (1 << CpsrChar[Index].BIT)) != 0) {
      // Concert to upper case if bit is set
      *Str &= ~0x20;
    }
  }

  *Str++ = '_';
  *Str = '\0';

  switch (Cpsr & 0x1f) {
  case 0x10:
    ModeStr = "usr";
    break;
  case 0x011:
    ModeStr = "fiq";
    break;
  case 0x12:
    ModeStr = "irq";
    break;
  case 0x13:
    ModeStr = "svc";
    break;
  case 0x16:
    ModeStr = "mon";
    break;
  case 0x17:
    ModeStr = "abt";
    break;
  case 0x1b:
    ModeStr = "und";
    break;
  case 0x1f:
    ModeStr = "sys";
    break;

  default:
    ModeStr = "???";
    break;
  }

  AsciiStrCat (Str, ModeStr);
  return;
}

CHAR8 *
FaultStatusToString (
  IN  UINT32  Status
  )
{
  CHAR8 *FaultSource;

  switch (Status) {
    case 0x01: FaultSource = "Alignment fault"; break;
    case 0x02: FaultSource = "Debug event fault"; break;
    case 0x03: FaultSource = "Access Flag fault on Section"; break;
    case 0x04: FaultSource = "Cache maintenance operation fault[2]"; break;
    case 0x05: FaultSource = "Translation fault on Section"; break;
    case 0x06: FaultSource = "Access Flag fault on Page"; break;
    case 0x07: FaultSource = "Translation fault on Page"; break;
    case 0x08: FaultSource = "Precise External Abort"; break;
    case 0x09: FaultSource = "Domain fault on Section"; break;
    case 0x0b: FaultSource = "Domain fault on Page"; break;
    case 0x0c: FaultSource = "External abort on translation, first level"; break;
    case 0x0d: FaultSource = "Permission fault on Section"; break;
    case 0x0e: FaultSource = "External abort on translation, second level"; break;
    case 0x0f: FaultSource = "Permission fault on Page"; break;
    case 0x16: FaultSource = "Imprecise External Abort"; break;
    default:   FaultSource = "No function"; break;
    }

  return FaultSource;
}

STATIC CHAR8 *gExceptionTypeString[] = {
  "Reset",
  "Undefined OpCode",
  "SVC",
  "Prefetch Abort",
  "Data Abort",
  "Undefined",
  "IRQ",
  "FIQ"
};

/**
  This is the default action to take on an unexpected exception

  Since this is exception context don't do anything crazy like try to allcoate memory.

  @param  ExceptionType    Type of the exception
  @param  SystemContext    Register state at the time of the Exception


**/
VOID
DefaultExceptionHandler (
  IN     EFI_EXCEPTION_TYPE           ExceptionType,
  IN OUT EFI_SYSTEM_CONTEXT           SystemContext
  )
{
  CHAR8     Buffer[500];
  UINTN     CharCount;
  UINT32    DfsrStatus;
  UINT32    IfsrStatus;
  BOOLEAN   DfsrWrite;
  UINT32    PcAdjust = 0;
  CHAR8     *Pdb;
  UINT32    ImageBase;
  UINT32    PeCoffSizeOfHeader;
  UINT32    Offset;
  CHAR8     CpsrStr[32];  // char per bit. Lower 5-bits are mode that is a 3 char string
  UINT8     *DisAsm;
  UINT32    ItBlock;

  CharCount = AsciiSPrint (Buffer,sizeof (Buffer),"\n%a Exception PC at 0x%08x  CPSR 0x%08x ",
         gExceptionTypeString[ExceptionType], SystemContext.SystemContextArm->PC, SystemContext.SystemContextArm->CPSR);
  SerialPortPrint (Buffer, CharCount);

  CpsrString (SystemContext.SystemContextArm->CPSR, CpsrStr);
  SerialPortPrint ("%a\n", CpsrStr);

  Pdb = GetImageName (SystemContext.SystemContextArm->PC, &ImageBase, &PeCoffSizeOfHeader);
  Offset = SystemContext.SystemContextArm->PC - ImageBase;
  if (Pdb != NULL) {
    SerialPortPrint ("%a\n", Pdb);

    //
    // A PE/COFF image loads its headers into memory so the headers are
    // included in the linked addresses. ELF and Mach-O images do not
    // include the headers so the first byte of the image is usually
    // text (code). If you look at link maps from ELF or Mach-O images
    // you need to subtract out the size of the PE/COFF header to get
    // get the offset that matches the link map.
    //
    SerialPortPrint ("loaded at 0x%08x (PE/COFF offset) 0x%x (ELF or Mach-O offset) 0x%x", ImageBase, Offset, Offset - PeCoffSizeOfHeader);
    if ((*((UINT32*)(ImageBase)) == 0x00005A4D) && (*((UINT32*)(ImageBase + 0x80)) == 0x00004550))
    {
      // Found "MZ" and "PE" signature
      SerialPortPrint ("\nModule entry at 0x%08x", (*((UINT32*)(ImageBase + 0x80 + 0x34)) + *((UINT32*)(ImageBase + 0x80 + 0x28))));
    }
    // If we come from an image it is safe to show the instruction. We know it should not fault
    DisAsm = (UINT8 *)(UINTN)SystemContext.SystemContextArm->PC;
    ItBlock = 0;
    DisassembleInstruction (&DisAsm, (SystemContext.SystemContextArm->CPSR & BIT5) == BIT5, TRUE, &ItBlock, Buffer, sizeof (Buffer));
    SerialPortPrint ("\n%a", Buffer);

    switch (ExceptionType) {
    case EXCEPT_ARM_UNDEFINED_INSTRUCTION:
    case EXCEPT_ARM_SOFTWARE_INTERRUPT:
    case EXCEPT_ARM_PREFETCH_ABORT:
    case EXCEPT_ARM_DATA_ABORT:
      // advance PC past the faulting instruction
      PcAdjust = (UINTN)DisAsm - SystemContext.SystemContextArm->PC;
      break;

    default:
      break;
    }

  }

  SerialPortPrint ("\n  R0 0x%08x   R1 0x%08x   R2 0x%08x   R3 0x%08x\n", SystemContext.SystemContextArm->R0, SystemContext.SystemContextArm->R1, SystemContext.SystemContextArm->R2, SystemContext.SystemContextArm->R3);
  SerialPortPrint ("  R4 0x%08x   R5 0x%08x   R6 0x%08x   R7 0x%08x\n", SystemContext.SystemContextArm->R4, SystemContext.SystemContextArm->R5, SystemContext.SystemContextArm->R6, SystemContext.SystemContextArm->R7);
  SerialPortPrint ("  R8 0x%08x   R9 0x%08x  R10 0x%08x  R11 0x%08x\n", SystemContext.SystemContextArm->R8, SystemContext.SystemContextArm->R9, SystemContext.SystemContextArm->R10, SystemContext.SystemContextArm->R11);
  SerialPortPrint (" R12 0x%08x   SP 0x%08x   LR 0x%08x   PC 0x%08x\n", SystemContext.SystemContextArm->R12, SystemContext.SystemContextArm->SP, SystemContext.SystemContextArm->LR, SystemContext.SystemContextArm->PC);
  SerialPortPrint ("DFSR 0x%08x DFAR 0x%08x IFSR 0x%08x IFAR 0x%08x\n", SystemContext.SystemContextArm->DFSR, SystemContext.SystemContextArm->DFAR, SystemContext.SystemContextArm->IFSR, SystemContext.SystemContextArm->IFAR);

  // Bit10 is Status[4] Bit3:0 is Status[3:0]
  DfsrStatus = (SystemContext.SystemContextArm->DFSR & 0xf) | ((SystemContext.SystemContextArm->DFSR >> 6) & 0x10);
  DfsrWrite = (SystemContext.SystemContextArm->DFSR & BIT11) != 0;
  if (DfsrStatus != 0x00) {
    SerialPortPrint (" %a: %a 0x%08x\n", FaultStatusToString (DfsrStatus), DfsrWrite ? "write to" : "read from", SystemContext.SystemContextArm->DFAR);
  }

  IfsrStatus = (SystemContext.SystemContextArm->IFSR & 0xf) | ((SystemContext.SystemContextArm->IFSR >> 6) & 0x10);
  if (IfsrStatus != 0) {
    SerialPortPrint (" Instruction %a at 0x%08x\n", FaultStatusToString (SystemContext.SystemContextArm->IFSR & 0xf), SystemContext.SystemContextArm->IFAR);
  }

  SerialPortPrint ("\n");
  ASSERT (FALSE);
  while(ExceptionType);

  // Clear the error registers that we have already displayed incase some one wants to keep going
  SystemContext.SystemContextArm->DFSR = 0;
  SystemContext.SystemContextArm->IFSR = 0;

  // If some one is stepping past the exception handler adjust the PC to point to the next instruction
  SystemContext.SystemContextArm->PC += PcAdjust;
}

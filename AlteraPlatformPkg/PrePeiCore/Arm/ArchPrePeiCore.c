/** @file
  Main file supporting the transition to PEI Core in Normal World for Versatile Express

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

  Copyright (c) 2012, ARM Limited. All rights reserved.

  This program and the accompanying materials
  are licensed and made available under the terms and conditions of the BSD License
  which accompanies this distribution.  The full text of the license may be found at
  http://opensource.org/licenses/bsd-license.php

  THE PROGRAM IS DISTRIBUTED UNDER THE BSD 3 CLAUSE LICENSE ON AN "AS IS" BASIS,
  WITHOUT WARRANTIES OR REPRESENTATIONS OF ANY KIND, EITHER EXPRESS OR IMPLIED.

**/

#include <Library/PrintLib.h>
#include <Library/SerialPortLib.h>
#include <Library/SerialPortPrintLib.h>
#include <Protocol/DebugSupport.h>
#include "PrePeiCore.h"

// This enables your firmware supporting Semihosting SYS_WRITE0 (print string only) to still work when Semihosting is disabled
// Good during development/debug:
//   1) no need to recompile two firmware semihosting and none semihosting
//   2) can turn off "set semihosting enabled false" at then start to speed up boot
//      and then suddently "set semihosting enabled true" when it reached place you want to debug
// Remember to turn off Semihosting for production,
// due to minor performance hit from CPU content switching SWI invoke by SVC opcode
#define ENABLE_SEMIHOSTING_SWI_SYS_WRITE0_TO_RETURN 1

// Software Interrupt operation numbers used by ARM.
#define SWI_SYS_OPEN                         0x01
#define SWI_SYS_CLOSE                        0x02
#define SWI_SYS_WRITEC                       0x03
#define SWI_SYS_WRITE0                       0x04
#define SWI_SYS_WRITE                        0x05
#define SWI_SYS_READ                         0x06
#define SWI_SYS_READC                        0x07
#define SWI_SYS_ISERROR                      0x08
#define SWI_SYS_ISTTY                        0x09
#define SWI_SYS_SEEK                         0x0A
#define SWI_SYS_FLEN                         0x0C
#define SWI_SYS_TMPNAM                       0x0D
#define SWI_SYS_REMOVE                       0x0E
#define SWI_SYS_RENAME                       0x0F
#define SWI_SYS_CLOCK                        0x10
#define SWI_SYS_TIME                         0x11
#define SWI_SYS_SYSTEM                       0x12
#define SWI_SYS_ERRNO                        0x13
#define SWI_SYS_GET_CMDLINE                  0x15
#define SWI_SYS_HEAPINFO                     0x16
#define SWI_angel_SWIreason_EnterSVC         0x17
#define SWI_angel_SWIreason_ReportException  0x18
#define SWI_SYS_ELAPSED                      0x30
#define SWI_SYS_TICKFREQ                     0x31

UINTN  mDeadLoop = 1;

VOID
EFIAPI
PeiCommonExceptionEntry (
  IN     EFI_EXCEPTION_TYPE  ExceptionType,
  IN OUT EFI_SYSTEM_CONTEXT  SystemContext
  )
{
  CHAR8           Buffer[500];
  UINTN           CharCount;

  switch (ExceptionType) {
  case 0:
    CharCount = AsciiSPrint (Buffer,sizeof (Buffer),"\r\nReset Exception at PC = 0x%08X\r\n",SystemContext.SystemContextArm->PC);
    break;
  case 1:
    CharCount = AsciiSPrint (Buffer,sizeof (Buffer),"\r\nUndefined Exception at PC = 0x%08X\r\n",SystemContext.SystemContextArm->PC);
    break;
  case 2:
    if ((SystemContext.SystemContextArm->R0 == SWI_angel_SWIreason_ReportException) ||
        (SystemContext.SystemContextArm->R1 == 0x20026)) // ADP_Stopped_ApplicationExit (0x20026)
    {
      // Baremetal Application would trigger  "SVC      #0xab" In _sys_exit
      CharCount = AsciiSPrint (Buffer,sizeof (Buffer),"\r\nExecution stopped at: S:0x%08X due to application reaching end of execution\r\n",SystemContext.SystemContextArm->PC);
    }
#ifdef ENABLE_SEMIHOSTING_SWI_SYS_WRITE0_TO_RETURN
    else if (SystemContext.SystemContextArm->R0 == SWI_SYS_WRITE0)
    {
      // Code reched here when firmware is build to print message to Semihosting App Console
      // but Semihosting is not enabled in debugger
      // however we want the interrupt exception to just return without going into a dead loop
      return;
    }
#endif
    else if (SystemContext.SystemContextArm->R0 <= 0x31)
    {
      CharCount = AsciiSPrint (Buffer,sizeof (Buffer),
          "Semihosting operations 0x%02x at PC = 0x%08X\r\n"
          "Please enable semihosting in DS-5 by typing the command:\r\n"
          "set semihosting enabled true\r\n"
          "and open DS5->Window->Show View->App Console\r\n"
          ,SystemContext.SystemContextArm->R0
          ,SystemContext.SystemContextArm->PC);
      // If we let it run till SerialPortPrint,
      // it might causes endless exception reentry if the print function uses semihosting
      // but memory log print is safe
      MemorySerialLogWrite ((UINT8 *) Buffer, CharCount);
      // We must Put CPU in Loop now
      while(mDeadLoop);
    }
    else
    {
      CharCount = AsciiSPrint (Buffer,sizeof (Buffer),"\r\nSWI Exception at PC = 0x%08X\r\n",SystemContext.SystemContextArm->PC);
    }
    break;
  case 3:
    CharCount = AsciiSPrint (Buffer,sizeof (Buffer),"\r\nPrefetchAbort Exception at PC = 0x%08X\r\n",SystemContext.SystemContextArm->PC);
    break;
  case 4:
    CharCount = AsciiSPrint (Buffer,sizeof (Buffer),
      "\r\n%a 0x%08X Data Abort Exception at PC = 0x%08X\r\n",
      ((SystemContext.SystemContextArm->DFSR & 0x800) == 0)? "Read from" : "Write to",
      SystemContext.SystemContextArm->DFAR,
      SystemContext.SystemContextArm->PC);
    break;
  case 5:
    CharCount = AsciiSPrint (Buffer,sizeof (Buffer),"\r\nReserved Exception at PC = 0x%08X\r\n",SystemContext.SystemContextArm->PC);
    break;
  case 6:
    CharCount = AsciiSPrint (Buffer,sizeof (Buffer),"\r\nIRQ Exception at PC = 0x%08X\r\n",SystemContext.SystemContextArm->PC);
    break;
  case 7:
    CharCount = AsciiSPrint (Buffer,sizeof (Buffer),"\r\nFIQ Exception at PC = 0x%08X\r\n",SystemContext.SystemContextArm->PC);
    break;
  default:
    CharCount = AsciiSPrint (Buffer,sizeof (Buffer),"\r\nUnknown Exception at PC = 0x%08X\r\n",SystemContext.SystemContextArm->PC);
    break;
  }
  //SerialPortWrite ((UINT8 *) Buffer, CharCount); // Via SerialPortLib
  SerialPortPrint (Buffer); // Via SerialPortPrintLib

  // Do not dump registers if Baremetal Application in _sys_exit
  if ((SystemContext.SystemContextArm->R0 == SWI_angel_SWIreason_ReportException) ||
      (SystemContext.SystemContextArm->R1 == 0x20026)) // ADP_Stopped_ApplicationExit (0x20026)
  {
    // Put CPU in Loop
    while(mDeadLoop);
  }

  // Dump all registers state
  CharCount = AsciiSPrint (Buffer,sizeof (Buffer),
    "\t R0 = 0x%08X" "\t R1 = 0x%08X"  "\t R2 = 0x%08X"  "\t R3 = 0x%08X\r\n"
    "\t R4 = 0x%08X" "\t R5 = 0x%08X"  "\t R6 = 0x%08X"  "\t R7 = 0x%08X\r\n"
    "\t R8 = 0x%08X" "\t R9 = 0x%08X"  "\t R10= 0x%08X"  "\t R11= 0x%08X\r\n"
    "\t R12= 0x%08X" "\t SP = 0x%08X"  "\t LR = 0x%08X"  "\tCPSR= 0x%08X\r\n"
    "\tDFSR= 0x%08X" "\tDFAR= 0x%08X"  "\tIFSR= 0x%08X"  "\tIFAR= 0x%08X\r\n",
    SystemContext.SystemContextArm->R0,
    SystemContext.SystemContextArm->R1,
    SystemContext.SystemContextArm->R2,
    SystemContext.SystemContextArm->R3,
    SystemContext.SystemContextArm->R4,
    SystemContext.SystemContextArm->R5,
    SystemContext.SystemContextArm->R6,
    SystemContext.SystemContextArm->R7,
    SystemContext.SystemContextArm->R8,
    SystemContext.SystemContextArm->R9,
    SystemContext.SystemContextArm->R10,
    SystemContext.SystemContextArm->R11,
    SystemContext.SystemContextArm->R12,
    SystemContext.SystemContextArm->SP,
    SystemContext.SystemContextArm->LR,
    SystemContext.SystemContextArm->CPSR,
    SystemContext.SystemContextArm->DFSR,
    SystemContext.SystemContextArm->DFAR,
    SystemContext.SystemContextArm->IFSR,
    SystemContext.SystemContextArm->IFAR
    );
  //SerialPortWrite ((UINT8 *) Buffer, CharCount); // Via SerialPortLib
  SerialPortPrint (Buffer); // Via SerialPortPrintLib

  // Put CPU in Loop
  while(mDeadLoop);

  return;
}


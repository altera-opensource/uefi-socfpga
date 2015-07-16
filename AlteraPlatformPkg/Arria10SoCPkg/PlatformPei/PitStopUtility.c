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

//
// Include files
//
#include <Library/BaseMemoryLib.h>
#include <Library/IoLib.h>
#include <Library/SerialPortLib.h>
#include <Library/SerialPortPrintLib.h>
#include <Library/TimerLib.h>
#include "AlteraSdMmcPei/AlteraSdMmcPei.h"
#include "Assert.h"
#include "Boot.h"
#include "BootSource.h"
#include "NandLib.h"
#include "NandBlockIo.h"
#include "PitStopUtility.h"
#include "QspiLib.h"
#include "RawBinaryFile.h"
#include "SdMmc.h"

#define CARRIAGE_RERTURN 13
#define NEWLINE_FEED     10
#define ESCAPE           27
#define DELETE           127
#define BACKSPACE           127
#define MmioHexDump     SerialPortMmioHexDump
//#define SerialPortScan  SemihostRead
#define SerialPortScan  SerialPortRead

VOID
EFIAPI
GetInputString (
  OUT  CHAR8*  ReadBuffer
  )
{
  CHAR8  Buffer[100];
  INTN   i;
  CHAR8  ReadData[100];
  CHAR8  Char;

  i = 0;
  do {
    SerialPortScan((UINT8*)Buffer,1);
    Char = ReadData[i++] = Buffer[0];
    SerialPortPrint("%c", Char);
    if (Char == BACKSPACE) {
      if (i > 1)
        i = i - 2; // minus backspace key and previous key
      else
        i = 0;
    }
  } while (Char != CARRIAGE_RERTURN && Char != NEWLINE_FEED );
  CopyMem(ReadBuffer, ReadData, i-1);
  *(ReadBuffer + i-1) = '\0';
  SerialPortPrint("\r\n");
}

UINTN
EFIAPI
GetInputThenConvert2Num (
  )
{
  UINTN Data;
  CHAR8  ReadData[100];

  GetInputString((CHAR8*)ReadData);
  Data = AsciiStrHexToUintn((CHAR8*)ReadData);
  return Data;
}
CHAR8* LastPos = 0;

VOID
StrSplit (
  IN CHAR8* Str,
  IN CHAR8 DeLim,
  OUT CHAR8* SplitString
  )
{
    if(!Str && !LastPos)
        return;

    if(!DeLim)
        return;

    if(Str) {
        LastPos = Str;
    }

    CHAR8* StrPtr = LastPos;
    INTN i, Count = 0;

    while(*LastPos != '\0') {
        BOOLEAN IsFound = FALSE;

        if(DeLim == *LastPos) {
            IsFound = TRUE;
        }

        LastPos++;
        if(IsFound)
            break;
        Count++;

    }

    for(i = 0; i < Count; i++) {
        SplitString[i] = *(StrPtr + i);
    }

    SplitString[Count] = '\0';
    if(*LastPos == '\0')
        LastPos = 0;
}


BOOLEAN
IsEnterPitStop (
  VOID
  )
{
  UINTN    TimeOutCount;
  CHAR8    Input[100];

  TimeOutCount = PcdGet32(PcdEnablePitStopUtility);
  if (TimeOutCount == 0)
  {
    // Pit Stop utility disabled
    return FALSE;
  } else if (TimeOutCount == 1) {
    // Always enter Pit Stop utility
    return TRUE;
  } else {
    // Enter Pit Stop utility if key pressed before timeout
    SerialPortPrint ("\r\n");
    do {
      SerialPortPrint ("Press any key in %d seconds to enter Pit Stop utility\r", TimeOutCount);
      if (SerialPortPoll() == TRUE)
      {
        SerialPortScan((UINT8*)Input,1);
        break;
      }
      MicroSecondDelay (1000000);
    } while (--TimeOutCount != 0);
    if (TimeOutCount == 0) {
      // Timeout
      SerialPortPrint ("\rBooting...                                           \r\n");
      return FALSE;
    } else {
      // Enter Pit Stop
      SerialPortPrint ("\r\n");
    }
  }
  return TRUE;
}


VOID
PitStopCmdLine (
  VOID
  )
{
  CHAR8 Input[100];
  CHAR8 Argument[100][100];
  INTN ArgCnt = 0;
  UINTN  Data, Addr, FromAddr, ToAddr;
  UINTN Addr1, Addr2;
  CHAR8  TextFileName[100];
  UINTN  Len, Offset;
  UINTN  R0, R1, R2;
  EFI_STATUS Status;
  UINTN i;

  while(1) {
    ArgCnt = 0;

    // get command
    SerialPortPrint("SOCFPGA_ARRIA10 # ");
    GetInputString((CHAR8*)Input);

    // get input argument
    StrSplit((CHAR8*)Input, ' ', (CHAR8*)Argument[0]);
    while(LastPos != 0) {
      StrSplit(NULL, ' ',  (CHAR8*)Argument[++ArgCnt]);
    }

	//INTN i = 0;
    //SerialPortPrint("Number of argument is %d\r\n", ArgCnt);
    //for (i = 0; i <= ArgCnt; i++)
    //SerialPortPrint("%a\r\n", (CHAR16*)Argument[i]);
    //SerialPortPrint("Strlen of 1st argument is %d\r\n", AsciiStrLen(Argument[0]));
    //for (i = 0; i <= AsciiStrLen(Argument[0]); i++)
    //SerialPortPrint("%c", Argument[0][i]);
    //SerialPortPrint("\r\n");

    //memory display
    if (AsciiStrCmp((CHAR8*)Argument[0], "mr") == 0) {
      Addr = AsciiStrHexToUintn((CHAR8*)Argument[1]);
      Len  = (ArgCnt < 2)? 1: AsciiStrHexToUintn((CHAR8*)Argument[2]);
      if (Addr & 0x3) {
        SerialPortPrint ("Error: Unaligned address\r\n");
        return;
      }
      for (i = 0; i < Len; i++)
        SerialPortPrint("0x%08x\r\n", MmioRead32(Addr + i*4));

    //memory write
    } else if (AsciiStrCmp((CHAR8*)Argument[0], "mw") == 0) {
      Addr = AsciiStrHexToUintn((CHAR8*)Argument[1]);
      Data    = AsciiStrHexToUintn((CHAR8*)Argument[2]);
      Len  = (ArgCnt < 3)? 1: AsciiStrHexToUintn((CHAR8*)Argument[3]);

      if (Addr & 0x3) {
        SerialPortPrint ("Error: Unaligned address\r\n");
        return;
      }
      for (i = 0; i < Len; i++)
        MmioWrite32(Addr + i*4, Data);

    // memory compare
    } else if (AsciiStrCmp((CHAR8*)Argument[0], "mcmp") == 0) {
      Addr1 = AsciiStrHexToUintn((CHAR8*)Argument[1]);
      Addr2    = AsciiStrHexToUintn((CHAR8*)Argument[2]);
      Len  = (ArgCnt < 3)? 1: AsciiStrHexToUintn((CHAR8*)Argument[3]);

      if ((Addr1 & 0x3) || (Addr2 & 0x3)) {
        SerialPortPrint ("Error: Unaligned address\r\n");
        return;
      }
      for (i = 0; i < Len; i++) {
        if (MmioRead32(Addr1 + i*4) != MmioRead32(Addr2 + i*4)) {
          SerialPortPrint ("Different\r\n");
          return;
        }
      }
      SerialPortPrint ("Same\r\n");

    // SPI flash access
    } else if (AsciiStrCmp((CHAR8*)Argument[0], "qspi") == 0) {
      if (AsciiStrCmp((CHAR8*)Argument[1], "probe") == 0) {
        Status = QspiInit ();

      } else if (AsciiStrCmp((CHAR8*)Argument[1], "read") == 0) {
        if(ArgCnt != 4){
          SerialPortPrint ("Error: Invalid Arguments\r\n");
          return;
        }
        Addr   = AsciiStrHexToUintn((CHAR8*)Argument[2]);
        Offset = AsciiStrHexToUintn((CHAR8*)Argument[3]);
        Len = AsciiStrHexToUintn((CHAR8*)Argument[4]);

        Status = QspiRead((VOID *)Addr, Offset, Len);
        ASSERT_PLATFORM_INIT(!EFI_ERROR(Status));
        MmioHexDump(Addr, Len/4);

      } else if (AsciiStrCmp((CHAR8*)Argument[1], "write") == 0) {
        if(ArgCnt != 4){
          SerialPortPrint ("Error: Invalid Arguments\r\n");
          return;
        }
        Addr    = AsciiStrHexToUintn((CHAR8*)Argument[2]);
        Offset  = AsciiStrHexToUintn((CHAR8*)Argument[3]);
        Len     = AsciiStrHexToUintn((CHAR8*)Argument[4]);

        SerialPortPrint("Write Data Content is:\r\n");
        MmioHexDump(Addr, Len/4);

        Status = QspiWrite((VOID *)Addr, Offset, Len);
        ASSERT_PLATFORM_INIT(!EFI_ERROR(Status));

      } else if (AsciiStrCmp((CHAR8*)Argument[1], "erase") == 0) {
        if(ArgCnt != 3){
          SerialPortPrint ("Error: Invalid Arguments\r\n");
          return;
        }
        Offset  = AsciiStrHexToUintn((CHAR8*)Argument[2]);
        Len  = AsciiStrHexToUintn((CHAR8*)Argument[3]);

        Status = QspiErase (Offset, Len);
        ASSERT_PLATFORM_INIT(!EFI_ERROR(Status));

      } else if (AsciiStrCmp((CHAR8*)Argument[1], "update") == 0) {
        if(ArgCnt != 4){
          SerialPortPrint ("Error: Invalid Arguments\r\n");
          return;
        }
        Addr   = AsciiStrHexToUintn((CHAR8*)Argument[2]);
        Offset = AsciiStrHexToUintn((CHAR8*)Argument[3]);
        Len    = AsciiStrHexToUintn((CHAR8*)Argument[4]);

        Status = QspiUpdate((VOID *)Addr, Offset, Len);
        ASSERT_PLATFORM_INIT(!EFI_ERROR(Status));

      }
        // nand flash access
    } else if (AsciiStrCmp((CHAR8*)Argument[0], "nand") == 0) {
      if (AsciiStrCmp((CHAR8*)Argument[1], "probe") == 0) {
        Status = NandInit ();

      } else if (AsciiStrCmp((CHAR8*)Argument[1], "read") == 0) {
        if(ArgCnt != 4){
          SerialPortPrint ("Error: Invalid Arguments\r\n");
          return;
        }
        Addr   = AsciiStrHexToUintn((CHAR8*)Argument[2]);
        Offset = AsciiStrHexToUintn((CHAR8*)Argument[3]);
        Len = AsciiStrHexToUintn((CHAR8*)Argument[4]);

        Status = NandRead((VOID *)Addr, Offset, Len);
        ASSERT_PLATFORM_INIT(!EFI_ERROR(Status));
        MmioHexDump(Addr, Len/4);

      } else if (AsciiStrCmp((CHAR8*)Argument[1], "write") == 0) {
        if(ArgCnt != 4){
          SerialPortPrint ("Error: Invalid Arguments\r\n");
          return;
        }
        Addr    = AsciiStrHexToUintn((CHAR8*)Argument[2]);
        Offset  = AsciiStrHexToUintn((CHAR8*)Argument[3]);
        Len     = AsciiStrHexToUintn((CHAR8*)Argument[4]);

        SerialPortPrint("Write Data Content is:\r\n");
        MmioHexDump(Addr, Len/4);

        Status = NandWrite((VOID *)Addr, Offset, Len);
        ASSERT_PLATFORM_INIT(!EFI_ERROR(Status));

      } else if (AsciiStrCmp((CHAR8*)Argument[1], "erase") == 0) {
        if(ArgCnt != 3){
          SerialPortPrint ("Error: Invalid Arguments\r\n");
          return;
        }
        Offset  = AsciiStrHexToUintn((CHAR8*)Argument[2]);
        Len  = AsciiStrHexToUintn((CHAR8*)Argument[3]);

        Status = NandErase (Offset, Len);
        ASSERT_PLATFORM_INIT(!EFI_ERROR(Status));

      } else if (AsciiStrCmp((CHAR8*)Argument[1], "update") == 0) {
        if(ArgCnt != 4){
          SerialPortPrint ("Error: Invalid Arguments\r\n");
          return;
        }
        Addr   = AsciiStrHexToUintn((CHAR8*)Argument[2]);
        Offset = AsciiStrHexToUintn((CHAR8*)Argument[3]);
        Len    = AsciiStrHexToUintn((CHAR8*)Argument[4]);

        Status = NandUpdate((VOID *)Addr, Offset, Len);
        ASSERT_PLATFORM_INIT(!EFI_ERROR(Status));

      } else if (AsciiStrCmp((CHAR8*)Argument[1], "test") == 0) {
        if(ArgCnt != 5){
          SerialPortPrint ("Error: Invalid Arguments\r\n");
          return;
        }
        FromAddr   = AsciiStrHexToUintn((CHAR8*)Argument[2]);
        ToAddr   = AsciiStrHexToUintn((CHAR8*)Argument[3]);
        Offset = AsciiStrHexToUintn((CHAR8*)Argument[4]);
        Len    = AsciiStrHexToUintn((CHAR8*)Argument[5]);
		MmioHexDump(FromAddr, Len/4);
        Status = NandUpdate((VOID *)FromAddr, Offset, Len);
        ASSERT_PLATFORM_INIT(!EFI_ERROR(Status));

				Status = NandRead((VOID *)ToAddr, Offset, Len);
        ASSERT_PLATFORM_INIT(!EFI_ERROR(Status));
				MmioHexDump(ToAddr, Len/4);

      }
		} else if (AsciiStrCmp((CHAR8*)Argument[0], "ls") == 0) {
      if(ArgCnt != 0){
        SerialPortPrint ("Error: Invalid Arguments\r\n");
        return;
      }
      ListFileInRootDirectory ();

    // hexdump
    } else if (AsciiStrCmp((CHAR8*)Argument[0], "hexdump") == 0) {
      if(ArgCnt != 1){
        SerialPortPrint ("Error: Invalid Arguments\r\n");
        return;
      }
      AsciiStrCpy ((CHAR8*)TextFileName, (CHAR8*)Argument[1]);
      HexDumpFile((CHAR8*)TextFileName);

    // load file to memory
    } else if (AsciiStrCmp((CHAR8*)Argument[0], "fatload") == 0) {
      if(ArgCnt != 3){
        SerialPortPrint ("Error: Invalid Arguments\r\n");
        return;
      }
      if (AsciiStrCmp((CHAR8*)Argument[1], "mmc") == 0) {
        Addr = AsciiStrHexToUintn((CHAR8*)Argument[2]);
        AsciiStrCpy ((CHAR8*)TextFileName, (CHAR8*)Argument[3]);

        LoadFileToMemory(TextFileName, Addr, NULL);
      }
    // jump
    } else if (AsciiStrCmp((CHAR8*)Argument[0], "jump") == 0) {

      Addr = AsciiStrHexToUintn((CHAR8*)Argument[1]);
      R0 = (ArgCnt < 2)? 0: AsciiStrHexToUintn((CHAR8*)Argument[2]);
      R1 = (ArgCnt < 3)? 0: AsciiStrHexToUintn((CHAR8*)Argument[3]);
      R2 = (ArgCnt < 4)? 0: AsciiStrHexToUintn((CHAR8*)Argument[4]);

      JumpToEntry (Addr, R0, R1, R2 );

     // bootlinux
    }  else if (AsciiStrCmp((CHAR8*)Argument[0], "mmcboot") == 0) {
      if(ArgCnt != 0){
        SerialPortPrint ("Error: Invalid Arguments\r\n");
        return;
      }
      //SdmmcLoadLinuxZImageAndLinuxDtbToRamAndBoot();

    // help
    }  else if (AsciiStrCmp((CHAR8*)Argument[0], "help") == 0) {
      if(ArgCnt != 0){
        SerialPortPrint ("Error: Invalid Arguments\r\n");
        return;
      }
      SerialPortPrint ("\r\n");
      SerialPortPrint ("Usage:\r\n");
      SerialPortPrint ("------\r\n");
      SerialPortPrint ("mr addr [count]\r\n");
      SerialPortPrint ("mw addr data [count]\r\n");
      SerialPortPrint ("mcmp addr1 addr2 [count]\r\n");

      SerialPortPrint ("qspi probe\r\n");
      SerialPortPrint ("qspi read addr offset len\r\n");
      SerialPortPrint ("qspi write addr offset len\r\n");
      SerialPortPrint ("qspi erase offset len\r\n");
      SerialPortPrint ("qspi update addr offset len\r\n");

      SerialPortPrint ("nand probe\r\n");
      SerialPortPrint ("nand read addr offset len\r\n");
      SerialPortPrint ("nand write addr offset len\r\n");
      SerialPortPrint ("nand erase offset len\r\n");
      SerialPortPrint ("nand update addr offset len\r\n");
      SerialPortPrint ("nand test from_addr to_addr offset len\r\n");

      SerialPortPrint ("ls\r\n");
      SerialPortPrint ("hexdump file \r\n");
      SerialPortPrint ("fatload mmc addr file\r\n");

      SerialPortPrint ("jump addr [r0] [r1] [r2]\r\n");
      SerialPortPrint ("mmcboot\r\n");
      SerialPortPrint ("\r\n");

    } else {
      SerialPortPrint("Invalid command\r\n");
      SerialPortPrint("Please type help for more information\r\n");
    }

  }

}


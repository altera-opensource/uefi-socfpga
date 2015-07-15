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

  Copyright (c) 2008 - 2009, Apple Inc. All rights reserved.<BR>
  Copyright (c) 2013 - 2014, ARM Ltd. All rights reserved.<BR>

  This program and the accompanying materials
  are licensed and made available under the terms and conditions of the BSD License
  which accompanies this distribution.  The full text of the license may be found at
  http://opensource.org/licenses/bsd-license.php

  THE PROGRAM IS DISTRIBUTED UNDER THE BSD 3 CLAUSE LICENSE ON AN "AS IS" BASIS,
  WITHOUT WARRANTIES OR REPRESENTATIONS OF ANY KIND, EITHER EXPRESS OR IMPLIED.

**/

#ifndef __SEMIHOST_PRIVATE_H__
#define __SEMIHOST_PRIVATE_H__

typedef struct {
  CHAR8   *FileName;
  UINTN    Mode;
  UINTN    NameLength;
} SEMIHOST_FILE_OPEN_BLOCK;

typedef struct {
  UINTN    Handle;
  VOID    *Buffer;
  UINTN    Length;
} SEMIHOST_FILE_READ_WRITE_BLOCK;

typedef struct {
  UINTN    Handle;
  UINTN    Location;
} SEMIHOST_FILE_SEEK_BLOCK;

typedef struct {
  CHAR8   *FileName;
  UINTN    NameLength;
} SEMIHOST_FILE_REMOVE_BLOCK;

typedef struct {
  CHAR8   *CommandLine;
  UINTN    CommandLength;
} SEMIHOST_SYSTEM_BLOCK;

#if defined(__CC_ARM)

#if defined(__thumb__)
#define SWI 0xAB
#else
#define SWI 0x123456
#endif

#define SEMIHOST_SUPPORTED  TRUE

// This implementation allow return from SVC call
UINT32
ArmccSemihostCall (
  IN UINT32   Operation,
  IN UINTN    SystemBlockAddress
  ); // __attribute__ ((interrupt ("SVC")));

#define Semihost_SYS_OPEN(OpenBlock)        ArmccSemihostCall(0x01, (UINTN)(OpenBlock))
#define Semihost_SYS_CLOSE(Handle)          ArmccSemihostCall(0x02, (UINTN)(Handle))
#define Semihost_SYS_WRITE0(String)         ArmccSemihostCall(0x04, (UINTN)(String))
#define Semihost_SYS_WRITEC(Character)      ArmccSemihostCall(0x03, (UINTN)(Character))
#define Semihost_SYS_WRITE(WriteBlock)      ArmccSemihostCall(0x05, (UINTN)(WriteBlock))
#define Semihost_SYS_READ(ReadBlock)        ArmccSemihostCall(0x06, (UINTN)(ReadBlock))
#define Semihost_SYS_READC()                ArmccSemihostCall(0x07, (UINTN)(0))
#define Semihost_SYS_SEEK(SeekBlock)        ArmccSemihostCall(0x0A, (UINTN)(SeekBlock))
#define Semihost_SYS_FLEN(Handle)           ArmccSemihostCall(0x0C, (UINTN)(Handle))
#define Semihost_SYS_REMOVE(RemoveBlock)    ArmccSemihostCall(0x0E, (UINTN)(RemoveBlock))
#define Semihost_SYS_SYSTEM(SystemBlock)    ArmccSemihostCall(0x12, (UINTN)(SystemBlock))

#elif defined(__GNUC__) // __CC_ARM

#define SEMIHOST_SUPPORTED  TRUE

UINT32
GccSemihostCall (
  IN UINT32   Operation,
  IN UINTN    SystemBlockAddress
  ); // __attribute__ ((interrupt ("SVC")));

#define Semihost_SYS_OPEN(OpenBlock)        GccSemihostCall(0x01, (UINTN)(OpenBlock))
#define Semihost_SYS_CLOSE(Handle)          GccSemihostCall(0x02, (UINTN)(Handle))
#define Semihost_SYS_WRITE0(String)         GccSemihostCall(0x04, (UINTN)(String))
#define Semihost_SYS_WRITEC(Character)      GccSemihostCall(0x03, (UINTN)(Character))
#define Semihost_SYS_WRITE(WriteBlock)      GccSemihostCall(0x05, (UINTN)(WriteBlock))
#define Semihost_SYS_READ(ReadBlock)        GccSemihostCall(0x06, (UINTN)(ReadBlock))
#define Semihost_SYS_READC()                GccSemihostCall(0x07, (UINTN)(0))
#define Semihost_SYS_SEEK(SeekBlock)        GccSemihostCall(0x0A, (UINTN)(SeekBlock))
#define Semihost_SYS_FLEN(Handle)           GccSemihostCall(0x0C, (UINTN)(Handle))
#define Semihost_SYS_REMOVE(RemoveBlock)    GccSemihostCall(0x0E, (UINTN)(RemoveBlock))
#define Semihost_SYS_SYSTEM(SystemBlock)    GccSemihostCall(0x12, (UINTN)(SystemBlock))

#else // __CC_ARM

#define SEMIHOST_SUPPORTED  FALSE

#define Semihost_SYS_OPEN(OpenBlock)        (-1)
#define Semihost_SYS_CLOSE(Handle)          (-1)
#define Semihost_SYS_WRITE0(String)
#define Semihost_SYS_WRITEC(Character)
#define Semihost_SYS_WRITE(WriteBlock)      (0)
#define Semihost_SYS_READ(ReadBlock)        ((ReadBlock)->Length)
#define Semihost_SYS_READC()                ('x')
#define Semihost_SYS_SEEK(SeekBlock)        (-1)
#define Semihost_SYS_FLEN(Handle)           (-1)
#define Semihost_SYS_REMOVE(RemoveBlock)    (-1)
#define Semihost_SYS_SYSTEM(SystemBlock)    (-1)

#endif // __CC_ARM

#endif //__SEMIHOST_PRIVATE_H__

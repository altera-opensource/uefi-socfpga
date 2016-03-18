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

  Copyright (c) 2011-2015, ARM Limited. All rights reserved.

  This program and the accompanying materials
  are licensed and made available under the terms and conditions of the BSD License
  which accompanies this distribution.  The full text of the license may be found at
  http://opensource.org/licenses/bsd-license.php

  THE PROGRAM IS DISTRIBUTED UNDER THE BSD 3 CLAUSE LICENSE ON AN "AS IS" BASIS,
  WITHOUT WARRANTIES OR REPRESENTATIONS OF ANY KIND, EITHER EXPRESS OR IMPLIED.

**/


#include <PiDxe.h>
#include <Library/HobLib.h>
#include <Library/TimerLib.h>
#include <Library/SerialPortLib.h>

#include "LinuxLoader.h"

STATIC CONST CHAR8 *mTokenList[] = {
  /*"SEC",*/
  "PEI",
  "DXE",
  "BDS",
  NULL
};

VOID
PrintPerformance (
  VOID
  )
{
  UINTN       Key;
  CONST VOID  *Handle;
  CONST CHAR8 *Token, *Module;
  UINT64      Start, Stop, TimeStamp;
  UINT64      Delta, TicksPerSecond, Milliseconds;
  UINTN       Index;
  CHAR8       Buffer[100];
  UINTN       CharCount;
  BOOLEAN     CountUp;

  TicksPerSecond = GetPerformanceCounterProperties (&Start, &Stop);
  if (Start < Stop) {
    CountUp = TRUE;
  } else {
    CountUp = FALSE;
  }

  TimeStamp = 0;
  Key       = 0;
  do {
    Key = GetPerformanceMeasurement (Key, (CONST VOID **)&Handle, &Token, &Module, &Start, &Stop);
    if (Key != 0) {
      for (Index = 0; mTokenList[Index] != NULL; Index++) {
        if (AsciiStriCmp (mTokenList[Index], Token) == 0) {
          Delta = CountUp ? (Stop - Start) : (Start - Stop);
          TimeStamp += Delta;
          Milliseconds = DivU64x64Remainder (MultU64x32 (Delta, 1000), TicksPerSecond, NULL);
          CharCount = AsciiSPrint (Buffer, sizeof (Buffer), "%6a %6ld ms\n", Token, Milliseconds);
          SerialPortWrite ((UINT8 *) Buffer, CharCount);
          break;
        }
      }
    }
  } while (Key != 0);

  CharCount = AsciiSPrint (Buffer, sizeof (Buffer), "Total Time = %ld ms\n\n",
      DivU64x64Remainder (MultU64x32 (TimeStamp, 1000), TicksPerSecond, NULL));
  SerialPortWrite ((UINT8 *) Buffer, CharCount);
}

STATIC
EFI_STATUS
InsertSystemMemoryResources (
  LIST_ENTRY *ResourceList,
  EFI_HOB_RESOURCE_DESCRIPTOR *ResHob
  )
{
  SYSTEM_MEMORY_RESOURCE  *NewResource;
  LIST_ENTRY              *Link;
  LIST_ENTRY              *NextLink;
  LIST_ENTRY              AttachedResources;
  SYSTEM_MEMORY_RESOURCE  *Resource;
  EFI_PHYSICAL_ADDRESS    NewResourceEnd;

  if (IsListEmpty (ResourceList)) {
    NewResource = AllocateZeroPool (sizeof (SYSTEM_MEMORY_RESOURCE));
    NewResource->PhysicalStart = ResHob->PhysicalStart;
    NewResource->ResourceLength = ResHob->ResourceLength;
    InsertTailList (ResourceList, &NewResource->Link);
    return EFI_SUCCESS;
  }

  InitializeListHead (&AttachedResources);

  Link = ResourceList->ForwardLink;
  ASSERT (Link != NULL);
  while (Link != ResourceList) {
    Resource = (SYSTEM_MEMORY_RESOURCE*)Link;

    // Sanity Check. The resources should not overlapped.
    ASSERT (!((ResHob->PhysicalStart >= Resource->PhysicalStart) && (ResHob->PhysicalStart < (Resource->PhysicalStart + Resource->ResourceLength))));
    ASSERT (!((ResHob->PhysicalStart + ResHob->ResourceLength - 1 >= Resource->PhysicalStart) &&
        ((ResHob->PhysicalStart + ResHob->ResourceLength - 1) < (Resource->PhysicalStart + Resource->ResourceLength))));

    // The new resource is attached after this resource descriptor
    if (ResHob->PhysicalStart == Resource->PhysicalStart + Resource->ResourceLength) {
      Resource->ResourceLength =  Resource->ResourceLength + ResHob->ResourceLength;

      NextLink = RemoveEntryList (&Resource->Link);
      InsertTailList (&AttachedResources, &Resource->Link);
      Link = NextLink;
    }
    // The new resource is attached before this resource descriptor
    else if (ResHob->PhysicalStart + ResHob->ResourceLength == Resource->PhysicalStart) {
      Resource->PhysicalStart = ResHob->PhysicalStart;
      Resource->ResourceLength =  Resource->ResourceLength + ResHob->ResourceLength;

      NextLink = RemoveEntryList (&Resource->Link);
      InsertTailList (&AttachedResources, &Resource->Link);
      Link = NextLink;
    } else {
      Link = Link->ForwardLink;
    }
  }

  if (!IsListEmpty (&AttachedResources)) {
    // See if we can merge the attached resource with other resources

    NewResource = (SYSTEM_MEMORY_RESOURCE*)GetFirstNode (&AttachedResources);
    Link = RemoveEntryList (&NewResource->Link);
    while (!IsListEmpty (&AttachedResources)) {
      // Merge resources
      Resource = (SYSTEM_MEMORY_RESOURCE*)Link;

      // Ensure they overlap each other
      ASSERT (
          ((NewResource->PhysicalStart >= Resource->PhysicalStart) && (NewResource->PhysicalStart < (Resource->PhysicalStart + Resource->ResourceLength))) ||
          (((NewResource->PhysicalStart + NewResource->ResourceLength) >= Resource->PhysicalStart) && ((NewResource->PhysicalStart + NewResource->ResourceLength) < (Resource->PhysicalStart + Resource->ResourceLength)))
      );

      NewResourceEnd = MAX (NewResource->PhysicalStart + NewResource->ResourceLength, Resource->PhysicalStart + Resource->ResourceLength);
      NewResource->PhysicalStart = MIN (NewResource->PhysicalStart, Resource->PhysicalStart);
      NewResource->ResourceLength = NewResourceEnd - NewResource->PhysicalStart;

      Link = RemoveEntryList (Link);
    }
  } else {
    // None of the Resource of the list is attached to this ResHob. Create a new entry for it
    NewResource = AllocateZeroPool (sizeof (SYSTEM_MEMORY_RESOURCE));
    NewResource->PhysicalStart = ResHob->PhysicalStart;
    NewResource->ResourceLength = ResHob->ResourceLength;
  }
  InsertTailList (ResourceList, &NewResource->Link);
  return EFI_SUCCESS;
}

EFI_STATUS
GetSystemMemoryResources (
  IN  LIST_ENTRY *ResourceList
  )
{
  EFI_HOB_RESOURCE_DESCRIPTOR *ResHob;

  InitializeListHead (ResourceList);

  // Find the System Memory Resource Descriptor for the SDRAM
  ResHob = (EFI_HOB_RESOURCE_DESCRIPTOR *)GetFirstHob (EFI_HOB_TYPE_RESOURCE_DESCRIPTOR);
  while (
    (ResHob != NULL) &&
	(ResHob->ResourceType != EFI_RESOURCE_SYSTEM_MEMORY) &&
	(ResHob->PhysicalStart != 0)
  )
  {
    ResHob = (EFI_HOB_RESOURCE_DESCRIPTOR *)GetNextHob (EFI_HOB_TYPE_RESOURCE_DESCRIPTOR, (VOID *)((UINTN)ResHob + ResHob->Header.HobLength));
  }

  // Did not find any
  if (ResHob == NULL) {
    return EFI_NOT_FOUND;
  } else {
    InsertSystemMemoryResources (ResourceList, ResHob);
  }

  return EFI_SUCCESS;
}

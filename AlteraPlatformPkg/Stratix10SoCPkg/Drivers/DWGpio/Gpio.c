/** @file
#  Altera SoC FPGA Package
#
#  Portions of the code modified by Altera to support SoC devices are licensed as follows:
#  Copyright (c) 2018, Intel Corporation. All rights reserved.
#
#  Redistribution and use in source and binary forms, with or without modification,
#  are permitted provided that the following conditions are met:
#
#  1. Redistributions of source code must retain the above copyright notice, this
#  list of conditions and the following disclaimer.
#
#  2. Redistributions in binary form must reproduce the above copyright notice, this
#  list of conditions and the following disclaimer in the documentation and/or other
#  materials provided with the distribution.
#
#  3. Neither the name of the copyright holder nor the names of its contributors may
#  be used to endorse or promote products derived from this software without specific
#  prior written permission.
#
#  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
#  EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
#  OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT
#  SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
#  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
#  TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
#  BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
#  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
#  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
#  DAMAGE.
**/

#include <Uefi.h>

#include <Library/IoLib.h>
#include <Library/MemoryAllocationLib.h>
#include <Library/DebugLib.h>
#include <Library/PrintLib.h>
#include <Library/UefiBootServicesTableLib.h>
#include <Library/SerialPortPrintLib.h>

#include <Protocol/EmbeddedGpio.h>

#define DW_SWPORT_DR_OFST   0x0
#define DW_SWPORT_DDR_OFST  0x4

#define DW_GPIO_PIN(x)    (1 << (x))


PLATFORM_GPIO_CONTROLLER *mDWPlatformGpio;

EFI_STATUS
EFIAPI
DWGpioLocate (
  IN  EMBEDDED_GPIO_PIN Gpio,
  OUT UINTN             *ControllerIndex,
  OUT UINTN             *ControllerOffset,
  OUT UINTN             *RegisterBase
  )
{
  UINT32    Index;

  for (Index = 0; Index < mDWPlatformGpio->GpioControllerCount; Index++) {
    if (    (Gpio >= mDWPlatformGpio->GpioController[Index].GpioIndex)
        &&  (Gpio < mDWPlatformGpio->GpioController[Index].GpioIndex
             + mDWPlatformGpio->GpioController[Index].InternalGpioCount)) {
      *ControllerIndex = Index;
      *ControllerOffset = Gpio - mDWPlatformGpio->GpioController[Index].GpioIndex;
      *RegisterBase = mDWPlatformGpio->GpioController[Index].RegisterBase;
      return EFI_SUCCESS;
    }
  }
  DEBUG ((EFI_D_ERROR, "%a, failed to locate gpio %d\n", __func__, Gpio));
  return EFI_INVALID_PARAMETER;
}

EFI_STATUS
Get (
  IN  EMBEDDED_GPIO     *This,
  IN  EMBEDDED_GPIO_PIN Gpio,
  OUT UINTN               *Value
  )
{
  EFI_STATUS Status = EFI_SUCCESS;
  UINTN  Index, Offset, RegisterBase;

  DWGpioLocate(Gpio, &Index, &Offset, &RegisterBase);
  ASSERT_EFI_ERROR(Status);

  if (Value == NULL)
  {
    return EFI_UNSUPPORTED;
  }

  if (MmioRead32 (RegisterBase + DW_SWPORT_DR_OFST) & DW_GPIO_PIN(Offset)) {
    *Value = 1;
  } else {
    *Value = 0;
  }

  return Status;
}

EFI_STATUS
Set (
  IN  EMBEDDED_GPIO       *This,
  IN  EMBEDDED_GPIO_PIN   Gpio,
  IN  EMBEDDED_GPIO_MODE  Mode
  )
{
  EFI_STATUS Status = EFI_SUCCESS;
  UINTN  Index, Offset, RegisterBase;
  DWGpioLocate(Gpio, &Index, &Offset, &RegisterBase);
  ASSERT_EFI_ERROR(Status);
  switch (Mode)
  {
    case GPIO_MODE_INPUT:
      MmioAnd32 (RegisterBase + DW_SWPORT_DDR_OFST, ~DW_GPIO_PIN(Offset));
      break;

    case GPIO_MODE_OUTPUT_0:
      MmioOr32 (RegisterBase + DW_SWPORT_DDR_OFST, DW_GPIO_PIN(Offset));
      MmioAnd32 (RegisterBase + DW_SWPORT_DR_OFST, ~DW_GPIO_PIN(Offset));

      break;

    case GPIO_MODE_OUTPUT_1:
      MmioOr32 (RegisterBase + DW_SWPORT_DDR_OFST, DW_GPIO_PIN(Offset));
      MmioOr32 (RegisterBase + DW_SWPORT_DR_OFST, DW_GPIO_PIN(Offset));
      break;

    default:
      return EFI_UNSUPPORTED;
  };

  return Status;
}

#define GPIO_PIN_MASK(Pin)              (1UL << ((UINTN)(Pin)))

EFI_STATUS
GetMode (
  IN  EMBEDDED_GPIO       *This,
  IN  EMBEDDED_GPIO_PIN   Gpio,
  OUT EMBEDDED_GPIO_MODE  *Mode
  )
{
  EFI_STATUS Status = EFI_SUCCESS;
  UINTN  PortDirection;
  UINTN  Index, Offset, RegisterBase;

  DWGpioLocate(Gpio, &Index, &Offset, &RegisterBase);
  ASSERT_EFI_ERROR(Status);
    
  PortDirection = MmioRead32 (RegisterBase + DW_SWPORT_DDR_OFST) & GPIO_PIN_MASK(Offset);
  /* If pin is input */
  if (PortDirection == 0) {
    *Mode = GPIO_MODE_INPUT;
  } else {
    if (MmioRead32(RegisterBase + DW_SWPORT_DR_OFST) >> Offset & 1)
      *Mode = GPIO_MODE_OUTPUT_0;
    else
      *Mode = GPIO_MODE_OUTPUT_1;
  }

  return Status;
}

EFI_STATUS
SetPull (
  IN  EMBEDDED_GPIO       *This,
  IN  EMBEDDED_GPIO_PIN   Gpio,
  IN  EMBEDDED_GPIO_PULL  Direction
  )
{
  return EFI_UNSUPPORTED;
}

EMBEDDED_GPIO Gpio = {
  Get,
  Set,
  GetMode,
  SetPull
};

/* Number of DW GPIO controllers */
#define PLATFORM_GPIO_NUM 2
#define GPIO_PIN_NUMS (PLATFORM_GPIO_NUM * 48)


EFI_STATUS
GpioInitialize (
  IN EFI_HANDLE         ImageHandle,
  IN EFI_SYSTEM_TABLE   *SystemTable
  )
{
  EFI_STATUS  Status;
  GPIO_CONTROLLER *GpioController;
  UINT32 GpioControllerCount = PcdGet32(PcdDWGpioControllerCount);
  INT32 Index;
  UINT32 GpioCount;
  UINT32 GpioIndex;

  Status = gBS->LocateProtocol (&gPlatformGpioProtocolGuid, NULL, (VOID **)&mDWPlatformGpio);
  if (EFI_ERROR (Status) && (Status == EFI_NOT_FOUND)) {
    mDWPlatformGpio = (PLATFORM_GPIO_CONTROLLER *)AllocateZeroPool (sizeof (PLATFORM_GPIO_CONTROLLER) + sizeof (GPIO_CONTROLLER) * GpioControllerCount);
    if (mDWPlatformGpio == NULL) {
      DEBUG ((EF_D_ERROR, "%a: failed to allocate PLATFORM_GPIO_CONTROLLER\n", __func__));
      return EFI_BAD_BUFFER_SIZE;
    }

    mDWPlatformGpio->GpioControllerCount = PcdGet32(PcdDWGpioControllerCount);
    GpioCount = 0;
    for (Index = 0; Index < mDWPlatformGpio->GpioControllerCount; Index++) {
      GpioCount += ((UINTN*) PcdGetPtr (PcdDWGpioCount))[Index];
    }
    mDWPlatformGpio->GpioCount = GpioCount;
    mDWPlatformGpio->GpioController = (GPIO_CONTROLLER *)((UINTN) mDWPlatformGpio + sizeof (PLATFORM_GPIO_CONTROLLER));

    GpioController = mDWPlatformGpio->GpioController;
    GpioIndex = 0;
    for (Index = 0; Index < mDWPlatformGpio->GpioControllerCount; Index++) {
      // assertion here!
      GpioController[Index].RegisterBase = ((UINT32*) PcdGetPtr (PcdDWGpioBase))[Index];
      GpioController[Index].GpioIndex = GpioIndex;
      GpioController[Index].InternalGpioCount = ((UINT8*) PcdGetPtr (PcdDWGpioCount))[Index];

      GpioIndex += GpioController[Index].InternalGpioCount;
    }
  }

  Status = gBS->InstallMultipleProtocolInterfaces(&ImageHandle, &gEmbeddedGpioProtocolGuid, &Gpio, NULL);
  return Status;
}


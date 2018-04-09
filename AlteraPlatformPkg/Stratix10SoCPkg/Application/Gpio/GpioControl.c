/** @file
  This application controls GPIO, used for toggling GPIO/LED on SoCFPGA board

  Copyright (c) 2018, Intel Corporation. All rights reserved.<BR>
  This program and the accompanying materials                          
  are licensed and made available under the terms and conditions of the BSD License         
  which accompanies this distribution.  The full text of the license may be found at        
  http://opensource.org/licenses/bsd-license.php                                            

  THE PROGRAM IS DISTRIBUTED UNDER THE BSD LICENSE ON AN "AS IS" BASIS,                     
  WITHOUT WARRANTIES OR REPRESENTATIONS OF ANY KIND, EITHER EXPRESS OR IMPLIED.             

**/

#include <Uefi.h>
#include <Library/UefiApplicationEntryPoint.h>
#include <Library/UefiLib.h>
#include <Library/BaseMemoryLib.h>
#include <Uefi/UefiSpec.h>
#include <Library/UefiBootServicesTableLib.h>
#include <Library/UefiRuntimeServicesTableLib.h>
#include <Library/BaseLib.h>
#include <Protocol/EmbeddedGpio.h>
#include <Library/DebugLib.h>

#include <stdlib.h>


/**
  The user Entry Point for Application. The user code starts with this function
  as the real entry point for the application.

  @param[in] ImageHandle    The firmware allocated handle for the EFI image.  
  @param[in] SystemTable    A pointer to the EFI System Table.
  
  @retval EFI_SUCCESS       The entry point is executed successfully.
  @retval other             Some error occurs when executing this entry point.

**/
/*EFI_STATUS
EFIAPI
UefiMain (
  IN EFI_HANDLE        ImageHandle,
  IN EFI_SYSTEM_TABLE  *SystemTable
  )*/
INTN
EFIAPI
ShellAppMain (
  IN UINTN Argc,
  IN CHAR16 **Argv
  )
{
  EFI_STATUS  Status;
  EMBEDDED_GPIO *mGpio;
  Status = gBS->LocateProtocol(&gEmbeddedGpioProtocolGuid, NULL, (VOID **)&mGpio);

  if (EFI_ERROR (Status)) {
    DEBUG ((EFI_D_ERROR, "Can't locate gEmbeddedGpioProtocolGuid\n"));
    return Status;
  }


  if (Argc == 2 || Argc == 3) {
    UINTN output_gpio = StrDecimalToUintn(Argv[1]);
    UINTN output_value;
    EMBEDDED_GPIO_MODE mode;

    if (Argc == 2) {
      Status = mGpio->GetMode (mGpio, output_gpio, &mode);
      if (EFI_ERROR (Status)) {
        DEBUG ((EFI_D_ERROR, "Failed reading GPIO\n"));
      }

      Status = mGpio->Get (mGpio, output_gpio, &output_value);
      if (EFI_ERROR (Status)) {
        DEBUG ((EFI_D_ERROR, "Failed reading GPIO\n"));
      }

      if (mode == GPIO_MODE_INPUT)
        Print(L"Gpio %d: is in input mode\n", output_gpio);
      else
        Print(L"Gpio %d: is in output with value %d\n", output_gpio, output_value);
    }
    if (Argc == 3) {
      output_value = StrDecimalToUintn(Argv[2]);
      if (output_value == 0)
          Status = mGpio->Set (mGpio, output_gpio, GPIO_MODE_OUTPUT_0);
        else
          Status = mGpio->Set (mGpio, output_gpio, GPIO_MODE_OUTPUT_1);
    }
  }
  else {
    Print(L"Parameters required (GpioControl <gpio_number> <optional, gpio value, 0|1>)");
  }

  return EFI_SUCCESS;
}

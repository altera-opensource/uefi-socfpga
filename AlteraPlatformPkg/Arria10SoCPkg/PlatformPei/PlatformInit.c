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
#include <AlteraPlatform.h>
#include <Library/BaseMemoryLib.h>
#include <Library/DebugLib.h>
#include <Library/IoLib.h>
#include <Library/MemoryAllocationLib.h>
#include <Library/PcdLib.h>
#include <Library/SerialPortPrintLib.h>
#include "Assert.h"
#include "Banner.h"
#include "Board.h"
#include "Boot.h"
#include "BootSource.h"
#include "ClockManager.h"
#include "DeviceTree.h"
#include "Firewall.h"
#include "FpgaManager.h"
#include "MemoryController.h"
#include "MemoryTest.h"
#include "NandLib.h"
#include "Pinmux.h"
#include "PitStopUtility.h"
#include "PlatformInit.h"
#include "QspiLib.h"
#include "ResetManager.h"
#include "SdMmc.h"
#include "SecurityManager.h"
#include "SystemManager.h"

#define InfoPrint  SerialPortPrint

//
// Functions
//
EFI_STATUS
EFIAPI
PeiStagePlatformInit (
  VOID
  )
{
  VOID*             Fdt;
  EFI_STATUS        Status;
  BOOT_SOURCE_TYPE  BootSourceType;
  BOOLEAN           AlreadyInitSerialPort;
  BOOLEAN           FlashDeviceIsAvailable;
  BOOLEAN           ReProgramFpgaEvenIfFpgaIsAlreadyInUserMode;
  BOOLEAN           DecidedToProgramFpga;

  Status = EFI_SUCCESS;
  AlreadyInitSerialPort = FALSE;
  DecidedToProgramFpga = FALSE;

  // ASSUMPTION - before entering PeiStagePlatformInit:
  // At this point of time,
  // 1) Reset has already been asserted to peripherals
  // 2) Watchdog Timer 0 has been disabled.
  // 3) Security Control Registers for OCRAM and DDR has been set
  // 4) System Timer 0 is running
  // Please see AlteraPlatformLibSec.c - ArmPlatformInitialize

  // Find the Flattened Device Tree Base Address
  Status = GetFlattenedDeviceTreePtr (&Fdt);
  ASSERT_PLATFORM_INIT(!EFI_ERROR(Status));

  // Clock Configuration
  ConfigureClockManager (Fdt);

  // Dedicated I/O Configuration
  ConfigureDedicatedIoElectricalBehavior (Fdt); // This need to be done before PinMux
  ConfigureDedicatedIoPinMux (Fdt);

  // Shared I/O Configuration
  SelectSharedIoBetweenHpsAndFpga (Fdt);
  ConfigureSharedIoPinMux (Fdt);

  // Reset Deassertion through the Reset Manager
  DeassertPeripheralsReset ();

  // Reset manager handshake with other modules before warm reset.
  ConfigureHpsSubsystemHandshakeBehaviorBeforeWarmReset ();

  // Start Talking if FPGA ready or FPGA is not ready but UART is using Dedicated IO.
  if (( FpgaIsInUserMode() == TRUE ) ||
      ( UartIsUsingDedicatedIo() == TRUE ))
  {
    // Start Talking
    SerialPortDisplayInfoForTheFirstTime ();
    AlreadyInitSerialPort = TRUE;
  }

  // Detect Boot Source Type
  BootSourceType = GetBootSourceType ();

  // Flash Device initialization
  switch (BootSourceType)
  {
    case BOOT_SOURCE_NAND:
      // Init NAND
      NandInit ();
      FlashDeviceIsAvailable = TRUE;
      break;
    case BOOT_SOURCE_QSPI:
      // Init QSPI
      QspiInit ();
      FlashDeviceIsAvailable = TRUE;
      break;
    case BOOT_SOURCE_SDMMC:
      // Init SDMMC
      InitSdMmc (Fdt);
      FlashDeviceIsAvailable = TRUE;
      break;
    case BOOT_SOURCE_RSVD:
    case BOOT_SOURCE_FPGA:
    default:
      // No Flash device.
      InfoPrint ("No Flash Device Available!\r\n");
      FlashDeviceIsAvailable = FALSE;
      break;
  }

  // Still program FPGA using RBF file from Flash device even if it is already programmed?
  ReProgramFpgaEvenIfFpgaIsAlreadyInUserMode = IsSkipFpgaConfig (Fdt)? FALSE : TRUE;

  // Configure the FPGA before Memory Init ?
  if ( FpgaIsInUserMode() == FALSE )
  {
    InfoPrint ("FPGA is not in User Mode\r\n");
    if (FlashDeviceIsAvailable == TRUE)
    {
      DecidedToProgramFpga = TRUE;
    }
  } else {
    InfoPrint ("FPGA is in User Mode\r\n");
    if (ReProgramFpgaEvenIfFpgaIsAlreadyInUserMode == TRUE)
    {
      if (FlashDeviceIsAvailable == TRUE)
      {
        DecidedToProgramFpga = TRUE;
      }
    }
  }

  // Start FPGA full configuration
  if ( DecidedToProgramFpga == TRUE )
  {
    Status = FpgaFullConfiguration (Fdt, BootSourceType);
    ASSERT_PLATFORM_INIT(!EFI_ERROR(Status));
  }

  // Start Talking if FPGA ready and UART is not yet init
  // (if Dediacted IO then it is init above before caling GetBootSourceType)
  if (( FpgaIsInUserMode() == TRUE ) &&
      ( AlreadyInitSerialPort == FALSE ))
  {
    // Start Talking
    SerialPortDisplayInfoForTheFirstTime ();
    AlreadyInitSerialPort = TRUE;
  }

  // FPGA must have entered User Mode before we can proceed with HMC init
  if (FpgaIsInUserMode() == FALSE )
  {
    InfoPrint ("FPGA not in User Mode, cannot begin HMC init!\r\n");
    EFI_DEADLOOP();
  }

  // Configure Memory Controller
  Status = InitHardMemoryController (Fdt);

  // Is memory initialization successful?
  if (Status == EFI_SUCCESS) {
    // Display Memory Info
    DisplayMemoryInfo ();

    // Memory Test
    MemoryTest ();
  }

  // Init Firewall
  InitFirewall (Fdt);
  // Display Firewall Info
  DisplayFirewallInfo ();

  // Enable Hps and Fpga Bridges
  if ( FpgaIsInUserMode() == TRUE )
  {
    EnableHpsAndFpgaBridges (Fdt);
  }

  //
  // Board Specific Initialization
  //
  BoardSpecificInitialization (Fdt);

  //
  // Enter Pit Stop utility ?
  //
  if ( IsEnterPitStop() == TRUE)
  {
    PitStopCmdLine ();
  }

  //
  // Transfer Control to BootImage now, without going through UEFI DXE phase ?
  //
  if (PcdGet32 (PcdBoot_LOAD_UEFI_DXE_PHASE) != 1)
  {
    LoadBootImageAndTransferControl (BootSourceType);
  }

  return Status;
}


VOID
EFIAPI
SerialPortDisplayInfoForTheFirstTime (
  VOID
  )
{
  // UART Initialization
  // Note:
  // If UART does not work please make sure you have the following items correct:
  // 1. PcdSerialRegisterBase (eg. UART0 is 0xFFC02000, UART1 is 0xFFC02100)
  // 2. PcdSerialClockRate (eg. 100000000 for l4_sp_clk 100 MHz)
  // 3. PcdSerialBaudRate (eg. 115200 baud)
  // 4. PcdSerialLineControl  (eg. 0x03 for 8 bits, 1 stop bit, No Parity)
  // 5. Dedicated/Shared IO pinmux especially the UART tx/rx signal
  // 6. If using Shared IO, mux selection between HPS and FPGA Interface (pinmux_uart[0,1]_usefpga)
  // 7. If available, io bank and buffer voltage, slew rate and drive strength settings
  // 8. Clock Manager settings, HPS-UART0/1 is connected to the l4_sp_clk clock.
  // 9. Reset Manager settings, check if HPS-UART0/1 reset signal is de-asserted.
  SerialPortInit ();

  // Display user customizable firmwave Banner
  DisplayBanner ();

  // Display System Manager Info
  DisplaySystemManagerInfo ();

  // Display Reset Manager Info
  DisplayResetManagerInfo ();

  // Display Security Manager Info
  DisplaySecurityManagerInfo ();

  // Display PinMux Info
  DisplayIo48PinMuxInfo ();

  // Display Clock Manager Info
  DisplayClockManagerInfo ();

}

EFI_BOOT_MODE
EFIAPI
PlatformPeiGetBootMode (
  VOID
  )
{
  return BOOT_WITH_FULL_CONFIGURATION;
}



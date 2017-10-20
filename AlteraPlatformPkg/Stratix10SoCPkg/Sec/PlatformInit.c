/** @file

  Copyright (c) 2016, Intel Corporation. All rights reserved.

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
#include "ClockManager.h"
#include "Ethernet.h"
#include "Handoff.h"
#include "MemoryController.h"
#include "MemoryTest.h"
#include "NandLib.h"
#include "Pinmux.h"
#include "PitStopUtility.h"
#include "PlatformInit.h"
#include "ResetManager.h"
#include "SdMmc.h"
#include "SystemManager.h"

#define InfoPrint  SerialPortPrint

VOID
EFIAPI
EnableNonsecureAccess (
  VOID
  )
{
  // this enables nonsecure access to UART0
  MmioWrite32(ALT_NOC_FW_L4_PER_SCR_OFST +
              ALT_NOC_FW_L4_PER_SCR_UART0_OFST,
  			  0x1);

  // enables nonsecure MPU accesses to SDMMC
  MmioWrite32(ALT_NOC_FW_L4_PER_SCR_OFST +
              ALT_NOC_FW_L4_PER_SCR_SDMMC_OFST,
              0x1 | BIT24 | BIT16);
  MmioWrite32(ALT_NOC_FW_L4_SYS_SCR_OFST +
              ALT_NOC_FW_L4_SYS_SCR_SDMMC_ECC_OFST,
              0x1 | BIT24 | BIT16);

  // enables nonsecure access to all the emacs
  MmioWrite32(ALT_NOC_FW_L4_PER_SCR_OFST +
              ALT_NOC_FW_L4_PER_SCR_EMAC0_OFST,
  			  0x1 | BIT24 | BIT16);
  MmioWrite32(ALT_NOC_FW_L4_PER_SCR_OFST +
              ALT_NOC_FW_L4_PER_SCR_EMAC1_OFST,
  			  0x1 | BIT24 | BIT16 );
  MmioWrite32(ALT_NOC_FW_L4_PER_SCR_OFST +
              ALT_NOC_FW_L4_PER_SCR_EMAC2_OFST,
  			  0x1 | BIT24 | BIT16 );
  
  MmioWrite32(ALT_NOC_FW_L4_SYS_SCR_OFST +
              ALT_NOC_FW_L4_SYS_SCR_EMAC0RX_ECC_OFST,
  			  0x1 | BIT24 | BIT16 );
  MmioWrite32(ALT_NOC_FW_L4_SYS_SCR_OFST +
              ALT_NOC_FW_L4_SYS_SCR_EMAC0TX_ECC_OFST,
  			  0x1 | BIT24 | BIT16 );
  MmioWrite32(ALT_NOC_FW_L4_SYS_SCR_OFST +
              ALT_NOC_FW_L4_SYS_SCR_EMAC1RX_ECC_OFST,
              0x1 | BIT24 | BIT16 );
  MmioWrite32(ALT_NOC_FW_L4_SYS_SCR_OFST +
              ALT_NOC_FW_L4_SYS_SCR_EMAC1TX_ECC_OFST,
              0x1 | BIT24 | BIT16 );
  MmioWrite32(ALT_NOC_FW_L4_SYS_SCR_OFST +
              ALT_NOC_FW_L4_SYS_SCR_EMAC2RX_ECC_OFST,
              0x1 | BIT24 | BIT16 );
  MmioWrite32(ALT_NOC_FW_L4_SYS_SCR_OFST +
              ALT_NOC_FW_L4_SYS_SCR_EMAC2TX_ECC_OFST,
              0x1 | BIT24 | BIT16 );

  // enables nonsecure access to i2c0 and i2c1
  MmioWrite32(ALT_NOC_FW_L4_PER_SCR_OFST +
              ALT_NOC_FW_L4_PER_SCR_I2C0_OFST,
              0x1 | BIT24 | BIT16 );
  MmioWrite32(ALT_NOC_FW_L4_PER_SCR_OFST +
              ALT_NOC_FW_L4_PER_SCR_I2C1_OFST,
              0x1 | BIT24 | BIT16 );

  // enables nonsecure access to usb0 and usb1
  MmioWrite32(ALT_NOC_FW_L4_PER_SCR_OFST +
              ALT_NOC_FW_L4_PER_SCR_USB0_REGISTER_OFST,
              0x1 | BIT24 | BIT16 );
  MmioWrite32(ALT_NOC_FW_L4_PER_SCR_OFST +
              ALT_NOC_FW_L4_PER_SCR_USB1_REGISTER_OFST,
              0x1 | BIT24 | BIT16 );

  // enables nonsecure access to gpio0 and gpio1
  MmioWrite32(ALT_NOC_FW_L4_PER_SCR_OFST +
              ALT_NOC_FW_L4_PER_SCR_GPIO0_OFST,
              0x1 | BIT24 | BIT16 );
  MmioWrite32(ALT_NOC_FW_L4_PER_SCR_OFST +
              ALT_NOC_FW_L4_PER_SCR_GPIO1_OFST,
              0x1 | BIT24 | BIT16 );

  // enables nonsecure access to clock manager
  MmioWrite32(ALT_NOC_FW_L4_SYS_SCR_OFST +
              ALT_NOC_FW_L4_SYS_SCR_CLOCK_MANAGER_OFST,
              0x1 | BIT24 | BIT16 );

  // enables nonsecure access to reset manager
  MmioWrite32(ALT_NOC_FW_L4_SYS_SCR_OFST +
              ALT_NOC_FW_L4_SYS_SCR_RESET_MANAGER_OFST,
              0x1 | BIT24 | BIT16 );
  
  // this enables nonsecure access to OCRAM ECC for flash DMA transaction AXI API & MPU
  MmioWrite32(ALT_NOC_FW_L4_SYS_SCR_OFST +
              ALT_NOC_FW_L4_SYS_SCR_OCRAM_ECC_OFST,
              ALT_NOC_FW_L4_SYS_SCR_OCRAM_ECC_AXI_AP_SET_MSK |
  		      ALT_NOC_FW_L4_SYS_SCR_OCRAM_ECC_MPU_SET_MSK);
  
  // disable ocram security at CCU, temporary hack
  MmioAnd32(ALT_CCU_NOC_OFST +
            ALT_CCU_NOC_BRIDGE_CPU0_MPRT_0_37_AM_ADMASK_MEM_RAM_SPRT_RAMSPACE0_0_OFST,
  		    ~0x03);
  MmioAnd32(ALT_CCU_NOC_OFST +
            ALT_CCU_NOC_BRIDGE_IOM_MPRT_5_63_AM_ADMASK_MEM_RAM_SPRT_RAMSPACE0_0_OFST,
  		    ~0x03);
}

EFI_STATUS
EFIAPI
PlatformInit (
  VOID
  )
{
  EFI_STATUS        Status;
  BOOT_SOURCE_TYPE  BootSourceType;

  Status = EFI_SUCCESS;

  // ASSUMPTION - before entering PeiStagePlatformInit:
  // At this point of time,
  // 1) Reset has already been asserted to peripherals
  // 2) Watchdog Timer 0 has been disabled.
  // 3) Security Control Registers for OCRAM and DDR has been set
  // 4) System Timer 0 is running
  // Please see AlteraPlatformLibSec.c - ArmPlatformInitialize

  /*
  handoff *handoff_ptr = (handoff*) 0xFFE3F000;
  if (verify_handoff_image(handoff_ptr))  {
  	InfoPrint("invalid hand off image\n");
  	return EFI_DEVICE_ERROR;
  }
  InfoPrint("handoff is valid\r\n");
	
  ConfigPinMuxHandoff(handoff_ptr);
  ConfigureClockManagerHandoff (handoff_ptr);
  */
  
  ConfigPinMux();
  ConfigureClockManager ();

  // Reset Deassertion through the Reset Manager
  DeassertPeripheralsReset ();

  // Reset manager handshake with other modules before warm reset.
  ConfigureHpsSubsystemHandshakeBehaviorBeforeWarmReset ();
  
  MmioOr32 (
    ALT_CLKMGR_PERPLL_OFST +
	ALT_CLKMGR_PERPLL_EN_OFST,
	ALT_CLKMGR_PERPLL_EN_SDMMCCLKEN_SET_MSK
	);
  
  EnableNonsecureAccess ();

  SerialPortInit ();
  SerialPortDisplayInfoForTheFirstTime ();
  // Configure Memory Controller
  Status = InitHardMemoryController ();
  // Detect Boot Source Type
  BootSourceType = BOOT_SOURCE_SDMMC;

  // Flash Device initialization
  switch (BootSourceType)
  {
    case BOOT_SOURCE_NAND:
      // Init NAND
      NandInit ();
      break;
    case BOOT_SOURCE_SDMMC:
      // Init SDMMC
      InitSdMmc ();
      break;
    case BOOT_SOURCE_RSVD:
    case BOOT_SOURCE_FPGA:
    default:
      // No Flash device.
      InfoPrint ("No Flash Device Available!\r\n");
      break;
  }
  
  // Is memory initialization successful?
  if (Status == EFI_SUCCESS) {
    // Display Memory Info
    DisplayMemoryInfo ();
  
    // Memory Test
    MemoryTest ();
  }
    
  // Enable Hps and Fpga Bridges
  //EnableHpsAndFpgaBridges ();
  EmacInit ();
   //
  // Board Specific Initialization
  //
  BoardSpecificInitialization ();

  //
  // Enter Pit Stop utility ?
  //
  if ( IsEnterPitStop() == TRUE)
  {
    PitStopCmdLine ();
  }
  
  LoadPei (BootSourceType);
   
  return Status;
}

//
// Private Functions
//
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


  // Display user customizable firmwave Banner
  DisplayBanner ();

  // Display System Manager Info
  DisplaySystemManagerInfo ();

  // Display Reset Manager Info
  DisplayResetManagerInfo ();

  // Display PinMux Info
 // DisplayIo48PinMuxInfo ();

  // Display Clock Manager Info
  DisplayClockManagerInfo ();

}


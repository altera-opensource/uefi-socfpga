#/** @file
#  Altera SoC FPGA Package
#
#  Portions of the code modified by Altera to support SoC devices are licensed as follows:
#  Copyright (c) 2016, Intel Corporation. All rights reserved.
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
#
#  The original software modules are licensed as follows:
#
#  Copyright (c) 2011-2014, ARM Limited. All rights reserved.
#
#  This program and the accompanying materials
#  are licensed and made available under the terms and conditions of the BSD License
#  which accompanies this distribution.  The full text of the license may be found at
#  http://opensource.org/licenses/bsd-license.php
#
#  THE PROGRAM IS DISTRIBUTED UNDER THE BSD 3 CLAUSE LICENSE ON AN "AS IS" BASIS,
#  WITHOUT WARRANTIES OR REPRESENTATIONS OF ANY KIND, EITHER EXPRESS OR IMPLIED.
#
#**/

################################################################################
#
# Defines Section - statements that will be processed to create a Makefile.
#
################################################################################
[Defines]
  PLATFORM_NAME                  = Altera(R) Stratix(R) 10 SoC Development Board
  PLATFORM_GUID                  = A2D10D02-7C36-4de8-831B-EFBFC2092D1B
  PLATFORM_VERSION               = 0.1
  FIRMWARE_VERSION               = 1.0
  DSC_SPECIFICATION              = 0x00010005
  SUPPORTED_ARCHITECTURES        = AARCH64
  BUILD_TARGETS                  = DEBUG|RELEASE
  SKUID_IDENTIFIER               = DEFAULT
  FLASH_DEFINITION               = AlteraPlatformPkg/Stratix10SoCPkg/Stratix10SoCPkg.fdf
  OUTPUT_DIRECTORY               = Build/Stratix10SoCPkg
  USE_ARM_BDS                    = FALSE
  SECURE_BOOT_ENABLE             = FALSE

################################################################################
#
# Pcd Section - list of all EDK II PCD Entries defined by this Platform
#
################################################################################

[PcdsFixedAtBuild.common]

  #-------------------------------
  # Baord Specific PCDs
  #-------------------------------
  gAlteraHpsTokenSpaceGuid.PcdPlatformNameString|L"$(PLATFORM_NAME)"
  gAlteraSocFpgaTokenSpaceGuid.PcdIsAlteraSoCFPGADevelopmentBoards|1

  ##*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*
  ## Altera SoC HPS modules PCD
  ##*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*

  #-------------------------------
  # gAlteraSocFpgaTokenSpaceGuid
  #-------------------------------

  #
  # Platform Init Features Enable / Disable
  #
  gAlteraSocFpgaTokenSpaceGuid.PcdSdmmcSweepAllDrvselAndSmplselValues|0
  gAlteraSocFpgaTokenSpaceGuid.PcdSdmmcDrvSel|3
  gAlteraSocFpgaTokenSpaceGuid.PcdSdmmcSmplSel|0
  gAlteraSocFpgaTokenSpaceGuid.PcdSdmmcSupport4BitMode|1
  gAlteraSocFpgaTokenSpaceGuid.PcdEnableMemoryTest|1
  gAlteraSocFpgaTokenSpaceGuid.PcdRemapOnChipRamTo1stOneMB|0

  # Maximum FAT32 Partition Volumn cluster size:
  # Valid value are:
  #     0x1000 (4KB/8GB), 0x2000 (8KB/16GB), 0x4000 (16KB/32GB), 0x8000 (32KB/Legacy FAT32x)
  # NOTE:
  #     The larger the size the more firmware volume and OCRAM space consumed
  gAlteraSocFpgaTokenSpaceGuid.PcdMaxFAT32ClusterSize|0x1000

  #
  # QSPI transfer mode
  # @Prompt 0 - Standard Single SPI (SIO-SPI) mode (bits always transferred on DQ0 only). Supported by all SPI flash devices.
  # @Prompt 1 - Use Dual SPI (DIO-SPI) SPI mode where bits are transferred on DQ0 and DQ1.
  # @Prompt 2,  Use Quad SPI (QIO-SPI) SPI mode where bits are transferred on DQ0, DQ1, DQ2, and DQ3.
  gAlteraSocFpgaTokenSpaceGuid.PcdQspiMode|2

  #
  # QSPI Clock Frequency
  gAlteraSocFpgaTokenSpaceGuid.PcdQspiClkFreq|54

  # GPIO Base
  gAlteraSocFpgaTokenSpaceGuid.PcdDWGpioBase|{0xFFC03200,0xFFC03300}
  gAlteraSocFpgaTokenSpaceGuid.PcdDWGpioCount|{24,24}
  gAlteraSocFpgaTokenSpaceGuid.PcdDWGpioControllerCount|2

  #
  # Nand will not erase/read/write if more than this defined numbers of bad block are detected
  gAlteraSocFpgaTokenSpaceGuid.PcdNandStopIfMoreThanThisNumberBadBlocks|1

  #
  # Semihosting Support
  # Semihosting is a mechanism that enables code running on an ARM target to communicate
  # and use the Input/Output facilities on a host computer that is running a debugger.
  # To use Semihosting in DS-5 debugger
  #  1) Type the command: set semihosting enabled true
  #  2) Open DS-5 Menu->Window->Show View->App Console
  #
  # Remember to disable Semihosting for production release, set to value 0
  # To turn off Semihosting, set to value 0
  # To turn  on Semihosting, set to value 1
  gAlteraHpsTokenSpaceGuid.PcdEnableSemihosting|0

  #
  # Platform Init Debug Message
  #
  # Suggest set to value 0 for RELEASE build to reduce boot time
  # Alternatively you can set PcdSerialRegisterBase to 0 to totally disable UART message
  gAlteraSocFpgaTokenSpaceGuid.PcdDebugMsg_BoardInit|1
  gAlteraSocFpgaTokenSpaceGuid.PcdDebugMsg_Boot|1
  gAlteraSocFpgaTokenSpaceGuid.PcdDebugMsg_BootSource|1
  gAlteraSocFpgaTokenSpaceGuid.PcdDebugMsg_ClockManager|1
  gAlteraSocFpgaTokenSpaceGuid.PcdDebugMsg_FDT|1
  gAlteraSocFpgaTokenSpaceGuid.PcdDebugMsg_Firewall|1
  gAlteraSocFpgaTokenSpaceGuid.PcdDebugMsg_FpgaManager|1
  gAlteraSocFpgaTokenSpaceGuid.PcdDebugMsg_MemoryController|1
  gAlteraSocFpgaTokenSpaceGuid.PcdDebugMsg_Pinmux|1
  gAlteraSocFpgaTokenSpaceGuid.PcdDebugMsg_ResetManager|1
  gAlteraSocFpgaTokenSpaceGuid.PcdDebugMsg_SdMmc|1
  gAlteraSocFpgaTokenSpaceGuid.PcdDebugMsg_SecurityManager|1
  gAlteraSocFpgaTokenSpaceGuid.PcdDebugMsg_SystemManager|1
  gAlteraSocFpgaTokenSpaceGuid.PcdDebugMsg_Rbf|1
  gAlteraSocFpgaTokenSpaceGuid.PcdDebugMsg_Nand|1
  gAlteraSocFpgaTokenSpaceGuid.PcdDebugMsg_Qspi|1
  gAlteraSocFpgaTokenSpaceGuid.PcdDebugMsg_EmacSnpDxe|1

  #
  # Platform Init Assertion
  #
  # Suggest set to value 0 for RELEASE build
  # Suggest set to value 1 when debugging
  gAlteraHpsTokenSpaceGuid.PcdPlatformInitAllowAssertion|1
  gAlteraHpsTokenSpaceGuid.PcdPlatformInitStopWhenAssert|1

  #
  # Pit Stop utility
  # Suggest set to value 0 for RELEASE build
  # Suggest set to value 1 to always enable
  # Suggest set to value 2+ for timeout if no key pressed
  gAlteraSocFpgaTokenSpaceGuid.PcdEnablePitStopUtility|5

  # On-Chip-Memory Serial Log Base address and Size
  # Come in handy for PlatformInit debug as alternative
  # when UART fail and Semihosting does not fit the situation
  # Set PcdMemorySerialLogBase to 0 to disable this feature
  # Suggest set to value 0 for RELEASE build
  # Suggest set to value 0xFFE31000 when debugging
  gAlteraHpsTokenSpaceGuid.PcdMemorySerialLogBase|0xFFE31000
  # Suggest set to value 0x1000 when debugging
  # If you change the size, to make sure there is no overlap,
  # please also adjust PcdCPUCoresStackBase and PcdCPUCorePrimaryStackSize
  gAlteraHpsTokenSpaceGuid.PcdMemorySerialLogSize|0x1000


  ## QSPI/NAND Flash Offset
  gAlteraSocFpgaTokenSpaceGuid.PcdQspiOrNand_BOOTLOADER_PEIROM_ADDR |0x00000000
  gAlteraSocFpgaTokenSpaceGuid.PcdQspiOrNand_BOOTLOADER_DXEROM_ADDR |0x00620000
  gAlteraSocFpgaTokenSpaceGuid.PcdQspiOrNand_LINUX_DTB_ADDR         |0x00100000
  gAlteraSocFpgaTokenSpaceGuid.PcdQspiOrNand_BOOTIMAGE_ADDR         |0x00120000

  ## Various Filename in root directory of the SD/MMC FAT32 partition
  gAlteraSocFpgaTokenSpaceGuid.PcdFileName_DXE_ROM                 |"DXE.ROM"
  gAlteraSocFpgaTokenSpaceGuid.PcdFileName_PEI_ROM                 |"PEI.ROM"
  gAlteraSocFpgaTokenSpaceGuid.PcdFileName_LINUX_DTB               |"fdt.dtb"
  gAlteraSocFpgaTokenSpaceGuid.PcdFileName_BOOTIMAGE_BIN           |"bootimage.bin"
  gAlteraSocFpgaTokenSpaceGuid.PcdFileName_ZIMAGE                  |"Image"

  # Boot Image Load Address and Entry Point
  # If the image have mkimage header, it will based on mkimage header
  # If the image do not have an mkimage header, it will base on PCDs
  gAlteraSocFpgaTokenSpaceGuid.PcdBoot_BOOTIMAGE_MEM_LOAD_ADDR      |0x100000
  gAlteraSocFpgaTokenSpaceGuid.PcdBoot_BOOTIMAGE_CPU_JUMP_ADDR      |0x100000

  # Extra Boot Option for PEI Phase
  gAlteraSocFpgaTokenSpaceGuid.PcdBoot_LOAD_ZIMAGE_AT_PEI_PHASE     |0
  gAlteraSocFpgaTokenSpaceGuid.PcdBoot_LOAD_UEFI_DXE_PHASE          |1


  ##------------------------------------------------------------------------------
  ## Pcd Settings - Timer Frequency
  ##------------------------------------------------------------------------------
  #  Clock souce from HPS_CLK1 pin to osc1_clk to Osc1Timer0 and Osc1Timer1
  gAlteraHpsTokenSpaceGuid.Pcd_Osc1Timer0_Base|0xFFD00000
  gAlteraHpsTokenSpaceGuid.Pcd_Osc1Timer0_ClkFreqInHz|400000000
  gAlteraHpsTokenSpaceGuid.Pcd_Osc1Timer1_Base|0xFFD00100
  gAlteraHpsTokenSpaceGuid.Pcd_Osc1Timer1_ClkFreqInHz|400000000
  gAlteraHpsTokenSpaceGuid.Pcd_Osc1Timer1_InterruptNum|116

  ##------------------------------------------------------------------------------
  ## Pcd Settings - HPS EMAC
  ##------------------------------------------------------------------------------
  #  EMAC Default Source Mac Address
  gAlteraSocFpgaTokenSpaceGuid.PcdDefaultMacAddress|0x021122334455
  gAlteraSocFpgaTokenSpaceGuid.PcdDefaultMacInterface|0

  #-------------------------------
  # gArmPlatformTokenSpaceGuid
  #-------------------------------
  gArmPlatformTokenSpaceGuid.PcdFirmwareVendor|""

  gArmPlatformTokenSpaceGuid.PcdCoreCount|1

  # Stacks for MPCores in PEI Phase
  gArmPlatformTokenSpaceGuid.PcdCPUCoresStackBase|0x6d000
  gArmPlatformTokenSpaceGuid.PcdCPUCorePrimaryStackSize|0xC000

  # Stacks for MPCores in SEC Phase
  gArmPlatformTokenSpaceGuid.PcdCPUCoresSecStackBase|0xFFE3F000
  gArmPlatformTokenSpaceGuid.PcdCPUCoreSecPrimaryStackSize|0x400
  gArmPlatformTokenSpaceGuid.PcdCPUCoreSecSecondaryStackSize|0x200

  # Stacks for MPCores in Monitor Mode
  gArmPlatformTokenSpaceGuid.PcdCPUCoresSecMonStackBase|0xFFE3F400
  gArmPlatformTokenSpaceGuid.PcdCPUCoreSecMonStackSize|0x400

  #-------------------------------
  # gArmTokenSpaceGuid
  #-------------------------------
  #
  # ARM L2x0 PCDs
  #
  gArmTokenSpaceGuid.PcdL2x0ControllerBase|0xFFFFF000

  #
  # ARM General Interrupt Controller
  #
  gArmTokenSpaceGuid.PcdGicDistributorBase|0xFFFC1000
  gArmTokenSpaceGuid.PcdGicInterruptInterfaceBase|0xFFFC2000

  # ARM Floating Point architecture (VFP)
  gArmTokenSpaceGuid.PcdVFPEnabled|1

  # System Memory (1GB)
  gArmTokenSpaceGuid.PcdSystemMemoryBase|0x01000000
  gArmTokenSpaceGuid.PcdSystemMemorySize|0x3f000000

  gArmTokenSpaceGuid.PcdArmUncachedMemoryMask|0x0000000040000000

  # Arm Architectural Timer
  gArmTokenSpaceGuid.PcdArmArchTimerFreqInHz|400000000

  # Trustzone Enable
  gArmTokenSpaceGuid.PcdTrustzoneSupport|TRUE

  #-------------------------------
  # gEfiMdeModulePkgTokenSpaceGuid
  #-------------------------------
  gEfiMdeModulePkgTokenSpaceGuid.PcdFirmwareVersionString|L"$(FIRMWARE_VERSION)"

  #-------------------------------
  # gEfiMdePkgTokenSpaceGuid
  #-------------------------------
  gEfiMdePkgTokenSpaceGuid.PcdMaximumUnicodeStringLength|1000000
  gEfiMdePkgTokenSpaceGuid.PcdMaximumAsciiStringLength|1000000
  gEfiMdePkgTokenSpaceGuid.PcdMaximumLinkedListLength|1000000
  gEfiMdePkgTokenSpaceGuid.PcdSpinLockTimeout|10000000
  gEfiMdePkgTokenSpaceGuid.PcdDebugClearMemoryValue|0xAF
  gEfiMdePkgTokenSpaceGuid.PcdPerformanceLibraryPropertyMask|0
  gEfiMdePkgTokenSpaceGuid.PcdPostCodePropertyMask|0
  gEfiMdePkgTokenSpaceGuid.PcdUefiLibMaxPrintBufferSize|320

  # DEBUG_ASSERT_ENABLED       0x01
  # DEBUG_PRINT_ENABLED        0x02
  # DEBUG_CODE_ENABLED         0x04
  # CLEAR_MEMORY_ENABLED       0x08
  # ASSERT_BREAKPOINT_ENABLED  0x10
  # ASSERT_DEADLOOP_ENABLED    0x20
!if $(TARGET) == RELEASE
  gEfiMdePkgTokenSpaceGuid.PcdDebugPropertyMask|0x2f
!else
  gEfiMdePkgTokenSpaceGuid.PcdDebugPropertyMask|0x2f
!endif

  #  DEBUG_INIT      0x00000001  // Initialization
  #  DEBUG_WARN      0x00000002  // Warnings
  #  DEBUG_LOAD      0x00000004  // Load events
  #  DEBUG_FS        0x00000008  // EFI File system
  #  DEBUG_POOL      0x00000010  // Alloc & Free's
  #  DEBUG_PAGE      0x00000020  // Alloc & Free's
  #  DEBUG_INFO      0x00000040  // Verbose
  #  DEBUG_DISPATCH  0x00000080  // PEI/DXE Dispatchers
  #  DEBUG_VARIABLE  0x00000100  // Variable
  #  DEBUG_BM        0x00000400  // Boot Manager
  #  DEBUG_BLKIO     0x00001000  // BlkIo Driver
  #  DEBUG_NET       0x00004000  // SNI Driver
  #  DEBUG_UNDI      0x00010000  // UNDI Driver
  #  DEBUG_LOADFILE  0x00020000  // UNDI Driver
  #  DEBUG_EVENT     0x00080000  // Event messages
  #  DEBUG_GCD       0x00100000  // Global Coherency Database changes
  #  DEBUG_CACHE     0x00200000  // Memory range cachability changes
  #  DEBUG_ERROR     0x80000000  // Error
  gEfiMdePkgTokenSpaceGuid.PcdDebugPrintErrorLevel|0x803000CF

  gEfiMdePkgTokenSpaceGuid.PcdReportStatusCodePropertyMask|0x00

  #-------------------------------
  # gEmbeddedTokenSpaceGuid
  #-------------------------------

  # ISP1761 USB OTG Controller
  #gEmbeddedTokenSpaceGuid.PcdIsp1761BaseAddress|0xFFB00000

  # LAN9118 Ethernet Driver PCDs
  #gEmbeddedTokenSpaceGuid.PcdLan9118DxeBaseAddress|0xFF800000

  gEmbeddedTokenSpaceGuid.PcdEmbeddedAutomaticBootCommand|""
  gEmbeddedTokenSpaceGuid.PcdEmbeddedDefaultTextColor|0x07
  gEmbeddedTokenSpaceGuid.PcdEmbeddedMemVariableStoreSize|0x10000

  #
  # Optional feature to help prevent EFI memory map fragments
  # Turned on and off via: PcdPrePiProduceMemoryTypeInformationHob
  # Values are in EFI Pages (4K). DXE Core will make sure that
  # at least this much of each type of memory can be allocated
  # from a single memory range. This way you only end up with
  # maximum of two fragements for each type in the memory map
  # (the memory used, and the free memory that was prereserved
  # but not used).
  #
  gEmbeddedTokenSpaceGuid.PcdMemoryTypeEfiACPIReclaimMemory|0
  gEmbeddedTokenSpaceGuid.PcdMemoryTypeEfiACPIMemoryNVS|0
  gEmbeddedTokenSpaceGuid.PcdMemoryTypeEfiReservedMemoryType|0
  gEmbeddedTokenSpaceGuid.PcdMemoryTypeEfiRuntimeServicesData|80
  gEmbeddedTokenSpaceGuid.PcdMemoryTypeEfiRuntimeServicesCode|65
  gEmbeddedTokenSpaceGuid.PcdMemoryTypeEfiBootServicesCode|400
  gEmbeddedTokenSpaceGuid.PcdMemoryTypeEfiBootServicesData|20000
  gEmbeddedTokenSpaceGuid.PcdMemoryTypeEfiLoaderCode|20
  gEmbeddedTokenSpaceGuid.PcdMemoryTypeEfiLoaderData|0

  # We want to use the Shell Libraries but don't want it to initialise
  # automatically. We initialise the libraries when the command is called by the
  # Shell.
  gEfiShellPkgTokenSpaceGuid.PcdShellLibAutoInitialize|FALSE

  ##----------------------------------------
  ## Pcd Settings - UART Serial Terminal
  ##----------------------------------------
  ## Base address of 16550 serial port registers in MMIO space.
  # @Prompt Base address of serial port registers.
  # Altera SoC FPGA HPS UART0 is 0xFFC02000.
  # Altera SoC FPGA HPS UART1 is 0xFFC02100.
  # To disable set 0 as base address
  gEfiMdeModulePkgTokenSpaceGuid.PcdSerialRegisterBase|0xFFC02000
  #gEfiMdeModulePkgTokenSpaceGuid.PcdSerialRegisterBase|0x60000
  ## UART clock frequency is for the baud rate configuration.
  # @Prompt Serial Port Clock Rate.
  # On PC this is 1.8432 MHz - 1843200
  # On Altera SoC FPGA HPS UART clock source is l4_sp_clk
  gEfiMdeModulePkgTokenSpaceGuid.PcdSerialClockRate|100000000

  ## Baud rate for the 16550 serial port.  Default is 115200 baud.
  # @Prompt Baud rate for serial port.
  # @ValidList  0x80000001 | 921600, 460800, 230400, 115200, 57600, 38400, 19200, 9600, 7200, 4800, 3600, 2400, 2000, 1800, 1200, 600, 300, 150, 134, 110, 75, 50
  gEfiMdeModulePkgTokenSpaceGuid.PcdSerialBaudRate|115200

  ## Line Control Register (LCR) for the 16550 serial port. This encodes data bits, parity, and stop bits.<BR><BR>
  #    BIT1..BIT0 - Data bits.  00b = 5 bits, 01b = 6 bits, 10b = 7 bits, 11b = 8 bits<BR>
  #    BIT2       - Stop Bits.  0 = 1 stop bit.  1 = 1.5 stop bits if 5 data bits selected, otherwise 2 stop bits.<BR>
  #    BIT5..BIT3 - Parity.  xx0b = No Parity, 001b = Odd Parity, 011b = Even Parity, 101b = Mark Parity, 111b=Stick Parity<BR>
  #    BIT7..BIT6 - Reserved.  Must be 0.<BR>
  #
  #  Default is No Parity, 8 Data Bits, 1 Stop Bit.<BR>
  # @Prompt Serial port Line Control settings.
  # @Expression 0x80000002 | (gEfiMdeModulePkgTokenSpaceGuid.PcdSerialLineControl & 0xC0) == 0
  gEfiMdeModulePkgTokenSpaceGuid.PcdSerialLineControl|0x03

  # 0-PCANSI, 1-VT100, 2-VT00+, 3-UTF8
  gEfiMdePkgTokenSpaceGuid.PcdDefaultTerminalType|1

  # Use the serial console (ConIn & ConOut) and the Graphic driver (ConOut)
  gArmPlatformTokenSpaceGuid.PcdDefaultConOutPaths|L"VenHw(D3987D4B-971A-435F-8CAF-4967EB627241)/Uart(115200,8,N,1)/VenMsg(7D916D80-5BB1-458C-A48F-E25FDD51EF94)"
  gArmPlatformTokenSpaceGuid.PcdDefaultConInPaths|L"VenHw(D3987D4B-971A-435F-8CAF-4967EB627241)/Uart(115200,8,N,1)/VenMsg(7D916D80-5BB1-458C-A48F-E25FDD51EF94)"

  # Column/Row
  #gEfiMdeModulePkgTokenSpaceGuid.PcdConOutColumn|L"ConOutSetupVar"|gArmGlobalVariableGuid|0x0|132
  #gEfiMdeModulePkgTokenSpaceGuid.PcdConOutRow|L"ConOutSetupVar"|gArmGlobalVariableGuid|0x4|43

  gEfiMdePkgTokenSpaceGuid.PcdPlatformBootTimeOut|5

  # RunAxf support via Dynamic Shell Command protocol
  # We want to use the Shell Libraries but don't want it to initialise
  # automatically. We initialise the libraries when the command is called by the
  # Shell.
  gEfiShellPkgTokenSpaceGuid.PcdShellLibAutoInitialize|FALSE

!if $(USE_ARM_BDS) == FALSE
  gEfiMdeModulePkgTokenSpaceGuid.PcdResetOnMemoryTypeInformationChange|FALSE
  gEfiIntelFrameworkModulePkgTokenSpaceGuid.PcdShellFile|{ 0x83, 0xA5, 0x04, 0x7C, 0x3E, 0x9E, 0x1C, 0x4F, 0xAD, 0x65, 0xE0, 0x52, 0x68, 0xD0, 0xB4, 0xD1 }
!endif

!if $(SECURE_BOOT_ENABLE) == TRUE
  # override the default values from SecurityPkg to ensure images from all sources are verified in secure boot
  gEfiSecurityPkgTokenSpaceGuid.PcdOptionRomImageVerificationPolicy|0x04
  gEfiSecurityPkgTokenSpaceGuid.PcdFixedMediaImageVerificationPolicy|0x04
  gEfiSecurityPkgTokenSpaceGuid.PcdRemovableMediaImageVerificationPolicy|0x04
!endif

  ##----------------------------------------
  ## UEFI Boot Menu Default Selections
  ##----------------------------------------
  gArmPlatformTokenSpaceGuid.PcdDefaultBootDescription|L"Default: Shell"
  gArmPlatformTokenSpaceGuid.PcdDefaultBootDevicePath|L"Fv(5EDA4200-2C5F-43CB-9DA3-0BAF74B1B30C)/Shell.efi"
  gArmPlatformTokenSpaceGuid.PcdDefaultBootArgument|L"-delay 0"

  #FDT address should be above the kernel.
  gArmTokenSpaceGuid.PcdArmLinuxFdtMaxOffset|0x10000

[PcdsFeatureFlag.common]

  #-------------------------------
  # gArmPlatformTokenSpaceGuid
  #-------------------------------
  gArmPlatformTokenSpaceGuid.PcdStandalone|TRUE
  gArmPlatformTokenSpaceGuid.PcdSystemMemoryInitializeInSec|FALSE
  gArmPlatformTokenSpaceGuid.PcdSendSgiToBringUpSecondaryCores|FALSE

  #-------------------------------
  # gArmTokenSpaceGuid
  #-------------------------------
  # Use the Vector Table location in CpuDxe. We will not copy the Vector Table at PcdCpuVectorBaseAddress
  gArmTokenSpaceGuid.PcdRelocateVectorTable|FALSE

  #-------------------------------
  # gEfiMdeModulePkgTokenSpaceGuid
  #-------------------------------
  gEfiMdeModulePkgTokenSpaceGuid.PcdTurnOffUsbLegacySupport|TRUE
  ## If TRUE, Graphics Output Protocol will be installed on virtual handle created by ConsplitterDxe.
  #  It could be set FALSE to save size.
  gEfiMdeModulePkgTokenSpaceGuid.PcdConOutGopSupport|FALSE

  #-------------------------------
  # gEfiMdePkgTokenSpaceGuid
  #-------------------------------
  gEfiMdePkgTokenSpaceGuid.PcdComponentNameDisable|TRUE
  gEfiMdePkgTokenSpaceGuid.PcdDriverDiagnosticsDisable|TRUE
  gEfiMdePkgTokenSpaceGuid.PcdComponentName2Disable|TRUE
  gEfiMdePkgTokenSpaceGuid.PcdDriverDiagnostics2Disable|TRUE

  #-------------------------------
  # gEmbeddedTokenSpaceGuid
  #-------------------------------
  #
  # Control what commands are supported from the UI
  # Turn these on and off to add features or save size
  #
  gEmbeddedTokenSpaceGuid.PcdEmbeddedMacBoot|TRUE
  gEmbeddedTokenSpaceGuid.PcdEmbeddedDirCmd|TRUE
  gEmbeddedTokenSpaceGuid.PcdEmbeddedHobCmd|TRUE
  gEmbeddedTokenSpaceGuid.PcdEmbeddedHwDebugCmd|TRUE
  gEmbeddedTokenSpaceGuid.PcdEmbeddedPciDebugCmd|TRUE
  gEmbeddedTokenSpaceGuid.PcdEmbeddedIoEnable|FALSE
  gEmbeddedTokenSpaceGuid.PcdEmbeddedScriptCmd|FALSE

  gEmbeddedTokenSpaceGuid.PcdCacheEnable|TRUE
  gEmbeddedTokenSpaceGuid.PcdPrePiProduceMemoryTypeInformationHob|TRUE

#*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-#
# Library Classes
#*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-#

[LibraryClasses.common]
  ArmGenericTimerCounterLib|ArmPkg/Library/ArmGenericTimerPhyCounterLib/ArmGenericTimerPhyCounterLib.inf
  AcpiLib|EmbeddedPkg/Library/AcpiLib/AcpiLib.inf
  AlteraSdMmcLib|AlteraPlatformPkg/Stratix10SoCPkg/Library/AlteraSdMmcLib/AlteraSdMmcLib.inf
  ArmCpuLib|ArmPkg/Drivers/ArmCpuLib/ArmCortexA5xLib/ArmCortexA5xLib.inf
  ArmDisassemblerLib|ArmPkg/Library/ArmDisassemblerLib/ArmDisassemblerLib.inf
  ArmGicLib|ArmPkg/Drivers/ArmGic/ArmGicLib.inf

  ArmLib|ArmPkg/Library/ArmLib/AArch64/AArch64Lib.inf

  ArmPlatformStackLib|ArmPlatformPkg/Library/ArmPlatformStackLib/ArmPlatformStackLib.inf
  ArmSmcLib|ArmPkg/Library/ArmSmcLib/ArmSmcLib.inf
  ArmTrustZoneLib|ArmPlatformPkg/Drivers/ArmTrustZone/ArmTrustZone.inf
  BaseLib|MdePkg/Library/BaseLib/BaseLib.inf
  BaseMemoryLib|ArmPkg/Library/BaseMemoryLibStm/BaseMemoryLibStm.inf
  BdsLib|ArmPkg/Library/BdsLib/BdsLib.inf
  CacheMaintenanceLib|ArmPkg/Library/ArmCacheMaintenanceLib/ArmCacheMaintenanceLib.inf
  CpuExceptionHandlerLib|MdeModulePkg/Library/CpuExceptionHandlerLibNull/CpuExceptionHandlerLibNull.inf
  CpuLib|MdePkg/Library/BaseCpuLib/BaseCpuLib.inf
  DefaultExceptionHandlerLib|ArmPkg/Library/DefaultExceptionHandlerLib/DefaultExceptionHandlerLib.inf
  DevicePathLib|MdePkg/Library/UefiDevicePathLib/UefiDevicePathLib.inf
  DmaLib|ArmPkg/Library/ArmDmaLib/ArmDmaLib.inf
  DxeServicesTableLib|MdePkg/Library/DxeServicesTableLib/DxeServicesTableLib.inf
  EblAddExternalCommandLib|EmbeddedPkg/Library/EblAddExternalCommandLib/EblAddExternalCommandLib.inf
  EblCmdLib|ArmPlatformPkg/Library/EblCmdLib/EblCmdLib.inf
  EblNetworkLib|EmbeddedPkg/Library/EblNetworkLib/EblNetworkLib.inf
  EfiFileLib|EmbeddedPkg/Library/EfiFileLib/EfiFileLib.inf
  EfiResetSystemLib|AlteraPlatformPkg/Library/AlteraHpsResetSystemLib/AlteraHpsResetSystemLib.inf
  FdtLib|EmbeddedPkg/Library/FdtLib/FdtLib.inf
  HiiLib|MdeModulePkg/Library/UefiHiiLib/UefiHiiLib.inf
  HobLib|MdePkg/Library/DxeHobLib/DxeHobLib.inf
  IoLib|MdePkg/Library/BaseIoLibIntrinsic/BaseIoLibIntrinsic.inf
  L2X0CacheLib|ArmPlatformPkg/Drivers/PL310L2Cache/PL310L2CacheSec.inf
  PcdLib|MdePkg/Library/BasePcdLibNull/BasePcdLibNull.inf
  PeCoffGetEntryPointLib|MdePkg/Library/BasePeCoffGetEntryPointLib/BasePeCoffGetEntryPointLib.inf
  PeCoffLib|MdePkg/Library/BasePeCoffLib/BasePeCoffLib.inf
  PerformanceLib|MdePkg/Library/BasePerformanceLibNull/BasePerformanceLibNull.inf
  PrintLib|MdePkg/Library/BasePrintLib/BasePrintLib.inf
  RealTimeClockLib|AlteraPlatformPkg/Library/AlteraNullRealTimeClockLib/AlteraNullRealTimeClockLib.inf
  SemihostLib|AlteraPlatformPkg/Library/SemihostLib/SemihostLib.inf
  SerialPortExtLib|EmbeddedPkg/Library/SerialPortExtLibNull/SerialPortExtLibNull.inf
  SerialPortLib|AlteraPlatformPkg/Library/Mmio16550SerialPortLib/Mmio16550SerialPortLib.inf
  SerialPortPrintLib|AlteraPlatformPkg/Library/SerialPortPrintLib/SerialPortPrintLib.inf
  SynchronizationLib|MdePkg/Library/BaseSynchronizationLib/BaseSynchronizationLib.inf
  UefiBootManagerLib|MdeModulePkg/Library/UefiBootManagerLib/UefiBootManagerLib.inf

  # replace hps timer by arm timer
  TimerLib|ArmPkg/Library/ArmArchTimerLib/ArmArchTimerLib.inf

  UefiApplicationEntryPoint|MdePkg/Library/UefiApplicationEntryPoint/UefiApplicationEntryPoint.inf
  UefiBootServicesTableLib|MdePkg/Library/UefiBootServicesTableLib/UefiBootServicesTableLib.inf
  UefiDecompressLib|MdePkg/Library/BaseUefiDecompressLib/BaseUefiDecompressLib.inf
  UefiDriverEntryPoint|MdePkg/Library/UefiDriverEntryPoint/UefiDriverEntryPoint.inf
  UefiHiiServicesLib|MdeModulePkg/Library/UefiHiiServicesLib/UefiHiiServicesLib.inf
  UefiLib|MdePkg/Library/UefiLib/UefiLib.inf
  UefiRuntimeLib|MdePkg/Library/UefiRuntimeLib/UefiRuntimeLib.inf
  UefiRuntimeServicesTableLib|MdePkg/Library/UefiRuntimeServicesTableLib/UefiRuntimeServicesTableLib.inf

  # RunAxf support via Dynamic Shell Command protocol
  # It uses the Shell libraries.
  ArmShellCmdRunAxfLib|ArmPlatformPkg/Library/ArmShellCmdRunAxf/ArmShellCmdRunAxf.inf
  ShellLib|ShellPkg/Library/UefiShellLib/UefiShellLib.inf
  ShellCEntryLib|ShellPkg/Library/UefiShellCEntryLib/UefiShellCEntryLib.inf
  FileHandleLib|MdePkg/Library/UefiFileHandleLib/UefiFileHandleLib.inf
  SortLib|MdeModulePkg/Library/UefiSortLib/UefiSortLib.inf

  #
  # Secure Boot dependencies
  #
!if $(SECURE_BOOT_ENABLE) == TRUE
  IntrinsicLib|CryptoPkg/Library/IntrinsicLib/IntrinsicLib.inf
  OpensslLib|CryptoPkg/Library/OpensslLib/OpensslLib.inf
  TpmMeasurementLib|SecurityPkg/Library/DxeTpmMeasurementLib/DxeTpmMeasurementLib.inf
  AuthVariableLib|SecurityPkg/Library/AuthVariableLib/AuthVariableLib.inf
  BaseCryptLib|CryptoPkg/Library/BaseCryptLib/BaseCryptLib.inf

  # re-use the UserPhysicalPresent() dummy implementation from the ovmf tree
  PlatformSecureLib|OvmfPkg/Library/PlatformSecureLib/PlatformSecureLib.inf
!else
  TpmMeasurementLib|MdeModulePkg/Library/TpmMeasurementLibNull/TpmMeasurementLibNull.inf
  AuthVariableLib|MdeModulePkg/Library/AuthVariableLibNull/AuthVariableLibNull.inf
!endif
  VarCheckLib|MdeModulePkg/Library/VarCheckLib/VarCheckLib.inf

!if $(USE_ARM_BDS) == FALSE
  CapsuleLib|MdeModulePkg/Library/DxeCapsuleLibNull/DxeCapsuleLibNull.inf
  GenericBdsLib|IntelFrameworkModulePkg/Library/GenericBdsLib/GenericBdsLib.inf
  PlatformBdsLib|ArmPlatformPkg/Library/PlatformIntelBdsLib/PlatformIntelBdsLib.inf
  CustomizedDisplayLib|MdeModulePkg/Library/CustomizedDisplayLib/CustomizedDisplayLib.inf
!endif

  #-------------------------------
  # Networking Requirements
  #-------------------------------
  DpcLib|MdeModulePkg/Library/DxeDpcLib/DxeDpcLib.inf
  IpIoLib|MdeModulePkg/Library/DxeIpIoLib/DxeIpIoLib.inf
  NetLib|MdeModulePkg/Library/DxeNetLib/DxeNetLib.inf
  UdpIoLib|MdeModulePkg/Library/DxeUdpIoLib/DxeUdpIoLib.inf

  #-------------------------------
  # These libraries are used by the dynamic EFI Shell commands
  #-------------------------------
  ShellLib|ShellPkg/Library/UefiShellLib/UefiShellLib.inf
  FileHandleLib|MdePkg/Library/UefiFileHandleLib/UefiFileHandleLib.inf
  SortLib|MdeModulePkg/Library/UefiSortLib/UefiSortLib.inf
  LibC|StdLib/LibC/LibC.inf

  #-------------------------------
  # Build Debug / Release
  #-------------------------------
!if $(TARGET) == RELEASE
  #DebugLib|MdePkg/Library/BaseDebugLibNull/BaseDebugLibNull.inf
  DebugLib|MdePkg/Library/BaseDebugLibSerialPort/BaseDebugLibSerialPort.inf
  UncachedMemoryAllocationLib|ArmPkg/Library/UncachedMemoryAllocationLib/UncachedMemoryAllocationLib.inf
!else
  DebugLib|MdePkg/Library/BaseDebugLibSerialPort/BaseDebugLibSerialPort.inf
  UncachedMemoryAllocationLib|ArmPkg/Library/UncachedMemoryAllocationLib/UncachedMemoryAllocationLib.inf
#  UncachedMemoryAllocationLib|ArmPkg/Library/DebugUncachedMemoryAllocationLib/DebugUncachedMemoryAllocationLib.inf
!endif
  DebugAgentLib|MdeModulePkg/Library/DebugAgentLibNull/DebugAgentLibNull.inf
  DebugAgentTimerLib|EmbeddedPkg/Library/DebugAgentTimerLibNull/DebugAgentTimerLibNull.inf
  DebugPrintErrorLevelLib|MdePkg/Library/BaseDebugPrintErrorLevelLib/BaseDebugPrintErrorLevelLib.inf
  PeCoffExtraActionLib|ArmPkg/Library/DebugPeCoffExtraActionLib/DebugPeCoffExtraActionLib.inf

[LibraryClasses.common.SEC, LibraryClasses.common.PEI_CORE]
  ArmGicArchLib|AlteraPlatformPkg/Library/ArmGicArchSecLib/ArmGicArchSecLib.inf
  ArmPlatformLib|AlteraPlatformPkg/Stratix10SoCPkg/Library/AlteraPlatformLib/AlteraPlatformLibSec.inf
  ArmLib|ArmPkg/Library/ArmLib/AArch64/AArch64LibSec.inf
  DefaultExceptionHandlerLib|ArmPkg/Library/DefaultExceptionHandlerLib/DefaultExceptionHandlerLibBase.inf 
 
[LibraryClasses.common.DXE_DRIVER, LibraryClasses.common.UEFI_APPLICATION, LibraryClasses.common.UEFI_DRIVER]
  ArmGicArchLib|ArmPkg/Library/ArmGicArchLib/ArmGicArchLib.inf

[LibraryClasses.common.PEI_CORE]
  ArmLib|ArmPkg/Library/ArmLib/AArch64/AArch64Lib.inf
  ArmPlatformGlobalVariableLib|ArmPlatformPkg/Library/ArmPlatformGlobalVariableLib/Pei/PeiArmPlatformGlobalVariableLib.inf
  ExtractGuidedSectionLib|EmbeddedPkg/Library/PrePiExtractGuidedSectionLib/PrePiExtractGuidedSectionLib.inf
  HobLib|EmbeddedPkg/Library/PrePiHobLib/PrePiHobLib.inf
  LzmaDecompressLib|MdeModulePkg/Library/LzmaCustomDecompressLib/LzmaCustomDecompressLib.inf
  MemoryAllocationLib|AlteraPlatformPkg/Library/AlteraPeiPostDdrMemoryAllocationLib/AlteraPeiPostDdrMemoryAllocationLib.inf
  OemHookStatusCodeLib|MdeModulePkg/Library/OemHookStatusCodeLibNull/OemHookStatusCodeLibNull.inf
  PeCoffGetEntryPointLib|MdePkg/Library/BasePeCoffGetEntryPointLib/BasePeCoffGetEntryPointLib.inf
  PeiCoreEntryPoint|MdePkg/Library/PeiCoreEntryPoint/PeiCoreEntryPoint.inf
  PeiServicesLib|MdePkg/Library/PeiServicesLib/PeiServicesLib.inf
  PeiServicesTablePointerLib|ArmPlatformPkg/Library/PeiServicesTablePointerLib/PeiServicesTablePointerLib.inf
  PerformanceLib|MdeModulePkg/Library/PeiPerformanceLib/PeiPerformanceLib.inf
  PrePiHobListPointerLib|ArmPlatformPkg/Library/PrePiHobListPointerLib/PrePiHobListPointerLib.inf
  PrePiLib|EmbeddedPkg/Library/PrePiLib/PrePiLib.inf
  ReportStatusCodeLib|MdeModulePkg/Library/PeiReportStatusCodeLib/PeiReportStatusCodeLib.inf
  UefiDecompressLib|MdePkg/Library/BaseUefiDecompressLib/BaseUefiDecompressLib.inf
  PeiCrc32GuidedSectionExtractLib|MdeModulePkg/Library/PeiCrc32GuidedSectionExtractLib/PeiCrc32GuidedSectionExtractLib.inf

[LibraryClasses.common.PEIM]
  ArmPlatformGlobalVariableLib|ArmPlatformPkg/Library/ArmPlatformGlobalVariableLib/Pei/PeiArmPlatformGlobalVariableLib.inf
  ExtractGuidedSectionLib|MdePkg/Library/PeiExtractGuidedSectionLib/PeiExtractGuidedSectionLib.inf
  HobLib|MdePkg/Library/PeiHobLib/PeiHobLib.inf
  MemoryAllocationLib|MdePkg/Library/PeiMemoryAllocationLib/PeiMemoryAllocationLib.inf
  OemHookStatusCodeLib|MdeModulePkg/Library/OemHookStatusCodeLibNull/OemHookStatusCodeLibNull.inf
  PeCoffGetEntryPointLib|MdePkg/Library/BasePeCoffGetEntryPointLib/BasePeCoffGetEntryPointLib.inf
  PeimEntryPoint|MdePkg/Library/PeimEntryPoint/PeimEntryPoint.inf
  PeiResourcePublicationLib|MdePkg/Library/PeiResourcePublicationLib/PeiResourcePublicationLib.inf
  PeiServicesLib|MdePkg/Library/PeiServicesLib/PeiServicesLib.inf
  PeiServicesTablePointerLib|ArmPlatformPkg/Library/PeiServicesTablePointerLib/PeiServicesTablePointerLib.inf
  PerformanceLib|MdeModulePkg/Library/PeiPerformanceLib/PeiPerformanceLib.inf
  ReportStatusCodeLib|MdeModulePkg/Library/PeiReportStatusCodeLib/PeiReportStatusCodeLib.inf
  UefiDecompressLib|MdePkg/Library/BaseUefiDecompressLib/BaseUefiDecompressLib.inf
  MemoryInitPeiLib|ArmPlatformPkg/MemoryInitPei/MemoryInitPeiLib.inf  

[LibraryClasses.common.DXE_CORE]
  DxeCoreEntryPoint|MdePkg/Library/DxeCoreEntryPoint/DxeCoreEntryPoint.inf
  DxeServicesLib|MdePkg/Library/DxeServicesLib/DxeServicesLib.inf
  ExtractGuidedSectionLib|MdePkg/Library/DxeExtractGuidedSectionLib/DxeExtractGuidedSectionLib.inf
  HobLib|MdePkg/Library/DxeCoreHobLib/DxeCoreHobLib.inf
  MemoryAllocationLib|MdeModulePkg/Library/DxeCoreMemoryAllocationLib/DxeCoreMemoryAllocationLib.inf
  PerformanceLib|MdeModulePkg/Library/DxeCorePerformanceLib/DxeCorePerformanceLib.inf
  ReportStatusCodeLib|IntelFrameworkModulePkg/Library/DxeReportStatusCodeLibFramework/DxeReportStatusCodeLib.inf
  UefiDecompressLib|MdePkg/Library/BaseUefiDecompressLib/BaseUefiDecompressLib.inf

[LibraryClasses.common.DXE_DRIVER]
  ArmPlatformGlobalVariableLib|ArmPlatformPkg/Library/ArmPlatformGlobalVariableLib/Dxe/DxeArmPlatformGlobalVariableLib.inf
  DxeServicesLib|MdePkg/Library/DxeServicesLib/DxeServicesLib.inf
  MemoryAllocationLib|MdePkg/Library/UefiMemoryAllocationLib/UefiMemoryAllocationLib.inf
  SecurityManagementLib|MdeModulePkg/Library/DxeSecurityManagementLib/DxeSecurityManagementLib.inf
  PerformanceLib|MdeModulePkg/Library/DxePerformanceLib/DxePerformanceLib.inf
  ReportStatusCodeLib|IntelFrameworkModulePkg/Library/DxeReportStatusCodeLibFramework/DxeReportStatusCodeLib.inf

[LibraryClasses.common.UEFI_APPLICATION]
  HiiLib|MdeModulePkg/Library/UefiHiiLib/UefiHiiLib.inf
  MemoryAllocationLib|MdePkg/Library/UefiMemoryAllocationLib/UefiMemoryAllocationLib.inf
  PerformanceLib|MdeModulePkg/Library/DxePerformanceLib/DxePerformanceLib.inf
  UefiDecompressLib|IntelFrameworkModulePkg/Library/BaseUefiTianoCustomDecompressLib/BaseUefiTianoCustomDecompressLib.inf

[LibraryClasses.common.UEFI_DRIVER]
  DxeServicesLib|MdePkg/Library/DxeServicesLib/DxeServicesLib.inf
  ExtractGuidedSectionLib|MdePkg/Library/DxeExtractGuidedSectionLib/DxeExtractGuidedSectionLib.inf
  MemoryAllocationLib|MdePkg/Library/UefiMemoryAllocationLib/UefiMemoryAllocationLib.inf
  PerformanceLib|MdeModulePkg/Library/DxePerformanceLib/DxePerformanceLib.inf
  ReportStatusCodeLib|IntelFrameworkModulePkg/Library/DxeReportStatusCodeLibFramework/DxeReportStatusCodeLib.inf
  UefiDecompressLib|IntelFrameworkModulePkg/Library/BaseUefiTianoCustomDecompressLib/BaseUefiTianoCustomDecompressLib.inf

[LibraryClasses.common.DXE_RUNTIME_DRIVER]
  ArmPlatformSysConfigLib|ArmPlatformPkg/ArmVExpressPkg/Library/ArmVExpressSysConfigRuntimeLib/ArmVExpressSysConfigRuntimeLib.inf
!if $(SECURE_BOOT_ENABLE) == TRUE
  BaseCryptLib|CryptoPkg/Library/BaseCryptLib/RuntimeCryptLib.inf
!endif
  CapsuleLib|MdeModulePkg/Library/DxeCapsuleLibNull/DxeCapsuleLibNull.inf
  HobLib|MdePkg/Library/DxeHobLib/DxeHobLib.inf
  MemoryAllocationLib|MdePkg/Library/UefiMemoryAllocationLib/UefiMemoryAllocationLib.inf
  ReportStatusCodeLib|IntelFrameworkModulePkg/Library/DxeReportStatusCodeLibFramework/DxeReportStatusCodeLib.inf

[LibraryClasses.AARCH64.DXE_RUNTIME_DRIVER]
  #
  # PSCI support in EL3 may not be available if we are not running under a PSCI
  # compliant secure firmware, but since the default VExpress EfiResetSystemLib
  # cannot be supported at runtime (due to the fact that the syscfg MMIO registers
  # cannot be runtime remapped), it is our best bet to get ResetSystem functionality
  # on these platforms.
  #
  EfiResetSystemLib|ArmPkg/Library/ArmPsciResetSystemLib/ArmPsciResetSystemLib.inf

[LibraryClasses.ARM, LibraryClasses.AARCH64]
  #
  # It is not possible to prevent the ARM compiler for generic intrinsic functions.
  # This library provides the instrinsic functions generate by a given compiler.
  # [LibraryClasses.ARM] and NULL mean link this library into all ARM images.
  #
  NULL|ArmPkg/Library/CompilerIntrinsicsLib/CompilerIntrinsicsLib.inf

  # Add support for GCC stack protector
  NULL|MdePkg/Library/BaseStackCheckLib/BaseStackCheckLib.inf

################################################################################
#
# Components Section - list of all EDK II Modules needed by this Platform
#
################################################################################
[Components.common]
  MdeModulePkg/Universal/PCD/Dxe/Pcd.inf


  #
  # Pre-PEI Phase modules
  #
  ArmPlatformPkg/PrePeiCore/PrePeiCoreMPCore.inf {
    <LibraryClasses>
      ArmPlatformGlobalVariableLib|ArmPlatformPkg/Library/ArmPlatformGlobalVariableLib/Pei/PeiArmPlatformGlobalVariableLib.inf
  }
  #
  # PEI Phase modules
  #
  AlteraPlatformPkg/Stratix10SoCPkg/PlatformPei/AlteraSocFpgaPeiMain.inf {
    <LibraryClasses>
      NULL|MdeModulePkg/Library/LzmaCustomDecompressLib/LzmaCustomDecompressLib.inf
  }
  #
  # DXE
  #
  MdeModulePkg/Core/Dxe/DxeMain.inf {
    <LibraryClasses>
      PcdLib|MdePkg/Library/BasePcdLibNull/BasePcdLibNull.inf
      NULL|MdeModulePkg/Library/DxeCrc32GuidedSectionExtractLib/DxeCrc32GuidedSectionExtractLib.inf
  }

  AlteraPlatformPkg/Bds/Bds.inf
  ArmPkg/Drivers/TimerDxe/TimerDxe.inf

  ArmPkg/Drivers/ArmGic/ArmGicDxe.inf
  ArmPkg/Drivers/CpuDxe/CpuDxe.inf
  ArmPkg/Filesystem/SemihostFs/SemihostFs.inf

  EmbeddedPkg/MetronomeDxe/MetronomeDxe.inf
  EmbeddedPkg/RealTimeClockRuntimeDxe/RealTimeClockRuntimeDxe.inf
  EmbeddedPkg/ResetRuntimeDxe/ResetRuntimeDxe.inf
  EmbeddedPkg/SerialDxe/SerialDxe.inf

  MdeModulePkg/Core/RuntimeDxe/RuntimeDxe.inf
  MdeModulePkg/Universal/CapsuleRuntimeDxe/CapsuleRuntimeDxe.inf
  MdeModulePkg/Universal/Console/ConPlatformDxe/ConPlatformDxe.inf
  MdeModulePkg/Universal/Console/ConSplitterDxe/ConSplitterDxe.inf
  MdeModulePkg/Universal/Console/GraphicsConsoleDxe/GraphicsConsoleDxe.inf
  MdeModulePkg/Universal/Console/TerminalDxe/TerminalDxe.inf
  MdeModulePkg/Universal/DevicePathDxe/DevicePathDxe.inf
  MdeModulePkg/Universal/Disk/DiskIoDxe/DiskIoDxe.inf
  MdeModulePkg/Universal/Disk/PartitionDxe/PartitionDxe.inf
  MdeModulePkg/Universal/Disk/UnicodeCollation/EnglishDxe/EnglishDxe.inf
  MdeModulePkg/Universal/FaultTolerantWriteDxe/FaultTolerantWriteDxe.inf
  MdeModulePkg/Universal/FvSimpleFileSystemDxe/FvSimpleFileSystemDxe.inf
  MdeModulePkg/Universal/HiiDatabaseDxe/HiiDatabaseDxe.inf
  MdeModulePkg/Universal/MonotonicCounterRuntimeDxe/MonotonicCounterRuntimeDxe.inf
  MdeModulePkg/Universal/SecurityStubDxe/SecurityStubDxe.inf
  MdeModulePkg/Universal/Variable/EmuRuntimeDxe/EmuVariableRuntimeDxe.inf
  MdeModulePkg/Universal/Variable/RuntimeDxe/VariableRuntimeDxe.inf {
    <LibraryClasses>
      NULL|MdeModulePkg/Library/VarCheckUefiLib/VarCheckUefiLib.inf
  }
  MdeModulePkg/Universal/WatchdogTimerDxe/WatchdogTimer.inf

  # Multimedia Card Interface
  EmbeddedPkg/Universal/MmcDxe/MmcDxe.inf
  AlteraPlatformPkg/Stratix10SoCPkg/Drivers/AlteraSdMmcDxe/AlteraSdMmcDxe.inf

  # DW GPIO
  AlteraPlatformPkg/Stratix10SoCPkg/Drivers/DWGpio/Gpio.inf

  #
  # Platform Specific Init for DXE phase
  #
  AlteraPlatformPkg/Stratix10SoCPkg/Drivers/AlteraPlatformDxe/AlteraPlatformDxe.inf

  #
  # Platform Specific Networking stack
  #
  AlteraPlatformPkg/Stratix10SoCPkg/Drivers/AlteraSnpDxe/AlteraSnpDxe.inf

  #
  # Universal Networking stack
  #
  MdeModulePkg/Universal/Network/ArpDxe/ArpDxe.inf
  MdeModulePkg/Universal/Network/Dhcp4Dxe/Dhcp4Dxe.inf
  MdeModulePkg/Universal/Network/DpcDxe/DpcDxe.inf
  MdeModulePkg/Universal/Network/Ip4Dxe/Ip4Dxe.inf
  MdeModulePkg/Universal/Network/IScsiDxe/IScsiDxe.inf
  MdeModulePkg/Universal/Network/MnpDxe/MnpDxe.inf
  MdeModulePkg/Universal/Network/Mtftp4Dxe/Mtftp4Dxe.inf
  MdeModulePkg/Universal/Network/Tcp4Dxe/Tcp4Dxe.inf
  MdeModulePkg/Universal/Network/Udp4Dxe/Udp4Dxe.inf
  MdeModulePkg/Universal/Network/UefiPxeBcDxe/UefiPxeBcDxe.inf
  MdeModulePkg/Universal/Network/VlanConfigDxe/VlanConfigDxe.inf

  # ISP1761 USB OTG Controller
  #EmbeddedPkg/Drivers/Isp1761UsbDxe/Isp1761UsbDxe.inf

  #
  # Android Fastboot
  #
  #EmbeddedPkg/Application/AndroidFastboot/AndroidFastbootApp.inf
  #EmbeddedPkg/Drivers/AndroidFastbootTransportUsbDxe/FastbootTransportUsbDxe.inf
  #AlteraPlatformPkg/Stratix10SoCPkg/ArmVExpressFastBootDxe/ArmVExpressFastBootDxe.inf

  #
  # FDT installation
  #
  EmbeddedPkg/Drivers/FdtPlatformDxe/FdtPlatformDxe.inf

  # Legacy Linux Loader
  AlteraPlatformPkg/Applications/LinuxLoader/LinuxLoader.inf

  # Standard EFI HelloWorld Application
  MdeModulePkg/Application/HelloWorld/HelloWorld.inf

  # DW GPIO
  AlteraPlatformPkg/Stratix10SoCPkg/Application/Gpio/GpioControl.inf
  
  # I2C
  AlteraPlatformPkg/Applications/AccessingI2C/I2c.inf

[BuildOptions]
  #-------------------------------
  # Common
  #-------------------------------
  GCC:RELEASE_*_*_CC_FLAGS  = -DMDEPKG_NDEBUG -fstack-protector -O2 -D_FORTIFY_SOURCE=2 -Wformat -Wformat-security
  GCC:RELEASE_*_*_DLINK_FLAGS  = -z noexecstack -z relro -z now

  #-------------------------------
  # AlteraPlatformPkg/...
  #-------------------------------
  CLANG35:*_*_AARCH64_PLATFORM_FLAGS   == -I$(WORKSPACE)/AlteraPlatformPkg/Stratix10SoCPkg/Include -I$(WORKSPACE)/AlteraPlatformPkg/Stratix10SoCPkg/Include
  GCC:DEBUG_*_AARCH64_PLATFORM_FLAGS   == -I$(WORKSPACE)/AlteraPlatformPkg/Stratix10SoCPkg/Include -I$(WORKSPACE)/AlteraPlatformPkg/Stratix10SoCPkg/Include -O0 -DMDEPKG_NDEBUG -Wno-unused-but-set-variable -mstrict-align
  GCC:RELEASE_*_AARCH64_PLATFORM_FLAGS == -I$(WORKSPACE)/AlteraPlatformPkg/Stratix10SoCPkg/Include -I$(WORKSPACE)/AlteraPlatformPkg/Stratix10SoCPkg/Include -O0 -mstrict-align
  XCODE:*_*_AARCH64_PLATFORM_FLAGS     == -I$(WORKSPACE)/AlteraPlatformPkg/Stratix10SoCPkg/Include -I$(WORKSPACE)/AlteraPlatformPkg/Stratix10SoCPkg/Include

[BuildOptions.AARCH64.EDKII.DXE_RUNTIME_DRIVER]
  GCC:*_*_AARCH64_DLINK_FLAGS = -z common-page-size=0x10000


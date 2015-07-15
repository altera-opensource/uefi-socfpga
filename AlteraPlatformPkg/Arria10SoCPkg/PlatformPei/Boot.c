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
#include <libfdt.h>
#include <Library/ArmGicLib.h>
#include <Library/ArmLib.h>
#include <Library/BaseMemoryLib.h>
#include <Library/DebugLib.h>
#include <Library/IoLib.h>
#include <Library/MemoryAllocationLib.h>
#include <Library/PcdLib.h>
#include <Library/PrintLib.h>
#include <Library/SerialPortPrintLib.h>
#include <Library/TimerLib.h>
#include "AlteraSdMmcPei/AlteraSdMmcPei.h"
#include "Assert.h"
#include "DeviceTree.h"
#include "SdMmc.h"
#include "Boot.h"
#include "BootSource.h"
#include "QspiLib.h"
#include "MemoryController.h"
#include "MkimageHeader.h"
#include "NandLib.h"

#if (FixedPcdGet32(PcdDebugMsg_Boot) == 0)
  //#define ProgressPrint(FormatString, ...)    /* do nothing */
  #define ProgressPrint                       SerialPortPrint
  #define InfoPrint(FormatString, ...)        /* do nothing */
  #define MmioHexDump(BaseAddr, Data32Size)   /* do nothing */
#else
  #define ProgressPrint SerialPortPrint
  #define InfoPrint     SerialPortPrint
  #define MmioHexDump   SerialPortMmioHexDump
#endif

//
// Functions
//

VOID
EFIAPI
PreparePlatformHardwareToBoot (
  VOID
  )
{
  UINTN    Index;
  UINT32   GicInterrupt;
  UINTN    MaxGicNumInterrupts;

  // Typical boot entry requirements are:
  // The CPU must be in SVC (supervisor) mode with both IRQ and FIQ interrupts disabled.
  // The MMU must be off, i.e. code running from physical RAM with no translated addressing.
  // Data cache must be off
  // Instruction cache may be either on or off

  // Disable all the interrupts
  MaxGicNumInterrupts = ArmGicGetMaxNumInterrupts (PcdGet32 (PcdGicDistributorBase));
  for (Index = 0; Index < MaxGicNumInterrupts; Index++) {
    ArmGicDisableInterrupt (PcdGet32 (PcdGicDistributorBase), Index);
  }

  // Acknowledge all pending interrupts
  #define PEI_ARM_GIC_ICCIAR_ACKINTID  0x3FF
  // Interrupts from 1020 to 1023 are considered as special interrupts (eg: spurious interrupts)
  #define PEI_ARM_GIC_IS_SPECIAL_INTERRUPTS(Interrupt) (((Interrupt) >= 1020) && ((Interrupt) <= 1023))
  do {
    // Read the Interrupt Acknowledge Register
    GicInterrupt = MmioRead32 (PcdGet32 (PcdGicInterruptInterfaceBase) + ARM_GIC_ICCIAR);
    if ((GicInterrupt & PEI_ARM_GIC_ICCIAR_ACKINTID) < MaxGicNumInterrupts) {
      //Set EndOfInterrupt
      MmioWrite32 (PcdGet32 (PcdGicInterruptInterfaceBase) + ARM_GIC_ICCEIOR, GicInterrupt);
    }
  } while (!PEI_ARM_GIC_IS_SPECIAL_INTERRUPTS (GicInterrupt));

  // Disable Gic Interface
  MmioWrite32 (PcdGet32 (PcdGicInterruptInterfaceBase) + ARM_GIC_ICCICR, 0x0);
  MmioWrite32 (PcdGet32 (PcdGicInterruptInterfaceBase) + ARM_GIC_ICCPMR, 0x0);

  // Disable Gic Distributor
  ArmGicDisableDistributor (PcdGet32 (PcdGicDistributorBase));

  // Clean before Disable else the Stack gets corrupted with old data.
  ArmCleanDataCache ();
  ArmDisableDataCache ();
  // Invalidate all the entries that might have snuck in.
  ArmInvalidateDataCache ();

  // Disable and invalidate the instruction cache
  ArmDisableInstructionCache ();
  ArmInvalidateInstructionCache ();

  // Turn off MMU
  ArmDisableMmu();

}


VOID
EFIAPI
JumpToEntry (
  IN UINTN    EntryMemoryAddr,
  IN UINTN    R0,
  IN UINTN    R1,
  IN UINTN    R2
  )
{

  BOOTIMAGE_ENTRY_POINT  EntryPoint;

  // Prepare hardware
  PreparePlatformHardwareToBoot ();

  // Jump to entry point
  EntryPoint = (BOOTIMAGE_ENTRY_POINT)(UINTN) EntryMemoryAddr;
  ProgressPrint ("Control transfered to 0x%08x with "
                 "R0 = 0x%08x "
                 "R1 = 0x%08x "
                 "R2 = 0x%08x\r\n\r\n\r\n",
                 EntryMemoryAddr, R0, R1, R2);
  EntryPoint (R0, R1, R2);
  InfoPrint ("\r\nControl returned to UEFI");
  ASSERT_PLATFORM_INIT(0);
}


VOID
EFIAPI
LoadDxeImageToRam (
  IN  UINTN            DestinationMemoryBase,
  OUT UINT32*          pFileSize
  )
{
  EFI_STATUS        Status;
  BOOT_SOURCE_TYPE  BootSourceType;

  // Detect Boot Source Type
  BootSourceType = GetBootSourceType ();

  // Flash Device Type
  switch (BootSourceType)
  {
    case BOOT_SOURCE_SDMMC:
      // Read DXE.ROM file from root folder of FAT32 partition on SD/MMC card
      LoadFileToMemory (
        (CHAR8*) PcdGetPtr (PcdSdmmc_BOOTLOADER_DXEROM_FILENAME),
        DestinationMemoryBase,
        pFileSize);
      break;
    case BOOT_SOURCE_NAND:
    case BOOT_SOURCE_QSPI:
      // Print message that we are going to read DXE.ROM from QSPI or NAND flash
      ProgressPrint ("Copying DXE from Flash Offset 0x%08lx to RAM Address 0x%08lx with image size 0x%08x\r\n",
        (UINT64) PcdGet64 (PcdQspiOrNand_BOOTLOADER_DXEROM_ADDR),
        (UINT64) DestinationMemoryBase,
        (UINT32) PcdGet32 (PcdDxeFvSize));
      if (BootSourceType == BOOT_SOURCE_QSPI)
      {
        // Read from QSPI
        Status = QspiRead(
          (VOID *) DestinationMemoryBase,
          PcdGet64 (PcdQspiOrNand_BOOTLOADER_DXEROM_ADDR),
          PcdGet32 (PcdDxeFvSize));
      } else {
        // Read from NAND
        Status = NandRead(
          (VOID *) DestinationMemoryBase,
          PcdGet64 (PcdQspiOrNand_BOOTLOADER_DXEROM_ADDR),
          PcdGet32 (PcdDxeFvSize));
      }
      // Error Checking
      if (EFI_ERROR(Status)) {
        *pFileSize = 0;
      } else {
        *pFileSize = PcdGet32 (PcdDxeFvSize);
      }
      break;
    case BOOT_SOURCE_RSVD:
    case BOOT_SOURCE_FPGA:
    default:
      // No Flash device.
      *pFileSize = 0;
      break;
  }

}


VOID
EFIAPI
LoadBootImageAndTransferControl (
  IN  BOOT_SOURCE_TYPE  BootSourceType
  )
{
  EFI_STATUS        Status;
  MKIMG_HEADER      ImgHdr;
  UINT64            FlashOffset;
  UINTN             LoadAddr;
  UINTN             EntryPoint;
  UINTN             DataSize;
  UINTN             R0;
  UINTN             R1;
  UINTN             R2;
  UINT32            BootImageDtbAddr;
  UINT32            BootImageDtbFileSize;


  LoadAddr = PcdGet32 (PcdBoot_BOOTIMAGE_MEM_LOAD_ADDR);
  EntryPoint = PcdGet32 (PcdBoot_BOOTIMAGE_CPU_JUMP_ADDR);
  DataSize = 0;
  R0 = 0;
  R1 = 0;
  R2 = 0;

  // Open and Load Boot Image
  // Objective:
  // 1. Get Data location and Size of the BootImage
  // 2. Get Memory Load Address and CPU Entry Point of the BootImage
  // 3. Copy BootImage from Flash to Memory
  switch (BootSourceType)
  {
    case BOOT_SOURCE_SDMMC:
      // Load the file to memory
      // If the file contain an MKIMAGE header,
      // it will utilize the LoadAddr and EntryPoint value from the MKIMAGE header
      // otherwiser, value from the PCDs will be used
      Status = LoadBootImageFile (
        (CHAR8*) PcdGetPtr (PcdSdmmc_BOOTIMAGE_FILENAME),
        &LoadAddr,
        &EntryPoint,
        &DataSize
        );
      // Check if BootImage loading failed?
      if (EFI_ERROR(Status)) {
        ASSERT_PLATFORM_INIT(0);
        // Halt the system
        EFI_DEADLOOP();
      }
      break;

    case BOOT_SOURCE_NAND:
    case BOOT_SOURCE_QSPI:
      FlashOffset = PcdGet64 (PcdQspiOrNand_BOOTIMAGE_ADDR);

      // Check if MKIMAGE header exist
      // if MKIMAGE header exist, image size and entry point will be based on MKIMAGE header
      // else value in PCDs will be used.
      if (BootSourceType == BOOT_SOURCE_QSPI)
      {
        // Read from QSPI
        Status = QspiRead ((VOID *) &ImgHdr, FlashOffset, sizeof(ImgHdr));
      } else {
        // Read from NAND
        Status = NandRead ((VOID *) &ImgHdr, FlashOffset, sizeof(ImgHdr));
      }
      // Flash Read Error Checking
      if (EFI_ERROR(Status)) {
        InfoPrint ("Flash Read Error!");
        ASSERT_PLATFORM_INIT(0);
        EFI_DEADLOOP();
      }

      Status = ValidateMkimageHeader(&ImgHdr);
      if (EFI_ERROR(Status)) {
        // Boot Image do not use mkimage header
        DataSize    = PcdGet32 (PcdQspiOrNand_BOOTIMAGE_SIZE);
      } else {
        // Boot Image have mkimage header
        FlashOffset = FlashOffset + sizeof(ImgHdr);
        DataSize    = ImgHdr.DataSize;
        LoadAddr    = ImgHdr.LoadAddr;
        EntryPoint  = ImgHdr.EntryPoint;
      }

      // Print message that we are going to read BootImage from QSPI or NAND flash
      ProgressPrint ("Copying BootImage from Flash Offset 0x%08lx to Memory Address 0x%08lx where BootImage's size is 0x%08x bytes\r\n",
        (UINT64) FlashOffset,
        (UINT64) LoadAddr,
        (UINT32) DataSize);
      if (BootSourceType == BOOT_SOURCE_QSPI)
      {
        // Read from QSPI
        Status = QspiRead ((VOID *) LoadAddr, FlashOffset, DataSize);
      } else {
        // Read from NAND
        Status = NandRead ((VOID *) LoadAddr, FlashOffset, DataSize);
      }
      // Flash Read Error Checking
      if (EFI_ERROR(Status)) {
        InfoPrint ("Flash Read Error!");
        ASSERT_PLATFORM_INIT(0);
        EFI_DEADLOOP();
      }
      break;

    case BOOT_SOURCE_RSVD:
    case BOOT_SOURCE_FPGA:
    default:
      // No Flash device.
      ASSERT_PLATFORM_INIT(0);
      EFI_DEADLOOP();
      break;
  }

  // Support loading of .DTB file for Boot Image ?
  if (PcdGet32 (PcdBoot_LOAD_BOOTIMAGE_DTB_FILE) == 1)
  {
    // If DTB for BootImage exist, then load it.
    BootImageDtbAddr = (LoadAddr + DataSize + 0x2000000) & 0xFE000000;
    switch (BootSourceType)
    {
      case BOOT_SOURCE_SDMMC:
        Status = LoadFileToMemory (
          (CHAR8*) PcdGetPtr (PcdSdmmc_BOOTIMAGE_DTB_FILENAME),
          BootImageDtbAddr,
          &BootImageDtbFileSize);
        if (Status == EFI_SUCCESS) {
          // Patch the DTB
          Status = UpdateBootImageDtbWithMemoryInfoAndBootArgs (BootImageDtbAddr);
          if (Status == EFI_SUCCESS) {
            R2 = BootImageDtbAddr;
          }
        }
        break;

      case BOOT_SOURCE_NAND:
      case BOOT_SOURCE_QSPI:
        FlashOffset = PcdGet64 (PcdQspiOrNand_BOOTIMAGE_DTB_ADDR);
        DataSize = PcdGet32 (PcdQspiOrNand_BOOTIMAGE_DTB_SIZE);
        if (BootSourceType == BOOT_SOURCE_QSPI)
        {
          // Read from QSPI
          Status = QspiRead ((VOID *) BootImageDtbAddr, FlashOffset, DataSize);
        } else {
          // Read from NAND
          Status = NandRead ((VOID *) BootImageDtbAddr, FlashOffset, DataSize);
        }
        break;

      case BOOT_SOURCE_RSVD:
      case BOOT_SOURCE_FPGA:
      default:
        break;
    }
  }

  // Transfer control
  JumpToEntry (EntryPoint, R0, R1, R2 );

}


STATIC
UINTN
cpu_to_fdtn (UINTN x) {
  if (sizeof (UINTN) == sizeof (UINT32)) {
    return cpu_to_fdt32 (x);
  } else {
    return cpu_to_fdt64 (x);
  }
}


EFI_STATUS
EFIAPI
UpdateBootImageDtbWithMemoryInfoAndBootArgs (
  IN     EFI_PHYSICAL_ADDRESS FdtBlobBase
  )
{
  VOID*                 fdt;
  INTN                  err;
  INTN                  node;
  INTN                  length;
  CONST VOID*           bootargs;
  FdtMemoryRegionType   Region;
  CHAR8                 NewBootArgsStr[1024];
  CHAR8*                NewBootArgsPtr = &NewBootArgsStr[0];

  //
  // Sanity checks on the original FDT blob.
  //
  fdt = (VOID*)(UINTN)(FdtBlobBase);
  err = fdt_check_header (fdt);
  if (err != 0) {
    InfoPrint ("Invalid BootImage's Device Tree header\n");
    return EFI_INVALID_PARAMETER;
  }

  //
  // Add Physical memory range to memory->reg
  //
  node = fdt_subnode_offset(fdt, 0, "memory");
  if (node > 0) {
    // Get RAM Base and Size
    Region.Base = cpu_to_fdtn ((UINTN)GetMpuWindowDramBaseAddr());
    Region.Size = cpu_to_fdtn ((UINTN)GetMpuWindowDramBaseAddr() + GetMpuWindowDramSize());
    // Update the DTB
    err = fdt_setprop(fdt, node, "reg", &Region, sizeof(Region));
    if (err) {
      InfoPrint ("Fail to update DTB with Memory Info\n");
      return EFI_INVALID_PARAMETER;
    }
  }


  //
  // Update chosen->bootargs
  //
  // Get "chosen" node, because bootargs is under "chosen"
  node = fdt_subnode_offset (fdt, 0, "chosen");
  if (node < 0) {
    // The 'chosen' node does not exist, create it
    node = fdt_add_subnode(fdt, 0, "chosen");
    if (node < 0) {
      InfoPrint ("Error on finding 'chosen' node\n");
      return EFI_INVALID_PARAMETER;
    }
  }

  // Print old bootargs
  bootargs = fdt_getprop(fdt, node, "bootargs", &length);
  if (bootargs != NULL) {
    InfoPrint ("old bootargs: %a\n", bootargs);
  }

  // Construct new bootargs
  length = AsciiSPrint (NewBootArgsPtr, 1024,
    "console=ttyS0,%d",
    PcdGet32 (PcdSerialBaudRate));

  // Update bootargs to new bootargs
  err = fdt_setprop(fdt, node, "bootargs", NewBootArgsStr, AsciiStrSize(NewBootArgsStr));
  if (err) {
    InfoPrint ("Fail to update bootargs\n");
    return EFI_INVALID_PARAMETER;
  }

  // Print new bootargs
  bootargs = fdt_getprop(fdt, node, "bootargs", &length);
  if (bootargs != NULL) {
    InfoPrint ("old bootargs: %a\n", bootargs);
  }

  return EFI_SUCCESS;

}





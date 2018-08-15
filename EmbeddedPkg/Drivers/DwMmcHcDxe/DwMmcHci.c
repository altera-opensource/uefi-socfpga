/** @file
  This driver is used to manage Designware SD/MMC PCI host controllers.

  It would expose EFI_SD_MMC_PASS_THRU_PROTOCOL for upper layer use.

  Copyright (c) 2015 - 2016, Intel Corporation. All rights reserved.<BR>
  Copyright (c) 2018, Linaro Ltd. All rights reserved.<BR>

  This program and the accompanying materials are licensed and made available
  under the terms and conditions of the BSD License which accompanies this
  distribution.  The full text of the license may be found at
  http://opensource.org/licenses/bsd-license.php

  THE PROGRAM IS DISTRIBUTED UNDER THE BSD LICENSE ON AN "AS IS" BASIS,
  WITHOUT WARRANTIES OR REPRESENTATIONS OF ANY KIND, EITHER EXPRESS OR IMPLIED.

**/

#include <IndustryStandard/Emmc.h>
#include <IndustryStandard/Sd.h>

#include <Library/ArmLib.h>
#include <Library/BaseMemoryLib.h>
#include <Library/DebugLib.h>
#include <Library/MemoryAllocationLib.h>
#include <Library/UefiBootServicesTableLib.h>

#include "DwMmcHcDxe.h"

/**
  Dump the content of SD/MMC host controller's Capability Register.

  @param[in]  Slot            The slot number of the SD card to send the
                              command to.
  @param[in]  Capability      The buffer to store the capability data.

**/
VOID
DumpCapabilityReg (
  IN UINT8                Slot,
  IN DW_MMC_HC_SLOT_CAP   *Capability
  )
{
  //
  // Dump Capability Data
  //
  DEBUG ((
    DEBUG_INFO,
    " == Slot [%d] Capability is 0x%x ==\n",
    Slot,
    Capability
    ));
  DEBUG ((
    DEBUG_INFO,
    "   Base Clk Freq     %dKHz\n",
    Capability->BaseClkFreq
    ));
  DEBUG ((
    DEBUG_INFO,
    "   BusWidth          %d\n",
    Capability->BusWidth
    ));
  DEBUG ((
    DEBUG_INFO,
    "   HighSpeed Support %a\n",
    Capability->HighSpeed ? "TRUE" : "FALSE"
    ));
  DEBUG ((
    DEBUG_INFO,
    "   Voltage 1.8       %a\n",
    Capability->Voltage18 ? "TRUE" : "FALSE"
    ));
  DEBUG ((
    DEBUG_INFO,
    "   64-bit Sys Bus    %a\n",
    Capability->SysBus64 ? "TRUE" : "FALSE"
    ));
  DEBUG ((DEBUG_INFO, "   SlotType          "));
  if (Capability->SlotType == 0x00) {
    DEBUG ((DEBUG_INFO, "%a\n", "Removable Slot"));
  } else if (Capability->SlotType == 0x01) {
    DEBUG ((DEBUG_INFO, "%a\n", "Embedded Slot"));
  } else if (Capability->SlotType == 0x02) {
    DEBUG ((DEBUG_INFO, "%a\n", "Shared Bus Slot"));
  } else {
    DEBUG ((DEBUG_INFO, "%a\n", "Reserved"));
  }
  DEBUG ((
    DEBUG_INFO,
    "   SDR50  Support    %a\n",
    Capability->Sdr50 ? "TRUE" : "FALSE"
    ));
  DEBUG ((
    DEBUG_INFO,
    "   SDR104 Support    %a\n",
    Capability->Sdr104 ? "TRUE" : "FALSE"
    ));
  DEBUG ((
    DEBUG_INFO,
    "   DDR50  Support    %a\n",
    Capability->Ddr50 ? "TRUE" : "FALSE"
    ));
  return;
}

/**
  Read/Write specified SD/MMC host controller mmio register.

  @param[in]      DevIo         The DEVICE IO protocol instance.
  @param[in]      Offset        The offset to start the memory operation.
  @param[in]      Read          A boolean to indicate it's read or write
                                operation.
  @param[in]      Count         The width of the mmio register in bytes.
                                Must be 1, 2 , 4 or 8 bytes.
  @param[in, out] Data          For read operations, the destination buffer to
                                store the results. For write operations, the
                                source buffer to write data from. The caller is
                                responsible for having ownership of the data
                                buffer and ensuring its size not less than
                                Count bytes.

  @retval EFI_INVALID_PARAMETER The DevIo or Data is NULL or the Count is not
                                valid.
  @retval EFI_SUCCESS           The read/write operation succeeds.
  @retval Others                The read/write operation fails.

**/
EFI_STATUS
EFIAPI
DwMmcHcRwMmio (
  IN     EFI_DEVICE_IO_PROTOCOL   *DevIo,
  IN     UINT32                   Offset,
  IN     BOOLEAN                  Read,
  IN     UINT8                    Count,
  IN OUT VOID                     *Data
  )
{
  EFI_STATUS                   Status;

  if ((DevIo == NULL) || (Data == NULL))  {
    return EFI_INVALID_PARAMETER;
  }

  if ((Count != 4) && (Count != 8)) {
    return EFI_INVALID_PARAMETER;
  }

  //
  // Since there's FIFO in Designware controller, map it to 32-bit word only.
  //
  Count = Count / sizeof (UINT32);
  if (Read) {
    Status = DevIo->Mem.Read (
                          DevIo,
                          IO_UINT32,
                          (UINT64) Offset,
                          Count,
                          Data
                          );
  } else {
    Status = DevIo->Mem.Write (
                          DevIo,
                          IO_UINT32,
                          (UINT64) Offset,
                          Count,
                          Data
                          );
  }

  return Status;
}

/**
  Do OR operation with the value of the specified SD/MMC host controller mmio
  register.

  @param[in] DevIo             The DEVICE IO protocol instance.
  @param[in] Offset            The offset to start the memory operation.
  @param[in] Count             The width of the mmio register in bytes.
                               Must be 1, 2 , 4 or 8 bytes.
  @param[in] OrData            The pointer to the data used to do OR operation.
                               The caller is responsible for having ownership of
                               the data buffer and ensuring its size not less
                               than Count bytes.

  @retval EFI_INVALID_PARAMETER The DevIo or OrData is NULL or the Count is not
                                valid.
  @retval EFI_SUCCESS           The OR operation succeeds.
  @retval Others                The OR operation fails.

**/
EFI_STATUS
EFIAPI
DwMmcHcOrMmio (
  IN  EFI_DEVICE_IO_PROTOCOL   *DevIo,
  IN  UINT32                   Offset,
  IN  UINT8                    Count,
  IN  VOID                     *OrData
  )
{
  EFI_STATUS                   Status;
  UINT64                       Data;
  UINT64                       Or;

  Status = DwMmcHcRwMmio (DevIo, Offset, TRUE, Count, &Data);
  if (EFI_ERROR (Status)) {
    return Status;
  }

  if (Count == 1) {
    Or = *(UINT8*) OrData;
  } else if (Count == 2) {
    Or = *(UINT16*) OrData;
  } else if (Count == 4) {
    Or = *(UINT32*) OrData;
  } else if (Count == 8) {
    Or = *(UINT64*) OrData;
  } else {
    return EFI_INVALID_PARAMETER;
  }

  Data  |= Or;
  Status = DwMmcHcRwMmio (DevIo, Offset, FALSE, Count, &Data);

  return Status;
}

/**
  Do AND operation with the value of the specified SD/MMC host controller mmio
  register.

  @param[in] DevIo             The DEVICE IO protocol instance.
  @param[in] Offset            The offset to start the memory operation.
  @param[in] Count             The width of the mmio register in bytes.
                               Must be 1, 2 , 4 or 8 bytes.
  @param[in] AndData           The pointer to the data used to do AND operation.
                               The caller is responsible for having ownership of
                               the data buffer and ensuring its size not less
                               than Count bytes.

  @retval EFI_INVALID_PARAMETER The DevIo or AndData is NULL or the Count is
                                not valid.
  @retval EFI_SUCCESS           The AND operation succeeds.
  @retval Others                The AND operation fails.

**/
EFI_STATUS
EFIAPI
DwMmcHcAndMmio (
  IN  EFI_DEVICE_IO_PROTOCOL   *DevIo,
  IN  UINT32                   Offset,
  IN  UINT8                    Count,
  IN  VOID                     *AndData
  )
{
  EFI_STATUS                   Status;
  UINT64                       Data;
  UINT64                       And;

  Status = DwMmcHcRwMmio (DevIo, Offset, TRUE, Count, &Data);
  if (EFI_ERROR (Status)) {
    return Status;
  }

  if (Count == 1) {
    And = *(UINT8*) AndData;
  } else if (Count == 2) {
    And = *(UINT16*) AndData;
  } else if (Count == 4) {
    And = *(UINT32*) AndData;
  } else if (Count == 8) {
    And = *(UINT64*) AndData;
  } else {
    return EFI_INVALID_PARAMETER;
  }

  Data  &= And;
  Status = DwMmcHcRwMmio (DevIo, Offset, FALSE, Count, &Data);

  return Status;
}

/**
  Wait for the value of the specified MMIO register set to the test value.

  @param[in]  DevIo         The DEVICE IO protocol instance.
  @param[in]  Offset        The offset to start the memory operation.
  @param[in]  Count         The width of the mmio register in bytes.
                            Must be 1, 2, 4 or 8 bytes.
  @param[in]  MaskValue     The mask value of memory.
  @param[in]  TestValue     The test value of memory.

  @retval EFI_NOT_READY     The MMIO register hasn't set to the expected value.
  @retval EFI_SUCCESS       The MMIO register has expected value.
  @retval Others            The MMIO operation fails.

**/
EFI_STATUS
EFIAPI
DwMmcHcCheckMmioSet (
  IN  EFI_DEVICE_IO_PROTOCOL    *DevIo,
  IN  UINT32                    Offset,
  IN  UINT8                     Count,
  IN  UINT64                    MaskValue,
  IN  UINT64                    TestValue
  )
{
  EFI_STATUS            Status;
  UINT64                Value;

  Value  = 0;
  Status = DwMmcHcRwMmio (DevIo, Offset, TRUE, Count, &Value);
  if (EFI_ERROR (Status)) {
    return Status;
  }

  Value &= MaskValue;

  if (Value == TestValue) {
    return EFI_SUCCESS;
  }

  return EFI_NOT_READY;
}

/**
  Wait for the value of the specified MMIO register set to the test value.

  @param[in]  DevIo         The DEVICE IO protocol instance.
  @param[in]  Offset        The offset to start the memory operation.
  @param[in]  Count         The width of the mmio register in bytes.
                            Must be 1, 2, 4 or 8 bytes.
  @param[in]  MaskValue     The mask value of memory.
  @param[in]  TestValue     The test value of memory.
  @param[in]  Timeout       The time out value for wait memory set, uses 1
                            microsecond as a unit.

  @retval EFI_TIMEOUT       The MMIO register hasn't expected value in timeout
                            range.
  @retval EFI_SUCCESS       The MMIO register has expected value.
  @retval Others            The MMIO operation fails.

**/
EFI_STATUS
EFIAPI
DwMmcHcWaitMmioSet (
  IN  EFI_DEVICE_IO_PROTOCOL    *DevIo,
  IN  UINT32                    Offset,
  IN  UINT8                     Count,
  IN  UINT64                    MaskValue,
  IN  UINT64                    TestValue,
  IN  UINT64                    Timeout
  )
{
  EFI_STATUS            Status;
  BOOLEAN               InfiniteWait;

  if (Timeout == 0) {
    InfiniteWait = TRUE;
  } else {
    InfiniteWait = FALSE;
  }

  while (InfiniteWait || (Timeout > 0)) {
    Status = DwMmcHcCheckMmioSet (
               DevIo,
               Offset,
               Count,
               MaskValue,
               TestValue
               );
    if (Status != EFI_NOT_READY) {
      return Status;
    }

    //
    // Stall for 1 microsecond.
    //
    gBS->Stall (1);

    Timeout--;
  }

  return EFI_TIMEOUT;
}

/**
  Set all interrupt status bits in Normal and Error Interrupt Status Enable
  register.

  @param[in] DevIo          The DEVICE IO protocol instance.

  @retval EFI_SUCCESS       The operation executes successfully.
  @retval Others            The operation fails.

**/
EFI_STATUS
DwMmcHcEnableInterrupt (
  IN EFI_DEVICE_IO_PROTOCOL *DevIo
  )
{
  EFI_STATUS                Status;
  UINT32                    IntStatus;
  UINT32                    IdIntEn;
  UINT32                    IdSts;

  //
  // Enable all bits in Interrupt Mask Register
  //
  IntStatus = 0;
  Status = DwMmcHcRwMmio (
             DevIo,
             DW_MMC_INTMASK,
             FALSE,
             sizeof (IntStatus),
             &IntStatus
             );
  if (EFI_ERROR (Status)) {
    return Status;
  }

  //
  // Clear status in Interrupt Status Register
  //
  IntStatus = ~0;
  Status = DwMmcHcRwMmio (
             DevIo,
             DW_MMC_RINTSTS,
             FALSE,
             sizeof (IntStatus),
             &IntStatus
             );
  if (EFI_ERROR (Status)) {
    return Status;
  }

  IdIntEn = ~0;
  Status = DwMmcHcRwMmio (
             DevIo,
             DW_MMC_IDINTEN,
             FALSE,
             sizeof (IdIntEn),
             &IdIntEn
             );
  if (EFI_ERROR (Status)) {
    DEBUG ((
      DEBUG_ERROR,
      "DwMmcHcEnableInterrupt: init dma interrupts fail: %r\n",
      Status
      ));
    return Status;
  }

  IdSts = ~0;
  Status = DwMmcHcRwMmio (
             DevIo,
             DW_MMC_IDSTS,
             FALSE,
             sizeof (IdSts),
             &IdSts
             );
  return Status;
}

EFI_STATUS
DwMmcHcGetCapability (
  IN     EFI_DEVICE_IO_PROTOCOL  *DevIo,
  IN     EFI_HANDLE              Controller,
  IN     UINT8                   Slot,
     OUT DW_MMC_HC_SLOT_CAP      *Capacity
  )
{
  PLATFORM_DW_MMC_PROTOCOL       *PlatformDwMmc;
  EFI_STATUS                     Status;

  if (Capacity == NULL) {
    return EFI_INVALID_PARAMETER;
  }
  Status = gBS->LocateProtocol (
                  &gPlatformDwMmcProtocolGuid,
                  NULL,
                  (VOID **) &PlatformDwMmc
                  );
  if (EFI_ERROR (Status)) {
    return Status;
  }
  Status = PlatformDwMmc->GetCapability (Controller, Slot, Capacity);
  return Status;
}

/**
  Detect whether there is a SD/MMC card attached at the specified SD/MMC host
  controller slot.

  Refer to SD Host Controller Simplified spec 3.0 Section 3.1 for details.

  @param[in]  DevIo         The DEVICE IO protocol instance.
  @param[in]  Slot          The slot number of the SD card to send the command
                            to.
  @param[out] MediaPresent  The pointer to the media present boolean value.

  @retval EFI_SUCCESS       There is no media change happened.
  @retval EFI_MEDIA_CHANGED There is media change happened.
  @retval Others            The detection fails.

**/
EFI_STATUS
DwMmcHcCardDetect (
  IN     EFI_DEVICE_IO_PROTOCOL *DevIo,
  IN     EFI_HANDLE             Controller,
  IN     UINT8                  Slot,
     OUT BOOLEAN                *MediaPresent
  )
{
  PLATFORM_DW_MMC_PROTOCOL  *PlatformDwMmc;
  EFI_STATUS                Status;

  if (MediaPresent == NULL) {
    return EFI_INVALID_PARAMETER;
  }
  Status = gBS->LocateProtocol (
                  &gPlatformDwMmcProtocolGuid,
                  NULL,
                  (VOID **) &PlatformDwMmc
                  );
  if (EFI_ERROR (Status)) {
    return Status;
  }
  *MediaPresent = PlatformDwMmc->CardDetect (Controller, Slot);
  return EFI_SUCCESS;
}

STATIC
EFI_STATUS
DwMmcHcUpdateClock (
  IN EFI_DEVICE_IO_PROTOCOL *DevIo
  )
{
  EFI_STATUS                Status;
  UINT32                    Cmd;
  UINT32                    IntStatus;

  Cmd = BIT_CMD_WAIT_PRVDATA_COMPLETE | BIT_CMD_UPDATE_CLOCK_ONLY |
        BIT_CMD_START;
  Status = DwMmcHcRwMmio (DevIo, DW_MMC_CMD, FALSE, sizeof (Cmd), &Cmd);
  if (EFI_ERROR (Status)) {
    return Status;
  }
  while (1) {
    Status = DwMmcHcRwMmio (DevIo, DW_MMC_CMD, TRUE, sizeof (Cmd), &Cmd);
    if (EFI_ERROR (Status)) {
      return Status;
    }
    if (!(Cmd & CMD_START_BIT)) {
      break;
    }
    Status = DwMmcHcRwMmio (
               DevIo,
               DW_MMC_RINTSTS,
               TRUE,
               sizeof (IntStatus),
               &IntStatus
               );
    if (EFI_ERROR (Status)) {
      return Status;
    }
    if (IntStatus & DW_MMC_INT_HLE) {
      DEBUG ((
        DEBUG_ERROR,
        "DwMmcHcUpdateClock: failed to update mmc clock frequency\n"
        ));
      return EFI_DEVICE_ERROR;
    }
  }
  return EFI_SUCCESS;

}

/**
  Stop SD/MMC card clock.

  @param[in] DevIo          The DEVICE IO protocol instance.

  @retval EFI_SUCCESS       Succeed to stop SD/MMC clock.
  @retval Others            Fail to stop SD/MMC clock.

**/
EFI_STATUS
DwMmcHcStopClock (
  IN EFI_DEVICE_IO_PROTOCOL *DevIo
  )
{
  EFI_STATUS                Status;
  UINT32                    ClkEna;

  //
  // Disable MMC clock first
  //
  ClkEna = 0;
  Status = DwMmcHcRwMmio (
             DevIo,
             DW_MMC_CLKENA,
             FALSE,
             sizeof (ClkEna),
             &ClkEna
             );
  if (EFI_ERROR (Status)) {
    return Status;
  }
  Status = DwMmcHcUpdateClock (DevIo);
  if (EFI_ERROR (Status)) {
    return Status;
  }
  return Status;
}

/**
  SD/MMC card clock supply.

  @param[in] DevIo          The DEVICE IO protocol instance.
  @param[in] ClockFreq      The max clock frequency to be set. The unit is KHz.
  @param[in] Capability     The capability of the slot.

  @retval EFI_SUCCESS       The clock is supplied successfully.
  @retval Others            The clock isn't supplied successfully.

**/
EFI_STATUS
DwMmcHcClockSupply (
  IN EFI_DEVICE_IO_PROTOCOL *DevIo,
  IN UINT64                 ClockFreq,
  IN DW_MMC_HC_SLOT_CAP     Capability
  )
{
  EFI_STATUS                Status;
  UINT32                    BaseClkFreq;
  UINT32                    SettingFreq;
  UINT32                    Divisor;
  UINT32                    Remainder;
  UINT32                    MmcStatus;
  UINT32                    ClkEna;
  UINT32                    ClkSrc;

  //
  // Calculate a divisor for SD clock frequency
  //
  ASSERT (Capability.BaseClkFreq != 0);

  BaseClkFreq = Capability.BaseClkFreq;
  if (ClockFreq == 0) {
    return EFI_INVALID_PARAMETER;
  }

  if (ClockFreq > BaseClkFreq) {
    ClockFreq = BaseClkFreq;
  }

  //
  // Calculate the divisor of base frequency.
  //
  Divisor     = 0;
  SettingFreq = BaseClkFreq;
  while (ClockFreq < SettingFreq) {
    Divisor++;

    SettingFreq = BaseClkFreq / (2 * Divisor);
    Remainder   = BaseClkFreq % (2 * Divisor);
    if ((ClockFreq == SettingFreq) && (Remainder == 0)) {
      break;
    }
    if ((ClockFreq == SettingFreq) && (Remainder != 0)) {
      SettingFreq ++;
    }
  }

  DEBUG ((
    DEBUG_INFO,
    "BaseClkFreq %dKHz Divisor %d ClockFreq %dKhz\n",
    BaseClkFreq,
    Divisor,
    ClockFreq
    ));

  //
  // Wait until MMC is idle
  //
  do {
    Status = DwMmcHcRwMmio (
               DevIo,
               DW_MMC_STATUS,
               TRUE,
               sizeof (MmcStatus),
               &MmcStatus
               );
    if (EFI_ERROR (Status)) {
      return Status;
    }
  } while (MmcStatus & DW_MMC_STS_DATA_BUSY);

  do {
    Status = DwMmcHcStopClock (DevIo);
  } while (EFI_ERROR (Status));

  do {
    ClkSrc = 0;
    Status = DwMmcHcRwMmio (
               DevIo,
               DW_MMC_CLKSRC,
               FALSE,
               sizeof (ClkSrc),
               &ClkSrc
               );
    if (EFI_ERROR (Status)) {
      continue;
    }
    //
    // Set clock divisor
    //
    Status = DwMmcHcRwMmio (
               DevIo,
               DW_MMC_CLKDIV,
               FALSE,
               sizeof (Divisor),
               &Divisor
               );
    if (EFI_ERROR (Status)) {
      continue;
    }
    //
    // Enable MMC clock
    //
    ClkEna = 1;
    Status = DwMmcHcRwMmio (
               DevIo,
               DW_MMC_CLKENA,
               FALSE,
               sizeof (ClkEna),
               &ClkEna
               );
    if (EFI_ERROR (Status)) {
      continue;
    }
    Status = DwMmcHcUpdateClock (DevIo);
  } while (EFI_ERROR (Status));

  return EFI_SUCCESS;
}

/**
  Set the SD/MMC bus width.

  Refer to SD Host Controller Simplified spec 3.0 Section 3.4 for details.

  @param[in] DevIo          The DEVICE IO protocol instance.
  @param[in] IsDdr          A boolean to indicate it's dual data rate or not.
  @param[in] BusWidth       The bus width used by the SD/MMC device, it must be
                            1, 4 or 8.

  @retval EFI_SUCCESS       The bus width is set successfully.
  @retval Others            The bus width isn't set successfully.

**/
EFI_STATUS
DwMmcHcSetBusWidth (
  IN EFI_DEVICE_IO_PROTOCOL *DevIo,
  IN BOOLEAN                IsDdr,
  IN UINT16                 BusWidth
  )
{
  EFI_STATUS                Status;
  UINT32                    Ctype;
  UINT32                    Uhs;

  switch (BusWidth) {
  case 1:
    Ctype = MMC_1BIT_MODE;
    break;
  case 4:
    Ctype = MMC_4BIT_MODE;
    break;
  case 8:
    Ctype = MMC_8BIT_MODE;
    break;
  default:
    return EFI_INVALID_PARAMETER;
  }
  Status = DwMmcHcRwMmio (
             DevIo,
             DW_MMC_CTYPE,
             FALSE,
             sizeof (Ctype),
             &Ctype
             );
  if (EFI_ERROR (Status)) {
    return Status;
  }
  Status = DwMmcHcRwMmio (DevIo, DW_MMC_UHSREG, TRUE, sizeof (Uhs), &Uhs);
  if (EFI_ERROR (Status)) {
    return Status;
  }
  if (IsDdr) {
    Uhs |= UHS_DDR_MODE;
  } else {
    Uhs &= ~(UHS_DDR_MODE);
  }
  Status = DwMmcHcRwMmio (
             DevIo,
             DW_MMC_UHSREG,
             FALSE,
             sizeof (Uhs),
             &Uhs
             );
  return Status;
}

/**
  Supply SD/MMC card with lowest clock frequency at initialization.

  @param[in] DevIo          The DEVICE IO protocol instance.
  @param[in] Capability     The capability of the slot.

  @retval EFI_SUCCESS       The clock is supplied successfully.
  @retval Others            The clock isn't supplied successfully.

**/
EFI_STATUS
DwMmcHcInitClockFreq (
  IN EFI_DEVICE_IO_PROTOCOL    *DevIo,
  IN DW_MMC_HC_SLOT_CAP        Capability
  )
{
  EFI_STATUS                Status;
  UINT32                    InitFreq;

  //
  // Calculate a divisor for SD clock frequency
  //
  if (Capability.BaseClkFreq == 0) {
    //
    // Don't support get Base Clock Frequency information via another method
    //
    return EFI_UNSUPPORTED;
  }
  //
  // Supply 400KHz clock frequency at initialization phase.
  //
  InitFreq = DWMMC_INIT_CLOCK_FREQ;
  Status = DwMmcHcClockSupply (DevIo, InitFreq, Capability);
  if (EFI_ERROR (Status)) {
    return Status;
  }
  MicroSecondDelay (100);
  return Status;
}

/**
  Supply SD/MMC card with maximum voltage at initialization.

  @param[in] DevIo          The DEVICE IO protocol instance.
  @param[in] Capability     The capability of the slot.

  @retval EFI_SUCCESS       The voltage is supplied successfully.
  @retval Others            The voltage isn't supplied successfully.

**/
EFI_STATUS
DwMmcHcInitPowerVoltage (
  IN EFI_DEVICE_IO_PROTOCOL *DevIo,
  IN DW_MMC_HC_SLOT_CAP     Capability
  )
{
  EFI_STATUS                Status;
  UINT32                    Data;

  Data = 0x1;
  Status  = DwMmcHcRwMmio (
              DevIo,
              DW_MMC_PWREN,
              FALSE,
              sizeof (Data),
              &Data
              );
  if (EFI_ERROR (Status)) {
    DEBUG ((
      DEBUG_ERROR,
      "DwMmcHcInitPowerVoltage: enable power fails: %r\n",
      Status
      ));
    return Status;
  }

  Data = DW_MMC_CTRL_RESET_ALL;
  Status = DwMmcHcRwMmio (
             DevIo,
             DW_MMC_CTRL,
             FALSE,
             sizeof (Data),
             &Data
             );
  if (EFI_ERROR (Status)) {
    DEBUG ((DEBUG_ERROR, "DwMmcHcInitPowerVoltage: reset fails: %r\n", Status));
    return Status;
  }
  Status = DwMmcHcWaitMmioSet (
             DevIo,
             DW_MMC_CTRL,
             sizeof (Data),
             DW_MMC_CTRL_RESET_ALL,
             0x00,
             DW_MMC_HC_GENERIC_TIMEOUT
             );
  if (EFI_ERROR (Status)) {
    DEBUG ((
      DEBUG_INFO,
      "DwMmcHcInitPowerVoltage: reset done with %r\n",
      Status
      ));
    return Status;
  }

  Data = DW_MMC_CTRL_INT_EN;
  Status = DwMmcHcRwMmio (
             DevIo,
             DW_MMC_CTRL,
             FALSE,
             sizeof (Data),
             &Data
             );
  return Status;
}

/**
  Initialize the Timeout Control register with most conservative value at
  initialization.

  @param[in] DevIo          The DEVICE IO protocol instance.

  @retval EFI_SUCCESS       The timeout control register is configured
                            successfully.
  @retval Others            The timeout control register isn't configured
                            successfully.

**/
EFI_STATUS
DwMmcHcInitTimeoutCtrl (
  IN EFI_DEVICE_IO_PROTOCOL *DevIo
  )
{
  EFI_STATUS                Status;
  UINT32                    Data;

  Data = ~0;
  Status = DwMmcHcRwMmio (
             DevIo,
             DW_MMC_TMOUT,
             FALSE,
             sizeof (Data),
             &Data
             );
  if (EFI_ERROR (Status)) {
    DEBUG ((
      DEBUG_ERROR,
      "DwMmcHcInitTimeoutCtrl: set timeout fails: %r\n",
      Status
      ));
    return Status;
  }

  Data = 0x00FFFFFF;
  Status = DwMmcHcRwMmio (
             DevIo,
             DW_MMC_DEBNCE,
             FALSE,
             sizeof (Data),
             &Data
             );
  return Status;
}

/**
  Initial SD/MMC host controller with lowest clock frequency, max power and
  max timeout value at initialization.

  @param[in] DevIo          The DEVICE IO protocol instance.
  @param[in] Slot           The slot number of the SD card to send the command
                            to.
  @param[in] Capability     The capability of the slot.

  @retval EFI_SUCCESS       The host controller is initialized successfully.
  @retval Others            The host controller isn't initialized successfully.

**/
EFI_STATUS
DwMmcHcInitHost (
  IN EFI_DEVICE_IO_PROTOCOL    *DevIo,
  IN DW_MMC_HC_SLOT_CAP        Capability
  )
{
  EFI_STATUS       Status;

  Status = DwMmcHcInitPowerVoltage (DevIo, Capability);
  if (EFI_ERROR (Status)) {
    return Status;
  }
  return Status;
}

EFI_STATUS
DwMmcHcStartDma (
  IN DW_MMC_HC_PRIVATE_DATA           *Private,
  IN DW_MMC_HC_TRB                    *Trb
  )
{
  EFI_STATUS                          Status;
  EFI_DEVICE_IO_PROTOCOL              *DevIo;
  UINT32                              Ctrl;
  UINT32                              Bmod;

  DevIo  = Trb->Private->DevIo;

  //
  // Reset DMA
  //
  Ctrl = DW_MMC_CTRL_DMA_RESET;
  Status = DwMmcHcRwMmio (
             DevIo,
             DW_MMC_CTRL,
             FALSE,
             sizeof (Ctrl),
             &Ctrl
             );
  if (EFI_ERROR (Status)) {
    DEBUG ((DEBUG_ERROR, "DwMmcHcStartDma: reset fails: %r\n", Status));
    return Status;
  }
  Status = DwMmcHcWaitMmioSet (
             DevIo,
             DW_MMC_CTRL,
             sizeof (Ctrl),
             DW_MMC_CTRL_DMA_RESET,
             0x00,
             DW_MMC_HC_GENERIC_TIMEOUT
             );
  if (EFI_ERROR (Status)) {
    DEBUG ((DEBUG_INFO, "DwMmcHcStartDma: reset done with %r\n", Status));
    return Status;
  }
  Bmod = DW_MMC_IDMAC_SWRESET;
  Status = DwMmcHcOrMmio (DevIo, DW_MMC_BMOD, sizeof (Bmod), &Bmod);
  if (EFI_ERROR (Status)) {
    DEBUG ((DEBUG_ERROR, "DwMmcHcStartDma: set BMOD fail: %r\n", Status));
    return Status;
  }

  //
  // Select IDMAC
  //
  Ctrl = DW_MMC_CTRL_IDMAC_EN;
  Status = DwMmcHcOrMmio (DevIo, DW_MMC_CTRL, sizeof (Ctrl), &Ctrl);
  if (EFI_ERROR (Status)) {
    DEBUG ((DEBUG_ERROR, "DwMmcHcStartDma: init IDMAC fail: %r\n", Status));
    return Status;
  }

  //
  // Enable IDMAC
  //
  Bmod = DW_MMC_IDMAC_ENABLE | DW_MMC_IDMAC_FB;
  Status = DwMmcHcOrMmio (DevIo, DW_MMC_BMOD, sizeof (Bmod), &Bmod);
  if (EFI_ERROR (Status)) {
    DEBUG ((DEBUG_ERROR, "DwMmcHcStartDma: set BMOD failure: %r\n", Status));
    return Status;
  }
  return Status;
}

EFI_STATUS
DwMmcHcStopDma (
  IN DW_MMC_HC_PRIVATE_DATA           *Private,
  IN DW_MMC_HC_TRB                    *Trb
  )
{
  EFI_STATUS                          Status;
  EFI_DEVICE_IO_PROTOCOL              *DevIo;
  UINT32                              Ctrl;
  UINT32                              Bmod;

  DevIo  = Trb->Private->DevIo;

  //
  // Disable and reset IDMAC
  //
  Status = DwMmcHcRwMmio (
             DevIo,
             DW_MMC_CTRL,
             TRUE,
             sizeof (Ctrl),
             &Ctrl
             );
  if (EFI_ERROR (Status)) {
    return Status;
  }
  Ctrl &= ~DW_MMC_CTRL_IDMAC_EN;
  Ctrl |= DW_MMC_CTRL_DMA_RESET;
  Status = DwMmcHcRwMmio (
             DevIo,
             DW_MMC_CTRL,
             FALSE,
             sizeof (Ctrl),
             &Ctrl
             );
  if (EFI_ERROR (Status)) {
    return Status;
  }
  //
  // Stop IDMAC
  //
  Status = DwMmcHcRwMmio (
             DevIo,
             DW_MMC_BMOD,
             TRUE,
             sizeof (Bmod),
             &Bmod
             );
  if (EFI_ERROR (Status)) {
    return Status;
  }
  Bmod &= ~(DW_MMC_BMOD_FB | DW_MMC_BMOD_DE);
  Bmod |= DW_MMC_BMOD_SWR;
  Status = DwMmcHcRwMmio (
             DevIo,
             DW_MMC_BMOD,
             FALSE,
             sizeof (Bmod),
             &Bmod
             );
  if (EFI_ERROR (Status)) {
    return Status;
  }
  return Status;
}

/**
  Build DMA descriptor table for transfer.

  @param[in] Trb            The pointer to the DW_MMC_HC_TRB instance.

  @retval EFI_SUCCESS       The DMA descriptor table is created successfully.
  @retval Others            The DMA descriptor table isn't created successfully.

**/
EFI_STATUS
BuildDmaDescTable (
  IN DW_MMC_HC_TRB          *Trb
  )
{
  EFI_PHYSICAL_ADDRESS      Data;
  UINT64                    DataLen;
  UINT64                    Entries;
  UINT32                    Index;
  UINT64                    Remaining;
  UINTN                     TableSize;
  EFI_DEVICE_IO_PROTOCOL    *DevIo;
  EFI_STATUS                Status;
  UINTN                     Bytes;
  UINTN                     Blocks;
  DW_MMC_HC_DMA_DESC_LINE   *DmaDesc;
  UINT32                    DmaDescPhy;
  UINT32                    Idsts;
  UINT32                    BytCnt;
  UINT32                    BlkSize;

  Data    = Trb->DataPhy;
  DataLen = Trb->DataLen;
  DevIo   = Trb->Private->DevIo;
  //
  // Only support 32bit DMA Descriptor Table
  //
  if ((Data >= 0x100000000ul) || ((Data + DataLen) > 0x100000000ul)) {
    return EFI_INVALID_PARAMETER;
  }
  //
  // Address field shall be set on 32-bit boundary (Lower 2-bit is always set
  // to 0) for 32-bit address descriptor table.
  //
  if ((Data & (BIT0 | BIT1)) != 0) {
    DEBUG ((
      DEBUG_INFO,
      "The buffer [0x%x] to construct DMA desc is not aligned to 4 bytes!\n",
      Data
      ));
  }

  Entries   = (DataLen + DWMMC_DMA_BUF_SIZE - 1) / DWMMC_DMA_BUF_SIZE;
  TableSize = Entries * sizeof (DW_MMC_HC_DMA_DESC_LINE);
  Blocks    = (DataLen + DW_MMC_BLOCK_SIZE - 1) / DW_MMC_BLOCK_SIZE;

  Trb->DmaDescPages = (UINT32)EFI_SIZE_TO_PAGES (Entries * DWMMC_DMA_BUF_SIZE);
  Status = DevIo->AllocateBuffer (
                    DevIo,
                    AllocateAnyPages,
                    EfiBootServicesData,
                    EFI_SIZE_TO_PAGES (TableSize),
                    (EFI_PHYSICAL_ADDRESS *)&Trb->DmaDesc
                    );
  if (EFI_ERROR (Status)) {
    return EFI_OUT_OF_RESOURCES;
  }
  ZeroMem (Trb->DmaDesc, TableSize);
  Bytes  = TableSize;
  Status = DevIo->Map (
                    DevIo,
                    EfiBusMasterCommonBuffer,
                    (EFI_PHYSICAL_ADDRESS *)Trb->DmaDesc,
                    &Bytes,
                    &Trb->DmaDescPhy,
                    &Trb->DmaMap
                    );

  if (EFI_ERROR (Status) || (Bytes != TableSize)) {
    //
    // Map error or unable to map the whole RFis buffer into a contiguous
    // region.
    //
    DevIo->FreeBuffer (
             DevIo,
             EFI_SIZE_TO_PAGES (TableSize),
             (EFI_PHYSICAL_ADDRESS)Trb->DmaDesc
             );
    return EFI_OUT_OF_RESOURCES;
  }

  if ((UINT64)(UINTN)Trb->DmaDescPhy > 0x100000000ul) {
    //
    // The DMA doesn't support 64bit addressing.
    //
    DevIo->Unmap (
      DevIo,
      Trb->DmaMap
    );
    return EFI_DEVICE_ERROR;
  }

  if (DataLen < DW_MMC_BLOCK_SIZE) {
    BlkSize = DataLen;
    BytCnt = DataLen;
    Remaining = DataLen;
  } else {
    BlkSize = DW_MMC_BLOCK_SIZE;
    BytCnt = DW_MMC_BLOCK_SIZE * Blocks;
    Remaining = DW_MMC_BLOCK_SIZE * Blocks;
  }
  Status = DwMmcHcRwMmio (
             DevIo,
             DW_MMC_BLKSIZ,
             FALSE,
             sizeof (BlkSize),
             &BlkSize
             );
  if (EFI_ERROR (Status)) {
    DEBUG ((
      DEBUG_ERROR,
      "BuildDmaDescTable: set block size fails: %r\n",
      Status
      ));
    return Status;
  }
  Status = DwMmcHcRwMmio (
             DevIo,
             DW_MMC_BYTCNT,
             FALSE,
             sizeof (BytCnt),
             &BytCnt
             );
  if (EFI_ERROR (Status)) {
    return Status;
  }
  DmaDesc = Trb->DmaDesc;
  for (Index = 0; Index < Entries; Index++, DmaDesc++) {
    DmaDesc->Des0 = DW_MMC_IDMAC_DES0_OWN | DW_MMC_IDMAC_DES0_CH |
                    DW_MMC_IDMAC_DES0_DIC;
    DmaDesc->Des1 = DW_MMC_IDMAC_DES1_BS1 (DWMMC_DMA_BUF_SIZE);
    //
    // Buffer Address
    //
    DmaDesc->Des2 = (UINT32)((UINTN)Trb->DataPhy +
                    (DWMMC_DMA_BUF_SIZE * Index));
    //
    // Next Descriptor Address
    //
    DmaDesc->Des3 = (UINT32)((UINTN)Trb->DmaDescPhy +
                    sizeof (DW_MMC_HC_DMA_DESC_LINE) * (Index + 1));
    Remaining = Remaining - DWMMC_DMA_BUF_SIZE;
  }
  //
  // First Descriptor
  //
  Trb->DmaDesc[0].Des0 |= DW_MMC_IDMAC_DES0_FS;
  //
  // Last Descriptor
  //
  Trb->DmaDesc[Entries - 1].Des0 &= ~(DW_MMC_IDMAC_DES0_CH |
                                    DW_MMC_IDMAC_DES0_DIC);
  Trb->DmaDesc[Entries - 1].Des0 |= DW_MMC_IDMAC_DES0_OWN |
                                    DW_MMC_IDMAC_DES0_LD;
  Trb->DmaDesc[Entries - 1].Des1 = DW_MMC_IDMAC_DES1_BS1 (Remaining +
                                   DWMMC_DMA_BUF_SIZE);
  //
  // Set the next field of the Last Descriptor
  //
  Trb->DmaDesc[Entries - 1].Des3 = 0;
  DmaDescPhy = (UINT32)Trb->DmaDescPhy;
  Status = DwMmcHcRwMmio (
             DevIo,
             DW_MMC_DBADDR,
             FALSE,
             sizeof (DmaDescPhy),
             &DmaDescPhy
             );
  if (EFI_ERROR (Status)) {
    return Status;
  }
  ArmDataSynchronizationBarrier ();
  ArmInstructionSynchronizationBarrier ();
  //
  // Clear interrupts
  //
  Idsts = ~0;
  Status = DwMmcHcRwMmio (
             DevIo,
             DW_MMC_IDSTS,
             FALSE,
             sizeof (Idsts),
             &Idsts
             );
  return Status;
}

EFI_STATUS
ReadFifo (
  IN DW_MMC_HC_TRB          *Trb
  )
{
  EFI_STATUS                Status;
  EFI_DEVICE_IO_PROTOCOL    *DevIo;
  UINT32                    Data;
  UINT32                    Received;
  UINT32                    Count;
  UINT32                    Intsts;
  UINT32                    Sts;
  UINT32                    FifoCount;
  UINT32                    Index;     /* count with bytes */
  UINT32                    Ascending;
  UINT32                    Descending;

  DevIo   = Trb->Private->DevIo;
  Received = 0;
  Count = 0;
  Index = 0;
  Ascending = 0;
  Descending = ((Trb->DataLen + 3) & ~3) - 4;
  do {
    Status = DwMmcHcRwMmio (
               DevIo,
               DW_MMC_RINTSTS,
               TRUE,
               sizeof (Intsts),
               &Intsts
               );
    if (EFI_ERROR (Status)) {
      DEBUG ((
        DEBUG_ERROR,
        "ReadFifo: failed to read RINTSTS, Status:%r\n",
        Status
        ));
      return Status;
    }
    if (Trb->DataLen && ((Intsts & DW_MMC_INT_RXDR) ||
       (Intsts & DW_MMC_INT_DTO))) {
      Status = DwMmcHcRwMmio (
                 DevIo,
                 DW_MMC_STATUS,
                 TRUE,
                 sizeof (Sts),
                 &Sts
                 );
      if (EFI_ERROR (Status)) {
        DEBUG ((
          DEBUG_ERROR,
          "ReadFifo: failed to read STATUS, Status:%r\n",
          Status
          ));
        return Status;
      }
      //
      // Convert to bytes
      //
      FifoCount = GET_STS_FIFO_COUNT (Sts) << 2;
      if ((FifoCount == 0) && (Received < Trb->DataLen)) {
        continue;
      }
      Index = 0;
      Count = (MIN (FifoCount, Trb->DataLen) + 3) & ~3;
      while (Index < Count) {
        Status = DwMmcHcRwMmio (
                   DevIo,
                   DW_MMC_FIFO_START,
                   TRUE,
                   sizeof (Data),
                   &Data
                   );
        if (EFI_ERROR (Status)) {
          DEBUG ((
            DEBUG_ERROR,
            "ReadFifo: failed to read FIFO, Status:%r\n",
            Status
            ));
          return Status;
        }
        if (Trb->UseBE) {
          *(UINT32 *)((UINTN)Trb->Data + Descending) = SwapBytes32 (Data);
          Descending = Descending - 4;
        } else {
          *(UINT32 *)((UINTN)Trb->Data + Ascending) = Data;
          Ascending += 4;
        }
        Index += 4;
        Received += 4;
      } /* while */
    } /* if */
  } while (((Intsts & DW_MMC_INT_CMD_DONE) == 0) || (Received < Trb->DataLen));
  //
  // Clear RINTSTS
  //
  Intsts = ~0;
  Status = DwMmcHcRwMmio (
             DevIo,
             DW_MMC_RINTSTS,
             FALSE,
             sizeof (Intsts),
             &Intsts
             );
  if (EFI_ERROR (Status)) {
    DEBUG ((
      DEBUG_ERROR,
      "ReadFifo: failed to write RINTSTS, Status:%r\n",
      Status
      ));
    return Status;
  }
  return EFI_SUCCESS;
}

/**
  Create a new TRB for the SD/MMC cmd request.

  @param[in] Private        A pointer to the DW_MMC_HC_PRIVATE_DATA instance.
  @param[in] Slot           The slot number of the SD card to send the command
                            to.
  @param[in] Packet         A pointer to the SD command data structure.
  @param[in] Event          If Event is NULL, blocking I/O is performed. If
                            Event is not NULL, then nonblocking I/O is
                            performed, and Event will be signaled when the
                            Packet completes.

  @return Created Trb or NULL.

**/
DW_MMC_HC_TRB *
DwMmcCreateTrb (
  IN DW_MMC_HC_PRIVATE_DATA              *Private,
  IN UINT8                               Slot,
  IN EFI_SD_MMC_PASS_THRU_COMMAND_PACKET *Packet,
  IN EFI_EVENT                           Event
  )
{
  DW_MMC_HC_TRB                 *Trb;
  EFI_STATUS                    Status;
  EFI_TPL                       OldTpl;
  EFI_IO_OPERATION_TYPE         Flag;
  EFI_DEVICE_IO_PROTOCOL        *DevIo;
  UINTN                         MapLength;

  Trb = AllocateZeroPool (sizeof (DW_MMC_HC_TRB));
  if (Trb == NULL) {
    return NULL;
  }

  Trb->Signature = DW_MMC_HC_TRB_SIG;
  Trb->Slot      = Slot;
  Trb->BlockSize = 0x200;
  Trb->Packet    = Packet;
  Trb->Event     = Event;
  Trb->Started   = FALSE;
  Trb->Timeout   = Packet->Timeout;
  Trb->Private   = Private;

  if ((Packet->InTransferLength != 0) && (Packet->InDataBuffer != NULL)) {
    Trb->Data    = Packet->InDataBuffer;
    Trb->DataLen = Packet->InTransferLength;
    Trb->Read    = TRUE;
    ZeroMem (Trb->Data, Trb->DataLen);
  } else if (Packet->OutTransferLength && (Packet->OutDataBuffer != NULL)) {
    Trb->Data    = Packet->OutDataBuffer;
    Trb->DataLen = Packet->OutTransferLength;
    Trb->Read    = FALSE;
  } else if (!Packet->InTransferLength && !Packet->OutTransferLength) {
    Trb->Data    = NULL;
    Trb->DataLen = 0;
  } else {
    goto Error;
  }

  if (((Private->Slot[Trb->Slot].CardType == EmmcCardType) &&
       (Packet->SdMmcCmdBlk->CommandIndex == EMMC_SEND_TUNING_BLOCK)) ||
      ((Private->Slot[Trb->Slot].CardType == SdCardType) &&
       (Packet->SdMmcCmdBlk->CommandIndex == SD_SEND_TUNING_BLOCK))) {
    Trb->Mode = SdMmcPioMode;
  } else {
    if (Trb->Read) {
      Flag = EfiBusMasterWrite;
    } else {
      Flag = EfiBusMasterRead;
    }

    DevIo = Private->DevIo;
    if (Private->Slot[Trb->Slot].CardType == SdCardType) {
      Trb->UseFifo = TRUE;
    } else {
      Trb->UseFifo = FALSE;
      if (Trb->DataLen) {
        MapLength = Trb->DataLen;
        Status = DevIo->Map (
                          DevIo,
                          Flag,
                          Trb->Data,
                          &MapLength,
                          &Trb->DataPhy,
                          &Trb->DataMap
                          );
        if (EFI_ERROR (Status) || (Trb->DataLen != MapLength)) {
          Status = EFI_BAD_BUFFER_SIZE;
          goto Error;
        }

        Status = BuildDmaDescTable (Trb);
        if (EFI_ERROR (Status)) {
          DevIo->Unmap (DevIo, Trb->DataMap);
          goto Error;
        }
        Status = DwMmcHcStartDma (Private, Trb);
        if (EFI_ERROR (Status)) {
          DevIo->Unmap (DevIo, Trb->DataMap);
          goto Error;
        }
      }
    }
  } /* TuningBlock */

  if (Event != NULL) {
    OldTpl = gBS->RaiseTPL (TPL_NOTIFY);
    InsertTailList (&Private->Queue, &Trb->TrbList);
    gBS->RestoreTPL (OldTpl);
  }

  return Trb;

Error:
  return NULL;
}

/**
  Free the resource used by the TRB.

  @param[in] Trb            The pointer to the DW_MMC_HC_TRB instance.

**/
VOID
DwMmcFreeTrb (
  IN DW_MMC_HC_TRB           *Trb
  )
{
  EFI_DEVICE_IO_PROTOCOL     *DevIo;

  DevIo = Trb->Private->DevIo;

  if (Trb->DmaMap != NULL) {
    DevIo->Unmap (DevIo, Trb->DmaMap);
  }
  if (Trb->DataMap != NULL) {
    DevIo->Unmap (DevIo, Trb->DataMap);
  }
  FreePool (Trb);
}

/**
  Check if the env is ready for execute specified TRB.

  @param[in] Private        A pointer to the DW_MMC_HC_PRIVATE_DATA instance.
  @param[in] Trb            The pointer to the DW_MMC_HC_TRB instance.

  @retval EFI_SUCCESS       The env is ready for TRB execution.
  @retval EFI_NOT_READY     The env is not ready for TRB execution.
  @retval Others            Some erros happen.

**/
EFI_STATUS
DwMmcCheckTrbEnv (
  IN DW_MMC_HC_PRIVATE_DATA           *Private,
  IN DW_MMC_HC_TRB                    *Trb
  )
{
  return EFI_SUCCESS;
}

/**
  Wait for the env to be ready for execute specified TRB.

  @param[in] Private        A pointer to the DW_MMC_HC_PRIVATE_DATA instance.
  @param[in] Trb            The pointer to the DW_MMC_HC_TRB instance.

  @retval EFI_SUCCESS       The env is ready for TRB execution.
  @retval EFI_TIMEOUT       The env is not ready for TRB execution in time.
  @retval Others            Some erros happen.

**/
EFI_STATUS
DwMmcWaitTrbEnv (
  IN DW_MMC_HC_PRIVATE_DATA           *Private,
  IN DW_MMC_HC_TRB                    *Trb
  )
{
  EFI_STATUS                          Status;
  EFI_SD_MMC_PASS_THRU_COMMAND_PACKET *Packet;
  UINT64                              Timeout;
  BOOLEAN                             InfiniteWait;

  //
  // Wait Command Complete Interrupt Status bit in Normal Interrupt Status
  // Register
  //
  Packet  = Trb->Packet;
  Timeout = Packet->Timeout;
  if (Timeout == 0) {
    InfiniteWait = TRUE;
  } else {
    InfiniteWait = FALSE;
  }

  while (InfiniteWait || (Timeout > 0)) {
    //
    // Check Trb execution result by reading Normal Interrupt Status register.
    //
    Status = DwMmcCheckTrbEnv (Private, Trb);
    if (Status != EFI_NOT_READY) {
      return Status;
    }
    //
    // Stall for 1 microsecond.
    //
    gBS->Stall (1);

    Timeout--;
  }

  return EFI_TIMEOUT;
}

EFI_STATUS
DwEmmcExecTrb (
  IN DW_MMC_HC_PRIVATE_DATA           *Private,
  IN DW_MMC_HC_TRB                    *Trb
  )
{
  EFI_STATUS                          Status;
  EFI_SD_MMC_PASS_THRU_COMMAND_PACKET *Packet;
  EFI_DEVICE_IO_PROTOCOL              *DevIo;
  UINT32                              Cmd;
  UINT32                              MmcStatus;
  UINT32                              IntStatus;
  UINT32                              Argument;
  UINT32                              ErrMask;
  UINT32                              Timeout;

  Packet = Trb->Packet;
  DevIo  = Trb->Private->DevIo;

  ArmDataSynchronizationBarrier ();
  ArmInstructionSynchronizationBarrier ();
  //
  // Wait until MMC is idle
  //
  do {
    Status = DwMmcHcRwMmio (
               DevIo,
               DW_MMC_STATUS,
               TRUE,
               sizeof (MmcStatus),
               &MmcStatus
               );
    if (EFI_ERROR (Status)) {
      return Status;
    }
  } while (MmcStatus & DW_MMC_STS_DATA_BUSY);

  IntStatus = ~0;
  Status = DwMmcHcRwMmio (
             DevIo,
             DW_MMC_RINTSTS,
             FALSE,
             sizeof (IntStatus),
             &IntStatus
             );
  if (EFI_ERROR (Status)) {
    return Status;
  }
  Cmd = CMD_INDEX (Packet->SdMmcCmdBlk->CommandIndex);
  if ((Packet->SdMmcCmdBlk->CommandType == SdMmcCommandTypeAc) ||
      (Packet->SdMmcCmdBlk->CommandType == SdMmcCommandTypeAdtc)) {
    switch (Packet->SdMmcCmdBlk->CommandIndex) {
    case EMMC_SET_RELATIVE_ADDR:
      Cmd |= BIT_CMD_SEND_INIT;
      break;
    case EMMC_SEND_STATUS:
      Cmd |= BIT_CMD_WAIT_PRVDATA_COMPLETE;
      break;
    case EMMC_STOP_TRANSMISSION:
      Cmd |= BIT_CMD_STOP_ABORT_CMD;
      break;
    }
    if (Packet->InTransferLength) {
      Cmd |= BIT_CMD_WAIT_PRVDATA_COMPLETE | BIT_CMD_DATA_EXPECTED |
             BIT_CMD_READ;
    } else if (Packet->OutTransferLength) {
      Cmd |= BIT_CMD_WAIT_PRVDATA_COMPLETE | BIT_CMD_DATA_EXPECTED |
             BIT_CMD_WRITE;
    }
    Cmd |= BIT_CMD_RESPONSE_EXPECT | BIT_CMD_CHECK_RESPONSE_CRC;
  } else {
    switch (Packet->SdMmcCmdBlk->CommandIndex) {
    case EMMC_GO_IDLE_STATE:
      Cmd |= BIT_CMD_SEND_INIT;
      break;
    case EMMC_SEND_OP_COND:
      Cmd |= BIT_CMD_RESPONSE_EXPECT;
      break;
    case EMMC_ALL_SEND_CID:
      Cmd |= BIT_CMD_RESPONSE_EXPECT | BIT_CMD_LONG_RESPONSE |
             BIT_CMD_CHECK_RESPONSE_CRC | BIT_CMD_SEND_INIT;
      break;
    }
  }
  switch (Packet->SdMmcCmdBlk->ResponseType) {
  case SdMmcResponseTypeR2:
    Cmd |= BIT_CMD_RESPONSE_EXPECT | BIT_CMD_CHECK_RESPONSE_CRC |
           BIT_CMD_LONG_RESPONSE;
    break;
  case SdMmcResponseTypeR3:
    Cmd |= BIT_CMD_RESPONSE_EXPECT;
    break;
  }
  Cmd |= BIT_CMD_USE_HOLD_REG | BIT_CMD_START;

  Argument = Packet->SdMmcCmdBlk->CommandArgument;
  Status = DwMmcHcRwMmio (
             DevIo,
             DW_MMC_CMDARG,
             FALSE,
             sizeof (Argument),
             &Argument
             );
  if (EFI_ERROR (Status)) {
    return Status;
  }
  ArmDataSynchronizationBarrier ();
  ArmInstructionSynchronizationBarrier ();
  Status = DwMmcHcRwMmio (
             DevIo,
             DW_MMC_CMD,
             FALSE,
             sizeof (Cmd),
             &Cmd
             );
  if (EFI_ERROR (Status)) {
    return Status;
  }
  ArmDataSynchronizationBarrier ();
  ArmInstructionSynchronizationBarrier ();

  ErrMask = DW_MMC_INT_EBE | DW_MMC_INT_HLE | DW_MMC_INT_RTO |
            DW_MMC_INT_RCRC | DW_MMC_INT_RE;
  ErrMask |= DW_MMC_INT_DCRC | DW_MMC_INT_DRT | DW_MMC_INT_SBE;
  do {
    Timeout = 10000;
    if (--Timeout == 0) {
      break;
    }
    Status = DwMmcHcRwMmio (
               DevIo,
               DW_MMC_RINTSTS,
               TRUE,
               sizeof (IntStatus),
               &IntStatus
               );
    if (EFI_ERROR (Status)) {
      return Status;
    }
    if (IntStatus & ErrMask) {
      return EFI_DEVICE_ERROR;
    }
    if (Trb->DataLen && ((IntStatus & DW_MMC_INT_DTO) == 0)) {
      //
      // Transfer Not Done
      //
      MicroSecondDelay (10);
      continue;
    }
    MicroSecondDelay (10);
  } while (!(IntStatus & DW_MMC_INT_CMD_DONE));
  switch (Packet->SdMmcCmdBlk->ResponseType) {
    case SdMmcResponseTypeR1:
    case SdMmcResponseTypeR1b:
    case SdMmcResponseTypeR3:
    case SdMmcResponseTypeR4:
    case SdMmcResponseTypeR5:
      Status = DwMmcHcRwMmio (
                 DevIo,
                 DW_MMC_RESP0,
                 TRUE,
                 sizeof (Packet->SdMmcStatusBlk->Resp0),
                 &Packet->SdMmcStatusBlk->Resp0
                 );
      if (EFI_ERROR (Status)) {
        return Status;
      }
      break;
    case SdMmcResponseTypeR2:
      Status = DwMmcHcRwMmio (
                 DevIo,
                 DW_MMC_RESP0,
                 TRUE,
                 sizeof (Packet->SdMmcStatusBlk->Resp0),
                 &Packet->SdMmcStatusBlk->Resp0
                 );
      if (EFI_ERROR (Status)) {
        return Status;
      }
      Status = DwMmcHcRwMmio (
                 DevIo,
                 DW_MMC_RESP1,
                 TRUE,
                 sizeof (Packet->SdMmcStatusBlk->Resp1),
                 &Packet->SdMmcStatusBlk->Resp1
                 );
      if (EFI_ERROR (Status)) {
        return Status;
      }
      Status = DwMmcHcRwMmio (
                 DevIo,
                 DW_MMC_RESP2,
                 TRUE,
                 sizeof (Packet->SdMmcStatusBlk->Resp2),
                 &Packet->SdMmcStatusBlk->Resp2
                 );
      if (EFI_ERROR (Status)) {
        return Status;
      }
      Status = DwMmcHcRwMmio (
                 DevIo,
                 DW_MMC_RESP3,
                 TRUE,
                 sizeof (Packet->SdMmcStatusBlk->Resp3),
                 &Packet->SdMmcStatusBlk->Resp3
                 );
      if (EFI_ERROR (Status)) {
        return Status;
      }
      break;
  }

  //
  // The workaround on EMMC_SEND_CSD is used to be compatible with SDHC.
  //
  if (Packet->SdMmcCmdBlk->CommandIndex == EMMC_SEND_CSD) {
    {
      UINT32   Buf[4];
      ZeroMem (Buf, sizeof (Buf));
      CopyMem (
        (UINT8 *)Buf,
        (UINT8 *)&Packet->SdMmcStatusBlk->Resp0 + 1,
        sizeof (Buf) - 1
        );
      CopyMem (
        (UINT8 *)&Packet->SdMmcStatusBlk->Resp0,
        (UINT8 *)Buf,
        sizeof (Buf) - 1
        );
    }
  }

  return EFI_SUCCESS;
}

EFI_STATUS
DwSdExecTrb (
  IN DW_MMC_HC_PRIVATE_DATA           *Private,
  IN DW_MMC_HC_TRB                    *Trb
  )
{
  EFI_STATUS                          Status;
  EFI_SD_MMC_PASS_THRU_COMMAND_PACKET *Packet;
  EFI_DEVICE_IO_PROTOCOL              *DevIo;
  UINT32                              Cmd;
  UINT32                              MmcStatus;
  UINT32                              IntStatus;
  UINT32                              Argument;
  UINT32                              ErrMask;
  UINT32                              Timeout;
  UINT32                              Idsts;
  UINT32                              BytCnt;
  UINT32                              BlkSize;

  Packet = Trb->Packet;
  DevIo  = Trb->Private->DevIo;

  ArmDataSynchronizationBarrier ();
  ArmInstructionSynchronizationBarrier ();
  //
  // Wait until MMC is idle
  //
  do {
    Status = DwMmcHcRwMmio (
               DevIo,
               DW_MMC_STATUS,
               TRUE,
               sizeof (MmcStatus),
               &MmcStatus
               );
    if (EFI_ERROR (Status)) {
      return Status;
    }
  } while (MmcStatus & DW_MMC_STS_DATA_BUSY);

  IntStatus = ~0;
  Status = DwMmcHcRwMmio (
             DevIo,
             DW_MMC_RINTSTS,
             FALSE,
             sizeof (IntStatus),
             &IntStatus
             );
  if (EFI_ERROR (Status)) {
    return Status;
  }
  Cmd = CMD_INDEX (Packet->SdMmcCmdBlk->CommandIndex);
  if ((Packet->SdMmcCmdBlk->CommandType == SdMmcCommandTypeAc) ||
      (Packet->SdMmcCmdBlk->CommandType == SdMmcCommandTypeAdtc)) {
    switch (Packet->SdMmcCmdBlk->CommandIndex) {
    case SD_SET_RELATIVE_ADDR:
      Cmd |= BIT_CMD_SEND_INIT;
      break;
    case SD_STOP_TRANSMISSION:
      Cmd |= BIT_CMD_STOP_ABORT_CMD;
      break;
    case SD_SEND_SCR:
      Trb->UseBE = TRUE;
      break;
    }
    if (Packet->InTransferLength) {
      Cmd |= BIT_CMD_WAIT_PRVDATA_COMPLETE | BIT_CMD_DATA_EXPECTED |
             BIT_CMD_READ;
    } else if (Packet->OutTransferLength) {
      Cmd |= BIT_CMD_WAIT_PRVDATA_COMPLETE | BIT_CMD_DATA_EXPECTED |
             BIT_CMD_WRITE;
    }
    Cmd |= BIT_CMD_RESPONSE_EXPECT | BIT_CMD_CHECK_RESPONSE_CRC |
           BIT_CMD_SEND_AUTO_STOP;
  } else {
    switch (Packet->SdMmcCmdBlk->CommandIndex) {
    case SD_GO_IDLE_STATE:
      Cmd |= BIT_CMD_SEND_INIT;
      break;
    }
  }
  switch (Packet->SdMmcCmdBlk->ResponseType) {
  case SdMmcResponseTypeR2:
    Cmd |= BIT_CMD_RESPONSE_EXPECT | BIT_CMD_CHECK_RESPONSE_CRC |
           BIT_CMD_LONG_RESPONSE;
    break;
  case SdMmcResponseTypeR3:
    Cmd |= BIT_CMD_RESPONSE_EXPECT;
    break;
  case SdMmcResponseTypeR1b:
  case SdMmcResponseTypeR4:
  case SdMmcResponseTypeR6:
  case SdMmcResponseTypeR7:
    Cmd |= BIT_CMD_RESPONSE_EXPECT | BIT_CMD_CHECK_RESPONSE_CRC;
    break;
  }
  Cmd |= BIT_CMD_USE_HOLD_REG | BIT_CMD_START;

  if (Trb->UseFifo == TRUE) {
    BytCnt = Packet->InTransferLength;
    Status = DwMmcHcRwMmio (
               DevIo,
               DW_MMC_BYTCNT,
               FALSE,
               sizeof (BytCnt),
               &BytCnt
               );
    if (EFI_ERROR (Status)) {
      return Status;
    }
    if (Packet->InTransferLength > DW_MMC_BLOCK_SIZE) {
      BlkSize = DW_MMC_BLOCK_SIZE;
    } else {
      BlkSize = Packet->InTransferLength;
    }
    Status = DwMmcHcRwMmio (
               DevIo,
               DW_MMC_BLKSIZ,
               FALSE,
               sizeof (BlkSize),
               &BlkSize
               );
    if (EFI_ERROR (Status)) {
      DEBUG ((DEBUG_ERROR, "DwSdExecTrb: set block size fails: %r\n", Status));
      return Status;
    }
  }

  Argument = Packet->SdMmcCmdBlk->CommandArgument;
  Status = DwMmcHcRwMmio (
             DevIo,
             DW_MMC_CMDARG,
             FALSE,
             sizeof (Argument),
             &Argument
             );
  if (EFI_ERROR (Status)) {
    return Status;
  }
  ArmDataSynchronizationBarrier ();
  ArmInstructionSynchronizationBarrier ();
  Status = DwMmcHcRwMmio (
             DevIo,
             DW_MMC_CMD,
             FALSE,
             sizeof (Cmd),
             &Cmd
             );
  if (EFI_ERROR (Status)) {
    return Status;
  }
  ArmDataSynchronizationBarrier ();
  ArmInstructionSynchronizationBarrier ();

  ErrMask = DW_MMC_INT_EBE | DW_MMC_INT_HLE | DW_MMC_INT_RTO |
            DW_MMC_INT_RCRC | DW_MMC_INT_RE;
  ErrMask |= DW_MMC_INT_DRT | DW_MMC_INT_SBE;
  if (Packet->InTransferLength || Packet->OutTransferLength) {
    ErrMask |= DW_MMC_INT_DCRC;
  }
  if (Trb->UseFifo == TRUE) {
    Status = ReadFifo (Trb);
    if (EFI_ERROR (Status)) {
      return Status;
    }
  } else {
    Timeout = 10000;
    do {
      if (--Timeout == 0) {
        break;
      }
      Status = DwMmcHcRwMmio (
                 DevIo,
                 DW_MMC_RINTSTS,
                 TRUE,
                 sizeof (IntStatus),
                 &IntStatus
                 );
      if (EFI_ERROR (Status)) {
        return Status;
      }
      if (IntStatus & ErrMask) {
        return EFI_DEVICE_ERROR;
      }
      if (Trb->DataLen && ((IntStatus & DW_MMC_INT_DTO) == 0)) {
        //
        // Transfer not Done
        //
        MicroSecondDelay (10);
        continue;
      }
      MicroSecondDelay (10);
    } while (!(IntStatus & DW_MMC_INT_CMD_DONE));
    if (Packet->InTransferLength) {
      do {
        Status = DwMmcHcRwMmio (
                   DevIo,
                   DW_MMC_IDSTS,
                   TRUE,
                   sizeof (Idsts),
                   &Idsts
                   );
        if (EFI_ERROR (Status)) {
          return Status;
        }
      } while ((Idsts & DW_MMC_IDSTS_RI) == 0);
      Status = DwMmcHcStopDma (Private, Trb);
      if (EFI_ERROR (Status)) {
        return Status;
      }
    } else if (Packet->OutTransferLength) {
      do {
        Status = DwMmcHcRwMmio (
                   DevIo,
                   DW_MMC_IDSTS,
                   TRUE,
                   sizeof (Idsts),
                   &Idsts
                   );
        if (EFI_ERROR (Status)) {
          return Status;
        }
      } while ((Idsts & DW_MMC_IDSTS_TI) == 0);
      Status = DwMmcHcStopDma (Private, Trb);
      if (EFI_ERROR (Status)) {
        return Status;
      }
    } /* Packet->InTransferLength */
  } /* UseFifo */
  switch (Packet->SdMmcCmdBlk->ResponseType) {
    case SdMmcResponseTypeR1:
    case SdMmcResponseTypeR1b:
    case SdMmcResponseTypeR3:
    case SdMmcResponseTypeR4:
    case SdMmcResponseTypeR5:
    case SdMmcResponseTypeR6:
    case SdMmcResponseTypeR7:
      Status = DwMmcHcRwMmio (
                 DevIo,
                 DW_MMC_RESP0,
                 TRUE,
                 sizeof (Packet->SdMmcStatusBlk->Resp0),
                 &Packet->SdMmcStatusBlk->Resp0
                 );
      if (EFI_ERROR (Status)) {
        return Status;
      }
      break;
    case SdMmcResponseTypeR2:
      Status = DwMmcHcRwMmio (
                 DevIo,
                 DW_MMC_RESP0,
                 TRUE,
                 sizeof (Packet->SdMmcStatusBlk->Resp0),
                 &Packet->SdMmcStatusBlk->Resp0
                 );
      if (EFI_ERROR (Status)) {
        return Status;
      }
      Status = DwMmcHcRwMmio (
                 DevIo,
                 DW_MMC_RESP1,
                 TRUE,
                 sizeof (Packet->SdMmcStatusBlk->Resp1),
                 &Packet->SdMmcStatusBlk->Resp1
                 );
      if (EFI_ERROR (Status)) {
        return Status;
      }
      Status = DwMmcHcRwMmio (
                 DevIo,
                 DW_MMC_RESP2,
                 TRUE,
                 sizeof (Packet->SdMmcStatusBlk->Resp2),
                 &Packet->SdMmcStatusBlk->Resp2
                 );
      if (EFI_ERROR (Status)) {
        return Status;
      }
      Status = DwMmcHcRwMmio (
                 DevIo,
                 DW_MMC_RESP3,
                 TRUE,
                 sizeof (Packet->SdMmcStatusBlk->Resp3),
                 &Packet->SdMmcStatusBlk->Resp3
                 );
      if (EFI_ERROR (Status)) {
        return Status;
      }
      break;
  }

  //
  // The workaround on SD_SEND_CSD is used to be compatible with SDHC.
  //
  if (Packet->SdMmcCmdBlk->CommandIndex == SD_SEND_CSD) {
    {
      UINT32   Buf[4];
      ZeroMem (Buf, sizeof (Buf));
      CopyMem (
        (UINT8 *)Buf,
        (UINT8 *)&Packet->SdMmcStatusBlk->Resp0 + 1,
        sizeof (Buf) - 1
        );
      CopyMem (
        (UINT8 *)&Packet->SdMmcStatusBlk->Resp0,
        (UINT8 *)Buf,
        sizeof (Buf) - 1
        );
    }
  }

  return EFI_SUCCESS;
}

/**
  Execute the specified TRB.

  @param[in] Private        A pointer to the DW_MMC_HC_PRIVATE_DATA instance.
  @param[in] Trb            The pointer to the DW_MMC_HC_TRB instance.

  @retval EFI_SUCCESS       The TRB is sent to host controller successfully.
  @retval Others            Some erros happen when sending this request to the
                            host controller.

**/
EFI_STATUS
DwMmcExecTrb (
  IN DW_MMC_HC_PRIVATE_DATA           *Private,
  IN DW_MMC_HC_TRB                    *Trb
  )
{
  EFI_STATUS                          Status = EFI_SUCCESS;
  UINT32                              Slot;

  Slot = Trb->Slot;
  if (Private->Slot[Slot].CardType == EmmcCardType) {
    Status = DwEmmcExecTrb (Private, Trb);
  } else if (Private->Slot[Slot].CardType == SdCardType) {
    Status = DwSdExecTrb (Private, Trb);
  } else {
    ASSERT (0);
  }
  return Status;
}

/**
  Check the TRB execution result.

  @param[in] Private        A pointer to the DW_MMC_HC_PRIVATE_DATA instance.
  @param[in] Trb            The pointer to the DW_MMC_HC_TRB instance.

  @retval EFI_SUCCESS       The TRB is executed successfully.
  @retval EFI_NOT_READY     The TRB is not completed for execution.
  @retval Others            Some erros happen when executing this request.

**/
EFI_STATUS
DwMmcCheckTrbResult (
  IN DW_MMC_HC_PRIVATE_DATA           *Private,
  IN DW_MMC_HC_TRB                    *Trb
  )
{
  EFI_STATUS                          Status;
  EFI_SD_MMC_PASS_THRU_COMMAND_PACKET *Packet;
  UINT32                              Idsts;

  Packet  = Trb->Packet;
  if (Trb->UseFifo == TRUE) {
    return EFI_SUCCESS;
  }
  if (Packet->InTransferLength) {
    do {
      Status = DwMmcHcRwMmio (
                 Private->DevIo,
                 DW_MMC_IDSTS,
                 TRUE,
                 sizeof (Idsts),
                 &Idsts
                 );
      if (EFI_ERROR (Status)) {
        return Status;
      }
    } while ((Idsts & BIT1) == 0);
  } else if (Packet->OutTransferLength) {
    do {
      Status = DwMmcHcRwMmio (
                 Private->DevIo,
                 DW_MMC_IDSTS,
                 TRUE,
                 sizeof (Idsts),
                 &Idsts
                 );
      if (EFI_ERROR (Status)) {
        return Status;
      }
    } while ((Idsts & BIT0) == 0);
  } else {
    return EFI_SUCCESS;
  }
  Idsts = ~0;
  Status = DwMmcHcRwMmio (
             Private->DevIo,
             DW_MMC_IDSTS,
             FALSE,
             sizeof (Idsts), &Idsts);
  if (EFI_ERROR (Status)) {
    return Status;
  }
  return EFI_SUCCESS;
}

/**
  Wait for the TRB execution result.

  @param[in] Private        A pointer to the DW_MMC_HC_PRIVATE_DATA instance.
  @param[in] Trb            The pointer to the DW_MMC_HC_TRB instance.

  @retval EFI_SUCCESS       The TRB is executed successfully.
  @retval Others            Some erros happen when executing this request.

**/
EFI_STATUS
DwMmcWaitTrbResult (
  IN DW_MMC_HC_PRIVATE_DATA           *Private,
  IN DW_MMC_HC_TRB                    *Trb
  )
{
  EFI_STATUS                          Status;
  EFI_SD_MMC_PASS_THRU_COMMAND_PACKET *Packet;
  UINT64                              Timeout;
  BOOLEAN                             InfiniteWait;

  Packet = Trb->Packet;
  //
  // Wait Command Complete Interrupt Status bit in Normal Interrupt Status
  // Register
  //
  Timeout = Packet->Timeout;
  if (Timeout == 0) {
    InfiniteWait = TRUE;
  } else {
    InfiniteWait = FALSE;
  }

  while (InfiniteWait || (Timeout > 0)) {
    //
    // Check Trb execution result by reading Normal Interrupt Status register.
    //
    Status = DwMmcCheckTrbResult (Private, Trb);
    if (Status != EFI_NOT_READY) {
      return Status;
    }
    //
    // Stall for 1 microsecond.
    //
    gBS->Stall (1);

    Timeout--;
  }

  return EFI_TIMEOUT;
}

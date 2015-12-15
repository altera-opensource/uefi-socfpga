/** @file
  Altera HPS SD/MMC controller Lib file

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

#include <Library/PcdLib.h>
#include <Library/SerialPortPrintLib.h>
#include <Library/AlteraSdMmcLib.h>


// ==================================================================
// Module level variables:
// ==================================================================

MMC_CMD  mPrevMmcCmd;

//#ifdef USE_4_BIT_MODE
UINT32  mRCA;
BOOLEAN mCMD7_detected;
BOOLEAN mSetBitMode_started;
//#endif

volatile IDMA_DES mDmaDes;

// ==================================================================
// Functions Implementation:
// ==================================================================

BOOLEAN
EFIAPI
IsCardPresent (
  VOID
  )
{
  BOOLEAN  Result;
  UINT32   Data32;
  Data32 = MmioRead32 (
    ALT_SDMMC_OFST +
    ALT_SDMMC_STAT_OFST
    );
  Result = (
    ALT_SDMMC_STAT_DATA_3_STAT_GET(Data32) ==
    ALT_SDMMC_STAT_DATA_3_STAT_E_CARDPRESENT
    );
  return (Result);
}


BOOLEAN
EFIAPI
IsWriteProtected (
  VOID
  )
{
  BOOLEAN  Result;
  UINT32   Data32;
  Data32 = MmioRead32 (
    ALT_SDMMC_OFST +
    ALT_SDMMC_WRTPRT_OFST
    );
  Result = (
    ALT_SDMMC_WRTPRT_WR_PROTECT_GET(Data32) ==
    ALT_SDMMC_WRTPRT_WR_PROTECT_E_END
    );
  return (Result);
}


/**
  Initialize the SD/MMC Controller

  @retval EFI_SUCCESS       Successfully.
  @retval other             Some error occurs.

**/
EFI_STATUS
EFIAPI
InitializeSdMmc (
  VOID
  )
{
  EFI_STATUS Status;

  if (PcdGet32 (PcdDebugMsg_SdMmc) != 0)
    SerialPortPrint ("\tSDMMC: %a ()\r\n", __FUNCTION__);

  // Initalize variables
  mPrevMmcCmd = 0;
  //#ifdef USE_4_BIT_MODE
  mRCA = 0xFFFF;
  mCMD7_detected = FALSE;
  mSetBitMode_started = FALSE;
  //#endif

  // Issues a Reset Commands
  MmioOr32  (
    ALT_RSTMGR_OFST +
    ALT_RSTMGR_PER0MODRST_OFST,
    ALT_RSTMGR_PER0MODRST_SDMMC_SET_MSK
    );

  MicroSecondDelay(1000);
  MmioAnd32 (
    ALT_RSTMGR_OFST +
    ALT_RSTMGR_PER0MODRST_OFST,
    ALT_RSTMGR_PER0MODRST_SDMMC_CLR_MSK);

  //
  // Power-On Reset Sequence (as per a10_5v4.pdf 2014.12.15)
  // Software must perform the following steps after the power-on-reset:
  //
  //  1. Before enabling power to the card, confirm that the voltage setting to the voltage regulator is correct.
  Status = InitOnboardSdmmcVoltageRegulator ();
  if (Status != EFI_SUCCESS) return Status;
  //  2. Enable power to the card by setting the power enable bit (power_enable) in the power enable register
  //     (pwren) to 1. Wait for the power ramp-up time before proceeding to the next step.
  Status = EnablePowerToTheCard ();
  if (Status != EFI_SUCCESS) return Status;
  //  3. Set the interrupt masks by resetting the appropriate bits to 0 in the intmask register.
  //  4. Set the int_enable bit of the ctrl register to 1.
  //     Note: Altera recommends that you write 0xFFFFFFFF to the rintsts register to clear any pending
  //           interrupts before setting the int_enable bit to 1.
  Status = InitSdmmcInterrupt ();
  if (Status != EFI_SUCCESS) return Status;
  //  5. Discover the card stack according to the card type. For discovery, you must restrict the clock frequency
  //     to 400 kHz in accordance with SD/MMC/CE-ATA standards. For more information, refer to
  //     Enumerate Card Stack.
  Status = InitBeforeEnumerateCardStack ();
  if (Status != EFI_SUCCESS) return Status;
  // The MmcDxe driver will send the command sequence to identify the card type,
  // Once it finished identifying the card, it will callback via MmcStandByState,
  // we can then perform aditional initialization such as changing the card clock frequency.
  //  6. Set the clock source assignments. Set the card frequency using the clkdiv and clksrc registers of the
  //     controller. For more information, refer to Clock Setup.

  //Step#6 will be done in MmcStandByState

  //  7. The following common registers and fields can be set during initialization process:
  //     a) The response timeout field (response_timeout) of the tmout register. A typical value is 64(0x40).
  //     b) The data timeout field (data_timeout) of the tmout register, highest of the following:
  //        i) 10 * NAC
  //           NAC = card device total access time
  //               = 10 * ((TAAC * FOP) + (100 * NSAC))
  //                 where:
  //                 ~ TAAC = Time-dependent factor of the data access time
  //                 ~ FOP  = The card clock frequency used for the card operation
  //                 ~ NSAC = Worst-case clock rate-dependent factor of the data access time
  //       ii) Host FIFO buffer latency
  //            On read: Time elapsed before host starts reading from a full FIFO buffer
  //            On write: Time elapsed before host starts writing to an empty FIFO buffer
  //      iii) Debounce counter register (debnce). A typical debounce value is 25 ms.
  //       iv) TX watermark field (tx_wmark) of the FIFO threshold watermark register (fifoth). Typically, the
  //           threshold value is set to 512, which is half the FIFO buffer depth.
  //        v) RX watermark field (rx_wmark) of the fifoth register. Typically, the threshold value is set to 511.
  //
  //  These registers do not need to be changed with every SD/MMC/CE-ATA command.
  //  Set them to a typical value according to the SD/MMC/CE-ATA specifications.
  //  Some sub-step in Step#7 that require card type to be known first will also be done in MmcStandByState

  Status = InitCommonRegisters ();
  if (Status != EFI_SUCCESS) return Status;

  Status = InitInternalDMAC ();
  if (Status != EFI_SUCCESS) return Status;

  return EFI_SUCCESS;
}


EFI_STATUS
EFIAPI
InitOnboardSdmmcVoltageRegulator(
  VOID
  )
{
  //
  // This is a stub for switching voltage level between 1.8V and 3.3V before enabling power to the card
  //
  // Most Altera SoC development kit boards are build with MicroSD card slot running at 3.3V.
  //
  // In case the board support card runs at 1.8V, the board will build with a voltage-translation transceiver
  // between the HPS and the SD/MMC card slot and power regulator/supply to support dual voltage.
  // For example, you have a card which is at 1.8V and you eject it and replace it with another card,
  // which is 3.3V, then voltage switching is required in order to have the right voltage level to power the card.
  //
  // The general steps to switch voltage level when the board have a SD/MMC voltage-translation transceiver in
  // between the HPS and the SD/MMC card are as follow:
  // 1. Power the HPS I/O pins for the SD/MMC controller to 3.3V.
  //    a. Connect the same power supply to one of the transceiver voltage input pins.
  // 2. Power the other transceiver voltage input pin with another power supply.
  //    a. Connect this power supply with the SD/MMC card.
  // 3. The SD/MMC will send predefined commands to check if the card supports dual voltage. The response
  //    from the card will indicate if dual voltage is supported
  // 4. Software stops all SD/MMC activity. If the card does not support dual voltage, do not perform the
  //    remaining steps.
  // 5. The GPIO on HPS sends a signal to the external power supply on the board notifying it to switch its
  //    voltage value.
  // 6. After voltage switching is completed, resume the SD/MMC activity.
  return EFI_SUCCESS;
}


EFI_STATUS
EFIAPI
EnablePowerToTheCard(
  VOID
  )
{
  // Enable power to the card
  MmioAndThenOr32 (
    ALT_SDMMC_OFST +
    ALT_SDMMC_PWREN_OFST,
    ALT_SDMMC_PWREN_POWER_EN_CLR_MSK,
    ALT_SDMMC_PWREN_POWER_EN_SET(ALT_SDMMC_PWREN_POWER_EN_E_ON)
    );

  if (PcdGet32 (PcdIsAlteraSoCFPGADevelopmentBoards != 0))
  {
    if (MmioRead32 (ALT_SYSMGR_OFST + ALT_SYSMGR_SILICONID1_OFST) == ALT_SYSMGR_SILICONID1_ES1)
    {
      // A10 Dev Kit Rev A Board Workaround
      MmioAndThenOr32 (
        ALT_SDMMC_OFST +
        ALT_SDMMC_PWREN_OFST,
        ALT_SDMMC_PWREN_POWER_EN_CLR_MSK,
        ALT_SDMMC_PWREN_POWER_EN_SET(ALT_SDMMC_PWREN_POWER_EN_E_OFF)
        );
      if (PcdGet32 (PcdDebugMsg_SdMmc) != 0)
        SerialPortPrint ("\tSDMMC: %a ()  A10 Dev Kit Rev A Board Workaround.\r\n", __FUNCTION__);
    }
  }
  // Time to wait for the power ramp-up to the card
  // The exact power ramp-up is card vendor specific
  // but in general the minimum ramp up time should be 100us.
  // and maximum ramp up time should be 35000us for 2.7-3.6V power supply.
  // most microSD card shall work with duration of at least 1000us
  MicroSecondDelay (TIME_DELAY_IN_MICRO_SECOND_FOR_CARD_POWER_TO_RAMP_UP);
  return EFI_SUCCESS;
}


EFI_STATUS
EFIAPI
InitSdmmcInterrupt(
  VOID
  )
{
  // Disable all interrupts
  MmioAndThenOr32 (
    ALT_SDMMC_OFST +
    ALT_SDMMC_CTL_OFST,
    ALT_SDMMC_CTL_INT_EN_CLR_MSK,
    ALT_SDMMC_CTL_INT_EN_SET(ALT_SDMMC_CTL_INT_EN_E_DISD)
    );
  // Clear any pending interrupts
  MmioWrite32 (
    ALT_SDMMC_OFST +
    ALT_SDMMC_RINTSTS_OFST,
    0xFFFFFFFF
    );
  // Mask all interrupts, we use polling method in UEFI
  MmioWrite32 (
    ALT_SDMMC_OFST +
    ALT_SDMMC_INTMSK_OFST,
    0
    );
  return EFI_SUCCESS;
}


EFI_STATUS
EFIAPI
WaitUntilCardDataNotBusy(
  VOID
  )
{
  UINTN  WaitCount;
  UINT32 Data32;
  WaitCount = 0;
  do {
    Data32 = MmioRead32 (ALT_SDMMC_OFST + ALT_SDMMC_STAT_OFST);
    // If card data not busy then break, else must be card data busy so wait.
    if (ALT_SDMMC_STAT_DATA_BUSY_GET(Data32) == ALT_SDMMC_STAT_DATA_BUSY_E_CARDNOTBUSY)
      break;
  } while ( (WaitCount++ < DATA_BUSY_TIMEOUT) );
  if (WaitCount >= DATA_BUSY_TIMEOUT) {
    if (PcdGet32 (PcdDebugMsg_SdMmc) != 0)
      SerialPortPrint ("\tSDMMC: %a () Timeout\r\n", __FUNCTION__);
    return EFI_TIMEOUT;
  }
  return EFI_SUCCESS;
}


EFI_STATUS
EFIAPI
ChangeCardClockFrequency(
  IN UINT32 clkdiv
  )
{
  EFI_STATUS Status;
  UINT32     Data32;
  UINTN      Count;
  UINT32     smplsel, drvsel;

  if (PcdGet32 (PcdDebugMsg_SdMmc) != 0)
    SerialPortPrint ("\tSDMMC: %a ( 0x%02x ) \r\n", __FUNCTION__, clkdiv);

  // Changing the Card Clock Frequency:
  // Steps:
  // 1. Before disabling the clocks, ensure that the card is not busy with any previous data command. To do so,
  //    verify that the data_busy bit of the status register (status) is 0.
  Status = WaitUntilCardDataNotBusy ();
  if (Status != EFI_SUCCESS) return Status;

  // 2. Reset the cclk_enable bit of the clkena register to 0, to disable the card clock generation.
  MmioAnd32 (
    ALT_SDMMC_OFST +
    ALT_SDMMC_CLKENA_OFST,
    ALT_SDMMC_CLKENA_CCLK_EN_CLR_MSK
    );

  // 3. Reset the clksrc register to 0.
  MmioAnd32 (
    ALT_SDMMC_OFST +
    ALT_SDMMC_CLKSRC_OFST,
    ALT_SDMMC_CLKSRC_CLK_SRC_CLR_MSK
    );

  // 4. Set the following bits in the cmd register to 1:
  //    a) update_clk_regs_only — Specifies the update clocks command
  //    b) wait_prvdata_complete — Wait until any ongoing data transfer is complete
  //    c) start_cmd — Initiates the command
  Data32 = ALT_SDMMC_CMD_UPDATE_CLK_REGS_ONLY_SET (ALT_SDMMC_CMD_UPDATE_CLK_REGS_ONLY_E_UPDATCLKREG) |
           ALT_SDMMC_CMD_WAIT_PRVDATA_COMPLETE_SET(ALT_SDMMC_CMD_WAIT_PRVDATA_COMPLETE_E_WAIT) |
           ALT_SDMMC_CMD_START_CMD_SET    ((UINT32)ALT_SDMMC_CMD_START_CMD_E_START);
  MmioWrite32 (
    ALT_SDMMC_OFST +
    ALT_SDMMC_CMD_OFST,
    Data32
    );

  // 5. Wait until the start_cmd and update_clk_regs_only bits change to 0. There is no interrupt when the
  //    clock modification completes. The controller does not set the command_done bit in the rintsts register
  //    upon command completion. The controller might signal a hardware lock error if it already has another
  //    command in the queue. In this case, return to Step 4.
  Count = 0;
  do {
    Data32 = MmioRead32 (ALT_SDMMC_OFST + ALT_SDMMC_CMD_OFST);
    if (ALT_SDMMC_CMD_START_CMD_GET(Data32) == ALT_SDMMC_CMD_START_CMD_E_NOSTART)
      break;
  } while (Count++ < SEND_CMD_WAIT_TIMEOUT);
  // Check if timeout
  if (Count >= SEND_CMD_WAIT_TIMEOUT) {
    if (PcdGet32 (PcdDebugMsg_SdMmc) != 0)
      SerialPortPrint ("\tSDMMC: %a () Timeout\r\n", __FUNCTION__);
    return EFI_TIMEOUT;
  }

  // 6. Reset the sdmmcclken enable bit to 0 in the enable register of the clock manager peripheral PLL group (perpllgrp).
  MmioAnd32 (
    ALT_CLKMGR_PERPLL_OFST +
    ALT_CLKMGR_PERPLL_EN_OFST,
    ALT_CLKMGR_PERPLL_EN_SDMMCCLKEN_CLR_MSK
    );

  // 7. In the control register (ctrl) of the SDMMC controller group (sdmmcgrp) in the system manager, set
  //    the drive clock phase shift select (drvsel) and sample clock phase shift select (smplsel) bits to specify
  //    the required phase shift value.

  if (PcdGet32 (PcdSdmmcSweepAllDrvselAndSmplselValues) != 1)
  {
    // Default value need to be consistent with value found in soc_system.dts
    //  --> altr,dw-mshc-sdr-timing = <0(smplsel) 3(drvsel)>;
    smplsel = PcdGet32 (PcdSdmmcSmplSel);
    drvsel = PcdGet32 (PcdSdmmcDrvSel);
    MmioAndThenOr32 (
      ALT_SYSMGR_OFST +
      ALT_SYSMGR_SDMMC_OFST,
      ALT_SYSMGR_SDMMC_SMPLSEL_CLR_MSK &
      ALT_SYSMGR_SDMMC_DRVSEL_CLR_MSK,
      ALT_SYSMGR_SDMMC_SMPLSEL_SET(smplsel) |
      ALT_SYSMGR_SDMMC_DRVSEL_SET (drvsel)
      );
  }

  // 8. Set the sdmmc_clk_enable bit in the Enable register of the clock manager perpllgrp group to 1.
  MmioOr32 (
    ALT_CLKMGR_PERPLL_OFST +
    ALT_CLKMGR_PERPLL_EN_OFST,
    ALT_CLKMGR_PERPLL_EN_SDMMCCLKEN_SET_MSK
    );

  // 9. Set the clkdiv register of the controller to the correct divider value for the required clock frequency.
  // Divides Clock sdmmc_clk (12.5 MHz max, See Table A-8: SD/MMC Controller CLKSEL Pin Settings)
  MmioAndThenOr32 (
    ALT_SDMMC_OFST +
    ALT_SDMMC_CLKDIV_OFST,
    ALT_SDMMC_CLKDIV_CLK_DIVR0_CLR_MSK,
    ALT_SDMMC_CLKDIV_CLK_DIVR0_SET(clkdiv)
    );

  //10. Set the cclk_enable bit of the clkena register to 1, to enable the card clock generation.
  //    You can also use the clkena register to enable low-power mode, which automatically stops the
  //    sdmmc_cclk_out clock when the card is idle for more than eight clock cycles.
  MmioOr32 (
    ALT_SDMMC_OFST +
    ALT_SDMMC_CLKENA_OFST,
    ALT_SDMMC_CLKENA_CCLK_EN_E_END
    );

  //11. Set the following bits in the cmd register to 1:
  //    a) update_clk_regs_only — Specifies the update clocks command
  //    b) wait_prvdata_complete — Wait until any ongoing data transfer is complete
  //    c) start_cmd — Initiates the command
  Data32 = ALT_SDMMC_CMD_UPDATE_CLK_REGS_ONLY_SET (ALT_SDMMC_CMD_UPDATE_CLK_REGS_ONLY_E_UPDATCLKREG) |
           ALT_SDMMC_CMD_WAIT_PRVDATA_COMPLETE_SET(ALT_SDMMC_CMD_WAIT_PRVDATA_COMPLETE_E_WAIT) |
           ALT_SDMMC_CMD_START_CMD_SET            ((UINT32)ALT_SDMMC_CMD_START_CMD_E_START);
  MmioWrite32 (
    ALT_SDMMC_OFST +
    ALT_SDMMC_CMD_OFST,
    Data32
    );

  return EFI_SUCCESS;
}


EFI_STATUS
EFIAPI
InitBeforeEnumerateCardStack(
  VOID
  )
{
  EFI_STATUS Status;
  //
  // In general the card connected to the controller can be an MMC, CE-ATA, SD or SDIO card.
  //
  // However most Altera SoC development kit boards are build with MicroSD card slot
  // which limit the posibility of card type other than MicroSD card.
  //
  // To odentify the Connected Card Type, the following tasks need to be performs:
  // In high level details:
  // - Discovers the connected card
  // - Sets the relative Card Address Register (RCA) in the connected card
  // - Reads the card specific information
  // - Stores the card specific information locally
  //
  // Steps:
  // 1. Set the card clock source frequency to the frequency of identification clock rate, 400 KHz.
  Status = ChangeCardClockFrequency (ALT_SDMMC_CLKDIV_VALUE_FOR_ID_MODE);
  if (Status != EFI_SUCCESS) return Status;

  // 2. Reset the card width 1 or 4 bit (card_width2) and card width 8 bit (card_width1) fields
  //  in the ctype register to 0.
  MmioWrite32 (
    ALT_SDMMC_OFST +
    ALT_SDMMC_CTYPE_OFST,
    0
    );
  return EFI_SUCCESS;
}


EFI_STATUS
EFIAPI
ChangeDataBusMode (
  VOID
  )
{
  EFI_STATUS Status;

  if (PcdGet32 (PcdSdmmcSupport4BitMode) != 0) //#ifdef USE_4_BIT_MODE
  {
    if ((mRCA != 0xFFFF) &&
        (mCMD7_detected == TRUE) &&
        (mSetBitMode_started == FALSE))
    {
      if (PcdGet32 (PcdDebugMsg_SdMmc) != 0)
        SerialPortPrint ("\tSDMMC: %a ( RCA = 0x%08x )\r\n",__FUNCTION__, mRCA);
      mSetBitMode_started = TRUE;
      // Try sending CMD55/ACMD6 (SET_BUS_WIDTH) = 4-bit mode
      // Send CMD55 to tell that the next command is an application specific command
      Status = SendCommand (MMC_CMD55, mRCA << mRCA_SHIFT_OFFSET);
      if (!EFI_ERROR (Status)) {
        // Set card to 4-bit data bus mode
        Status = SendCommand (mACMD6, 0x2);
        if (!EFI_ERROR (Status)) {
          // Change host controler ctype to 4-bit mode
          MmioAndThenOr32 (
            ALT_SDMMC_OFST +
            ALT_SDMMC_CTYPE_OFST,
            ALT_SDMMC_CTYPE_CARD_WIDTH2_CLR_MSK &
            ALT_SDMMC_CTYPE_CARD_WIDTH1_CLR_MSK,
            ALT_SDMMC_CTYPE_CARD_WIDTH2_SET(ALT_SDMMC_CTYPE_CARD_WIDTH2_E_MOD4BIT) |
            ALT_SDMMC_CTYPE_CARD_WIDTH1_SET(ALT_SDMMC_CTYPE_CARD_WIDTH1_E_NON8BIT)
            );
          if (PcdGet32 (PcdDebugMsg_SdMmc) != 0)
            SerialPortPrint ("\tSDMMC: Switch to 4-bit mode\r\n");
        } else {
          // ACMD6 command can only be send in transfer state, eg. after CMD7 is send.
          if (PcdGet32 (PcdDebugMsg_SdMmc) != 0)
            SerialPortPrint ("\tSDMMC unable to switch to 4-bit mode.\r\n");
        }
      }
    }
  }
  return EFI_SUCCESS;
}


EFI_STATUS
EFIAPI
InitAfterEnumerateCardStack(
  VOID
  )
{
  // Change the clock rate from card identification frequency, to data mode clock rate.
  ChangeCardClockFrequency (ALT_SDMMC_CLKDIV_VALUE_FOR_DATA_MODE);
  return EFI_SUCCESS;
}


EFI_STATUS
EFIAPI
InitCommonRegisters(
  VOID
  )
{
  // Set Response timeout value.
  // Set Data Read Timeout value which is also used for Data Starvation by Host timeout.
  // Value is in number of card output clocks sdmmc_cclk_out of selected card.
  MmioAndThenOr32 (
    ALT_SDMMC_OFST +
    ALT_SDMMC_TMOUT_OFST,
    ALT_SDMMC_TMOUT_RESPONSE_TMO_CLR_MSK &
    ALT_SDMMC_TMOUT_DATA_TMO_CLR_MSK,
    ALT_SDMMC_TMOUT_RESPONSE_TMO_SET(RESPONSE_TIMEOUT_VALUE) |
    ALT_SDMMC_TMOUT_DATA_TMO_SET((UINT32)DATA_TIMEOUT_VALUE)
    );
  return EFI_SUCCESS;
}


EFI_STATUS
EFIAPI
SendCommand (
  IN MMC_CMD MmcCmd,
  IN UINT32  CmdArgument
  )
{
  UINTN  Count;
  UINT32 cmd_index;
  UINT32 Data32;

  if ( IgnoreCommand (MmcCmd) )
    return EFI_SUCCESS;

  // 1. Construct Command Register Settings
  cmd_index = ( MMC_GET_INDX (MmcCmd) & ALT_SDMMC_CMD_CMD_INDEX_SET_MSK );
  Data32 = ALT_SDMMC_CMD_START_CMD_SET            ((UINT32)ALT_SDMMC_CMD_START_CMD_E_START) |
           ALT_SDMMC_CMD_USE_HOLD_REG_SET         (ALT_SDMMC_CMD_USE_HOLD_REG_E_NOBYPASS) |
           ALT_SDMMC_CMD_UPDATE_CLK_REGS_ONLY_SET (ALT_SDMMC_CMD_UPDATE_CLK_REGS_ONLY_E_NORMCMD) |
           ALT_SDMMC_CMD_VOLT_SWITCH_SET          (ALT_SDMMC_CMD_VOLT_SWITCH_E_NOVOLTSW) |
           ALT_SDMMC_CMD_BOOT_MOD_SET             (ALT_SDMMC_CMD_BOOT_MOD_E_MANDATORY) |
           ALT_SDMMC_CMD_DIS_BOOT_SET             (ALT_SDMMC_CMD_DIS_BOOT_E_NOTERMBOOT) |
           ALT_SDMMC_CMD_EXPECT_BOOT_ACK_SET      (ALT_SDMMC_CMD_EXPECT_BOOT_ACK_E_NOBOOTACK) |
           ALT_SDMMC_CMD_EN_BOOT_SET              (ALT_SDMMC_CMD_EN_BOOT_E_DISD) |
           ALT_SDMMC_CMD_CCS_EXPECTED_SET         (ALT_SDMMC_CMD_CCS_EXPECTED_E_DISD) |
           ALT_SDMMC_CMD_RD_CEATA_DEVICE_SET      (ALT_SDMMC_CMD_RD_CEATA_DEVICE_E_NORD) |
           ALT_SDMMC_CMD_TFR_MOD_SET              (ALT_SDMMC_CMD_TFR_MOD_E_BLK) |
           ALT_SDMMC_CMD_CHECK_RESPONSE_CRC_SET   (ALT_SDMMC_CMD_CHECK_RESPONSE_CRC_E_NOCHK) |
           ALT_SDMMC_CMD_CARD_NUMBER_SET          (0) |
           cmd_index;

  // Is CMD 0 (a.k.a. GO_IDLE_STATE) ?
  if (MmcCmd == MMC_CMD0) {
    // Initialize clocks before sending command to card
    Data32 |= ALT_SDMMC_CMD_SEND_INITIALIZATION_SET(ALT_SDMMC_CMD_SEND_INITIALIZATION_E_INIT);
  } else {
    Data32 |= ALT_SDMMC_CMD_SEND_INITIALIZATION_SET(ALT_SDMMC_CMD_SEND_INITIALIZATION_E_NOINIT);
  }

  // Is Read/Write Data Transfer Command?
  if ((MmcCmd == MMC_CMD24) ||
      (MMC_GET_INDX(MmcCmd) == 25)) {
    // Write Data Command
    Data32 |= ALT_SDMMC_CMD_DATA_EXPECTED_SET (ALT_SDMMC_CMD_DATA_EXPECTED_E_DATAXFEREXP);
    Data32 |= ALT_SDMMC_CMD_RD_WR_SET         (ALT_SDMMC_CMD_RD_WR_E_WR);
    Data32 |= ALT_SDMMC_CMD_SEND_AUTO_STOP_SET(ALT_SDMMC_CMD_SEND_AUTO_STOP_E_NOSEND);
  } else if ((MmcCmd == MMC_CMD17) || (MmcCmd == MMC_CMD18) || (MmcCmd == MMC_CMD8)) {
    // Read Data Command
    Data32 |= ALT_SDMMC_CMD_DATA_EXPECTED_SET (ALT_SDMMC_CMD_DATA_EXPECTED_E_DATAXFEREXP);
    Data32 |= ALT_SDMMC_CMD_RD_WR_SET         (ALT_SDMMC_CMD_RD_WR_E_RD);
    // Do Multi Block Read?
    // Work together with MmcIoBlocks() in PEI phase's MmcBlockIo.c
    // which send CMD18 followed by a 4 KB read
    // 4K size is also the FIFO size and also the minimum FAT32 cluster size
    if (MmcCmd == MMC_CMD18) {
      Data32 |= ALT_SDMMC_CMD_SEND_AUTO_STOP_SET(ALT_SDMMC_CMD_SEND_AUTO_STOP_E_SEND);
      // Set 4 KB BYTCNT
      MmioWrite32 (
        ALT_SDMMC_OFST +
        ALT_SDMMC_BYTCNT_OFST,
        4096
        );
    } else {
      Data32 |= ALT_SDMMC_CMD_SEND_AUTO_STOP_SET(ALT_SDMMC_CMD_SEND_AUTO_STOP_E_NOSEND);
      // Default 512 BYTCNT
      MmioWrite32 (
        ALT_SDMMC_OFST +
        ALT_SDMMC_BYTCNT_OFST,
        ALT_SDMMC_BYTCNT_BYTE_COUNT_RESET
        );
    }
  } else {
    // Non-Data Transfer Command
    Data32 |= ALT_SDMMC_CMD_DATA_EXPECTED_SET (ALT_SDMMC_CMD_DATA_EXPECTED_E_NODATXFEREXP);
    Data32 |= ALT_SDMMC_CMD_RD_WR_SET         (ALT_SDMMC_CMD_RD_WR_E_RD);
    Data32 |= ALT_SDMMC_CMD_SEND_AUTO_STOP_SET(ALT_SDMMC_CMD_SEND_AUTO_STOP_E_NOSEND);

    // Print debug message for Non-Data Transfer Command
    if ((MmcCmd & 0x3F) != 13)
    {
      if (PcdGet32 (PcdDebugMsg_SdMmc) >= 2)
        SerialPortPrint ("\tSDMMC: Send CMD = %d ARG = 0x%x\r\n",(MmcCmd & 0x3F), CmdArgument);
    }
  }

  // Forcing the card to stop transmission?
  if (MmcCmd == MMC_CMD12) {
    Data32 |= ALT_SDMMC_CMD_STOP_ABT_CMD_SET(ALT_SDMMC_CMD_STOP_ABT_CMD_E_STOPABRT) |
              ALT_SDMMC_CMD_WAIT_PRVDATA_COMPLETE_SET(ALT_SDMMC_CMD_WAIT_PRVDATA_COMPLETE_E_NOWAIT);

  } else {
    Data32 |= ALT_SDMMC_CMD_STOP_ABT_CMD_SET(ALT_SDMMC_CMD_STOP_ABT_CMD_E_NOSTOPABRT) |
              ALT_SDMMC_CMD_WAIT_PRVDATA_COMPLETE_SET(ALT_SDMMC_CMD_WAIT_PRVDATA_COMPLETE_E_WAIT);
  }

  if (MmcCmd & MMC_CMD_WAIT_RESPONSE) {
    // Response expected from card
    Data32 |= ALT_SDMMC_CMD_RESPONSE_EXPECT_SET(1); // Not using SoCal constant yet due to case:243277
  } else {
    // No response expected from card
    Data32 |= ALT_SDMMC_CMD_RESPONSE_EXPECT_SET(0);
  }

  if (MmcCmd & MMC_CMD_LONG_RESPONSE) {
    // Long response expected from card
    Data32 |= ALT_SDMMC_CMD_RESPONSE_LEN_SET(ALT_SDMMC_CMD_RESPONSE_LEN_E_LONG);
  } else {
    // Short response expected from card
    Data32 |= ALT_SDMMC_CMD_RESPONSE_LEN_SET(ALT_SDMMC_CMD_RESPONSE_LEN_E_SHORT);
  }

  // 2. Clear any pending interrupts
  MmioWrite32 (
    ALT_SDMMC_OFST +
    ALT_SDMMC_RINTSTS_OFST,
    0xFFFFFFFF
    );

  // 3. Write the cmdarg register and cmd register
  MmioWrite32 (
    ALT_SDMMC_OFST +
    ALT_SDMMC_CMDARG_OFST,
    CmdArgument
    );
  MmioWrite32 (
    ALT_SDMMC_OFST +
    ALT_SDMMC_CMD_OFST,
    Data32
    );

  // 4. Wait for the start_cmd bit changes to 0 or timeout.
  Count = 0;
  do {
    Data32 = MmioRead32 (ALT_SDMMC_OFST + ALT_SDMMC_CMD_OFST);
    if (ALT_SDMMC_CMD_START_CMD_GET(Data32) == ALT_SDMMC_CMD_START_CMD_E_NOSTART)
      break;
  } while (Count++ < SEND_CMD_WAIT_TIMEOUT);

  // Check if timeout
  if (Count >= SEND_CMD_WAIT_TIMEOUT) {
    return EFI_TIMEOUT;
  }

  // 5. Wait for command execution to complete by polling the command_done bit in the rintsts register to 1.
  Count = 0;
  do {
    Data32 = MmioRead32 (ALT_SDMMC_OFST + ALT_SDMMC_RINTSTS_OFST);
    // Check if Hardware locked error when the controller cannot load a command issued by software.
    if (ALT_SDMMC_RINTSTS_HLE_GET(Data32) == ALT_SDMMC_RINTSTS_HLE_E_ACT) {
      // The command buffer already contains a command, and the new command is discarded,
      // Software can try to reload the command later.
      MmioWrite32 (
        ALT_SDMMC_OFST +
        ALT_SDMMC_RINTSTS_OFST,
        ALT_SDMMC_RINTSTS_HLE_E_ACT
        );
      return EFI_LOAD_ERROR;
    }
    // Is command_done bit set?
    if (ALT_SDMMC_RINTSTS_CMD_GET(Data32) == ALT_SDMMC_RINTSTS_CMD_E_ACT)
      break;
  } while (Count++ < SEND_CMD_WAIT_TIMEOUT);

  // Check if timeout
  if (Count >= SEND_CMD_WAIT_TIMEOUT) {
    return EFI_TIMEOUT;
  }

  // 6. Check if the response is valid
  // Clear command done bit and interrupts that we are going to check
  MmioWrite32 (
    ALT_SDMMC_OFST +
    ALT_SDMMC_RINTSTS_OFST,
    ALT_SDMMC_RINTSTS_CMD_E_ACT  |
    ALT_SDMMC_RINTSTS_RCRC_E_ACT |
    ALT_SDMMC_RINTSTS_DCRC_E_ACT |
    ALT_SDMMC_RINTSTS_BAR_E_ACT  |
    ALT_SDMMC_RINTSTS_BDS_E_ACT  |
    ALT_SDMMC_RINTSTS_HTO_E_ACT  |
    ALT_SDMMC_RINTSTS_FRUN_E_ACT |
    ALT_SDMMC_RINTSTS_SBE_E_ACT  |
    ALT_SDMMC_RINTSTS_EBE_E_ACT  |
    ALT_SDMMC_RINTSTS_RE_E_ACT
    );

  if ((ALT_SDMMC_RINTSTS_RCRC_GET(Data32) == ALT_SDMMC_RINTSTS_RCRC_E_ACT) ||
      (ALT_SDMMC_RINTSTS_DCRC_GET(Data32) == ALT_SDMMC_RINTSTS_DCRC_E_ACT)) {
    return EFI_CRC_ERROR;
  }
  if ((ALT_SDMMC_RINTSTS_BAR_GET(Data32) == ALT_SDMMC_RINTSTS_BAR_E_ACT) ||
      (ALT_SDMMC_RINTSTS_BDS_GET(Data32) == ALT_SDMMC_RINTSTS_BDS_E_ACT) ||
      (ALT_SDMMC_RINTSTS_HTO_GET(Data32) == ALT_SDMMC_RINTSTS_HTO_E_ACT)) {
    if (PcdGet32 (PcdDebugMsg_SdMmc) >= 2)
      SerialPortPrint ("\tTimeout\n\r");
    return EFI_TIMEOUT;
  }
  if ((ALT_SDMMC_RINTSTS_FRUN_GET(Data32) == ALT_SDMMC_RINTSTS_FRUN_E_ACT) ||
      (ALT_SDMMC_RINTSTS_SBE_GET(Data32) == ALT_SDMMC_RINTSTS_SBE_E_ACT) ||
      (ALT_SDMMC_RINTSTS_EBE_GET(Data32) == ALT_SDMMC_RINTSTS_EBE_E_ACT) ||
      (ALT_SDMMC_RINTSTS_RE_GET(Data32) == ALT_SDMMC_RINTSTS_RE_E_ACT)) {
    return EFI_DEVICE_ERROR;
  }

  return EFI_SUCCESS;
}


BOOLEAN
EFIAPI
IgnoreCommand (
  IN MMC_CMD MmcCmd
  )
{
  mPrevMmcCmd = MmcCmd;

  switch(MmcCmd) {
//#ifdef USE_4_BIT_MODE
    case MMC_CMD7:
      mCMD7_detected = TRUE;
      return FALSE;
//#endif
/*
eg. to ignore command
    case MMC_CMD13:
      return TRUE;
*/
    default:
      return FALSE;
  }
}


EFI_STATUS
EFIAPI
ReceiveCommandResponse(
  IN MMC_RESPONSE_TYPE   Type,
  IN UINT32*             Buffer
  )
{
  Buffer[0] = MmioRead32 (ALT_SDMMC_OFST + ALT_SDMMC_RESP0_OFST);
  Buffer[1] = MmioRead32 (ALT_SDMMC_OFST + ALT_SDMMC_RESP1_OFST);
  Buffer[2] = MmioRead32 (ALT_SDMMC_OFST + ALT_SDMMC_RESP2_OFST);
  Buffer[3] = MmioRead32 (ALT_SDMMC_OFST + ALT_SDMMC_RESP3_OFST);
  // Print debug message for Non-Data Transfer Command
  if ((mPrevMmcCmd & 0x3F) != 13)
  {
    if (PcdGet32 (PcdDebugMsg_SdMmc) >= 2)
      SerialPortPrint ("\tSDMMC: Received 0x%08x 0x%08x 0x%08x 0x%08x\n\r", Buffer[0], Buffer[1], Buffer[2], Buffer[3]);
  }
//#ifdef USE_4_BIT_MODE
  if ((mPrevMmcCmd==MMC_CMD3) && (Type == MMC_RESPONSE_TYPE_RCA))
  {
    mRCA = Buffer[0] >> mRCA_SHIFT_OFFSET;
  }
//#endif
  return EFI_SUCCESS;
}


EFI_STATUS
EFIAPI
ReadFifoData (
  IN  UINTN     Length,
  OUT UINT32*   Buffer
  )
{
  EFI_STATUS Status;
  UINTN*     DataBuffer = Buffer;
  UINTN      BufSize = Length / 4;
  UINTN      FifoCount = 0;
  UINTN      Count = 0;
  UINT32     Data32;

  while (BufSize)
  {
    Data32 = MmioRead32 (ALT_SDMMC_OFST + ALT_SDMMC_RINTSTS_OFST);
    if ((Data32 & ALT_SDMMC_RINTSTS_RXDR_SET_MSK) ||
        (Data32 & ALT_SDMMC_RINTSTS_DTO_SET_MSK))
    {
      FifoCount = ALT_SDMMC_STAT_FIFO_COUNT_GET(MmioRead32 (ALT_SDMMC_OFST + ALT_SDMMC_STAT_OFST));
      for (Count = 0; Count < FifoCount; Count++)
      {
        *DataBuffer++ = MmioRead32 (ALT_SDMMC_OFST + ALT_SDMMC_DATA_OFST);
      }
      MmioWrite32(
        ALT_SDMMC_OFST +
        ALT_SDMMC_RINTSTS_OFST,
        ALT_SDMMC_RINTSTS_RXDR_SET_MSK |
        ALT_SDMMC_RINTSTS_DTO_SET_MSK
        );
      BufSize -= FifoCount;
    }
    else if(Data32 &
           (ALT_SDMMC_RINTSTS_DCRC_E_ACT |
            ALT_SDMMC_RINTSTS_BDS_E_ACT  |
            ALT_SDMMC_RINTSTS_HTO_E_ACT  |
            ALT_SDMMC_RINTSTS_FRUN_E_ACT))
    {
      return EFI_DEVICE_ERROR;
    }
  }

  if(BufSize==0)
  {
      Status = EFI_SUCCESS;
  }
  else
  {
      Status = EFI_BAD_BUFFER_SIZE;
  }
  return Status;
}


EFI_STATUS
EFIAPI
WriteFifoData (
  IN UINTN      Length,
  IN UINT32*    Buffer
  )
{
  UINTN *DataBuffer = Buffer;
  UINTN BufSize = Length / 4;
  UINTN Count = 0;

  for (Count = 0; Count < BufSize; Count++)
  {
    MmioWrite32 (
      ALT_SDMMC_OFST +
      ALT_SDMMC_DATA_OFST,
      *DataBuffer++
      );
  }
  return EFI_SUCCESS;
}


EFI_STATUS
EFIAPI
InitInternalDMAC(
  VOID
  )
{
  // Initalize Internal DMA Controller

  // Set FIFOTH
  MmioAndThenOr32 (
    ALT_SDMMC_OFST +
    ALT_SDMMC_FIFOTH_OFST,
    ALT_SDMMC_FIFOTH_TX_WMARK_CLR_MSK  &
    ALT_SDMMC_FIFOTH_RX_WMARK_CLR_MSK,
    ALT_SDMMC_FIFOTH_TX_WMARK_SET(512) |
    ALT_SDMMC_FIFOTH_RX_WMARK_SET(511)
    );

  // Soft Reset DMA
  MmioOr32 (
    ALT_SDMMC_OFST +
    ALT_SDMMC_BMOD_OFST,
    ALT_SDMMC_BMOD_SWR_SET_MSK
    );

  // Disable DMAC Interrupt
  MmioAndThenOr32 (
    ALT_SDMMC_OFST +
    ALT_SDMMC_IDINTEN_OFST,
    ALT_SDMMC_IDINTEN_AI_CLR_MSK  &
    ALT_SDMMC_IDINTEN_NI_CLR_MSK  &
    ALT_SDMMC_IDINTEN_CES_CLR_MSK &
    ALT_SDMMC_IDINTEN_DU_CLR_MSK  &
    ALT_SDMMC_IDINTEN_FBE_CLR_MSK &
    ALT_SDMMC_IDINTEN_RI_CLR_MSK  &
    ALT_SDMMC_IDINTEN_TI_CLR_MSK,
    ALT_SDMMC_IDINTEN_AI_SET (ALT_SDMMC_IDINTEN_NI_E_DISD)  |
    ALT_SDMMC_IDINTEN_NI_SET (ALT_SDMMC_IDINTEN_AI_E_DISD)  |
    ALT_SDMMC_IDINTEN_CES_SET(ALT_SDMMC_IDINTEN_CES_E_DISD) |
    ALT_SDMMC_IDINTEN_DU_SET (ALT_SDMMC_IDINTEN_DU_E_DISD)  |
    ALT_SDMMC_IDINTEN_FBE_SET(ALT_SDMMC_IDINTEN_FBE_E_DISD) |
    ALT_SDMMC_IDINTEN_RI_SET (ALT_SDMMC_IDINTEN_RI_E_DISD)  |
    ALT_SDMMC_IDINTEN_TI_SET (ALT_SDMMC_IDINTEN_TI_E_DISD)
    );

  // Clear Interrupt status bits
  MmioOr32 (
    ALT_SDMMC_OFST +
    ALT_SDMMC_IDSTS_OFST,
    ALT_SDMMC_IDSTS_AIS_SET_MSK |
    ALT_SDMMC_IDSTS_NIS_SET_MSK |
    ALT_SDMMC_IDSTS_CES_SET_MSK |
    ALT_SDMMC_IDSTS_DU_SET_MSK  |
    ALT_SDMMC_IDSTS_FBE_SET_MSK |
    ALT_SDMMC_IDSTS_RI_SET_MSK  |
    ALT_SDMMC_IDSTS_TI_SET_MSK
    );

  // Initalize descriptor variable
  mDmaDes.des0 = IDMAC_DIC | IDMAC_FS | IDMAC_LD;
  mDmaDes.des1 = 0;
  mDmaDes.des2 = 0;
  mDmaDes.des3 = (UINT32)(IDMA_DES*)&mDmaDes;;

  // Set DMA Descriptor base address
  MmioWrite32 (
    ALT_SDMMC_OFST +
    ALT_SDMMC_DBADDR_OFST,
    (UINT32) &mDmaDes
    );

  if (PcdGet32 (PcdSdmmcBlockReadUseInternalDMA) != 0)
  {
    // Enable the DMA at SDMMC CTRL
    MmioAndThenOr32 (
      ALT_SDMMC_OFST +
      ALT_SDMMC_CTL_OFST,
      ALT_SDMMC_CTL_USE_INTERNAL_DMAC_CLR_MSK,
      ALT_SDMMC_CTL_USE_INTERNAL_DMAC_SET(ALT_SDMMC_CTL_USE_INTERNAL_DMAC_E_END)
      );

    // Enable the DMA at IDMAC
    MmioOr32 (
      ALT_SDMMC_OFST +
      ALT_SDMMC_BMOD_OFST,
      ALT_SDMMC_BMOD_DE_SET_MSK
      );
  }

  return EFI_SUCCESS;
}

EFI_STATUS
EFIAPI
ReadDmaData (
  IN  UINTN     Length,
  OUT UINT32*   Buffer
  )
{
  UINTN   WaitCount;
  UINT32  Des0_flags;
  UINT32  Des1_Bs1;
  UINT32  Des2_Bap1;
  UINT32  Des3_Next;

  // Construct Internal DMA Descriptor
  Des0_flags = (IDMAC_OWN | IDMAC_DIC | IDMAC_FS | IDMAC_LD);
  Des1_Bs1   = Length;
  Des2_Bap1  = (UINT32)(UINT32*)Buffer;
  Des3_Next  = (UINT32)(IDMA_DES*)&mDmaDes;

  mDmaDes.des0 = Des0_flags;
  mDmaDes.des1 = Des1_Bs1;
  mDmaDes.des2 = Des2_Bap1;
  mDmaDes.des3 = Des3_Next;

  // Clear Poll Demand Register
  MmioWrite32 (
    ALT_SDMMC_OFST +
    ALT_SDMMC_PLDMND_OFST,
    ALT_SDMMC_PLDMND_RESET
    );

  // Wait for DMA to finish transfering the data
  WaitCount = 0;
  do {
    if ((mDmaDes.des0 & IDMAC_OWN) == 0)
    {
      return EFI_SUCCESS;
    }
  } while ( (WaitCount++ < DATA_BUSY_TIMEOUT) );

  // Timeout
  if (PcdGet32 (PcdDebugMsg_SdMmc) != 0)
    SerialPortPrint ("\tSDMMC iDMA RX Timeout\n\r");
  return EFI_TIMEOUT;
}


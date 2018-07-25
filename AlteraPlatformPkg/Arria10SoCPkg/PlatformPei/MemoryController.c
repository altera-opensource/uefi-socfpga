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
#include <Library/TimerLib.h>
#include "Assert.h"
#include "MemoryController.h"

#if (FixedPcdGet32(PcdDebugMsg_MemoryController) == 0)
  #define ProgressPrint(FormatString, ...)    /* do nothing */
  #define InfoPrint(FormatString, ...)        /* do nothing */
#else
  #define ProgressPrint SerialPortPrint
  #define InfoPrint     SerialPortPrint
#endif

#define DDR_READ_LATENCY_DELAY 40

/// Definition for HMC and EMIF related register bit
#define ALT_FPGAMGR_GPO_31_HPS2EMIF_RST_OVR_ACT_LOW      BIT31
#define ALT_FPGAMGR_GPO_30_HPS2OCT_OCT_CAL_REQ_ACT_HIGH  BIT30
#define ALT_FPGAMGR_GPI_31_OCT2HPS_OCT_CAL_RDY_ACT_HIGH  BIT31
#define ALT_RSTMGR_HMCGPI_7_SEQ2CORE_OCT_REQ_ACT_HIGH    BIT7
#define ALT_RSTMGR_HMCGPO_7_CORE2SEQ_OCT_RDY_ACT_HIGH    BIT7
#define ALT_RSTMGR_HMCGPI_F_SEQ2CORE_MASK                0x0F
#define ALT_RSTMGR_HMCGPO_F_CORE2SEQ_INT_REQ             0x0F
#define ALT_RSTMGR_HMCGPI_3_SEQ2CORE_INT_RESP_BIT        BIT3
#define CLEAR_EMIF_DELAY                                 50000
#define CLEAR_EMIF_TIMEOUT                               0x100000
#define TIMEOUT_INT_RESP                                 10000

//
// Functions
//


EFI_STATUS
EFIAPI
InitHardMemoryController (
  IN  CONST VOID*  Fdt
  )
{
  EFI_STATUS      Status;

  ProgressPrint ("Initializing Hard Memory Controller\r\n");

  // De-assert the reset signal of DDR Scheduler in the NOC
  MmioAnd32 (ALT_RSTMGR_OFST +
             ALT_RSTMGR_BRGMODRST_OFST,
             ALT_RSTMGR_BRGMODRST_DDRSCH_CLR_MSK);

  // Memory Calibration
  Status = MemoryCalibration ();
  ASSERT_PLATFORM_INIT(!EFI_ERROR(Status));
  if (EFI_ERROR(Status)) {
    return Status;
  }

  // HMC Adaptor is a component between NOC and IO48_HMC which also do ECC checking.
  ConfigureHmcAdaptorRegisters ();

  // DDR Scheduler
  ConfigureDdrSchedulerControlRegisters ();

  return Status;
}


EFI_STATUS
EFIAPI
ClearEMIF (
  VOID
  )
{
  UINT32          Data32;
  UINTN           TimeOutCount;
  // Request HMC to Clear EMIF
  MmioWrite32 (ALT_RSTMGR_OFST +
               ALT_RSTMGR_HMCGPOUT_OFST,
               ALT_RSTMGR_HMCGPOUT_RESET);
  // Wait for Clear EMIF done.
  TimeOutCount = 0;
  do {
    Data32 = MmioRead32 (ALT_RSTMGR_OFST + ALT_RSTMGR_HMCGPIN_OFST);
    if ((Data32 & ALT_RSTMGR_HMCGPI_F_SEQ2CORE_MASK) == 0)
      break;
    MicroSecondDelay (CLEAR_EMIF_DELAY);
  } while (++TimeOutCount < CLEAR_EMIF_TIMEOUT);
  if (TimeOutCount >= CLEAR_EMIF_TIMEOUT) {
    return EFI_TIMEOUT;
  }
  return EFI_SUCCESS;
}


EFI_STATUS
EFIAPI
MemoryCalibration (
  VOID
  )
{
  EFI_STATUS      Status;
  UINT32          Data32;
  UINTN           RetryCount;
  UINTN           TimeOutCount;

  Status = EFI_DEVICE_ERROR;

  #define MAX_MEM_CAL_RETRY         3
  #define PRE_CALIBRATION_DELAY     0
  #define POST_CALIBRATION_DELAY    0
  #define TIMEOUT_EMIF_CALIBRATION  20000000

  MicroSecondDelay (PRE_CALIBRATION_DELAY);

  ProgressPrint ("\t HMC Calibrating Memory...\r\n");

  RetryCount = 0;
  do {

    if (RetryCount != 0) {
      ProgressPrint ("\t\t Retry DRAM calibration\r\n");
    }

    //
    // ES1 Silicon Specific Initialization Sequence
    //
    Data32 = MmioRead32 (ALT_SYSMGR_OFST + ALT_SYSMGR_SILICONID1_OFST);
    if (Data32 == ALT_SYSMGR_SILICONID1_ES1)
    {
      // case:248063 HPS/EMIF OCT Handshake
      #define RESET_DELAY_OCT                                  5000
      #define RESET_DELAY_EMIF                                 10000
      #define TIMEOUT_OCT                                      10000
      #define TIMEOUT_EMIF_HAND_SHAKE                          10000

      // 1. Assert EMIF Reset signal
      MmioAnd32 (ALT_FPGAMGR_OFST +
                 ALT_FPGAMGR_GPO_OFST,
               ~(ALT_FPGAMGR_GPO_31_HPS2EMIF_RST_OVR_ACT_LOW));
      MicroSecondDelay (RESET_DELAY_EMIF);

      // 2. Clear HPS to EMIF NIOS II communication register
      MmioWrite32 (ALT_RSTMGR_OFST +
                   ALT_RSTMGR_HMCGPOUT_OFST,
                   ALT_RSTMGR_HMCGPOUT_RESET);

      // 3. De-assert HPS to OCT FSM calibration request signal
      MmioAnd32 (ALT_FPGAMGR_OFST +
                 ALT_FPGAMGR_GPO_OFST,
               ~(ALT_FPGAMGR_GPO_30_HPS2OCT_OCT_CAL_REQ_ACT_HIGH));
      MicroSecondDelay (RESET_DELAY_OCT);

      // 4. De-assert EMIF Reset signal
      MmioOr32 (ALT_FPGAMGR_OFST +
                ALT_FPGAMGR_GPO_OFST,
                ALT_FPGAMGR_GPO_31_HPS2EMIF_RST_OVR_ACT_LOW);
      MicroSecondDelay (RESET_DELAY_EMIF);

      //-------------------------------------------------------------------

      // 5. Wait for OCT FSM to send HPS an OCT Ready = 1
      TimeOutCount = 0;
      do {
        Data32 = MmioRead32 (ALT_FPGAMGR_OFST + ALT_FPGAMGR_GPI_OFST);
        if (((Data32) & ALT_FPGAMGR_GPI_31_OCT2HPS_OCT_CAL_RDY_ACT_HIGH) != 0)
          break;
        MicroSecondDelay (1);
      } while (++TimeOutCount < TIMEOUT_OCT);
      if (TimeOutCount >= TIMEOUT_OCT) {
        ProgressPrint ("\t\t Timeout during HPS/EMIF/OCT Handshake - Step 5\r\n");
        // skip the current loop and start from the top again.
        continue;
      }

      //-------------------------------------------------------------------

      // 6. Inform EMIF NIOS II to get ready for OCT calibration
      MmioOr32 (ALT_RSTMGR_OFST +
                ALT_RSTMGR_HMCGPOUT_OFST,
                ALT_RSTMGR_HMCGPO_7_CORE2SEQ_OCT_RDY_ACT_HIGH);

      // 7. Wait for EMIF NIOS II to send HPS an OCT request = 1
      TimeOutCount = 0;
      do {
        Data32 = MmioRead32 (ALT_RSTMGR_OFST + ALT_RSTMGR_HMCGPIN_OFST);
        if (((Data32) & ALT_RSTMGR_HMCGPI_7_SEQ2CORE_OCT_REQ_ACT_HIGH) != 0)
          break;
        MicroSecondDelay (1);
      } while (++TimeOutCount < TIMEOUT_EMIF_HAND_SHAKE);
      if (TimeOutCount >= TIMEOUT_EMIF_HAND_SHAKE) {
        ProgressPrint ("\t\t Timeout during HPS/EMIF/OCT Handshake - Step 7\r\n");
        // skip the current loop and start from the top again.
        continue;
      }

      // 8. Acknowledge to EMIF NIOS II on receive of the OCT request signal
      MmioAnd32 (ALT_RSTMGR_OFST +
                 ALT_RSTMGR_HMCGPOUT_OFST,
               ~(ALT_RSTMGR_HMCGPO_7_CORE2SEQ_OCT_RDY_ACT_HIGH));

      // 9. Wait for EMIF NIOS II to response with OCT request = 0
      TimeOutCount = 0;
      do {
        Data32 = MmioRead32 (ALT_RSTMGR_OFST + ALT_RSTMGR_HMCGPIN_OFST);
        if (((Data32) & ALT_RSTMGR_HMCGPI_7_SEQ2CORE_OCT_REQ_ACT_HIGH) == 0)
          break;
        MicroSecondDelay (1);
      } while (++TimeOutCount < TIMEOUT_EMIF_HAND_SHAKE);
      if (TimeOutCount >= TIMEOUT_EMIF_HAND_SHAKE) {
        ProgressPrint ("\t\t Timeout during HPS/EMIF/OCT Handshake - Step 9\r\n");
        // skip the current loop and start from the top again.
        continue;
      }

      //-------------------------------------------------------------------

      // 10. Assert HPS to OCT FSM calibration request signal
      MmioOr32 (ALT_FPGAMGR_OFST +
                ALT_FPGAMGR_GPO_OFST,
                ALT_FPGAMGR_GPO_30_HPS2OCT_OCT_CAL_REQ_ACT_HIGH);
      MicroSecondDelay (RESET_DELAY_OCT);

      // 11. Wait for OCT FSM's OCT Ready = 0
      TimeOutCount = 0;
      do {
        Data32 = MmioRead32 (ALT_FPGAMGR_OFST + ALT_FPGAMGR_GPI_OFST);
        if (((Data32) & ALT_FPGAMGR_GPI_31_OCT2HPS_OCT_CAL_RDY_ACT_HIGH) == 0)
          break;
        MicroSecondDelay (1);
      } while (++TimeOutCount < TIMEOUT_OCT);
      if (TimeOutCount >= TIMEOUT_OCT) {
        ProgressPrint ("\t\t Timeout during HPS/EMIF/OCT Handshake - Step 11\r\n");
        // skip the current loop and start from the top again.
        continue;
      }

      // 12. De-assert HPS to OCT FSM calibration request signal
      MmioAnd32 (ALT_FPGAMGR_OFST +
                 ALT_FPGAMGR_GPO_OFST,
               ~(ALT_FPGAMGR_GPO_30_HPS2OCT_OCT_CAL_REQ_ACT_HIGH));
      MicroSecondDelay (RESET_DELAY_OCT);

      // 13. Wait for OCT FSM to send HPS an OCT Ready = 1
      TimeOutCount = 0;
      do {
        Data32 = MmioRead32 (ALT_FPGAMGR_OFST + ALT_FPGAMGR_GPI_OFST);
        if (((Data32) & ALT_FPGAMGR_GPI_31_OCT2HPS_OCT_CAL_RDY_ACT_HIGH) != 0)
          break;
        MicroSecondDelay (1);
      } while (++TimeOutCount < TIMEOUT_OCT);
      if (TimeOutCount >= TIMEOUT_OCT) {
        ProgressPrint ("\t\t Timeout during HPS/EMIF/OCT Handshake - Step 13\r\n");
        // skip the current loop and start from the top again.
        continue;
      }

      //-------------------------------------------------------------------

      // 14. Allow EMIF NIOS II to start OCT calibration
      MmioOr32 (ALT_RSTMGR_OFST +
                ALT_RSTMGR_HMCGPOUT_OFST,
                ALT_RSTMGR_HMCGPO_7_CORE2SEQ_OCT_RDY_ACT_HIGH);

      MicroSecondDelay (RESET_DELAY_EMIF);

    }

    // Common Code - Applicable to all Silicon Revision
    //--------------------------------------------------------
    // Wait for EMIF NIOS II to set calibration success bit.
    //--------------------------------------------------------
    TimeOutCount = 0;
    do {
      Data32 = MmioRead32 (ALT_ECC_HMC_OCP_OFST + ALT_ECC_HMC_OCP_DDRCALSTAT_OFST);
      if (ALT_ECC_HMC_OCP_DDRCALSTAT_CAL_GET(Data32) == 1)
        break;
      MicroSecondDelay (1);
    } while (++TimeOutCount < TIMEOUT_EMIF_CALIBRATION);
    if (TimeOutCount >= TIMEOUT_EMIF_CALIBRATION) {
      ProgressPrint ("\t\t Timeout during DDR calibration\r\n");
      // skip the current loop and start from the top again.
      continue;
    }
    ProgressPrint ("\t\t DRAM calibration successful.\r\n");
    ProgressPrint ("\t\t Calibration wait time = ~%d us\r\n", TimeOutCount);

    //
    // ES2 & Production Silicon Specific Initialization Sequence
    //
    Data32 = MmioRead32 (ALT_SYSMGR_OFST + ALT_SYSMGR_SILICONID1_OFST);
    if (Data32 != ALT_SYSMGR_SILICONID1_ES1)
    {
      // Step 1 - if SEQ2CORE (HMCGPI) register not already zero then Clear EMIF
      Data32 = MmioRead32 (ALT_RSTMGR_OFST + ALT_RSTMGR_HMCGPIN_OFST);
      if ((Data32 & ALT_RSTMGR_HMCGPI_F_SEQ2CORE_MASK) &&
           EFI_ERROR(ClearEMIF())) {
        ProgressPrint ("\t\t Timeout during Clear EMIF #1\r\n");
      }

      // Step 2 -  Send EMIF interrupt request
      MmioWrite32 (ALT_RSTMGR_OFST +
                   ALT_RSTMGR_HMCGPOUT_OFST,
                   ALT_RSTMGR_HMCGPO_F_CORE2SEQ_INT_REQ);

      // Step 3 -  Wait for Interrupt Response Bits = 0
      TimeOutCount = 0;
      do {
        Data32 = MmioRead32 (ALT_RSTMGR_OFST + ALT_RSTMGR_HMCGPIN_OFST);
        if ((Data32 & ALT_RSTMGR_HMCGPI_3_SEQ2CORE_INT_RESP_BIT) == 0)
          break;
        MicroSecondDelay (1);
      } while (++TimeOutCount < TIMEOUT_INT_RESP);
      if (TimeOutCount >= TIMEOUT_INT_RESP) {
        InfoPrint ("\t\t Timeout EMIF INT_RESP\r\n");
      }

      // Step 4 - Clear EMIF
      if (EFI_ERROR(ClearEMIF())) {
        ProgressPrint ("\t\t Timeout during Clear EMIF #2\r\n");
      }
    }

    // If it reach here, calibration has success.
    Status = EFI_SUCCESS;
    // Stop the while loop, because we have success.
    break;

  } while (++RetryCount < MAX_MEM_CAL_RETRY);

  if (EFI_ERROR(Status)) {
    ProgressPrint ("\t\t DRAM calibration fail.\r\n");
  }

  MicroSecondDelay (POST_CALIBRATION_DELAY);

  return Status;
}


VOID
EFIAPI
ConfigureHmcAdaptorRegisters (
  VOID
  )
{
  UINT32          Data32;
  UINT32          DramIoWidth;

  ProgressPrint ("\t Init HMC Adaptor.\r\n");

  //
  // Init Register: DDRIOCTL
  //
  // case:270660 - Try to get IoSize from io48_hmc_mmr.niosreserve(N) registers
  DramIoWidth = ALT_IO48_HMC_MMR_NIOSRESERVE0_IOSIZE_GET(
    MmioRead32 (ALT_IO48_HMC_MMR_OFST +
                ALT_IO48_HMC_MMR_NIOSRESERVE0_OFST));
  Data32 = MmioRead32 (ALT_IO48_HMC_MMR_OFST + ALT_IO48_HMC_MMR_NIOSRESERVE1_OFST);
  // Check if the IO Size value from HMC MMR is legit
  if ((DramIoWidth >= 16) && (DramIoWidth <= 64))
  {
    // niosreserve(N) registers looks legit
    ProgressPrint ("\t\t DRAM IoSize from HMC_MMR.\r\n");
    DramIoWidth = DramIoWidth >> 5; // Convert it to DDRIOCTL_IO_SIZE format
  } else {
    ProgressPrint ("\t\t ERROR! DRAM IoSize from HMC_MMR_NIOSRESERVE0 does not look legit\r\n");
    DramIoWidth = 1;
  }
  // Program the Dram IO Width
  MmioAndThenOr32 (ALT_ECC_HMC_OCP_OFST +
                   ALT_ECC_HMC_OCP_DDRIOCTL_OFST,
                   ALT_ECC_HMC_OCP_DDRIOCTL_IO_SIZE_CLR_MSK,
                   ALT_ECC_HMC_OCP_DDRIOCTL_IO_SIZE_SET(DramIoWidth));

  //
  // Does HMC enabled the generation and checking of ECC ?
  //
  Data32 = MmioRead32 (ALT_IO48_HMC_MMR_OFST + ALT_IO48_HMC_MMR_CTLCFG1_OFST);
  if (ALT_IO48_HMC_MMR_CTLCFG1_CFG_CTL_EN_ECC_GET(Data32) == 1) {
    // Yes, ECC is enabled.
    ProgressPrint ("\t\t ECC is enabled.\r\n");

    //
    // Init Register: ECCCTRL1 and ECCCTRL2
    //

    // ECCCTRL1:Perform counter reset
    MmioAndThenOr32 (ALT_ECC_HMC_OCP_OFST +
                     ALT_ECC_HMC_OCP_ECCCTL1_OFST,
                     ALT_ECC_HMC_OCP_ECCCTL1_AUTOWB_CNT_RST_CLR_MSK &
                     ALT_ECC_HMC_OCP_ECCCTL1_CNT_RST_CLR_MSK &
                     ALT_ECC_HMC_OCP_ECCCTL1_ECC_EN_CLR_MSK,
                     ALT_ECC_HMC_OCP_ECCCTL1_AUTOWB_CNT_RST_SET(1) |
                     ALT_ECC_HMC_OCP_ECCCTL1_CNT_RST_SET(1) |
                     ALT_ECC_HMC_OCP_ECCCTL1_ECC_EN_SET(0));

    // ECCCTRL2: Enable read modify write logic and auto write back correction.
    MmioAndThenOr32 (ALT_ECC_HMC_OCP_OFST +
                     ALT_ECC_HMC_OCP_ECCCTL2_OFST,
                     ALT_ECC_HMC_OCP_ECCCTL2_OVRW_RB_ECC_EN_CLR_MSK &
                     ALT_ECC_HMC_OCP_ECCCTL2_RMW_EN_CLR_MSK &
                     ALT_ECC_HMC_OCP_ECCCTL2_AUTOWB_EN_CLR_MSK,
                     ALT_ECC_HMC_OCP_ECCCTL2_OVRW_RB_ECC_EN_SET(0) |
                     ALT_ECC_HMC_OCP_ECCCTL2_RMW_EN_SET(1) |
                     ALT_ECC_HMC_OCP_ECCCTL2_AUTOWB_EN_SET(1));

    // ECCCTRL1:Enable the ECC
    MmioAndThenOr32 (ALT_ECC_HMC_OCP_OFST +
                     ALT_ECC_HMC_OCP_ECCCTL1_OFST,
                     ALT_ECC_HMC_OCP_ECCCTL1_AUTOWB_CNT_RST_CLR_MSK &
                     ALT_ECC_HMC_OCP_ECCCTL1_CNT_RST_CLR_MSK &
                     ALT_ECC_HMC_OCP_ECCCTL1_ECC_EN_CLR_MSK,
                     ALT_ECC_HMC_OCP_ECCCTL1_AUTOWB_CNT_RST_SET(0) |
                     ALT_ECC_HMC_OCP_ECCCTL1_CNT_RST_SET(0) |
                     ALT_ECC_HMC_OCP_ECCCTL1_ECC_EN_SET(1));

  } else {
    // No, ECC is disabled.
    ProgressPrint ("\t\t ECC is disabled.\r\n");
  }

}


VOID
EFIAPI
ConfigureDdrSchedulerControlRegisters (
  VOID
  )
{
  UINT32          Data32;
  // Variable Used by Step 1
  UINT32          DramAddrOrder;
  UINT32          DdrConf;
  UINT32          Bank, Row, Col;
  BOOLEAN         IsDdrConfFound;
  // Variable Used by Step 2
  UINT32          RdToMiss;
  UINT32          WrToMiss;
  UINT32          BurstLen;
  UINT32          BurstLenInDdrClockUnit;
  UINT32          BurstLenInSchedulerClockUnit;
  UINT32          ActToAct;
  UINT32          RdToWr;
  UINT32          WrToRd;
  UINT32          BwRatio;
  UINT32          tRTP;
  UINT32          tRP;
  UINT32          tRCD;
  UINT32          RdLatency; // Also used by Step 4
  UINT32          tWRinClockCycles;
  // Variable Used by Step 3
  UINT32          BwRatioExtended;
  UINT32          AutoPreCharge;
  // Variable Used by Step 5
  UINT32          ActToActDiffBank;
  UINT32          Faw;
  UINT32          FawBank;
  // Variable Used by Step 6
  UINT32          BusRdToRd;
  UINT32          BusRdToWr;
  UINT32          BusWrToRd;

  ProgressPrint ("\t Init HPS NOC's DDR Scheduler.\r\n");

  //
  // Step 1 - Init Selector of predefined ddrConf configuration.
  //
  DdrConf = ALT_NOC_MPU_DDR_T_SCHED_DDRCONF_DDRCONF_RESET;
  IsDdrConfFound = FALSE;
  // First, find out the DRAM Address Ordering format
  // where "00" - chip, row, bank(BG, BA), column;
  //       "01" - chip, bank(BG, BA), row, column;
  //       "10" - row, chip, bank(BG, BA), column;
  // Only do this step 1 when "00" or "01", because this DdrConf selector
  // only support "00":Chip_R(n)_B(n)_C(n) and "01":Chip_B(n)_R(n)_C(n)
  Data32 = MmioRead32 (ALT_IO48_HMC_MMR_OFST + ALT_IO48_HMC_MMR_CTLCFG1_OFST);
  DramAddrOrder = ALT_IO48_HMC_MMR_CTLCFG1_CFG_ADDR_ORDER_GET(Data32);

  // Next, get the row/column/bank address width information
  Data32 = MmioRead32 (ALT_IO48_HMC_MMR_OFST + ALT_IO48_HMC_MMR_DRAMADDRW_OFST);

  Col  = ALT_IO48_HMC_MMR_DRAMADDRW_CFG_COL_ADDR_WIDTH_GET(Data32);
  Row  = ALT_IO48_HMC_MMR_DRAMADDRW_CFG_ROW_ADDR_WIDTH_GET(Data32);
  Bank = ALT_IO48_HMC_MMR_DRAMADDRW_CFG_BANK_ADDR_WIDTH_GET(Data32) +
         ALT_IO48_HMC_MMR_DRAMADDRW_CFG_BANK_GROUP_ADDR_WIDTH_GET(Data32);

  // Determine what value to be set for DdrConf
  ProgressPrint ("\t\t DRAM Address Order: ");
  if (DramAddrOrder == 0) {
    IsDdrConfFound = TRUE;
    // "00":Chip, Row(n)_Bank(n)_Column(n)
    ProgressPrint ("Chip-Row-Bank-Col\r\n");
    #define MERGE_ROW_BANK_COL(ROW, BANK, COL) ((ROW<<16)|(BANK<<8)|COL)
    switch (MERGE_ROW_BANK_COL(Row, Bank, Col))
    {
      case MERGE_ROW_BANK_COL(12, 3, 10):
        // 0x00
        DdrConf = ALT_NOC_MPU_DDR_T_SCHED_DDRCONF_DDRCONF_E_R12_B3_C10;
        break;
      case MERGE_ROW_BANK_COL(13, 3, 10):
        // 0x01
        DdrConf = ALT_NOC_MPU_DDR_T_SCHED_DDRCONF_DDRCONF_E_R13_B3_C10;
        break;
      case MERGE_ROW_BANK_COL(14, 3, 10):
        // 0x02
        DdrConf = ALT_NOC_MPU_DDR_T_SCHED_DDRCONF_DDRCONF_E_R14_B3_C10;
        break;
      case MERGE_ROW_BANK_COL(15, 3, 10):
        // 0x03
        DdrConf = ALT_NOC_MPU_DDR_T_SCHED_DDRCONF_DDRCONF_E_R15_B3_C10;
        break;
      case MERGE_ROW_BANK_COL(16, 3, 10):
        // 0x04
        DdrConf = ALT_NOC_MPU_DDR_T_SCHED_DDRCONF_DDRCONF_E_R16_B3_C10;
        break;
      case MERGE_ROW_BANK_COL(17, 3, 10):
        // 0x05
        DdrConf = ALT_NOC_MPU_DDR_T_SCHED_DDRCONF_DDRCONF_E_R17_B3_C10;
        break;
      case MERGE_ROW_BANK_COL(14, 3, 11):
        // 0x06
        DdrConf = ALT_NOC_MPU_DDR_T_SCHED_DDRCONF_DDRCONF_E_R14_B3_C11;
        break;
      case MERGE_ROW_BANK_COL(15, 3, 11):
        // 0x07
        DdrConf = ALT_NOC_MPU_DDR_T_SCHED_DDRCONF_DDRCONF_E_R15_B3_C11;
        break;
      case MERGE_ROW_BANK_COL(16, 3, 11):
        // 0x08
        DdrConf = ALT_NOC_MPU_DDR_T_SCHED_DDRCONF_DDRCONF_E_R16_B3_C11;
        break;
      case MERGE_ROW_BANK_COL(15, 3, 12):
        // 0x09
        DdrConf = ALT_NOC_MPU_DDR_T_SCHED_DDRCONF_DDRCONF_E_R15_B3_C12;
        break;
      case MERGE_ROW_BANK_COL(14, 4, 10):
        // 0x0A
        DdrConf = ALT_NOC_MPU_DDR_T_SCHED_DDRCONF_DDRCONF_E_R14_B4_C10;
        break;
      case MERGE_ROW_BANK_COL(15, 4, 10):
        // 0x0B
        DdrConf = ALT_NOC_MPU_DDR_T_SCHED_DDRCONF_DDRCONF_E_R15_B4_C10;
        break;
      case MERGE_ROW_BANK_COL(16, 4, 10):
        // 0x0C
        DdrConf = ALT_NOC_MPU_DDR_T_SCHED_DDRCONF_DDRCONF_E_R16_B4_C10;
        break;
      case MERGE_ROW_BANK_COL(17, 4, 10):
        // 0x0D
        DdrConf = ALT_NOC_MPU_DDR_T_SCHED_DDRCONF_DDRCONF_E_R17_B4_C10;
        break;
      default:
        IsDdrConfFound = FALSE;
        break;
    }
  } else if (DramAddrOrder == 1) {
    IsDdrConfFound = TRUE;
    // "01":Chip, Bank(n)_Row(n)_Column(n)
    ProgressPrint ("Chip-Bank-Row-Col\r\n");
    #define MERGE_BANK_ROW_COL(BANK, ROW, COL) ((BANK<<16)|(ROW<<8)|COL)
    switch (MERGE_BANK_ROW_COL(Bank, Row, Col))
    {
      case MERGE_BANK_ROW_COL(3, 12, 10):
        // 0x0E
        DdrConf = ALT_NOC_MPU_DDR_T_SCHED_DDRCONF_DDRCONF_E_B3_R12_C10;
        break;
      case MERGE_BANK_ROW_COL(3, 13, 10):
        // 0x0F
        DdrConf = ALT_NOC_MPU_DDR_T_SCHED_DDRCONF_DDRCONF_E_B3_R13_C10;
        break;
      case MERGE_BANK_ROW_COL(3, 14, 10):
        // 0x10
        DdrConf = ALT_NOC_MPU_DDR_T_SCHED_DDRCONF_DDRCONF_E_B3_R14_C10;
        break;
      case MERGE_BANK_ROW_COL(3, 15, 10):
        // 0x11
        DdrConf = ALT_NOC_MPU_DDR_T_SCHED_DDRCONF_DDRCONF_E_B3_R15_C10;
        break;
      case MERGE_BANK_ROW_COL(3, 16, 10):
        // 0x12
        DdrConf = ALT_NOC_MPU_DDR_T_SCHED_DDRCONF_DDRCONF_E_B3_R16_C10;
        break;
      case MERGE_BANK_ROW_COL(3, 17, 10):
        // 0x13
        DdrConf = ALT_NOC_MPU_DDR_T_SCHED_DDRCONF_DDRCONF_E_B3_R17_C10;
        break;
      case MERGE_BANK_ROW_COL(3, 14, 11):
        // 0x14
        DdrConf = ALT_NOC_MPU_DDR_T_SCHED_DDRCONF_DDRCONF_E_B3_R14_C11;
        break;
      case MERGE_BANK_ROW_COL(3, 15, 11):
        // 0x15
        DdrConf = ALT_NOC_MPU_DDR_T_SCHED_DDRCONF_DDRCONF_E_B3_R15_C11;
        break;
      case MERGE_BANK_ROW_COL(3, 16, 11):
        // 0x16
        DdrConf = ALT_NOC_MPU_DDR_T_SCHED_DDRCONF_DDRCONF_E_B3_R16_C11;
        break;
      case MERGE_BANK_ROW_COL(3, 15, 12):
        // 0x17
        DdrConf = ALT_NOC_MPU_DDR_T_SCHED_DDRCONF_DDRCONF_E_B3_R15_C12;
        break;
      case MERGE_BANK_ROW_COL(4, 14, 10):
        // 0x18
        DdrConf = ALT_NOC_MPU_DDR_T_SCHED_DDRCONF_DDRCONF_E_B4_R14_C10;
        break;
      case MERGE_BANK_ROW_COL(4, 15, 10):
        // 0x19
        DdrConf = ALT_NOC_MPU_DDR_T_SCHED_DDRCONF_DDRCONF_E_B4_R15_C10;
        break;
      case MERGE_BANK_ROW_COL(4, 16, 10):
        // 0x1A
        DdrConf = ALT_NOC_MPU_DDR_T_SCHED_DDRCONF_DDRCONF_E_B4_R16_C10;
        break;
      case MERGE_BANK_ROW_COL(4, 17, 10):
        // 0x1B
        DdrConf = ALT_NOC_MPU_DDR_T_SCHED_DDRCONF_DDRCONF_E_B4_R17_C10;
        break;
      default:
        IsDdrConfFound = FALSE;
        break;
    }
  } else if (DramAddrOrder == 2) {
    ProgressPrint ("Row-Chip-Bank-Col\r\n");
  } else {
    ProgressPrint ("Undefined\r\n");
  }

  // Set the DdrConf register
  if (IsDdrConfFound == TRUE) {
    MmioAndThenOr32 (
      ALT_NOC_MPU_DDR_T_SCHED_OFST +
      ALT_NOC_MPU_DDR_T_SCHED_DDRCONF_OFST,
      ALT_NOC_MPU_DDR_T_SCHED_DDRCONF_DDRCONF_CLR_MSK,
      ALT_NOC_MPU_DDR_T_SCHED_DDRCONF_DDRCONF_SET(DdrConf)
    );
  } else {
    ProgressPrint ("\t\t Cannot find predefined ddrConf configuration.\r\n");
  }

  //
  // Step 2 - Init DDR Timing for scheduler clock cycles between DRAM commands.
  // Register : ddr_T_main_Scheduler_DdrTiming
  //
  // WRTOMISS formula note:
  // Begin with, WRTOMISS = WL + tWR + tRP + tRCD
  //      Where, WL = RL + BL/2 + 2 - rd-to-wr
  //        and, tWR = 15ns (see JEDEC standard eg. DDR4 = JESD79-4.pdf)
  // The equation is in memory clock units,
  // We need to divide the first part by 2 to make it HMC clock units.
  // A note for BL (on question why is it BL's final value is equal to original BL divide by 4):
  // BL/2 - the 1ST divide by 2 is because data is transferred on both edges DDR clock.
  // (BL/2)/2 - the 2nd divide by 2 again is to convert to scheduler or controller clocks.
  // Final WRTOMISS = ((RL + BL/2 + 2 + tWR) / 2) - rd-to-wr + tRP + tRCD
  //
  // RDTOMISS formula note:
  // Begin with, RDTOMISS = tRTP + tRP + tRCD - BL/2
  // After convert BL to HMC clock units.
  // Final RDTOMISS = tRTP + tRP + tRCD - ((BL/2)/2)
  //
  // For DDR3, Maximum Frequency is 1066 MHz, to get 15ns tWR:
  //   1066MHz is 0.938ns which is close to 1ns, so we can use 15 directly.
  //
  // For DDR4, Maximum Frequency is 1333 MHz, to get 15ns tWR:
  //   1333MHz is 0.75ns, to get 15ns, we need 15/0.75 = 20 clock cycles
  //
  // BURSTLEN = hmc_mmr.ctrlcfg0.cfg_ctrl_burst_length / 2
  // ACTTOACT = hmc_mmr.caltiming0.cfg_t_param_act_to_act
  // RDTOWR   = hmc_mmr.caltiming1.cfg_t_param_rd_to_wr
  // WRTORD   = hmc_mmr.caltiming3.cfg_t_param_wr_to_rd
  // BWRATIO  = ((ecc_hmc.DDRIOCTL == 0) ? 0 : 1)

  // Calculate the value to be set for DdrTiming register

  Data32 = MmioRead32 (ALT_IO48_HMC_MMR_OFST + ALT_IO48_HMC_MMR_DRAMTIMING0_OFST);
  RdLatency = ALT_IO48_HMC_MMR_DRAMTIMING0_CFG_TCL_GET(Data32);

  Data32 = MmioRead32 (ALT_IO48_HMC_MMR_OFST + ALT_IO48_HMC_MMR_CALTIMING0_OFST);
  ActToAct = ALT_IO48_HMC_MMR_CALTIMING0_CFG_T_PARAM_ACT_TO_ACT_GET(Data32);
  tRCD = ALT_IO48_HMC_MMR_CALTIMING0_CFG_T_PARAM_ACT_TO_RDWR_GET(Data32);
  ActToActDiffBank = ALT_IO48_HMC_MMR_CALTIMING0_CFG_T_PARAM_ACT_TO_ACT_DIFF_BANK_GET(Data32);

  Data32 = MmioRead32 (ALT_IO48_HMC_MMR_OFST + ALT_IO48_HMC_MMR_CALTIMING1_OFST);
  RdToWr = ALT_IO48_HMC_MMR_CALTIMING1_CFG_T_PARAM_RD_TO_WR_GET(Data32);
  BusRdToRd = ALT_IO48_HMC_MMR_CALTIMING1_CFG_T_PARAM_RD_TO_RD_DIFF_CHIP_GET(Data32);
  BusRdToWr = ALT_IO48_HMC_MMR_CALTIMING1_CFG_T_PARAM_RD_TO_WR_DIFF_CHIP_GET(Data32);

  Data32 = MmioRead32 (ALT_IO48_HMC_MMR_OFST + ALT_IO48_HMC_MMR_CALTIMING2_OFST);
  tRTP = ALT_IO48_HMC_MMR_CALTIMING2_CFG_T_PARAM_RD_TO_PCH_GET(Data32);

  Data32 = MmioRead32 (ALT_IO48_HMC_MMR_OFST + ALT_IO48_HMC_MMR_CALTIMING3_OFST);
  WrToRd = ALT_IO48_HMC_MMR_CALTIMING3_CFG_T_PARAM_WR_TO_RD_GET(Data32);
  BusWrToRd = ALT_IO48_HMC_MMR_CALTIMING3_CFG_T_PARAM_WR_TO_RD_DIFF_CHIP_GET(Data32);

  Data32 = MmioRead32 (ALT_IO48_HMC_MMR_OFST + ALT_IO48_HMC_MMR_CALTIMING4_OFST);
  tRP = ALT_IO48_HMC_MMR_CALTIMING4_CFG_T_PARAM_PCH_TO_VALID_GET(Data32);

  Data32 = MmioRead32 (ALT_ECC_HMC_OCP_OFST + ALT_ECC_HMC_OCP_DDRIOCTL_OFST);
  BwRatio = ((ALT_ECC_HMC_OCP_DDRIOCTL_IO_SIZE_GET(Data32) == 0) ? 0 : 1);

  Data32 = MmioRead32 (ALT_IO48_HMC_MMR_OFST + ALT_IO48_HMC_MMR_CTLCFG0_OFST);
  BurstLen = ALT_IO48_HMC_MMR_CTLCFG0_CFG_CTL_BURST_LEN_GET(Data32);
  BurstLenInDdrClockUnit = BurstLen / 2;
  BurstLenInSchedulerClockUnit = ((BurstLen/2) / 2);

  // tWR = Min. 15ns constant, see JEDEC standard eg. DDR4 is JESD79-4.pdf
  #define tWR_IN_NANOSECONDS 15
  Data32 = MmioRead32 (ALT_IO48_HMC_MMR_OFST + ALT_IO48_HMC_MMR_CTLCFG0_OFST);
  switch (ALT_IO48_HMC_MMR_CTLCFG0_CFG_MEM_TYPE_GET(Data32))
  {
    case 1:
      // DDR4 - 1333MHz
      // 20 (19.995) clock cycles = 15ns
      // Calculate with rounding
      tWRinClockCycles = (((tWR_IN_NANOSECONDS * 1333) % 1000) >= 500) ?
                          ((tWR_IN_NANOSECONDS * 1333) / 1000) + 1 :
                          ((tWR_IN_NANOSECONDS * 1333) / 1000);
      break;
    default:
      // Others - 1066MHz or slower
      // 16 (15.990) clock cycles = 15ns
      // Calculate with rounding
      tWRinClockCycles = (((tWR_IN_NANOSECONDS * 1066) % 1000) >= 500) ?
                          ((tWR_IN_NANOSECONDS * 1066) / 1000) + 1 :
                          ((tWR_IN_NANOSECONDS * 1066) / 1000);
      break;
  }

  RdToMiss = tRTP + tRP + tRCD - BurstLenInSchedulerClockUnit;
  WrToMiss = ((RdLatency + BurstLenInDdrClockUnit + 2 + tWRinClockCycles) / 2) - RdToWr + tRP + tRCD;

  ProgressPrint (
    "\t\t RdLatency = %d\r\n"
    "\t\t ActToAct = %d\r\n"
    "\t\t tRCD = %d\r\n"
    "\t\t ActToActDiffBank = %d\r\n"
    "\t\t BusRdToRd = %d\r\n"
    "\t\t BusRdToWr = %d\r\n"
    "\t\t tRTP = %d\r\n"
    "\t\t WrToRd = %d\r\n"
    "\t\t BusWrToRd = %d\r\n"
    "\t\t tRP = %d\r\n"
    "\t\t BwRatio = %d\r\n"
    "\t\t BurstLen = %d\r\n"
    "\t\t BurstLen in one DDR Clock cycles = %d\r\n"
    "\t\t BurstLen in DDR Scheduler Clock cycles = %d\r\n"
    "\t\t tWR in Nano Seconds = %d\r\n"
    "\t\t tWR in Clock Cycles = %d\r\n"
    "\t\t RdToMiss = %d\r\n"
    "\t\t WrToMiss = %d\r\n",
    RdLatency,
    ActToAct,
    tRCD,
    ActToActDiffBank,
    BusRdToRd,
    BusRdToWr,
    tRTP,
    WrToRd,
    BusWrToRd,
    tRP,
    BwRatio,
    BurstLen,
    BurstLenInDdrClockUnit,
    BurstLenInSchedulerClockUnit,
    tWR_IN_NANOSECONDS,
    tWRinClockCycles,
    RdToMiss,
    WrToMiss);

  // Set the DdrTiming register
  MmioAndThenOr32 (
    ALT_NOC_MPU_DDR_T_SCHED_OFST +
    ALT_NOC_MPU_DDR_T_SCHED_DDRTIMING_OFST,
    ALT_NOC_MPU_DDR_T_SCHED_DDRTIMING_BWRATIO_CLR_MSK &
    ALT_NOC_MPU_DDR_T_SCHED_DDRTIMING_WRTORD_CLR_MSK &
    ALT_NOC_MPU_DDR_T_SCHED_DDRTIMING_RDTOWR_CLR_MSK &
    ALT_NOC_MPU_DDR_T_SCHED_DDRTIMING_BURSTLEN_CLR_MSK &
    ALT_NOC_MPU_DDR_T_SCHED_DDRTIMING_WRTOMISS_CLR_MSK &
    ALT_NOC_MPU_DDR_T_SCHED_DDRTIMING_RDTOMISS_CLR_MSK &
    ALT_NOC_MPU_DDR_T_SCHED_DDRTIMING_ACTTOACT_CLR_MSK,
    ALT_NOC_MPU_DDR_T_SCHED_DDRTIMING_BWRATIO_SET(BwRatio) |
    ALT_NOC_MPU_DDR_T_SCHED_DDRTIMING_WRTORD_SET(WrToRd) |
    ALT_NOC_MPU_DDR_T_SCHED_DDRTIMING_RDTOWR_SET(RdToWr) |
    ALT_NOC_MPU_DDR_T_SCHED_DDRTIMING_BURSTLEN_SET(BurstLenInSchedulerClockUnit) |
    ALT_NOC_MPU_DDR_T_SCHED_DDRTIMING_WRTOMISS_SET(WrToMiss) |
    ALT_NOC_MPU_DDR_T_SCHED_DDRTIMING_RDTOMISS_SET(RdToMiss) |
    ALT_NOC_MPU_DDR_T_SCHED_DDRTIMING_ACTTOACT_SET(ActToAct)
  );

  //
  // Step 3 - Init DDR mode concerning Page Auto Precharge and BW Ratio.
  //
  Data32 = MmioRead32 (ALT_ECC_HMC_OCP_OFST + ALT_ECC_HMC_OCP_DDRIOCTL_OFST);
  BwRatioExtended = ((ALT_ECC_HMC_OCP_DDRIOCTL_IO_SIZE_GET(Data32) == 0) ? 1 : 0);

  AutoPreCharge = 0;

  MmioAndThenOr32 (
    ALT_NOC_MPU_DDR_T_SCHED_OFST +
    ALT_NOC_MPU_DDR_T_SCHED_DDRMOD_OFST,
    ALT_NOC_MPU_DDR_T_SCHED_DDRMOD_BWRATIOEXTENDED_CLR_MSK &
    ALT_NOC_MPU_DDR_T_SCHED_DDRMOD_AUTOPRECHARGE_CLR_MSK,
    ALT_NOC_MPU_DDR_T_SCHED_DDRMOD_BWRATIOEXTENDED_SET(BwRatioExtended) |
    ALT_NOC_MPU_DDR_T_SCHED_DDRMOD_AUTOPRECHARGE_SET(AutoPreCharge)
  );

  //
  // Step 4 - Init Read Latency between a read request and the first data response.
  //
  // case 314587: Update DDR Scheduler ReadLatency calculation
  MmioAndThenOr32 (
    ALT_NOC_MPU_DDR_T_SCHED_OFST +
    ALT_NOC_MPU_DDR_T_SCHED_RDLATENCY_OFST,
    ALT_NOC_MPU_DDR_T_SCHED_RDLATENCY_RDLATENCY_CLR_MSK,
    ALT_NOC_MPU_DDR_T_SCHED_RDLATENCY_RDLATENCY_SET(RdLatency / 2 + DDR_READ_LATENCY_DELAY)
  );

  //
  // Step 5 - Init Timing values concerning Activate commands
  //
  Data32 = MmioRead32 (ALT_IO48_HMC_MMR_OFST + ALT_IO48_HMC_MMR_CALTIMING9_OFST);
  Faw = ALT_IO48_HMC_MMR_CALTIMING9_CFG_T_PARAM_4_ACT_TO_ACT_GET(Data32);

  // Number of Bank of a given device involved in the FAW period.
  FawBank = 1; // always 1 because we always have 4 bank DDR.

  MmioAndThenOr32 (
    ALT_NOC_MPU_DDR_T_SCHED_OFST +
    ALT_NOC_MPU_DDR_T_SCHED_ACTIVATE_OFST,
    ALT_NOC_MPU_DDR_T_SCHED_ACTIVATE_FAWBANK_CLR_MSK &
    ALT_NOC_MPU_DDR_T_SCHED_ACTIVATE_FAW_CLR_MSK &
    ALT_NOC_MPU_DDR_T_SCHED_ACTIVATE_RRD_CLR_MSK,
    ALT_NOC_MPU_DDR_T_SCHED_ACTIVATE_FAWBANK_SET(FawBank) |
    ALT_NOC_MPU_DDR_T_SCHED_ACTIVATE_FAW_SET(Faw) |
    ALT_NOC_MPU_DDR_T_SCHED_ACTIVATE_RRD_SET(ActToActDiffBank)
  );

  //
  // Step 6 - Init Timing values concerning Device to Device Data Bus ownership change.
  //
  MmioAndThenOr32 (
    ALT_NOC_MPU_DDR_T_SCHED_OFST +
    ALT_NOC_MPU_DDR_T_SCHED_DEVTODEV_OFST,
    ALT_NOC_MPU_DDR_T_SCHED_DEVTODEV_BUSRDTORD_CLR_MSK &
    ALT_NOC_MPU_DDR_T_SCHED_DEVTODEV_BUSRDTOWR_CLR_MSK &
    ALT_NOC_MPU_DDR_T_SCHED_DEVTODEV_BUSWRTORD_CLR_MSK,
    ALT_NOC_MPU_DDR_T_SCHED_DEVTODEV_BUSRDTORD_SET(BusRdToRd) |
    ALT_NOC_MPU_DDR_T_SCHED_DEVTODEV_BUSRDTOWR_SET(BusRdToWr) |
    ALT_NOC_MPU_DDR_T_SCHED_DEVTODEV_BUSWRTORD_SET(BusWrToRd)
  );

}


UINT64
EFIAPI
GetPhysicalDramSize (
  VOID
)
{
  UINT32          Data32;
  UINT64          RamSize;
  UINTN           RamAddrWidth;
  UINTN           RamExtIfIoWidth;

  // Get DRAM external interface IO size
  Data32 = MmioRead32 (ALT_ECC_HMC_OCP_OFST + ALT_ECC_HMC_OCP_DDRIOCTL_OFST);
  switch (ALT_ECC_HMC_OCP_DDRIOCTL_IO_SIZE_GET(Data32))
  {
    case 0:
      RamExtIfIoWidth = 16;
      break;
    case 1:
      RamExtIfIoWidth = 32;
      break;
    case 2:
      RamExtIfIoWidth = 64;
      break;
    default:
      RamExtIfIoWidth = 0;
      break;
  }
  // Get RAM Address Width
  Data32 = MmioRead32 (ALT_IO48_HMC_MMR_OFST + ALT_IO48_HMC_MMR_DRAMADDRW_OFST);
  RamAddrWidth = ALT_IO48_HMC_MMR_DRAMADDRW_CFG_COL_ADDR_WIDTH_GET(Data32) +
                 ALT_IO48_HMC_MMR_DRAMADDRW_CFG_ROW_ADDR_WIDTH_GET(Data32) +
                 ALT_IO48_HMC_MMR_DRAMADDRW_CFG_BANK_ADDR_WIDTH_GET(Data32) +
                 ALT_IO48_HMC_MMR_DRAMADDRW_CFG_BANK_GROUP_ADDR_WIDTH_GET(Data32) +
                 ALT_IO48_HMC_MMR_DRAMADDRW_CFG_CS_ADDR_WIDTH_GET(Data32);
  // Calcualte total RAM size in number of bytes
  RamSize = ((UINT64)1 << RamAddrWidth) * (RamExtIfIoWidth / 8);

  return RamSize;
}


UINT64
EFIAPI
GetMpuWindowDramSize (
  VOID
)
{
  UINT64 RamSize;
  // Considered the case where MPU DRAM windows does not start at 0
  RamSize = GetPhysicalDramSize() - GetMpuWindowDramBaseAddr();
  return RamSize;
}


UINTN
EFIAPI
GetMpuWindowDramBaseAddr (
  VOID
)
{
  UINTN  MpuDramWindowsBottom;
  UINTN  DramBaseAddr = 0;
  UINT32 Data32;

  Data32 =   MmioRead32 (ARM_MPUL2_OFST + ARM_MPUL2_ADDR_FILTERING_START_OFST);
  if ( ARM_MPUL2_ADDR_FILTERING_START_EN_GET(Data32) ==
       ARM_MPUL2_ADDR_FILTERING_ENABLED )
  {
    MpuDramWindowsBottom = ARM_MPUL2_ADDR_FILTERING_ADDR_GET(Data32);
    DramBaseAddr = MpuDramWindowsBottom;
  } else {
    // Should not reach here, you forget to set the MPU address filtering enable bit?
    // This should have already been done in InitMpuDramWindowBoundary function
    ASSERT_PLATFORM_INIT(0);
  }

  return DramBaseAddr;
}


VOID
EFIAPI
DisplayMemoryInfo (
  VOID
  )
{
  UINT32  Data32;
  UINT64  RamSize;
  UINTN   RamAddrWidth;
  UINTN   RamExtIfIoWidth;
  CHAR8*  DramTypeAsciiStrPtr;

  // Display RAM external IO size
  Data32 = MmioRead32 (ALT_ECC_HMC_OCP_OFST + ALT_ECC_HMC_OCP_DDRIOCTL_OFST);
  InfoPrint (
    "HMC Info:\r\n"
    "\t ECC_HMC_OCP_DDRIOCTL: 0x%08x\r\n"
    "\t\t IO Size : ",
    Data32);
  switch (ALT_ECC_HMC_OCP_DDRIOCTL_IO_SIZE_GET(Data32))
  {
    case 0:
      RamExtIfIoWidth = 16;
      InfoPrint ("x16\r\n");
      break;
    case 1:
      RamExtIfIoWidth = 32;
      InfoPrint ("x32\r\n");
      break;
    case 2:
      RamExtIfIoWidth = 64;
      InfoPrint ("x64\r\n");
      break;
    default:
      RamExtIfIoWidth = 0;
      InfoPrint ("Unknown\r\n");
      break;
  }

  // Display RAM Address Width
  Data32 = MmioRead32 (ALT_IO48_HMC_MMR_OFST + ALT_IO48_HMC_MMR_DRAMADDRW_OFST);
  InfoPrint (
    "\t HMC_MMR_DRAMADDRW: 0x%08x\r\n"
    "\t\t column address width\t\t: %d bits\r\n"
    "\t\t row address width\t\t: %d bits\r\n"
    "\t\t bank address width\t\t: %d bits\r\n"
    "\t\t bank group address width\t: %d bits\r\n"
    "\t\t chip select address width\t: %d bits\r\n",
    Data32,
    ALT_IO48_HMC_MMR_DRAMADDRW_CFG_COL_ADDR_WIDTH_GET(Data32),
    ALT_IO48_HMC_MMR_DRAMADDRW_CFG_ROW_ADDR_WIDTH_GET(Data32),
    ALT_IO48_HMC_MMR_DRAMADDRW_CFG_BANK_ADDR_WIDTH_GET(Data32),
    ALT_IO48_HMC_MMR_DRAMADDRW_CFG_BANK_GROUP_ADDR_WIDTH_GET(Data32),
    ALT_IO48_HMC_MMR_DRAMADDRW_CFG_CS_ADDR_WIDTH_GET(Data32));

  // Display total RAM size
  RamAddrWidth = ALT_IO48_HMC_MMR_DRAMADDRW_CFG_COL_ADDR_WIDTH_GET(Data32) +
                 ALT_IO48_HMC_MMR_DRAMADDRW_CFG_ROW_ADDR_WIDTH_GET(Data32) +
                 ALT_IO48_HMC_MMR_DRAMADDRW_CFG_BANK_ADDR_WIDTH_GET(Data32) +
                 ALT_IO48_HMC_MMR_DRAMADDRW_CFG_BANK_GROUP_ADDR_WIDTH_GET(Data32) +
                 ALT_IO48_HMC_MMR_DRAMADDRW_CFG_CS_ADDR_WIDTH_GET(Data32);
  RamSize = ((UINT64)1 << RamAddrWidth) * (RamExtIfIoWidth / 8);
  InfoPrint ("\t\t Memory Size\t\t\t: %Ld\r\n", RamSize);

  // Display RAM Type
  Data32 = MmioRead32 (ALT_IO48_HMC_MMR_OFST + ALT_IO48_HMC_MMR_CTLCFG0_OFST);
  InfoPrint (
    "\t HMC_MMR_CTLCFG0: 0x%08x\r\n"
    "\t\t Memory Type\t\t\t: ",
    Data32);
  switch (ALT_IO48_HMC_MMR_CTLCFG0_CFG_MEM_TYPE_GET(Data32))
  {
    case 0:
      DramTypeAsciiStrPtr = "DDR3";
      break;
    case 1:
      DramTypeAsciiStrPtr = "DDR4";
      break;
    case 2:
      DramTypeAsciiStrPtr = "LPDDR3";
      break;
    case 3:
      DramTypeAsciiStrPtr = "RLDRAM3";
      break;
    default:
      DramTypeAsciiStrPtr = "Unknown";
      break;
  }
  InfoPrint ("%a\r\n",  DramTypeAsciiStrPtr);

  //----------------------------------------------------------------------------------

  // Display IO48_HMC_MMR_NIOSRESERVE0
  Data32 = MmioRead32 (ALT_IO48_HMC_MMR_OFST + ALT_IO48_HMC_MMR_NIOSRESERVE0_OFST);
  InfoPrint ("\t IO48_HMC_MMR_NIOSRESERVE0: 0x%08x\r\n", Data32);

  // Display EMIF IP ACDS Version
  Data32 = MmioRead32 (ALT_IO48_HMC_MMR_OFST + ALT_IO48_HMC_MMR_NIOSRESERVE1_OFST);
  InfoPrint ( "\t IO48_HMC_MMR_NIOSRESERVE1: 0x%08x\r\n", Data32);

  if (ALT_IO48_HMC_MMR_NIOSRESERVE1_ACDS_MAJOR_VER_GET(Data32) != 0) {
    InfoPrint ("\t\t EMIF IP ACDS version : ");
    // Print "Major.Minor"
    InfoPrint ( "%d.%d",
                ALT_IO48_HMC_MMR_NIOSRESERVE1_ACDS_MAJOR_VER_GET(Data32),
                ALT_IO48_HMC_MMR_NIOSRESERVE1_ACDS_MINOR_VER_GET(Data32));
    // Print Sevice Pack version if valid
    if (ALT_IO48_HMC_MMR_NIOSRESERVE1_ACDS_SP_VER_GET(Data32) != 0) {
      InfoPrint("sp%d",
                ALT_IO48_HMC_MMR_NIOSRESERVE1_ACDS_SP_VER_GET(Data32));
    }

    // Print ACDS Special Variant suffix
    switch (ALT_IO48_HMC_MMR_NIOSRESERVE1_ACDS_VARIANT_GET(Data32))
    {
      case 0:
        // Not a special variant
        break;
      case 3:
        InfoPrint ("eap");
        break;
      default:
        // unrecongized variant
        InfoPrint ( " [ACDS Variant = %d]",
                    ALT_IO48_HMC_MMR_NIOSRESERVE1_ACDS_VARIANT_GET(Data32));
        break;
    }
    InfoPrint ("\r\n");
  }

  //-----------------------------------------------------------------------------
  InfoPrint ("DDR Scheduler Info:\r\n");

  // Display DDR Scheduler DDRCONF register
  Data32 = MmioRead32 (ALT_NOC_MPU_DDR_T_SCHED_OFST + ALT_NOC_MPU_DDR_T_SCHED_DDRCONF_OFST);
  InfoPrint ("\t DDRCONF: 0x%08x\r\n", Data32);

  // Display DDR Scheduler DDRTIMING register
  Data32 = MmioRead32 (ALT_NOC_MPU_DDR_T_SCHED_OFST + ALT_NOC_MPU_DDR_T_SCHED_DDRTIMING_OFST);
  InfoPrint (
    "\t DDRTIMING: 0x%08x\r\n"
    "\t\t BwRatio \t: %d\r\n"
    "\t\t WrToRd  \t: %d\r\n"
    "\t\t RdToWr  \t: %d\r\n"
    "\t\t BurstLen\t: %d\r\n"
    "\t\t WrToMiss\t: %d\r\n"
    "\t\t RdToMiss\t: %d\r\n"
    "\t\t ActToAct\t: %d\r\n",
    Data32,
    ALT_NOC_MPU_DDR_T_SCHED_DDRTIMING_BWRATIO_GET(Data32),
    ALT_NOC_MPU_DDR_T_SCHED_DDRTIMING_WRTORD_GET(Data32),
    ALT_NOC_MPU_DDR_T_SCHED_DDRTIMING_RDTOWR_GET(Data32),
    ALT_NOC_MPU_DDR_T_SCHED_DDRTIMING_BURSTLEN_GET(Data32),
    ALT_NOC_MPU_DDR_T_SCHED_DDRTIMING_WRTOMISS_GET(Data32),
    ALT_NOC_MPU_DDR_T_SCHED_DDRTIMING_RDTOMISS_GET(Data32),
    ALT_NOC_MPU_DDR_T_SCHED_DDRTIMING_ACTTOACT_GET(Data32));

  // Display DDR Scheduler DDRMODE register
  Data32 = MmioRead32 (ALT_NOC_MPU_DDR_T_SCHED_OFST + ALT_NOC_MPU_DDR_T_SCHED_DDRMOD_OFST);
  InfoPrint (
    "\t DDRMODE: 0x%08x\r\n"
    "\t\t BwRatioExtended : %d\r\n"
    "\t\t AutoPreCharge   : %d\r\n",
    Data32,
    ALT_NOC_MPU_DDR_T_SCHED_DDRMOD_BWRATIOEXTENDED_GET(Data32),
    ALT_NOC_MPU_DDR_T_SCHED_DDRMOD_AUTOPRECHARGE_GET(Data32));

  // Display DDR Scheduler RDLATENCY register
  Data32 = MmioRead32 (ALT_NOC_MPU_DDR_T_SCHED_OFST + ALT_NOC_MPU_DDR_T_SCHED_RDLATENCY_OFST);
  InfoPrint (
    "\t RDLATENCY: 0x%08x (%d)\r\n",
     Data32, ALT_NOC_MPU_DDR_T_SCHED_RDLATENCY_RDLATENCY_GET(Data32));

  // Display DDR Scheduler ACTIVATE register
  Data32 = MmioRead32 (ALT_NOC_MPU_DDR_T_SCHED_OFST + ALT_NOC_MPU_DDR_T_SCHED_ACTIVATE_OFST);
  InfoPrint (
    "\t ACTIVATE: 0x%08x\r\n"
    "\t\t FAW BANK : %d\r\n"
    "\t\t FAW      : %d\r\n"
    "\t\t RRD      : %d\r\n",
    Data32,
    ALT_NOC_MPU_DDR_T_SCHED_ACTIVATE_FAWBANK_GET(Data32),
    ALT_NOC_MPU_DDR_T_SCHED_ACTIVATE_FAW_GET(Data32),
    ALT_NOC_MPU_DDR_T_SCHED_ACTIVATE_RRD_GET(Data32));

  // Display DDR Scheduler DEVTODEV register
  Data32 = MmioRead32 (ALT_NOC_MPU_DDR_T_SCHED_OFST + ALT_NOC_MPU_DDR_T_SCHED_DEVTODEV_OFST);
  InfoPrint (
    "\t DEVTODEV: 0x%08x\r\n"
    "\t\t BusRdToRd : %d\r\n"
    "\t\t BusRdToWr : %d\r\n"
    "\t\t BusWrToRd : %d\r\n",
    Data32,
    ALT_NOC_MPU_DDR_T_SCHED_DEVTODEV_BUSRDTORD_GET(Data32),
    ALT_NOC_MPU_DDR_T_SCHED_DEVTODEV_BUSRDTOWR_GET(Data32),
    ALT_NOC_MPU_DDR_T_SCHED_DEVTODEV_BUSWRTORD_GET(Data32));

  //-----------------------------------------------------------------------------

  InfoPrint (
    "Memory Info:\r\n"
    "\t Physical DRAM Type  = "
  );
  if ((RamSize >= (1024*1024*1024)) && ((RamSize % 1024) == 0)) {
    InfoPrint ( "%Ld GB ",
                RamSize / 1024 / 1024 / 1024);
  } else {
    InfoPrint ( "%Ld MB ",
                RamSize / 1024 / 1024);
  }
  InfoPrint (
    "%a\r\n"
    "\t Physical DRAM Size  = 0x%08Lx\r\n",
    DramTypeAsciiStrPtr,
    (UINT64)GetPhysicalDramSize()
  );

  InfoPrint (
    "Memory Map : \r\n");

  if (PcdGet32(PcdRemapOnChipRamTo1stOneMB) == 1) {
  InfoPrint (
    "\t HPS OCRAM Remapped  = 0x%08Lx - 0x%08Lx\r\n",
    (UINT64) 0,
    (UINT64) ((UINT32) ALT_OCRAM_UB_ADDR - ALT_OCRAM_OFST)
    );
  InfoPrint (
    "\t Undefined           = 0x%08Lx - 0x%08Lx\r\n",
    (UINT64) ((UINT32) ALT_OCRAM_UB_ADDR - ALT_OCRAM_OFST + 1),
    (UINT64) 0xFFFFF
    );
  }

  InfoPrint (
    "\t SDRAM Window        = 0x%08Lx - 0x%08Lx\r\n"
    "\t Unused SDRAM Window = 0x%08Lx - 0x%08Lx\r\n"
    "\t FPGA Slaves         = 0x%08Lx - 0x%08Lx\r\n"
    "\t STM Module          = 0x%08Lx - 0x%08Lx\r\n"
    "\t DAP Moudle          = 0x%08Lx - 0x%08Lx\r\n"
    "\t LW FPGA slaves      = 0x%08Lx - 0x%08Lx\r\n"
    "\t Undefined           = 0x%08Lx - 0x%08Lx\r\n"
    "\t Peripherals Region  = 0x%08Lx - 0x%08Lx\r\n"
    "\t HPS OCRAM           = 0x%08Lx - 0x%08Lx\r\n"
    "\t Undefined           = 0x%08Lx - 0x%08Lx\r\n"
    "\t HPS Boot ROM        = 0x%08Lx - 0x%08Lx\r\n"
    "\t Undefined           = 0x%08Lx - 0x%08Lx\r\n"
    "\t SCU and L2 Region   = 0x%08Lx - 0x%08Lx\r\n",
     (UINT64) GetMpuWindowDramBaseAddr(),
     (UINT64)(GetMpuWindowDramBaseAddr() + GetMpuWindowDramSize() - 1),
     (UINT64) GetMpuWindowDramBaseAddr() + GetMpuWindowDramSize(),
     (UINT64) ALT_FPGA_BRIDGE_H2F128_OFST - 1,
     (UINT64) ALT_FPGA_BRIDGE_H2F128_OFST,
     (UINT64) (UINT32) ALT_FPGA_BRIDGE_H2F128_UB_ADDR,
     (UINT64) 0xFC000000,
     (UINT64) 0xFEFFFFFF,
     (UINT64) 0xFF000000,
     (UINT64) 0xFF1FFFFF,
     (UINT64) ALT_FPGA_BRIDGE_LWH2F_OFST,
     (UINT64) (UINT32) ALT_FPGA_BRIDGE_LWH2F_UB_ADDR,
     (UINT64) (UINT32) ALT_FPGA_BRIDGE_LWH2F_UB_ADDR + 1,
     (UINT64) 0XFF800000 - 1,
     (UINT64) 0XFF800000,
     (UINT64) ALT_OCRAM_OFST - 1,
     (UINT64) ALT_OCRAM_OFST,
     (UINT64) (UINT32) ALT_OCRAM_UB_ADDR,
     (UINT64) 0XFFE40000,
     (UINT64) 0XFFFBFFFF,
     (UINT64) ALT_ROM_OFST,
     (UINT64) (UINT32) ALT_ROM_UB_ADDR,
     (UINT64) 0XFFFE0000,
     (UINT64) 0XFFFFBFFF,
     (UINT64) ALT_MPU_REGS_MPUSCU_OFST,
     (UINT64) (UINT32) ALT_L2_REGS_L2DBG_UB_ADDR
     );

}



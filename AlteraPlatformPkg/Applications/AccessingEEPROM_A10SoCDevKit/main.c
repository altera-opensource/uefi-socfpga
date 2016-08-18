/*****************************************************************************
 *
 * Copyright (c) 2016, Intel Corporation. All Rights Reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors
 * may be used to endorse or promote products derived from this software without
 * specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 *****************************************************************************/

#include <Library/BaseMemoryLib.h>
#include <Library/TimerLib.h>
#include <Library/IoLib.h>

#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "alt_i2c.h"
#include "socal/socal.h"
#include "socal/hps.h"



#define EMAC_ADDRESS_BYTES          6              // Number of bytes of EMAC Address
#define I2C_EEPROM_ADDRESS          (0xA2 >> 1)    // I2C address of EEPROM module
#define IC_CLK_FREQ                 50             // l4_sp_clk frequency in MHz
#define I2C_TIMEOUT                 10000          // Timeout value for I2C
#define MIN_SS_SCL_HIGHTIME         4000           // Min standard speed scl high period in ns
#define MIN_SS_SCL_LOWTIME          4700           // Min standard speed scl low period in ns
#define NANO_TO_MICRO               1000

/**
  Initialization of the I2C Controller

  @retval  EFI_SUCCESS  Successfully initialize the I2C controller
  @retval  Other        Failed to initialize the I2C controller
**/
EFI_STATUS
EFIAPI
I2cInit (
  VOID
  )
{
  UINT16  HighCount;
  UINT16  LowCount;

  //Disable I2C Controller
  printf ("INFO: Disable I2C Controller\n");
  MmioAndThenOr32 (ALT_I2C1_OFST +
                   ALT_I2C_EN_OFST,
                   ALT_I2C_EN_EN_CLR_MSK,
                   ALT_I2C_EN_EN_SET(ALT_I2C_EN_EN_E_DIS));

  //Slave mode disabled
  printf ("INFO: Disable Slave Mode\n");
  MmioAndThenOr32 (ALT_I2C1_OFST +
                   ALT_I2C_CON_OFST,
                   ALT_I2C_CON_IC_SLV_DIS_CLR_MSK,
                   ALT_I2C_CON_IC_SLV_DIS_SET(ALT_I2C_CON_IC_SLV_DIS_E_DIS));

  //Restart Mode Enabled
  printf ("INFO: Enable Restart Mode\n");
  MmioAndThenOr32 (ALT_I2C1_OFST +
                   ALT_I2C_CON_OFST,
                   ALT_I2C_CON_IC_RESTART_EN_CLR_MSK,
                   ALT_I2C_CON_IC_RESTART_EN_SET(ALT_I2C_CON_IC_RESTART_EN_E_EN));

  //Master Address mode 7bit
  printf ("INFO: Enable 7-bit Master Address Mode\n");
  MmioAndThenOr32 (ALT_I2C1_OFST +
                   ALT_I2C_CON_OFST,
                   ALT_I2C_CON_IC_10BITADDR_MST_CLR_MSK,
                   ALT_I2C_CON_IC_10BITADDR_MST_SET(ALT_I2C_CON_IC_10BITADDR_MST_E_MSTADDR7BIT));

  //Slave Address mode 7bit
  printf ("INFO: Enable 7-bit Slave Address Mode\n");
  MmioAndThenOr32 (ALT_I2C1_OFST +
                   ALT_I2C_CON_OFST,
                   ALT_I2C_CON_IC_10BITADDR_SLV_CLR_MSK,
                   ALT_I2C_CON_IC_10BITADDR_SLV_SET(ALT_I2C_CON_IC_10BITADDR_SLV_E_SLVADDR7BIT));

  //Speed mode
  printf ("INFO: Enable Standard Speed Mode\n");
  MmioAndThenOr32 (ALT_I2C1_OFST +
                   ALT_I2C_CON_OFST,
                   ALT_I2C_CON_SPEED_CLR_MSK,
                   ALT_I2C_CON_SPEED_SET(ALT_I2C_CON_SPEED_E_STANDARD));

  //Master mode enabled
  printf ("INFO: Enable Master Mode\n");
  MmioAndThenOr32 (ALT_I2C1_OFST +
                   ALT_I2C_CON_OFST,
                   ALT_I2C_CON_MST_MOD_CLR_MSK,
                   ALT_I2C_CON_MST_MOD_SET(ALT_I2C_CON_MST_MOD_E_EN));

  //Set target address register
  printf("INFO: Set EEPROM Slave Address: 0x%x\n", I2C_EEPROM_ADDRESS);
  MmioWrite32 (ALT_I2C1_OFST +
               ALT_I2C_TAR_OFST,
               ALT_I2C_TAR_IC_TAR_SET(I2C_EEPROM_ADDRESS));

  //Set I2c bus speed
  HighCount = ceil (MIN_SS_SCL_HIGHTIME * IC_CLK_FREQ) / NANO_TO_MICRO;
  printf ("INFO: Set Minimum SCL High Count: %d\n", HighCount);
  MmioWrite32 (ALT_I2C1_OFST +
               ALT_I2C_SS_SCL_HCNT_OFST,
               ALT_I2C_SS_SCL_HCNT_IC_SS_SCL_HCNT_SET(HighCount));

  LowCount  = ceil (MIN_SS_SCL_LOWTIME  * IC_CLK_FREQ) / NANO_TO_MICRO;
  printf ("INFO: Set Minimum SCL Low Count : %d\n", LowCount);
  MmioWrite32 (ALT_I2C1_OFST +
               ALT_I2C_SS_SCL_LCNT_OFST,
               ALT_I2C_SS_SCL_LCNT_IC_SS_SCL_LCNT_SET(LowCount));

  //Enable all interrupt
  printf ("INFO: Enable All Interrupts\n");
  MmioWrite32 (ALT_I2C1_OFST +
               ALT_I2C_INTR_MSK_OFST,
               ALT_I2C_INTR_MSK_M_RX_UNDER_SET_MSK|
               ALT_I2C_INTR_MSK_M_RX_OVER_SET_MSK|
               ALT_I2C_INTR_MSK_M_RX_FULL_SET_MSK|
               ALT_I2C_INTR_MSK_M_TX_OVER_SET_MSK|
               ALT_I2C_INTR_MSK_M_TX_EMPTY_SET_MSK|
               ALT_I2C_INTR_MSK_M_RD_REQ_SET_MSK|
               ALT_I2C_INTR_MSK_M_TX_ABRT_SET_MSK|
               ALT_I2C_INTR_MSK_M_RX_DONE_SET_MSK|
               ALT_I2C_INTR_MSK_M_ACTIVITY_SET_MSK|
               ALT_I2C_INTR_MSK_M_STOP_DET_SET_MSK|
               ALT_I2C_INTR_MSK_M_START_DET_SET_MSK|
               ALT_I2C_INTR_MSK_M_GEN_CALL_SET_MSK|
               ALT_I2C_INTR_MSK_M_RESTART_DET_SET_MSK|
               ALT_I2C_INTR_MSK_M_MST_ON_HOLD_SET_MSK);

  //Clear TX and RX FIFO Register
  printf ("INFO: Clear RX FIFO\n");
  MmioWrite32 (ALT_I2C1_OFST +
               ALT_I2C_RX_TL_OFST,
               ALT_I2C_RX_TL_RX_TL_CLR_MSK);

  printf ("INFO: Clear TX FIFO\n");
  MmioWrite32 (ALT_I2C1_OFST +
               ALT_I2C_TX_TL_OFST,
               ALT_I2C_TX_TL_TX_TL_CLR_MSK);

  //Re-enable I2C Controller
  printf ("INFO: Enable I2C Controller\n");
  MmioAndThenOr32 (ALT_I2C1_OFST +
                   ALT_I2C_EN_OFST,
                   ALT_I2C_EN_EN_CLR_MSK,
                   ALT_I2C_EN_EN_SET(ALT_I2C_EN_EN_E_EN));

  return EFI_SUCCESS;
}

/**
  Timeout counter for RX FIFO

  @retval  EFI_SUCCESS  RX Fifo is not empty
  @retval  Other        RX Fifo is empty
**/
EFI_STATUS
EFIAPI
PollingRxFifoNotEmpty (
  VOID
  )
{
  UINT32 Counter;
  UINT32 Data32;

  Counter = 0;
  do {
    Data32 = MmioRead32 (ALT_I2C1_OFST +
                         ALT_I2C_STAT_OFST);
    if (ALT_I2C_STAT_RFNE_GET(Data32) == ALT_I2C_STAT_RFNE_E_NOTEMPTY)
      break;
    MicroSecondDelay(10);
    Counter++;
  } while (Counter < I2C_TIMEOUT);

  if (Counter >= I2C_TIMEOUT)  return EFI_TIMEOUT;

  return EFI_SUCCESS;
}

/**
  Timeout counter for TX FIFO

  @retval  EFI_SUCCESS  TX Fifo is not empty
  @retval  Other        TX Fifo is empty
**/
EFI_STATUS
EFIAPI
PollingTxFifoNotEmpty (
  VOID
  )
{
  UINT32 Counter;
  UINT32 Data32;

  Counter = 0;
  do {
    Data32 = MmioRead32 (ALT_I2C1_OFST +
                         ALT_I2C_STAT_OFST);
    if (ALT_I2C_STAT_TFNF_GET(Data32)==ALT_I2C_STAT_TFNF_E_NOTFULL)
      break;
    MicroSecondDelay(10);
    Counter++;
  } while (Counter < I2C_TIMEOUT);

  if (Counter >= I2C_TIMEOUT)  return EFI_TIMEOUT;

  return EFI_SUCCESS;
}

/**
  Clear RX Fifo

  @retval  EFI_SUCCESS  Successfully clear RX Fifo
  @retval  Other        Failed to clear RX Fifo
**/
EFI_STATUS
EFIAPI
ClearRxFifo (
  VOID
  )
{
  UINT32 Counter;
  UINT32 Data32;

  Counter = 0;
  do {
    Data32 = MmioRead32 (ALT_I2C1_OFST +
                         ALT_I2C_STAT_OFST);
    if (ALT_I2C_STAT_RFNE_GET(Data32) == ALT_I2C_STAT_RFNE_E_NOTEMPTY) {
      Counter++;
      MmioRead32 (ALT_I2C1_OFST +
                  ALT_I2C_DATA_CMD_OFST);
    } else {
      break;
    }
  } while (Counter < I2C_TIMEOUT);

  if (Counter >= I2C_TIMEOUT)  return EFI_TIMEOUT;

  return EFI_SUCCESS;
}

/**
  Write high byte address, followed by low byte address to
  DATA_CMD register

  @retval  EFI_SUCCESS  Successfully write to DATA_CMD register
  @retval  Other        Failed to write to DATA_CMD register
**/
EFI_STATUS
EFIAPI
SetAddress (
  IN UINT32 Address,
  IN UINT32 AddrLength
  )
{
  UINT32 ByteAddress;

  //Set Master Write Register
  MmioWrite32 (ALT_I2C1_OFST +
               ALT_I2C_DATA_CMD_OFST,
               ALT_I2C_DATA_CMD_CMD_SET(ALT_I2C_DATA_CMD_CMD_E_WR));

  //Write the High Byte address, then Low Byte Address
  while (AddrLength) {
    AddrLength--;
    ByteAddress = (Address >> (AddrLength * 8)) & 0xff;
    MmioWrite32 (ALT_I2C1_OFST +
                 ALT_I2C_DATA_CMD_OFST,
                 ByteAddress);
  }

  return EFI_SUCCESS;
}

/**
  Timeout for Bus Busy

  @retval  EFI_SUCCESS  Bus is not busy
  @retval  Other        Bus is still busy
**/
EFI_STATUS
EFIAPI
PollingForBusBusy (
  VOID
  )
{
  UINT32 Data32;
  UINT32 Counter;

  Counter = 0;

  do {
    Data32 = MmioRead32 (ALT_I2C1_OFST +
                         ALT_I2C_STAT_OFST);
    if ((ALT_I2C_STAT_TFE_GET(Data32) == ALT_I2C_STAT_TFE_E_EMPTY) &&
        (ALT_I2C_STAT_MST_ACTIVITY_GET(Data32) == ALT_I2C_STAT_MST_ACTIVITY_E_IDLE))
      break;
    MicroSecondDelay(10);
    Counter++;
  } while (Counter < I2C_TIMEOUT);

  if (Counter >= I2C_TIMEOUT)  return EFI_TIMEOUT;

  return EFI_SUCCESS;
}

/**
  Transfer data finish

  @retval  EFI_SUCCESS  Successfully stop data transfer
  @retval  Other        Failed to stop data transfer
**/
EFI_STATUS
EFIAPI
StopDataTransfer (
  VOID
  )
{
  UINT32 Counter;
  UINT32 Data32;
  EFI_STATUS Status;
  Counter=0;
  do {
    Data32 = MmioRead32 (ALT_I2C1_OFST +
                         ALT_I2C_RAW_INTR_STAT_OFST);
    if (Data32 & ALT_I2C_RAW_INTR_STAT_STOP_DET_SET_MSK) {
      MmioRead32 (ALT_I2C1_OFST +
                  ALT_I2C_CLR_STOP_DET_OFST);
      break;
    }
    MicroSecondDelay(10);
    Counter++;
  } while (Counter < I2C_TIMEOUT);

  if (Counter >= I2C_TIMEOUT)  return EFI_TIMEOUT;

  Status = PollingForBusBusy();
  if (Status != EFI_SUCCESS)  return Status;

  return EFI_SUCCESS;
}

/**
  Read EEPROM

  @retval  EFI_SUCCESS  Successfully read from EEPROM
  @retval  Other        Failed to read from EEPROM
**/
EFI_STATUS
EFIAPI
EepromReadByte (
  IN UINT16 AddrLength,
  IN UINT16 Address,
  IN UINT8 BufferLength,
  OUT UINT8 *Buffer
  )
{
  UINT8*     ReadData;
  EFI_STATUS Status;

  //Wait for bus busy
  Status = PollingForBusBusy();
  if (Status != EFI_SUCCESS) {
    printf ("ERROR: Bus is Still Busy\n");
    return Status;
  }

  //Set address to read from
  Status = SetAddress(Address, AddrLength);
  if (Status != EFI_SUCCESS) {
    printf ("ERROR: Failed to Set Reading Address\n");
    return Status;
  }

  //Start reading
  ReadData = Buffer;
  while (BufferLength) {
    MmioWrite32 (ALT_I2C1_OFST +
                 ALT_I2C_DATA_CMD_OFST,
                 ALT_I2C_DATA_CMD_CMD_SET_MSK);
    if (BufferLength == 1) {
      MmioWrite32 (ALT_I2C1_OFST +
                   ALT_I2C_DATA_CMD_OFST,
                   ALT_I2C_DATA_CMD_STOP_SET_MSK);
    }
    //Check whether RX Fifo is Empty
    Status = PollingRxFifoNotEmpty();
    if (Status != EFI_SUCCESS) {
      printf("ERROR: RX FIFO is empty\r\n");
      return Status;
    }
    *ReadData = MmioRead32 (ALT_I2C1_OFST +
                            ALT_I2C_DATA_CMD_OFST);
    ReadData++;
    BufferLength--;
  }

  //Transfer finish
  Status = StopDataTransfer();
  if (Status != EFI_SUCCESS) {
    printf ("ERROR: Failed to Issue STOP Condition\r\n");
    return Status;
  }

  //Flush I2c RX FIFO
  Status = ClearRxFifo();
  if (Status != EFI_SUCCESS) {
    printf ("ERROR: Failed to Flush RX FIFO\r\n");
    return Status;
  }

  return EFI_SUCCESS;
}

/**
  Write EEPROM

  @retval  EFI_SUCCESS  Successfully write to EEPROM
  @retval  Other        Failed to write to EEPROM
**/
EFI_STATUS
EFIAPI
EepromWriteByte (
  IN UINT8 AddrLength,
  IN UINT16 Address,
  IN UINT8 BufferLength,
  IN UINT8* Buffer
  )
{
  EFI_STATUS Status;
  UINT8*     WriteData;

  //Wait for bus busy
  Status = PollingForBusBusy();
  if (Status != EFI_SUCCESS) {
    printf ("ERROR: Bus is Still Busy\r\n");
    return Status;
  }
  //Write Address to write to
  Status = SetAddress(Address, AddrLength);
  if (Status != EFI_SUCCESS) {
    printf ("ERROR: Failed to Set Address To Write\r\n");
    return Status;
  }

  //Start writing
  //Assign value to temporary buffer
  WriteData = Buffer;
  while (BufferLength) {
    //Check whether TX Fifo is Empty
    Status = PollingTxFifoNotEmpty();
    if (Status != EFI_SUCCESS) {
      printf ("ERROR: TX FIFO Is Empty\r\n");
      return Status;
    }
    MmioWrite32 (ALT_I2C1_OFST +
                 ALT_I2C_DATA_CMD_OFST,
                 *WriteData);
    if (BufferLength == 1) {
      MmioWrite32 (ALT_I2C1_OFST +
                   ALT_I2C_DATA_CMD_OFST,
                   ALT_I2C_DATA_CMD_STOP_SET_MSK);
    }
    MicroSecondDelay(10);
    BufferLength--;
    WriteData++;
  }

  //Transfer finish
  Status = StopDataTransfer();
  if (Status != EFI_SUCCESS) {
    printf ("ERROR: Failed to Issue STOP Condition\r\n");
    return Status;
  }

  //Flush I2c RX FIFO
  Status = ClearRxFifo();
  if (Status != EFI_SUCCESS) {
    printf ("ERROR: Failed to Flush RX FIFO\r\n");
    return Status;
  }

  return EFI_SUCCESS;
}

/**
  Program Entry Point

  @param [in] Argc  The number of arguments
  @param [in] Argv  The argument value array

  @retval  0        The application exited normally.
  @retval  Other    An error occurred.
**/
int
main (
  IN int Argc,
  IN char **Argv
  )
{
  UINT16     Address;
  UINT8      BufferData[EMAC_ADDRESS_BYTES];
  UINT8      Counter;
  UINT8      AddressLength;
  EFI_STATUS Status;
  UINT8      WriteData[EMAC_ADDRESS_BYTES];

  Address           = 0x174; //EEPROM Address to read EMAC Address
  AddressLength     = 2;     //the length of the address you want to read

  //Initialisation of I2C Controller
  Status = I2cInit();
  if (Status != EFI_SUCCESS) {
    printf ("ERROR: Failed to Initialize I2C Controller\r\n");
    return Status;
  }
  printf ("\nINFO: Writing to EEPROM EMAC Address 0x%04x, Size = %d bytes.\n", Address, EMAC_ADDRESS_BYTES);
  //Insert random data to be written into EEPROM Address
  printf ("\tWritten Data : 0x");
  for (Counter=0; Counter < EMAC_ADDRESS_BYTES; Counter++) {
    WriteData[Counter] = rand();
    printf("%02x", WriteData[Counter]);
  }
  printf ("\n");

  //Write Operation
  Status = EepromWriteByte(AddressLength, Address, EMAC_ADDRESS_BYTES, WriteData);
  if (Status != EFI_SUCCESS) {
    printf ("ERROR: Fail to write to EEPROM EMAC Address\t: 0x%04x\n", Address);
    return Status;
  }

  //Read Operation
  printf ("INFO: Reading from EEPROM EMAC Address 0x%04x, Size = %d bytes.\n", Address, EMAC_ADDRESS_BYTES);
  Status = EepromReadByte(AddressLength, Address, EMAC_ADDRESS_BYTES, BufferData);
  if (Status != EFI_SUCCESS) {
    printf ("ERROR: Fail to read from EEPROM Address\t: 0x%04x\n", Address);
    return Status;
  }

  //Verify read and write operation
  if (CompareMem(WriteData, BufferData, EMAC_ADDRESS_BYTES) != 0) {
    printf ("ERROR: Read Data and Write Data are Mismatched\r\n");
    return EFI_ABORTED;
  }

  //Print data received
  printf ("\tReceived Data: 0x");
  for (Counter = 0; Counter < EMAC_ADDRESS_BYTES; Counter++) {
    printf ("%02x", BufferData[Counter]);
  }
  printf ("\n");
  printf ("INFO: EEPROM Read and Write Success\n");

  return EFI_SUCCESS;
}

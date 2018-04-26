/** @file
  A shell application that triggers I2c control.

  Copyright (c) 2016 - 2018, Intel Corporation. All rights reserved.<BR>
  This program and the accompanying materials
  are licensed and made available under the terms and conditions of the BSD License
  which accompanies this distribution.  The full text of the license may be found at
  http://opensource.org/licenses/bsd-license.php

  THE PROGRAM IS DISTRIBUTED UNDER THE BSD LICENSE ON AN "AS IS" BASIS,
  WITHOUT WARRANTIES OR REPRESENTATIONS OF ANY KIND, EITHER EXPRESS OR IMPLIED.

**/

#include <Uefi.h>
#include <Library/BaseLib.h>
#include <Library/DebugLib.h>
#include <Library/BaseMemoryLib.h>
#include <Library/MemoryAllocationLib.h>
#include <Library/UefiBootServicesTableLib.h>
#include <Library/UefiRuntimeServicesTableLib.h>
#include <Library/UefiLib.h>
#include <Library/PrintLib.h>
#include <Library/TimerLib.h>
#include <Library/IoLib.h>
#include <AlteraPlatform.h>
#include <math.h>
#include "I2c.h"


extern UINTN  Argc;
extern CHAR16 **Argv;

extern I2C_CTLR_t  I2cCtrl;
UINT32 I2cSpeedinHz;

/* Min frequency during standard speed */
#define ALT_I2C_SS_MIN_SPEED        8000
/* Max frequency during standard speed */
#define ALT_I2C_SS_MAX_SPEED        100000
/* Min frequency during fast speed */
#define ALT_I2C_FS_MIN_SPEED        100000
/* Max frequency during fast speed */
#define ALT_I2C_FS_MAX_SPEED        400000
/* Default spike suppression limit during standard speed */
#define ALT_I2C_SS_DEFAULT_SPKLEN   11
/* Default spike suppression limit during fast speed */
#define ALT_I2C_FS_DEFAULT_SPKLEN   4

#define IC_CLK_FREQ                 100000000      // l4_sp_clk frequency in MHz
#define I2C_TIMEOUT                 10000          // Timeout value for I2C
#define MIN_SS_SCL_HIGHTIME         400            // Min standard speed scl high period 
#define MIN_SS_SCL_LOWTIME          470            // Min standard speed scl low period
#define NANO_TO_MICRO               1000
#define CONFIG_EXT_OSC1_FREQ        25000000

#define ALT_I2C_DIFF_LCNT_HCNT      70

/**
  Read I2c

  @retval  EFI_SUCCESS  Successfully read from I2c
  @retval  Other        Failed to read from I2c
**/
EFI_STATUS
EFIAPI
I2cReadByte (
  IN UINT32 BusIndex,
  IN UINT16 AddrLength,
  IN UINT16 Address,
  IN UINT8 BufferLength,
  OUT UINT8 *Buffer
  );

/**
  Initialization of the I2C Controller

  @retval  EFI_SUCCESS  Successfully initialize the I2C controller
  @retval  Other        Failed to initialize the I2C controller
**/
EFI_STATUS
EFIAPI
I2cInit (
  IN UINT32 BusIndex
  )
{
  UINT16  HighCount;
  UINT16  LowCount;

  //Disable I2C Controller
  MmioAndThenOr32 (BusIndex +
                   ALT_I2C_IC_ENABLE_OFST,
                   ALT_I2C_IC_ENABLE_ENABLE_CLR_MSK,
                   (UINT32)ALT_I2C_IC_ENABLE_STATUS_IC_EN_SET(ALT_I2C_IC_ENABLE_STATUS_IC_EN_E_DISABLED));  

  //Slave mode disabled
  MmioAndThenOr32 (BusIndex +
                   ALT_I2C_IC_CON_OFST,
                   ALT_I2C_IC_CON_IC_SLAVE_DISABLE_CLR_MSK,
                   ALT_I2C_IC_CON_IC_SLAVE_DISABLE_SET_MSK);

  //Restart Mode Enabled
  MmioAndThenOr32 (BusIndex +
                   ALT_I2C_IC_CON_OFST,
                   ALT_I2C_IC_CON_IC_RESTART_EN_CLR_MSK,
                   (UINT32)ALT_I2C_IC_CON_IC_RESTART_EN_SET(ALT_I2C_IC_CON_IC_RESTART_EN_E_ENABLED));

  //Master Address mode 7bit
  MmioAndThenOr32 (BusIndex +
                   ALT_I2C_IC_TAR_OFST,
                   ALT_I2C_IC_TAR_IC_10BITADDR_MASTER_CLR_MSK,
                   (UINT32)ALT_I2C_IC_TAR_IC_10BITADDR_MASTER_SET(ALT_I2C_IC_TAR_IC_10BITADDR_MASTER_E_ADDR_7BITS));

  //Slave Address mode 7bit
  MmioAndThenOr32 (BusIndex +
                   ALT_I2C_IC_CON_OFST,
                   ALT_I2C_IC_CON_IC_10BITADDR_SLAVE_CLR_MSK,
                   (UINT32)ALT_I2C_IC_CON_IC_10BITADDR_SLAVE_SET(ALT_I2C_IC_CON_IC_10BITADDR_SLAVE_E_ADDR_7BITS));

  //Speed mode
  MmioAndThenOr32 (BusIndex +
                   ALT_I2C_IC_CON_OFST,
                   ALT_I2C_IC_CON_SPEED_CLR_MSK,
                   (UINT32)ALT_I2C_IC_CON_SPEED_SET(ALT_I2C_IC_CON_SPEED_E_STANDARD));

  //Master mode enabled
  MmioAndThenOr32 (BusIndex +
                   ALT_I2C_IC_CON_OFST,
                   ALT_I2C_IC_CON_MASTER_MODE_CLR_MSK,
                   (UINT32)ALT_I2C_IC_CON_MASTER_MODE_SET(ALT_I2C_IC_CON_MASTER_MODE_E_ENABLED));

  //Set target address register
  MmioWrite32 (BusIndex +
               ALT_I2C_IC_TAR_OFST,
               (UINT32)ALT_I2C_IC_TAR_IC_TAR_SET(0x1c));

  //Set I2c bus speed
  HighCount = MIN_SS_SCL_HIGHTIME;
  MmioWrite32 (BusIndex +
               ALT_I2C_IC_SS_SCL_HCNT_OFST,
               (UINT32)ALT_I2C_IC_SS_SCL_HCNT_IC_SS_SCL_HCNT_SET(HighCount));

  LowCount  = MIN_SS_SCL_LOWTIME;
  MmioWrite32 (BusIndex +
               ALT_I2C_IC_SS_SCL_LCNT_OFST,
               (UINT32)ALT_I2C_IC_SS_SCL_LCNT_IC_SS_SCL_LCNT_SET(LowCount));
               
  //Enable all interrupt
  MmioAndThenOr32 (BusIndex +
                ALT_I2C_IC_INTR_MASK_OFST, 
                ALT_I2C_IC_INTR_MASK_M_RX_UNDER_CLR_MSK&
                ALT_I2C_IC_INTR_MASK_M_RX_OVER_CLR_MSK&
                ALT_I2C_IC_INTR_MASK_M_RX_FULL_CLR_MSK&
                ALT_I2C_IC_INTR_MASK_M_TX_OVER_CLR_MSK&
                ALT_I2C_IC_INTR_MASK_M_TX_EMPTY_CLR_MSK&
                ALT_I2C_IC_INTR_MASK_M_RD_REQ_CLR_MSK&
                ALT_I2C_IC_INTR_MASK_M_TX_ABRT_CLR_MSK&
                ALT_I2C_IC_INTR_MASK_M_RX_DONE_CLR_MSK&
                ALT_I2C_IC_INTR_MASK_M_ACTIVITY_CLR_MSK&
                ALT_I2C_IC_INTR_MASK_M_STOP_DET_CLR_MSK&
                ALT_I2C_IC_INTR_MASK_M_START_DET_CLR_MSK&
                ALT_I2C_IC_INTR_MASK_M_GEN_CALL_CLR_MSK&
                ALT_I2C_IC_INTR_MASK_M_RESTART_DET_CLR_MSK&
                ALT_I2C_IC_INTR_MASK_M_MASTER_ON_HOLD_CLR_MSK,
                0);

  //Clear TX and RX FIFO Register
  MmioWrite32 (BusIndex +
               ALT_I2C_IC_RX_TL_OFST,
               ALT_I2C_IC_RX_TL_RX_TL_CLR_MSK);

  MmioWrite32 (BusIndex +
               ALT_I2C_IC_TX_TL_OFST,
               ALT_I2C_IC_TX_TL_TX_TL_CLR_MSK);

  //Re-enable I2C Controller
  MmioAndThenOr32 (BusIndex +
                   ALT_I2C_IC_ENABLE_OFST,
                   ALT_I2C_IC_ENABLE_ENABLE_CLR_MSK,
                   (UINT32)ALT_I2C_IC_ENABLE_STATUS_IC_EN_SET(ALT_I2C_IC_ENABLE_STATUS_IC_EN_E_ENABLED));
                   
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
  IN UINT32 BusIndex
  )
{
  UINT32 Counter;
  UINT32 Data32;

  Counter = 0;
  do {
    Data32 = MmioRead32(BusIndex + ALT_I2C_IC_STATUS_OFST);
    if ( Data32 & ALT_I2C_IC_STATUS_RFNE_SET_MSK )
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
  IN UINT32 BusIndex
  )
{
  UINT32 Counter;
  UINT32 Data32;

  Counter = 0;
  do {
    Data32 = MmioRead32(BusIndex + ALT_I2C_IC_STATUS_OFST);
    if ( Data32 & ALT_I2C_IC_STATUS_TFNF_SET_MSK )
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
  IN UINT32 BusIndex
  )
{
  UINT32 Counter;
  UINT32 Data32;

  Counter = 0;
  do {
    Data32 = MmioRead32(BusIndex + ALT_I2C_IC_STATUS_OFST);
    if ( Data32 & ALT_I2C_IC_STATUS_RFNE_SET_MSK ) {
      Counter++;
      Data32 = MmioRead32(BusIndex + ALT_I2C_IC_DATA_CMD_OFST);
    } else {
      break;
    }
  } while (Counter < I2C_TIMEOUT);

  if (Counter >= I2C_TIMEOUT)  return EFI_TIMEOUT;

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
  IN UINT32 BusIndex
  )
{
  UINT32 Data32;
  UINT32 Counter;

  Counter = 0;

  do {
    Data32 = MmioRead32(BusIndex + ALT_I2C_IC_STATUS_OFST);
    if ((Data32 & ALT_I2C_IC_STATUS_TFE_SET_MSK) &&
        ((Data32 & ALT_I2C_IC_STATUS_MST_ACTIVITY_SET_MSK) == ALT_I2C_IC_STATUS_MST_ACTIVITY_E_IDLE))
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
  IN UINT32 BusIndex
  )
{
  UINT32 Counter;
  UINT32 Data32;
  EFI_STATUS Status;
  Counter=0;
  do {
    Data32 = MmioRead32(BusIndex + ALT_I2C_IC_RAW_INTR_STAT_OFST);
    if (Data32 & ALT_I2C_IC_RAW_INTR_STAT_STOP_DET_SET_MSK) {
      Data32 = MmioRead32(BusIndex + ALT_I2C_IC_CLR_STOP_DET_OFST);
      break;
    }
    MicroSecondDelay(10);
    Counter++;
  } while (Counter < I2C_TIMEOUT);

  if (Counter >= I2C_TIMEOUT)  return EFI_TIMEOUT;

  Status = PollingForBusBusy(BusIndex);
  if (Status != EFI_SUCCESS)  return Status;

  return EFI_SUCCESS;
}


/**
  Get current bus

  @retval  EFI_SUCCESS  Successfully process I2C command
  @retval  Other        Failed to process command
**/
EFI_STATUS
EFIAPI
I2cGetCurrentBus (
  IN UINT32 BusIndex
  )
{
  UINT32 Data32;
  
  Print(L"Current I2c addr %x\n", BusIndex); 
  Data32 = MmioRead32(BusIndex + ALT_I2C_IC_ENABLE_STATUS_OFST);  
    
  if (Data32 & ALT_I2C_IC_ENABLE_STATUS_IC_EN_SET_MSK){
     Print(L"I2c enabled\n");
  } else {
     Print(L"I2c disabled\n");       
  }

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
  IN UINT32 BusIndex,
  IN UINT32 Address,
  IN UINT32 AddrLength
  )
{
  UINT32 ByteAddress;

  //Set Master Write Register
  MmioWrite32 (BusIndex +
               ALT_I2C_IC_DATA_CMD_OFST,
               ALT_I2C_IC_DATA_CMD_CMD_SET(ALT_I2C_IC_DATA_CMD_CMD_E_WRITE));

  //Write the High Byte address, then Low Byte Address
  while (AddrLength) {
    AddrLength--;
    ByteAddress = (Address >> (AddrLength * 8)) & 0xff;
    MmioWrite32 (BusIndex +
                 ALT_I2C_IC_DATA_CMD_OFST,
                 ByteAddress);

  }

  return EFI_SUCCESS;
}

/**
  Get current bus speed

  @retval  EFI_SUCCESS  Successfully process I2C command
  @retval  Other        Failed to process command
**/
EFI_STATUS
EFIAPI
I2cGetCurrentBusSpeed (
  IN UINT32 BusIndex
  )
{
  UINT32 SpeedinHz;
  
  SpeedinHz = I2cSpeedinHz;
  Print(L"Current I2c speed is %x\n", SpeedinHz);   

  return EFI_SUCCESS;
}

/**
  Set current bus speed

  @retval  EFI_SUCCESS  Successfully process I2C command
  @retval  Other        Failed to process command
**/
EFI_STATUS
EFIAPI
I2cSetCurrentBusSpeed (
  IN UINT32 BusIndex,
  IN UINT32 SpeedinHz  
  )
{
  UINT16  HighCount;
  UINT16  LowCount;

  /* If speed is not standard or fast return range error */
  if ((SpeedinHz > ALT_I2C_FS_MAX_SPEED) || (SpeedinHz < ALT_I2C_SS_MIN_SPEED))
  {
    Print(L"ERROR: I2c speed out of range\n");
    return EFI_DEVICE_ERROR;
  }
  
  I2cSpeedinHz = SpeedinHz;
  if (SpeedinHz > ALT_I2C_FS_MIN_SPEED)
  {
    MmioAndThenOr32 (BusIndex +
                     ALT_I2C_IC_CON_OFST,
                     ALT_I2C_IC_CON_SPEED_CLR_MSK,
                     (UINT32)ALT_I2C_IC_CON_SPEED_SET(ALT_I2C_IC_CON_SPEED_E_FAST));      
    MmioAndThenOr32 (BusIndex +
                     ALT_I2C_IC_FS_SPKLEN_OFST,
                     ALT_I2C_IC_FS_SPKLEN_IC_FS_SPKLEN_CLR_MSK,
                     (UINT32)ALT_I2C_IC_FS_SPKLEN_IC_FS_SPKLEN_SET(ALT_I2C_FS_DEFAULT_SPKLEN));   
  }
  else
  {
    MmioAndThenOr32 (BusIndex +
                     ALT_I2C_IC_CON_OFST,
                     ALT_I2C_IC_CON_SPEED_CLR_MSK,
                     (UINT32)ALT_I2C_IC_CON_SPEED_SET(ALT_I2C_IC_CON_SPEED_E_STANDARD));   
    MmioAndThenOr32 (BusIndex +
                     ALT_I2C_IC_FS_SPKLEN_OFST,
                     ALT_I2C_IC_FS_SPKLEN_IC_FS_SPKLEN_CLR_MSK,
                     (UINT32)ALT_I2C_IC_FS_SPKLEN_IC_FS_SPKLEN_SET(ALT_I2C_SS_DEFAULT_SPKLEN));   
  }

  /* <lcount> = <internal clock> / 2 * <speed, Hz> */
  LowCount = IC_CLK_FREQ / (SpeedinHz << 1);

 
  /* hcount = <lcount> - 70 */
  HighCount = LowCount - ALT_I2C_DIFF_LCNT_HCNT;

  MmioWrite32 (BusIndex +
               ALT_I2C_IC_SS_SCL_HCNT_OFST,
               (UINT32)ALT_I2C_IC_SS_SCL_HCNT_IC_SS_SCL_HCNT_SET(HighCount));


  MmioWrite32 (BusIndex +
               ALT_I2C_IC_SS_SCL_LCNT_OFST,
               (UINT32)ALT_I2C_IC_SS_SCL_LCNT_IC_SS_SCL_LCNT_SET(LowCount));  
  
  return EFI_SUCCESS;
}

/**
  Initialization of the I2C Target Address

  @retval  EFI_SUCCESS  Successfully initialize the I2C controller
  @retval  Other        Failed to initialize the I2C controller
**/
EFI_STATUS
EFIAPI
I2cSetTargetAddress (
  IN UINT32 BusIndex,
  IN UINT32 DevAddr
  )
{ 
  //Set Target Address
  MmioWrite32 (BusIndex +
                 ALT_I2C_IC_TAR_OFST,
                (UINT32)ALT_I2C_IC_TAR_IC_TAR_SET(DevAddr)); 
                     
  return EFI_SUCCESS;
}

/**
  Read I2c

  @retval  EFI_SUCCESS  Successfully read from I2c
  @retval  Other        Failed to read from I2c
**/
EFI_STATUS
EFIAPI
I2cReadByte (
  IN UINT32 BusIndex,
  IN UINT16 AddrLength,
  IN UINT16 Address,
  IN UINT8 BufferLength,
  OUT UINT8 *Buffer
  )
{
  UINT8*     ReadData;
  EFI_STATUS Status;

  //Wait for bus busy
  Status = PollingForBusBusy(BusIndex);
  if (Status != EFI_SUCCESS) {
    return Status;
  }

  //Set address to read from
  Status = SetAddress(BusIndex, Address, AddrLength);
  if (Status != EFI_SUCCESS) {
    return Status;
  }

  //Start reading
  ReadData = Buffer;
  while (BufferLength) {
    MmioWrite32(BusIndex + 
                   ALT_I2C_IC_DATA_CMD_OFST, 
                   ALT_I2C_IC_DATA_CMD_CMD_SET_MSK);
    if (BufferLength == 1) {
      MmioWrite32(BusIndex + 
                     ALT_I2C_IC_DATA_CMD_OFST,
                     ALT_I2C_IC_DATA_CMD_STOP_SET_MSK);
    }
    //Check whether RX Fifo is Empty
    Status = PollingRxFifoNotEmpty(BusIndex);
    if (Status != EFI_SUCCESS) {
      return Status;
    }

    *ReadData = MmioRead32(BusIndex + ALT_I2C_IC_DATA_CMD_OFST);
    ReadData++;
    BufferLength--;
  }

  //Transfer finish
  Status = StopDataTransfer(BusIndex);
  if (Status != EFI_SUCCESS) {
    return Status;
  }

  //Flush I2c RX FIFO
  Status = ClearRxFifo(BusIndex);
  if (Status != EFI_SUCCESS) {
    return Status;
  }

  return EFI_SUCCESS;
}

/**
  Write I2c

  @retval  EFI_SUCCESS  Successfully write to I2c
  @retval  Other        Failed to write to I2c
**/
EFI_STATUS
EFIAPI
I2cWriteByte (
  IN UINT32 BusIndex,
  IN UINT16 AddrLength,
  IN UINT16 Address,
  IN UINT8 BufferLength,
  IN UINT8* Buffer
  )
{
  EFI_STATUS Status;
  UINT8*     WriteData;

  //Wait for bus busy
  Status = PollingForBusBusy(BusIndex);
  if (Status != EFI_SUCCESS) {
    return Status;
  }
  //Write Address to write to
  Status = SetAddress(BusIndex, Address, AddrLength);
  if (Status != EFI_SUCCESS) {
    return Status;
  }

  //Start writing
  //Assign value to temporary buffer
  WriteData = Buffer;
  while (BufferLength) {
    //Check whether TX Fifo is Empty
    Status = PollingTxFifoNotEmpty(BusIndex);
    if (Status != EFI_SUCCESS) {
      return Status;
    }
    MmioWrite32(BusIndex + 
                   ALT_I2C_IC_DATA_CMD_OFST, 
                   *WriteData);

    if (BufferLength == 1) {
      MmioWrite32(BusIndex + 
                     ALT_I2C_IC_DATA_CMD_OFST,
                     ALT_I2C_IC_DATA_CMD_STOP_SET_MSK);
    }
    MicroSecondDelay(10);
    BufferLength--;
    WriteData++;
  }

  //Transfer finish
  Status = StopDataTransfer(BusIndex);
  if (Status != EFI_SUCCESS) {
    return Status;
  }

  //Flush I2c RX FIFO
  Status = ClearRxFifo(BusIndex);
  if (Status != EFI_SUCCESS) {
    return Status;
  }

  return EFI_SUCCESS;
}

/** @file
  Memory Mapped I/O based 16550 UART Serial Port library functions

  Portions of the code modified by Altera to support SoC devices are licensed as follows:
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

  The original software modules are licensed as follows:

  Copyright (c) 2006 - 2014, Intel Corporation. All rights reserved.<BR>

  This program and the accompanying materials
  are licensed and made available under the terms and conditions of the BSD License
  which accompanies this distribution.  The full text of the license may be found at
  http://opensource.org/licenses/bsd-license.php

  THE PROGRAM IS DISTRIBUTED UNDER THE BSD 3 CLAUSE LICENSE ON AN "AS IS" BASIS,
  WITHOUT WARRANTIES OR REPRESENTATIONS OF ANY KIND, EITHER EXPRESS OR IMPLIED.

**/

#include <AlteraPlatform.h>
#include <Base.h>
#include <Library/SerialPortLib.h>
#include <Library/PcdLib.h>
#include <Library/IoLib.h>
#include <Library/BaseLib.h>

//
// Memory Mapped Register Width in number of bytes
//
#define R_WIDTH_IN_BYTES      4

//
// 16550 UART register offsets and bitfields
//
#define R_UART_RXBUF          0
#define R_UART_TXBUF          0
#define R_UART_BAUD_LOW       (0 * R_WIDTH_IN_BYTES)
#define R_UART_BAUD_HIGH      (1 * R_WIDTH_IN_BYTES)
#define R_UART_FCR            (2 * R_WIDTH_IN_BYTES)
#define   B_UART_FCR_FIFOE    BIT0
#define   B_UART_FCR_FIFO64   BIT5
#define R_UART_LCR            (3 * R_WIDTH_IN_BYTES)
#define   B_UART_LCR_DLAB     BIT7
#define R_UART_MCR            (4 * R_WIDTH_IN_BYTES)
#define   B_UART_MCR_RTS      BIT1
#define R_UART_LSR            (5 * R_WIDTH_IN_BYTES)
#define   B_UART_LSR_RXRDY    BIT0
#define   B_UART_LSR_TXRDY    BIT5
#define   B_UART_LSR_TEMT     BIT6
#define R_UART_MSR            (6 * R_WIDTH_IN_BYTES)
#define   B_UART_MSR_CTS      BIT4
#define   B_UART_MSR_DSR      BIT5

/**
  Read an 8-bit 16550 register from the MMIO space.
  The parameter Offset is added to the base address of the 16550 registers that is specified
  by PcdSerialRegisterBase.

  @param  Base    The base address register of UART device.
  @param  Offset  The offset of the 16550 register to read.

  @return The value read from the 16550 register.

**/
UINT8
SerialPortReadRegister (
  UINTN  Base,
  UINTN  Offset
  )
{
  return (UINT8) MmioRead32 (Base + Offset);
}

/**
  Write an 8-bit 16550 register to the MMIO space.
  The parameter Offset is added to the base address of the 16550 registers that is specified
  by PcdSerialRegisterBase.

  @param  Base    The base address register of UART device.
  @param  Offset  The offset of the 16550 register to write.
  @param  Value   The value to write to the 16550 register specified by Offset.

  @return The value written to the 16550 register.

**/
UINT8
SerialPortWriteRegister (
  UINTN  Base,
  UINTN  Offset,
  UINT8  Value
  )
{
  return (UINT8) MmioWrite32 (Base + Offset, (UINT32) Value);
}

/**
  Retrieve the I/O or MMIO base address register for the PCI UART device.

  This function assumes Root Bus Numer is Zero, and enables I/O and MMIO in PCI UART
  Device if they are not already enabled.

  @return  The base address register of the UART device.

**/
UINTN
GetSerialRegisterBase (
  VOID
  )
{
  return (UINTN)PcdGet64 (PcdSerialRegisterBase);
}

/**
  Return whether the hardware flow control signal allows writing.

  @param  SerialRegisterBase The base address register of UART device.

  @retval TRUE  The serial port is writable.
  @retval FALSE The serial port is not writable.
**/
BOOLEAN
SerialPortWritable (
  UINTN  SerialRegisterBase
  )
{
  if (PcdGetBool (PcdSerialUseHardwareFlowControl)) {
    if (PcdGetBool (PcdSerialDetectCable)) {
      //
      // Wait for both DSR and CTS to be set
      //   DSR is set if a cable is connected.
      //   CTS is set if it is ok to transmit data
      //
      //   DSR  CTS  Description                               Action
      //   ===  ===  ========================================  ========
      //    0    0   No cable connected.                       Wait
      //    0    1   No cable connected.                       Wait
      //    1    0   Cable connected, but not clear to send.   Wait
      //    1    1   Cable connected, and clear to send.       Transmit
      //
      return (BOOLEAN) ((SerialPortReadRegister (SerialRegisterBase, R_UART_MSR) & (B_UART_MSR_DSR | B_UART_MSR_CTS)) == (B_UART_MSR_DSR | B_UART_MSR_CTS));
    } else {
      //
      // Wait for both DSR and CTS to be set OR for DSR to be clear.
      //   DSR is set if a cable is connected.
      //   CTS is set if it is ok to transmit data
      //
      //   DSR  CTS  Description                               Action
      //   ===  ===  ========================================  ========
      //    0    0   No cable connected.                       Transmit
      //    0    1   No cable connected.                       Transmit
      //    1    0   Cable connected, but not clear to send.   Wait
      //    1    1   Cable connected, and clar to send.        Transmit
      //
      return (BOOLEAN) ((SerialPortReadRegister (SerialRegisterBase, R_UART_MSR) & (B_UART_MSR_DSR | B_UART_MSR_CTS)) != (B_UART_MSR_DSR));
    }
  }

  return TRUE;
}

/**
  Initialize the serial device hardware.

  If no initialization is required, then return RETURN_SUCCESS.
  If the serial device was successfully initialized, then return RETURN_SUCCESS.
  If the serial device could not be initialized, then return RETURN_DEVICE_ERROR.

  @retval RETURN_SUCCESS        The serial device was initialized.
  @retval RETURN_DEVICE_ERROR   The serial device could not be initialized.

**/
RETURN_STATUS
EFIAPI
SerialPortInitialize (
  VOID
  )
{
  UINTN          SerialRegisterBase;
  UINT32         Divisor;
  UINT32         CurrentDivisor;
  BOOLEAN        Initialized;
  UINT32         Data32;
  UINT32         l4_sp_clk_InMhz;
  UINT32         SerialClockRateInHz;

  // ClockManager.c stored l4_sp clock frequency in MHz at the 8th register of isw_handoff[8]
  // The SoCFPGA's HPS UART IP ref clock is from l4_sp clock
  // Get the ref clock frequency to calculate the divisor for baud generator
  Data32 = MmioRead32 (ALT_SYSMGR_ROM_OFST + ALT_SYSMGR_ROM_ISW_HANDOFF_OFST + ISW_HANDOFF_SLOT8_L4_SP_CLK_IN_MHZ_OFST);
  if (Data32 != 0) {
    l4_sp_clk_InMhz = ALT_SYSMGR_ROM_ISW_HANDOFF_ISW_HANDOFF_GET(Data32);
    SerialClockRateInHz = l4_sp_clk_InMhz * 1000000;
  } else {
    // Fallback to PcdSerialClockRate when isw_handoff[8] does not contain valid data.
    SerialClockRateInHz = PcdGet32 (PcdSerialClockRate);
  }
  //
  // Calculate divisor for baud generator
  //    Ref_Clk_Rate / (Baud_Rate * 16)
  //
  Divisor = SerialClockRateInHz / (PcdGet32 (PcdSerialBaudRate) * 16);
  if ((SerialClockRateInHz % (PcdGet32 (PcdSerialBaudRate) * 16)) >= PcdGet32 (PcdSerialBaudRate) * 8) {
    Divisor++;
  }

  //
  // Get the base address of the serial port in MMIO space
  //
  SerialRegisterBase = GetSerialRegisterBase ();
  if (SerialRegisterBase ==0) {
    return RETURN_DEVICE_ERROR;
  }

  //
  // See if the serial port is already initialized
  //
  Initialized = TRUE;
  if ((SerialPortReadRegister (SerialRegisterBase, R_UART_LCR) & 0x3F) != (PcdGet8 (PcdSerialLineControl) & 0x3F)) {
    Initialized = FALSE;
  }
  SerialPortWriteRegister (SerialRegisterBase, R_UART_LCR, (UINT8)(SerialPortReadRegister (SerialRegisterBase, R_UART_LCR) | B_UART_LCR_DLAB));
  CurrentDivisor =  SerialPortReadRegister (SerialRegisterBase, R_UART_BAUD_HIGH) << 8;
  CurrentDivisor |= (UINT32) SerialPortReadRegister (SerialRegisterBase, R_UART_BAUD_LOW);
  SerialPortWriteRegister (SerialRegisterBase, R_UART_LCR, (UINT8)(SerialPortReadRegister (SerialRegisterBase, R_UART_LCR) & ~B_UART_LCR_DLAB));
  if (CurrentDivisor != Divisor) {
    Initialized = FALSE;
  }
  if (Initialized) {
    return RETURN_SUCCESS;
  }

  //
  // Wait for the serial port to be ready.
  // Verify that both the transmit FIFO and the shift register are empty.
  //
  while ((SerialPortReadRegister (SerialRegisterBase, R_UART_LSR) & (B_UART_LSR_TEMT | B_UART_LSR_TXRDY)) != (B_UART_LSR_TEMT | B_UART_LSR_TXRDY));

  //
  // Configure baud rate
  //
  SerialPortWriteRegister (SerialRegisterBase, R_UART_LCR, B_UART_LCR_DLAB);
  SerialPortWriteRegister (SerialRegisterBase, R_UART_BAUD_HIGH, (UINT8) (Divisor >> 8));
  SerialPortWriteRegister (SerialRegisterBase, R_UART_BAUD_LOW, (UINT8) (Divisor & 0xff));

  //
  // Clear DLAB and configure Data Bits, Parity, and Stop Bits.
  // Strip reserved bits from PcdSerialLineControl
  //
  SerialPortWriteRegister (SerialRegisterBase, R_UART_LCR, (UINT8)(PcdGet8 (PcdSerialLineControl) & 0x3F));

  //
  // Enable and reset FIFOs
  // Strip reserved bits from PcdSerialFifoControl
  //
  SerialPortWriteRegister (SerialRegisterBase, R_UART_FCR, 0x00);
  SerialPortWriteRegister (SerialRegisterBase, R_UART_FCR, (UINT8)(PcdGet8 (PcdSerialFifoControl) & (B_UART_FCR_FIFOE | B_UART_FCR_FIFO64)));

  //
  // Put Modem Control Register(MCR) into its reset state of 0x00.
  //
  SerialPortWriteRegister (SerialRegisterBase, R_UART_MCR, 0x00);

  return RETURN_SUCCESS;
}

/**
  Write data from buffer to serial device.

  Writes NumberOfBytes data bytes from Buffer to the serial device.
  The number of bytes actually written to the serial device is returned.
  If the return value is less than NumberOfBytes, then the write operation failed.

  If Buffer is NULL, then ASSERT().

  If NumberOfBytes is zero, then return 0.

  @param  Buffer           Pointer to the data buffer to be written.
  @param  NumberOfBytes    Number of bytes to written to the serial device.

  @retval 0                NumberOfBytes is 0.
  @retval >0               The number of bytes written to the serial device.
                           If this value is less than NumberOfBytes, then the read operation failed.

**/
UINTN
EFIAPI
SerialPortWrite (
  IN UINT8     *Buffer,
  IN UINTN     NumberOfBytes
  )
{
  UINTN  SerialRegisterBase;
  UINTN  Result;
  UINTN  Index;
  UINTN  FifoSize;

  if (Buffer == NULL) {
    return 0;
  }

  SerialRegisterBase = GetSerialRegisterBase ();
  if (SerialRegisterBase ==0) {
    return 0;
  }

  if (NumberOfBytes == 0) {
    //
    // Flush the hardware
    //

    //
    // Wait for both the transmit FIFO and shift register empty.
    //
    while ((SerialPortReadRegister (SerialRegisterBase, R_UART_LSR) & (B_UART_LSR_TEMT | B_UART_LSR_TXRDY)) != (B_UART_LSR_TEMT | B_UART_LSR_TXRDY));

    //
    // Wait for the hardware flow control signal
    //
    while (!SerialPortWritable (SerialRegisterBase));
    return 0;
  }

  //
  // Compute the maximum size of the Tx FIFO
  //
  FifoSize = 1;
  if ((PcdGet8 (PcdSerialFifoControl) & B_UART_FCR_FIFOE) != 0) {
    if ((PcdGet8 (PcdSerialFifoControl) & B_UART_FCR_FIFO64) == 0) {
      FifoSize = 16;
    } else {
      FifoSize = PcdGet32 (PcdSerialExtendedTxFifoSize);
    }
  }

  Result = NumberOfBytes;
  while (NumberOfBytes != 0) {
    //
    // Wait for the serial port to be ready, to make sure both the transmit FIFO
    // and shift register empty.
    //
    while ((SerialPortReadRegister (SerialRegisterBase, R_UART_LSR) & B_UART_LSR_TEMT) == 0);

    //
    // Fill then entire Tx FIFO
    //
    for (Index = 0; Index < FifoSize && NumberOfBytes != 0; Index++, NumberOfBytes--, Buffer++) {
      //
      // Wait for the hardware flow control signal
      //
      while (!SerialPortWritable (SerialRegisterBase));

      //
      // Write byte to the transmit buffer.
      //
      SerialPortWriteRegister (SerialRegisterBase, R_UART_TXBUF, *Buffer);
    }
  }
  return Result;
}

/**
  Reads data from a serial device into a buffer.

  @param  Buffer           Pointer to the data buffer to store the data read from the serial device.
  @param  NumberOfBytes    Number of bytes to read from the serial device.

  @retval 0                NumberOfBytes is 0.
  @retval >0               The number of bytes read from the serial device.
                           If this value is less than NumberOfBytes, then the read operation failed.

**/
UINTN
EFIAPI
SerialPortRead (
  OUT UINT8     *Buffer,
  IN  UINTN     NumberOfBytes
  )
{
  UINTN  SerialRegisterBase;
  UINTN  Result;
  UINT8  Mcr;

  if (NULL == Buffer) {
    return 0;
  }

  SerialRegisterBase = GetSerialRegisterBase ();
  if (SerialRegisterBase ==0) {
    return 0;
  }

  Mcr = (UINT8)(SerialPortReadRegister (SerialRegisterBase, R_UART_MCR) & ~B_UART_MCR_RTS);

  for (Result = 0; NumberOfBytes-- != 0; Result++, Buffer++) {
    //
    // Wait for the serial port to have some data.
    //
    while ((SerialPortReadRegister (SerialRegisterBase, R_UART_LSR) & B_UART_LSR_RXRDY) == 0) {
      if (PcdGetBool (PcdSerialUseHardwareFlowControl)) {
        //
        // Set RTS to let the peer send some data
        //
        SerialPortWriteRegister (SerialRegisterBase, R_UART_MCR, (UINT8)(Mcr | B_UART_MCR_RTS));
      }
    }
    if (PcdGetBool (PcdSerialUseHardwareFlowControl)) {
      //
      // Clear RTS to prevent peer from sending data
      //
      SerialPortWriteRegister (SerialRegisterBase, R_UART_MCR, Mcr);
    }

    //
    // Read byte from the receive buffer.
    //
    *Buffer = SerialPortReadRegister (SerialRegisterBase, R_UART_RXBUF);
  }

  return Result;
}


/**
  Polls a serial device to see if there is any data waiting to be read.

  Polls aserial device to see if there is any data waiting to be read.
  If there is data waiting to be read from the serial device, then TRUE is returned.
  If there is no data waiting to be read from the serial device, then FALSE is returned.

  @retval TRUE             Data is waiting to be read from the serial device.
  @retval FALSE            There is no data waiting to be read from the serial device.

**/
BOOLEAN
EFIAPI
SerialPortPoll (
  VOID
  )
{
  UINTN  SerialRegisterBase;

  SerialRegisterBase = GetSerialRegisterBase ();
  if (SerialRegisterBase ==0) {
    return FALSE;
  }

  //
  // Read the serial port status
  //
  if ((SerialPortReadRegister (SerialRegisterBase, R_UART_LSR) & B_UART_LSR_RXRDY) != 0) {
    if (PcdGetBool (PcdSerialUseHardwareFlowControl)) {
      //
      // Clear RTS to prevent peer from sending data
      //
      SerialPortWriteRegister (SerialRegisterBase, R_UART_MCR, (UINT8)(SerialPortReadRegister (SerialRegisterBase, R_UART_MCR) & ~B_UART_MCR_RTS));
    }
    return TRUE;
  }

  if (PcdGetBool (PcdSerialUseHardwareFlowControl)) {
    //
    // Set RTS to let the peer send some data
    //
    SerialPortWriteRegister (SerialRegisterBase, R_UART_MCR, (UINT8)(SerialPortReadRegister (SerialRegisterBase, R_UART_MCR) | B_UART_MCR_RTS));
  }

  return FALSE;
}

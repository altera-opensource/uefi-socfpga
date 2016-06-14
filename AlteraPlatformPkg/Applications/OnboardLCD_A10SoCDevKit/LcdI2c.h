/*****************************************************************************
 *
 * Copyright 2016 Altera Corporation. All Rights Reserved.
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

#ifndef __LCD_I2C_H
#define __LCD_I2C_H

// Command lists
typedef enum
{
  LcdDisplayOn = 0,
  LcdDisplayOff,
  LcdSetCursorPosition,
  LcdCursorHome,
  LcdUnderlineCursorOn,
  LcdUnderlineCursorOff,
  LcdMoveCursorLeftOne,
  LcdMoveCursorRightOne,
  LcdBlinkingCursorOn,
  LcdBlinkingCursorOff,
  LcdBackspace,
  LcdEraseScreen,
  LcdSetContrast,
  LcdSetBacklightBrighness,
  LcdLoadCustomCharacter,
  LcdShiftDisplayLeftOne,
  LcdShiftDisplayRightOne,
  LcdSetBaudRateRs232,
  LcdSetI2cAddress
} LcdCmdList;

/**
  This function conduct the initialisation of the LCD and I2C
  controller.
  The initialization process includes:
  - Initialize the I2C controller instance
  - Enable the I2C Controller
  - Get the present configurations of I2C controller in master mode
  - Get the speed of the I2C master configuration
  - Set the speed of the I2C master to 8000
  - Set the rx and tx hold time to 15
  - Set the I2C to LCD bus address
**/
EFI_STATUS
EFIAPI
InitLcd (
  ALT_I2C_DEV_t *I2cDev,
  ALT_I2C_CTLR_t I2cControllerNumber
  );

/**
  This function will clear the LCD Screen. The command line for the
  clear screen operation is {0xfe, 0x51}

  @retval  EFI_SUCCESS  The LCD backlight was turned on successfully
  @retval  Other        Failed to turn on the LCD backlight
**/
EFI_STATUS
EFIAPI
LcdClearScreen (
  ALT_I2C_DEV_t   *I2cDev
  );

/**
  This function sets the position of the cursor on LCD. The command
  used to set cursor is {0xfe,  0x51}

  Row = Y; (Option: 0-1)
  Column = X; (Option: 0-15)

  @retval  EFI_SUCCESS  The position of the cursor was set successfully
  @retval  Other        Failed to set the position of the LCD
**/
EFI_STATUS
EFIAPI
LcdSetCursor (
  ALT_I2C_DEV_t *I2cDev,
  UINT8  Y,
  UINT8  X
  );

 /**
  Display character on LCD

  @retval  EFI_SUCCESS  Characters on LCD was displayed successfully
  @retval  Other        Failed to display characters on LCD
**/
EFI_STATUS
EFIAPI
LcdPrint (
  ALT_I2C_DEV_t   *I2cDev,
  IN  CONST CHAR8  *Msg
  );

#endif

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

#include <Library/DebugLib.h>
#include <Library/TimerLib.h>
#include <stdio.h>
#include <string.h>

#include <hwlib.h>
#include <alt_i2c.h>
#include "LcdI2c.h"

//
// Define constant parameters
//
#define ESC_CHAR          0xfe        // Reserved character
#define I2C_LCD_ADDRESS  (0x50 >>1)   // I2C to LCD module address
#define LCD_CURSOR_L1     0x00        // LCD set cursor position for line 1
#define LCD_CURSOR_L2     0x40        // LCD set cursor position for line 2
#define LCD_COLLENGTH     16          // LCD column length
#define LCD_SPEED         8000        // I2C speed to access LCD module
#define SDA_HOLD_TIME     15          // SDA hold time in ic_clk period

//
// Descriptions for all supported commands
//
UINT8 LcdCommand[] =
{
  0x41,   // LCD_COMMAND_DISPLAY_ON
  0x42,   // LCD_COMMAND_DISPLAY_OFF
  0x45,   // LCD_COMMAND_SET_CURSOR
  0x46,   // LCD_COMMAND_CURSOR_HOME
  0x47,   // LCD_COMMAND_UNDERLINE_CURSOR_ON
  0x48,   // LCD_COMMAND_UNDERLINE_CURSOR_OFF
  0x49,   // LCD_COMMAND_MOVE_CURSOR_LEFT_ONE_PLACE
  0x4A,   // LCD_COMMAND_MOVE_CURSOR_RIGHT_ONE_PLACE
  0x4B,   // LCD_COMMAND_BLINKING_CURSOR_ON
  0x4C,   // LCD_COMMAND_BLINKING_CURSOR_OFF
  0x4E,   // LCD_COMMAND_BACKSPACE
  0x51,   // LCD_COMMAND_CLEAR_SCREEN
  0x52,   // LCD_COMMAND_SET_CONTRAST
  0x53,   // LCD_COMMAND_SET_BACKLIGHT_BRIGHTNESS
  0x54,   // LCD_COMMAND_LOAD_CUSTOM_CHARACTER
  0x55,   // LCD_COMMAND_MOVE_DISPLAY_ONE_PLACE_TO_THE_LEFT
  0x56,   // LCD_COMMAND_MOVE_DISPLAY_ONE_PLACE_TO_THE_RIGHT
  0x61,   // LCD_COMMAND_CHANGE_RS_232_BAUD_RATE
  0x62,   // LCD_COMMAND_CHANGE_I2C_ADDRESS
  0x70,   // LCD_COMMAND_DISPLAY_FIRMWARE_VERSION_NUMBER
  0x71,   // LCD_COMMAND_DISPLAY_RS_232_BAUD_RATE
  0x72    // LCD_COMMAND_DISPLAY_I2C_ADDRESS
};

UINTN LcdCommandDelay[] =
{
  100,    // LCD_COMMAND_DISPLAY_ON
  100,    // LCD_COMMAND_DISPLAY_OFF
  100,    // LCD_COMMAND_SET_CURSOR
  1500,   // LCD_COMMAND_CURSOR_HOME
  1500,   // LCD_COMMAND_UNDERLINE_CURSOR_ON
  1500,   // LCD_COMMAND_UNDERLINE_CURSOR_OFF
  100,    // LCD_COMMAND_MOVE_CURSOR_LEFT_ONE_PLACE
  100,    // LCD_COMMAND_MOVE_CURSOR_RIGHT_ONE_PLACE
  100,    // LCD_COMMAND_BLINKING_CURSOR_ON
  100,    // LCD_COMMAND_BLINKING_CURSOR_OFF
  100,    // LCD_COMMAND_BACKSPACE
  2000,   // LCD_COMMAND_CLEAR_SCREEN
  500,    // LCD_COMMAND_SET_CONTRAST
  100,    // LCD_COMMAND_SET_BACKLIGHT_BRIGHTNESS
  200,    // LCD_COMMAND_LOAD_CUSTOM_CHARACTER
  100,    // LCD_COMMAND_MOVE_DISPLAY_ONE_PLACE_TO_THE_LEFT
  100,    // LCD_COMMAND_MOVE_DISPLAY_ONE_PLACE_TO_THE_RIGHT
  3000,   // LCD_COMMAND_CHANGE_RS_232_BAUD_RATE
  3000,   // LCD_COMMAND_CHANGE_I2C_ADDRESS
  4000,   // LCD_COMMAND_DISPLAY_FIRMWARE_VERSION_NUMBER
  10000,  // LCD_COMMAND_DISPLAY_RS_232_BAUD_RATE
  4000    // LCD_COMMAND_DISPLAY_I2C_ADDRESS
};

/**
  Initialize the LCD

  @retval  EFI_SUCCESS  The LCD was initialized successfully
  @retval  Other        Failed to initialized the LCD
**/
EFI_STATUS
EFIAPI
InitLcd (
  ALT_I2C_DEV_t *I2cDev,
  ALT_I2C_CTLR_t I2cControllerNumber
  )
{
  EFI_STATUS               Status;
  ALT_STATUS_CODE          AltStatus;
  ALT_I2C_MASTER_CONFIG_t  Cfg;
  UINT32                   Speed;

  DEBUG (( DEBUG_INFO,
           "Initializing the LCD\n" ));

  AltStatus = alt_i2c_init ( I2cControllerNumber, I2cDev );
  if ( AltStatus != ALT_E_SUCCESS ) {
    DEBUG (( DEBUG_ERROR,
             "ERROR - Failed to initialise the I2C controller, Status: %r\r\n",
             AltStatus ));
    return EFI_ABORTED;
  }

  AltStatus = alt_i2c_enable ( I2cDev );
  if ( AltStatus != ALT_E_SUCCESS ) {
    DEBUG (( DEBUG_ERROR,
             "ERROR - Failed to enable the I2C controller, Status: %r\r\n",
             AltStatus ));
    return EFI_ABORTED;
  }

  AltStatus = alt_i2c_master_config_get ( I2cDev, &Cfg );
  if ( AltStatus != ALT_E_SUCCESS ) {
    DEBUG (( DEBUG_ERROR,
             "ERROR - Failed to get I2C present configuration, Status: %r\r\n",
             AltStatus ));
    return EFI_ABORTED;
  }

  AltStatus = alt_i2c_master_config_speed_get ( I2cDev, &Cfg, &Speed );
  if ( AltStatus != ALT_E_SUCCESS ) {
    DEBUG (( DEBUG_ERROR,
             "ERROR - Failed to get initial speed of I2C, Status: %r\r\n",
             AltStatus ));
    return EFI_ABORTED;
  } else {
    DEBUG (( DEBUG_INFO,
             "INFO: Current Speed of I2C = %d\n", (int)Speed ));
  }

  AltStatus = alt_i2c_master_config_speed_set ( I2cDev, &Cfg, LCD_SPEED );
  if ( AltStatus != ALT_E_SUCCESS ) {
    DEBUG (( DEBUG_ERROR,
             "ERROR - Failed to set speed of I2C, Status: %r\r\n",
             AltStatus ));
    return EFI_ABORTED;
  }

  AltStatus = alt_i2c_master_config_speed_get ( I2cDev, &Cfg, &Speed );
  if ( AltStatus != ALT_E_SUCCESS ) {
    DEBUG (( DEBUG_ERROR,
             "ERROR - Failed to get current speed of I2C, Status: %r\r\n",
             AltStatus ));
    return EFI_ABORTED;
  }

  Cfg.addr_mode      = ALT_I2C_ADDR_MODE_7_BIT;
  Cfg.restart_enable = ALT_E_TRUE;

  DEBUG (( DEBUG_INFO,
           "INFO: Set Speed of I2C to %d\n", (int)Speed ));

  AltStatus = alt_i2c_sda_tx_hold_time_set ( I2cDev, SDA_HOLD_TIME );
  if ( AltStatus != ALT_E_SUCCESS ) {
    DEBUG (( DEBUG_ERROR,
             "ERROR - Failed to set the sda hold time tx, Status: %r\r\n",
             AltStatus ));
    return EFI_ABORTED;
  }

  AltStatus = alt_i2c_sda_rx_hold_time_set ( I2cDev, SDA_HOLD_TIME );
  if ( AltStatus != ALT_E_SUCCESS ) {
     DEBUG (( DEBUG_ERROR,
              "ERROR - Failed to set the sda hold time rx, Status: %r\r\n",
              AltStatus ));
    return EFI_ABORTED;
  }

  AltStatus = alt_i2c_master_target_set ( I2cDev, I2C_LCD_ADDRESS );
  if ( AltStatus != ALT_E_SUCCESS ) {
    DEBUG (( DEBUG_ERROR,
             "ERROR - Failed to set the I2C to LCD bus address, Status: %r\r\n",
             AltStatus ));
    return EFI_ABORTED;
  }

  return EFI_SUCCESS;
}

/**
  Clear LCD Screen

  @retval  EFI_SUCCESS  The LCD backlight was turned on successfully
  @retval  Other        Failed to turn on the LCD backlight
**/
EFI_STATUS
EFIAPI
LcdClearScreen (
  ALT_I2C_DEV_t   *I2cDev
  )
{
  EFI_STATUS       Status;
  ALT_STATUS_CODE  AltStatus;
  UINT8            LcdCmdDesc;
  UINT8            Data[2];

  LcdCmdDesc = LcdCommand[LcdEraseScreen];

  Data[0]    = ESC_CHAR;
  Data[1]    = LcdCmdDesc;

  AltStatus = alt_i2c_master_transmit ( I2cDev,
                                        Data, sizeof(Data),
                                        false, true );
  if ( AltStatus != ALT_E_SUCCESS ) {
    DEBUG (( DEBUG_ERROR,
             "ERROR - Failed to transmit LcdEraseScreen to LCD, Status: %r\r\n",
             AltStatus ));
    return EFI_ABORTED;
  }

  MicroSecondDelay( LcdCommandDelay[LcdEraseScreen] );

  return EFI_SUCCESS;
}

/**
  Set the position of the cursor on LCD

  @retval  EFI_SUCCESS  The position of the cursor was set successfully
  @retval  Other        Failed to set the position of the LCD
**/
EFI_STATUS
EFIAPI
LcdSetCursor (
  ALT_I2C_DEV_t *I2cDev,
  UINT8  Y,
  UINT8  X
  )
{
  ALT_STATUS_CODE  AltStatus;
  EFI_STATUS       Status;
  UINT8            LcdCmdDesc;
  UINT8            Data[3];
  UINT8            Row;
  UINT8            Position;

  LcdCmdDesc = LcdCommand[LcdSetCursorPosition];

  if (Y < 1) {
    Row = LCD_CURSOR_L1;
  } else {
    Row = LCD_CURSOR_L2;
  }

  if (X > LCD_COLLENGTH) {
    DEBUG (( DEBUG_ERROR,
             "ERROR - Column position exceeded" ));
    return EFI_ABORTED;
  } else {
    Position = Row | X;

    Data[0] = ESC_CHAR;
    Data[1] = LcdCmdDesc;
    Data[2] = Position;
  }

  AltStatus= alt_i2c_master_transmit ( I2cDev,
                                       Data,
                                       sizeof(Data), false, true);
  if ( AltStatus != ALT_E_SUCCESS ) {
    DEBUG (( DEBUG_ERROR,
             "ERROR - Failed to transmit LcdSetCursorPosition to LCD, Status: %r\r\n",
             AltStatus ));
    return EFI_ABORTED;
  }

  MicroSecondDelay(LcdCommandDelay[LcdSetCursorPosition]);

  return EFI_SUCCESS;
}

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
  )
{
  ALT_STATUS_CODE AltStatus;
  EFI_STATUS      Status;

  AltStatus = alt_i2c_master_transmit( I2cDev, Msg, strlen(Msg), false, true);
  if ( AltStatus != ALT_E_SUCCESS ) {
    DEBUG (( DEBUG_ERROR,
             "ERROR - Failed to transmit characters to LCD, Status: %r\r\n",
             AltStatus ));
    return EFI_ABORTED;
  }
  MicroSecondDelay(500000);
  return EFI_SUCCESS;
}

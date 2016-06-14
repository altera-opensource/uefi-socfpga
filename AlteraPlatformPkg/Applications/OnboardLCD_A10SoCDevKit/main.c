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

#include <errno.h>
#include <Uefi.h>
#include <Library/BaseMemoryLib.h>
#include <Library/DebugLib.h>
#include <Library/PcdLib.h>
#include <Library/UefiBootServicesTableLib.h>
#include <Library/UefiLib.h>
#include <Library/TimerLib.h>

#include <stdio.h>
#include <inttypes.h>
#include "socal/socal.h"
#include "socal/hps.h"
#include <hwlib.h>
#include <alt_i2c.h>
#include <alt_clock_manager.h>
#include <string.h>
#include "LcdI2c.h"

//
// Define constant parameters
//
#define LCD_COL_LENGTH 16

/**
  Move LCD characters to the left and right continuously
  until ESC is pressed

  @retval  EFI_SUCCESS  The LCD characters successfully move left and right
  @retval  Other        Failed to turn on the LCD backlight
**/
EFI_STATUS
EFIAPI
LcdMoveCharactersLeftRight (
  ALT_I2C_DEV_t   *I2cDev,
  IN  CONST CHAR8  *Msg
  )
{
  BOOLEAN          IsShiftRight;
  EFI_STATUS       Status;
  EFI_INPUT_KEY    Key;
  int              X,
                   Y;
  UINT8            EmptySpace;

  IsShiftRight    = TRUE;
  Key.UnicodeChar = CHAR_NULL;
  Key.ScanCode    = 0;
  X               = 0;
  Y               = 0;

  // Move characters in LCD until ESC key pressed
  printf ( "Press ESC key to exit.\n" );

  EmptySpace = LCD_COL_LENGTH - strlen( Msg );

  while ( Key.ScanCode != SCAN_ESC ){
    if ( IsShiftRight == TRUE ) {
      if ( X < EmptySpace ) {
        X++;
      } else {
        IsShiftRight = FALSE;
        X--;
      }
    } else {
      if ( X > 0 ) {
        X--;
      } else {
        IsShiftRight = TRUE;
        X++;
      }
    }

    // Update the text position
    Status = LcdClearScreen( I2cDev );
    if ( Status != EFI_SUCCESS ) {
      DEBUG (( DEBUG_ERROR,
               "ERROR - Failed to clear screen for scrolling msg" ));
      return EFI_NO_RESPONSE;
    }

    Status = LcdSetCursor( I2cDev, Y, X );
    if ( Status != EFI_SUCCESS ) {
      DEBUG (( DEBUG_ERROR,
               "ERROR - Failed to set cursor position for scrolling msg" ));
      return EFI_NO_RESPONSE;
    }

    Status = LcdPrint( I2cDev, Msg );
    if ( Status != EFI_SUCCESS ) {
      DEBUG (( DEBUG_ERROR,
               "ERROR - Failed to print characters for scrolling msg" ));
      return EFI_NO_RESPONSE;
    }

    // speed of scrolling text
    MicroSecondDelay( 50000 );

    // key pressed? exit loop
    Status = gST->ConIn->ReadKeyStroke ( gST->ConIn, &Key );
    if ( Status != EFI_SUCCESS ) {
      DEBUG (( DEBUG_ERROR,
               "ERROR - Failed to receive ESC key input" ));
    }
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
  ALT_I2C_DEV_t   Device;
  ALT_I2C_CTLR_t  I2cContNumber;
  EFI_STATUS      Status;
  EFI_INPUT_KEY   Key;
  CONST CHAR8     *LcdMessage;
  UINT8           Row;
  UINT8           Column;

  Key.UnicodeChar = CHAR_NULL;
  Key.ScanCode    = 0;
  I2cContNumber   = ALT_I2C_I2C1;

  DEBUG (( DEBUG_INFO,
           "16x2 LCD Application\n" ));

  // Initiate LCD
  Status = InitLcd ( &Device, I2cContNumber );
  if ( Status != EFI_SUCCESS ) {
    DEBUG (( DEBUG_ERROR,
             "ERROR - Failed to initiate LCD" ));
    return EFI_ABORTED;
  }

  // Clear LCD screen before printing character onto the LCD
  Status = LcdClearScreen ( &Device );
  if ( Status != EFI_SUCCESS ){
    DEBUG (( DEBUG_ERROR,
             "ERROR - Failed to clear screen" ));
    return EFI_ABORTED;
  } else {
    DEBUG (( DEBUG_INFO,
             "LCD Screen Cleared\n" ));
  }

  // Set cursor position
  Row = 0;
  Column = 0;
  Status = LcdSetCursor ( &Device, Row, Column );
  if ( Status != EFI_SUCCESS ) {
    DEBUG (( DEBUG_ERROR,
             "ERROR - Failed to set cursor position" ));
    return EFI_ABORTED;
  } else {
    DEBUG (( DEBUG_INFO,
             "Set cursor to Row- %d , Col- %d\n", Row, Column ));
  }

  // Printing characters onto the LCD
  DEBUG (( DEBUG_INFO,
           "Printing Characters To LCD. . .\n"  ));
  if ( Argc == 1 ) {
    // Scroll Hello World
    LcdMessage = "Hello World";

    DEBUG (( DEBUG_INFO,
             "Scrolling LCD Message: %s\n", LcdMessage ));

    Status = LcdMoveCharactersLeftRight ( &Device, LcdMessage );
  } else {
    // Print the first argument to LCD as text
    LcdMessage = Argv[1];
    printf ("Print LCD Message: %s\n", LcdMessage);
    Status = LcdPrint ( &Device, LcdMessage );
  }
  if ( Status != EFI_SUCCESS ){
      DEBUG (( DEBUG_ERROR,
               "ERROR - Failed to display the message on LCD" ));
      return EFI_ABORTED;
  }

  // Return the operation status
  DEBUG (( DEBUG_INFO,
           "Application exiting, Status: %r\r\n",
            Status ));
  return EFI_SUCCESS;;
}

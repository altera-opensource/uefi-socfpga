/*****************************************************************************
 *
 * Copyright 2015 Altera Corporation. All Rights Reserved.
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

#include <stdio.h>
#include <inttypes.h>
#include "alt_clock_manager.h"
#include "socal/socal.h"
#include "socal/hps.h"

volatile BOOLEAN bTick;
BOOLEAN          bTimerRunning;
EFI_EVENT        pTimer;
int              gLEDcode = 0;


/* LED functions defined in leds_xx.c */
void setLEDS(uint8_t data);
ALT_STATUS_CODE setupLEDs(void);
ALT_STATUS_CODE cleanupLEDs(void);

void setLEDsGrayCode(int code)
{
    int gray;
    gray = (code >> 1) ^ code;
    setLEDS(gray);
}


/**
  Handle the timer callback

  @param [in] Event     Event that caused this callback
  @param [in] pContext  Context for this routine
**/
VOID
EFIAPI
TimerCallback (
  IN EFI_EVENT Event,
  IN VOID * pContext
  )
{
    /*
     * Update the LEDs
    */
	setLEDsGrayCode (gLEDcode++);
}


/**
  Create the timer

  @retval  EFI_SUCCESS  The timer was successfully created
  @retval  Other        Timer initialization failed
**/
EFI_STATUS
TimerCreate (
  )
{
  EFI_STATUS Status;

  //
  //  Create the timer
  //
  Status = gBS->CreateEvent ( EVT_TIMER | EVT_NOTIFY_SIGNAL,
                              TPL_CALLBACK,
                              TimerCallback,
                              NULL,
                              &pTimer );
  if ( EFI_ERROR ( Status )) {
    DEBUG (( DEBUG_ERROR,
              "ERROR - Failed to allocate the timer event, Status: %r\r\n",
              Status ));
  }
  else {
    DEBUG (( DEBUG_INFO,
              "0x%08x: Timer created\r\n",
              pTimer ));
  }

  //
  //  Return the operation status
  //
  return Status;
}


/**
  Stop the timer

  @retval  EFI_SUCCESS  The timer was stopped successfully
  @retval  Other        The timer failed to stop
**/
EFI_STATUS
TimerStop (
  )
{
  EFI_STATUS Status;

  //
  //  Assume success
  //
  Status = EFI_SUCCESS;

  //
  //  Determine if the timer is running
  //
  if ( bTimerRunning ) {
    //
    //  Stop the timer
    //
    Status = gBS->SetTimer ( pTimer,
                             TimerCancel,
                             0 );
    if ( EFI_ERROR ( Status )) {
      DEBUG (( DEBUG_ERROR,
                "ERROR - Failed to stop the timer, Status: %r\r\n",
                Status ));
    }
    else {
      //
      //  Timer timer is now stopped
      //
      bTimerRunning = FALSE;
      DEBUG (( DEBUG_INFO,
                "0x%08x: Timer stopped\r\n",
                pTimer ));
    }
  }

  //
  //  Return the operation status
  //
  return Status;
}


/**
  Start the timer

  @param [in] Milliseconds  The number of milliseconds between timer callbacks

  @retval  EFI_SUCCESS  The timer was successfully created
  @retval  Other        Timer initialization failed
**/
EFI_STATUS
TimerStart (
  UINTN Milliseconds
  )
{
  EFI_STATUS Status;
  UINT64 TimeDelay;

  //
  //  Stop the timer if necessary
  //
  Status = EFI_SUCCESS;
  if ( bTimerRunning ) {
    Status = TimerStop ( );
  }
  if ( !EFI_ERROR ( Status )) {
    //
    //  Compute the new delay
    //
    TimeDelay = Milliseconds;
    TimeDelay *= 1000 * 10;

    //
    //  Start the timer
    //
    Status = gBS->SetTimer ( pTimer,
                             TimerPeriodic,
                             TimeDelay );
    if ( EFI_ERROR ( Status )) {
      DEBUG (( DEBUG_ERROR,
                "ERROR - Failed to start the timer, Status: %r\r\n",
                Status ));
    }
    else {
      //
      //  The timer is now running
      //
      bTimerRunning = TRUE;
      DEBUG (( DEBUG_INFO,
        "0x%08x: Timer running\r\n",
        pTimer ));
    }
  }

  //
  //  Return the operation status
  //
  return Status;
}


/**
  Destroy the timer

  @retval  EFI_SUCCESS  The timer was destroyed successfully
  @retval  Other        Failed to destroy the timer
**/
EFI_STATUS
TimerDestroy (
  )
{
  EFI_STATUS Status;

  //
  //  Assume success
  //
  Status = EFI_SUCCESS;

  //
  //  Determine if the timer is running
  //
  if ( bTimerRunning ) {
    //
    //  Stop the timer
    //
    Status = TimerStop ( );
  }
  if (( !EFI_ERROR ( Status )) && ( NULL != pTimer )) {
    //
    //  Done with this timer
    //
    Status = gBS->CloseEvent ( pTimer );
    if ( EFI_ERROR ( Status )) {
      DEBUG (( DEBUG_ERROR,
                "ERROR - Failed to free the timer event, Status: %r\r\n",
                Status ));
    }
    else {
      DEBUG (( DEBUG_INFO,
                "0x%08x: Timer Destroyed\r\n",
                pTimer ));
      pTimer = NULL;
    }
  }

  //
  //  Return the operation status
  //
  return Status;
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
  ALT_STATUS_CODE status = ALT_E_SUCCESS;
  EFI_STATUS      Status;
  EFI_INPUT_KEY   Key;
  Key.UnicodeChar = CHAR_NULL;
  Key.ScanCode    = 0;

  printf("Blinking LED Example for A10 SoC Dev Kit Board\n");

  //
  // Setup the LEDs
  //
  if (status == ALT_E_SUCCESS)
  {
    status = setupLEDs();
  }
  if (status != ALT_E_SUCCESS) {
    printf("ERROR! Unable to Setup the LEDs\n");
	return status;
  }

  //
  //  Create the timer
  //
  bTick = TRUE;
  Status = TimerCreate ();
  if ( EFI_ERROR ( Status )) {
    DEBUG (( DEBUG_INFO,
            "TimerCreate ERROR! Status: %r\r\n",
            Status ));
  } else {
    printf("Timer Created\n");
  }

  //
  //  Start a timer to perform updates
  //
  Status = TimerStart ( 500 );
  if ( EFI_ERROR ( Status )) {
    DEBUG (( DEBUG_INFO,
            "TimerStart ERROR! Status: %r\r\n",
            Status ));
  } else {
    printf("Timer Started\n");
  }

  //
  //  Loop until ESC key pressed
  //
  printf("LED is blinking......\n");
  printf("Press ESC key to exit.\n");

  do {
    Status = gST->ConIn->ReadKeyStroke (gST->ConIn, &Key);
    //
    // ESC was pressed
    //
    if (Status == EFI_SUCCESS && Key.UnicodeChar == 0 && Key.ScanCode == SCAN_ESC)
	{
      break;
    }
  } while(1);

  printf("Stopping LEDs from blinking\n");

  // Stop the LED system
  cleanupLEDs();

  //
  //  Stop the timer if necessary
  //
  TimerStop ();
  TimerDestroy ();
  printf("Timer Stopped\n");

  //
  //  Return the operation status
  //
  DEBUG (( DEBUG_INFO,
            "Applicaion exiting, Status: %r\r\n",
            Status ));
  return Status;
}

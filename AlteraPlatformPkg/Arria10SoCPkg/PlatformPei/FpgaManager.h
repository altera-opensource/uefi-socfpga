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

#ifndef __FPGAMANAGER_H__
#define __FPGAMANAGER_H__

#include "BootSource.h"

// ==================================================================
// CONSTANT Definition
// ==================================================================

/* Timeout counter */
#define FPGA_TIMEOUT_CNT      0x1000000

// ==================================================================
// MACRO Definition
// ==================================================================

#define ALT_FPGAMGR_IMGCFG_STAT_F2S_MSEL_SET_MSD (\
	ALT_FPGAMGR_IMGCFG_STAT_F2S_MSEL0_SET_MSK |\
	ALT_FPGAMGR_IMGCFG_STAT_F2S_MSEL1_SET_MSK |\
	ALT_FPGAMGR_IMGCFG_STAT_F2S_MSEL2_SET_MSK)

#define GET_MSEL \
       (((MmioRead32 (ALT_FPGAMGR_OFST + \
                      ALT_FPGAMGR_IMGCFG_STAT_OFST)) & \
                 (    ALT_FPGAMGR_IMGCFG_STAT_F2S_MSEL2_SET_MSK | \
                      ALT_FPGAMGR_IMGCFG_STAT_F2S_MSEL1_SET_MSK | \
                      ALT_FPGAMGR_IMGCFG_STAT_F2S_MSEL0_SET_MSK ) \
                 ) >> ALT_FPGAMGR_IMGCFG_STAT_F2S_MSEL0_LSB)

#define GET_F2S_USERMODE ( \
                      ALT_FPGAMGR_IMGCFG_STAT_F2S_USERMOD_GET( \
          MmioRead32 (ALT_FPGAMGR_OFST + \
                      ALT_FPGAMGR_IMGCFG_STAT_OFST)))

#define GET_F2S_NCONFIG_PIN ( \
                      ALT_FPGAMGR_IMGCFG_STAT_F2S_NCFG_PIN_GET( \
          MmioRead32 (ALT_FPGAMGR_OFST + \
                      ALT_FPGAMGR_IMGCFG_STAT_OFST)))

#define GET_F2S_NSTATUS_PIN ( \
                      ALT_FPGAMGR_IMGCFG_STAT_F2S_NSTAT_PIN_GET( \
          MmioRead32 (ALT_FPGAMGR_OFST + \
                      ALT_FPGAMGR_IMGCFG_STAT_OFST)))

#define GET_F2S_CONDONE_PIN ( \
                      ALT_FPGAMGR_IMGCFG_STAT_F2S_CONDONE_PIN_GET( \
          MmioRead32 (ALT_FPGAMGR_OFST + \
                      ALT_FPGAMGR_IMGCFG_STAT_OFST)))

#define GET_F2S_CONDONE_OE ( \
                      ALT_FPGAMGR_IMGCFG_STAT_F2S_CONDONE_OE_GET( \
          MmioRead32 (ALT_FPGAMGR_OFST + \
                      ALT_FPGAMGR_IMGCFG_STAT_OFST)))

#define GET_F2S_PR_READY ( \
                      ALT_FPGAMGR_IMGCFG_STAT_F2S_PR_READY_GET( \
          MmioRead32 (ALT_FPGAMGR_OFST + \
                      ALT_FPGAMGR_IMGCFG_STAT_OFST)))

#define GET_F2S_PR_DONE ( \
                      ALT_FPGAMGR_IMGCFG_STAT_F2S_PR_DONE_GET( \
          MmioRead32 (ALT_FPGAMGR_OFST + \
                      ALT_FPGAMGR_IMGCFG_STAT_OFST)))

#define GET_F2S_PR_ERROR ( \
                      ALT_FPGAMGR_IMGCFG_STAT_F2S_PR_ERROR_GET( \
          MmioRead32 (ALT_FPGAMGR_OFST + \
                      ALT_FPGAMGR_IMGCFG_STAT_OFST)))

#define GET_F2S_CRC_ERROR ( \
                      ALT_FPGAMGR_IMGCFG_STAT_F2S_CRC_ERROR_GET( \
          MmioRead32 (ALT_FPGAMGR_OFST + \
                      ALT_FPGAMGR_IMGCFG_STAT_OFST)))
#define GET_F2S_INITDONE_OE ( \
                      ALT_FPGAMGR_IMGCFG_STAT_F2S_INITDONE_OE_GET( \
          MmioRead32 (ALT_FPGAMGR_OFST + \
                      ALT_FPGAMGR_IMGCFG_STAT_OFST)))

// ==================================================================
// Functions Definition
// ==================================================================

//
// Public Functions
//

EFI_STATUS
EFIAPI
FpgaFullConfiguration (
  IN  VOID*             Fdt,
  IN  BOOT_SOURCE_TYPE  BootSourceType
  );

VOID
EFIAPI
EnableHpsAndFpgaBridges (
  IN  VOID*             Fdt
  );

VOID
EFIAPI
DisplayFpgaManagerInfo (
  VOID
  );

//
// Private Functions
//

VOID
EFIAPI
GetCdRatio (
  IN  UINT32    CfgWidth,
  OUT UINT32*   CdRatioPtr
  );

BOOLEAN
EFIAPI
FpgaIsInUserMode (
  VOID
  );

EFI_STATUS
EFIAPI
WaitForUserMode (
  VOID
  );

EFI_STATUS
EFIAPI
WaitForNconfigAndNstatusToGoesHigh (
  VOID
  );

EFI_STATUS
EFIAPI
WaitForF2sNstatus (
  UINT32 Value
  );

EFI_STATUS
EFIAPI
WaitForFpgaProgrammingCompletion (
  VOID
  );

EFI_STATUS
EFIAPI
GeneratesDclkPulses (
  IN UINT32 NumberOfDclkPulses
  );

EFI_STATUS
EFIAPI
FpgaProgramWrite (
  IN UINTN  RbfSize
  );

#endif



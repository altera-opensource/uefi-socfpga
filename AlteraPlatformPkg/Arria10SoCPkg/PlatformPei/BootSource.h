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

#ifndef __BOOTSOURCE_H__
#define __BOOTSOURCE_H__

//
// Definition Used by GetBootSourceType()
//
typedef enum {
	BOOT_SOURCE_RSVD = 0,
    BOOT_SOURCE_FPGA,
    BOOT_SOURCE_NAND,
    BOOT_SOURCE_SDMMC,
	BOOT_SOURCE_QSPI,
} BOOT_SOURCE_TYPE;

//
// Definition Used by GetBootSourceTypeViaFuse()
//
typedef enum {
	FUSE_BOOT_SOURCE_NONE = 0,
    FUSE_BOOT_SOURCE_FPGA, // hps_fusesec.fpga_boot_f
} FUSE_BOOT_SOURCE_TYPE;

//
// Definition Used by GetBootSourceTypeViaBsel()
//
typedef enum {
	BSEL_BOOT_SOURCE_RSVD = 0,
    BSEL_BOOT_SOURCE_FPGA,
    BSEL_BOOT_SOURCE_NAND,
    BSEL_BOOT_SOURCE_SDMMC,
	BSEL_BOOT_SOURCE_QSPI,
} BSEL_BOOT_SOURCE_TYPE;

//
// Definition Used by GetBootSourceTypeViaDedicatedIoMuxSelects()
//
typedef enum {
	PINMUX_BOOT_SOURCE_UNKNOWN = 0,
    PINMUX_BOOT_SOURCE_QSPI,
    PINMUX_BOOT_SOURCE_SDMMC,
    PINMUX_BOOT_SOURCE_NAND,
} PINMUX_BOOT_SOURCE_TYPE;
// NOTE: (Ref: A10_5v4.pdf Table A-2: Boot Source Mux Selects)
#define DCTD_IO_4to9_qspi      4
#define DCTD_IO_11_16_17_qspi  8
#define DCTD_IO_sdmmc          8
#define DCTD_IO_nand          14

// ==================================================================
// Functions Definition
// ==================================================================

//
// Public Functions
//
BOOT_SOURCE_TYPE
EFIAPI
GetBootSourceType (
  VOID
  );

FUSE_BOOT_SOURCE_TYPE
EFIAPI
GetBootSourceTypeViaFuse (
  VOID
  );

BSEL_BOOT_SOURCE_TYPE
EFIAPI
GetBootSourceTypeViaBsel (
  VOID
  );

PINMUX_BOOT_SOURCE_TYPE
EFIAPI
GetBootSourceTypeViaDedicatedIoMuxSelects (
  VOID
  );

//
// Private Functions
//
VOID
EFIAPI
ErrorHandlerBselIsNandButDtbPinMuxIsNotForNand (
  VOID
  );

VOID
EFIAPI
ErrorHandlerBselIsSdMmcButDtbPinMuxIsNotForSdMmc (
  VOID
  );

VOID
EFIAPI
ErrorHandlerBselIsQspiButDtbPinMuxIsNotForQspi (
  VOID
  );

#endif


/** @file

  Copyright (c) 2016, Intel Corporation. All rights reserved.

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
#include "Altera_Hps_Socal.h"
#include "Handoff.h"

#define InfoPrint     SerialPortPrint
#define SWAP_UINT32(x) (((x) >> 24) | (((x) & 0x00FF0000) >> 8) | (((x) & 0x0000FF00) << 8) | ((x) << 24))

EFI_STATUS
EFIAPI
verify_handoff_image (handoff *hoff_ptr)
{
	UINT32 i;
	UINT32 *buffer;

	buffer = (UINT32*)hoff_ptr;

	/* convert big indian to little indian */
	for(i = 0; i < sizeof(handoff) / 4; i++) {
		buffer[i] = SWAP_UINT32(buffer[i]);
	}
	InfoPrint("hoff_ptr->header_magic  = %x\n", hoff_ptr->header_magic);
	/* check for magic header */
	if (hoff_ptr->header_magic != HANDOFF_MAGIC_HEADER) {
		InfoPrint("invalid magic header\n");
		return EFI_DEVICE_ERROR;
	}
	InfoPrint("valid magic header\r\n");
	/* check for magic pinmux sel header */
	if (hoff_ptr->pinmux_sel_magic != HANDOFF_MAGIC_PINMUX_SEL)
		return EFI_DEVICE_ERROR;
	/* check for magic pinmux ioctrl header */
	if (hoff_ptr->pinmux_io_magic != HANDOFF_MAGIC_IOCTLR)
		return EFI_DEVICE_ERROR;
	/* check for magic pinmux fpga header */
	if (hoff_ptr->pinmux_fpga_magic != HANDOFF_MAGIC_FPGA)
		return EFI_DEVICE_ERROR;
	/* check for magic pinmux io delay header */
	if (hoff_ptr->pinmux_delay_magic != HANDOFF_MAGIC_IODELAY)
		return EFI_DEVICE_ERROR;
	return EFI_SUCCESS;
}

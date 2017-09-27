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

#ifndef	_HANDOFF_H_
#define	_HANDOFF_H_

#define HANDOFF_MAGIC_HEADER	0x424f4f54	/* BOOT */
#define HANDOFF_MAGIC_PINMUX_SEL	0x504d5558	/* PMUX */
#define HANDOFF_MAGIC_IOCTLR	0x494f4354	/* IOCT */
#define HANDOFF_MAGIC_FPGA		0x46504741	/* FPGA */
#define HANDOFF_MAGIC_IODELAY	0x444c4159	/* DLAY */
#define HANDOFF_MAGIC_CLOCK		0x434c4b53	/* CLKS */
#define HANDOFF_MAGIC_MISC		0x4d495343	/* MISC */

typedef struct handoff_t {
	/* header */
	UINT32	header_magic;
	UINT32	header_device;
	UINT32	_pad_0x08_0x10[2];

	/* pinmux configuration - select */
	UINT32	pinmux_sel_magic;
	UINT32	pinmux_sel_length;
	UINT32	_pad_0x18_0x20[2];
	UINT32	pinmux_sel_array[96];	/* offset, value */

	/* pinmux configuration - io control */
	UINT32	pinmux_io_magic;
	UINT32	pinmux_io_length;
	UINT32	_pad_0x1a8_0x1b0[2];
	UINT32	pinmux_io_array[96];	/* offset, value */

	/* pinmux configuration - use fpga switch */
	UINT32	pinmux_fpga_magic;
	UINT32	pinmux_fpga_length;
	UINT32	_pad_0x338_0x340[2];
	UINT32	pinmux_fpga_array[42];	/* offset, value */
	UINT32	_pad_0x3e8_0x3f0[2];

	/* pinmux configuration - io delay */
	UINT32	pinmux_delay_magic;
	UINT32	pinmux_delay_length;
	UINT32	_pad_0x3f8_0x400[2];
	UINT32	pinmux_iodelay_array[96];	/* offset, value */

	/* clock configuration */
	UINT32	clock_magic;
	UINT32	clock_length;
	UINT32	_pad_0x588_0x590[2];
	UINT32	main_pll_mpuclk;
	UINT32	main_pll_nocclk;
	UINT32	main_pll_cntr2clk;
	UINT32	main_pll_cntr3clk;
	UINT32	main_pll_cntr4clk;
	UINT32	main_pll_cntr5clk;
	UINT32	main_pll_cntr6clk;
	UINT32	main_pll_cntr7clk;
	UINT32	main_pll_cntr8clk;
	UINT32	main_pll_cntr9clk;
	UINT32	main_pll_nocdiv;
	UINT32	main_pll_pllglob;
	UINT32	main_pll_fdbck;
	UINT32	main_pll_pllc0;
	UINT32	main_pll_pllc1;
	UINT32	_pad_0x5cc_0x5d0[1];
	UINT32	per_pll_cntr2clk;
	UINT32	per_pll_cntr3clk;
	UINT32	per_pll_cntr4clk;
	UINT32	per_pll_cntr5clk;
	UINT32	per_pll_cntr6clk;
	UINT32	per_pll_cntr7clk;
	UINT32	per_pll_cntr8clk;
	UINT32	per_pll_cntr9clk;
	UINT32	per_pll_emacctl;
	UINT32	per_pll_gpiodiv;
	UINT32	per_pll_pllglob;
	UINT32	per_pll_fdbck;
	UINT32	per_pll_pllc0;
	UINT32	per_pll_pllc1;
	UINT32	hps_osc_clk_h;
	UINT32	fpga_clk_hz;

	/* misc configuration */
	UINT32	misc_magic;
	UINT32	misc_length;
	UINT32	_pad_0x618_0x620[2];
	UINT32	boot_source;
} handoff;

EFI_STATUS
EFIAPI
verify_handoff_image (handoff *hoff_ptr);

#endif



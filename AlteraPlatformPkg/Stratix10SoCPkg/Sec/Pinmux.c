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
#include <Library/PrintLib.h>
#include <Library/SerialPortPrintLib.h>
#include "Pinmux.h"

#if (FixedPcdGet32(PcdDebugMsg_Pinmux) == 0)
  #define ProgressPrint(FormatString, ...)    /* do nothing */
  #define InfoPrint(FormatString, ...)        /* do nothing */
#else
  #define ProgressPrint SerialPortPrint
  #define InfoPrint     SerialPortPrint
#endif



const UINT32 sysmgr_pinmux_array_sel[] = {
	0x00000000, 0x00000001, /* usb */
	0x00000004, 0x00000001,
	0x00000008, 0x00000001,
	0x0000000c, 0x00000001,
	0x00000010, 0x00000001,
	0x00000014, 0x00000001,
	0x00000018, 0x00000001,
	0x0000001c, 0x00000001,
	0x00000020, 0x00000001,
	0x00000024, 0x00000001,
	0x00000028, 0x00000001,
	0x0000002c, 0x00000001,
	0x00000030, 0x00000000, /* emac0 */
	0x00000034, 0x00000000,
	0x00000038, 0x00000000,
	0x0000003c, 0x00000000,
	0x00000040, 0x00000000,
	0x00000044, 0x00000000,
	0x00000048, 0x00000000,
	0x0000004c, 0x00000000,
	0x00000050, 0x00000000,
	0x00000054, 0x00000000,
	0x00000058, 0x00000000,
	0x0000005c, 0x00000000,
	0x00000060, 0x00000008, /* gpio1 */
	0x00000064, 0x00000008,
	0x00000068, 0x00000005,  // uart0 tx
	0x0000006c, 0x00000005,  // uart 0 rx
	0x00000070, 0x00000008,  // gpio
	0x00000074, 0x00000008,
	0x00000078, 0x00000004, //i2c1
	0x0000007c, 0x00000004,
	0x00000080, 0x00000007,  //jtag
	0x00000084, 0x00000007,
	0x00000088, 0x00000007,
	0x0000008c, 0x00000007,
	0x00000090, 0x00000001,  //sdmmc data0
	0x00000094, 0x00000001,
	0x00000098, 0x00000001,
	0x0000009c, 0x00000001,
	0x00000100, 0x00000001,
	0x00000104, 0x00000001,  //sdmmc.data3
	0x00000108, 0x00000008,  //loan
	0x0000010c, 0x00000008,   //gpio
	0x00000110, 0x00000008,  //
	0x00000114, 0x00000008,  //gpio1.io21
	0x00000118, 0x00000005,  // mdio0.mdio
	0x0000011c, 0x00000005  //mdio0.mdc
};

const UINT32 sysmgr_pinmux_array_ctrl[] = {
	0x00000000, 0x00502c38, /* Q1_1 */
	0x00000004, 0x00102c38,
	0x00000008, 0x00502c38,
	0x0000000c, 0x00502c38,
	0x00000010, 0x00502c38,
	0x00000014, 0x00502c38,
	0x00000018, 0x00502c38,
	0x0000001c, 0x00502c38,
	0x00000020, 0x00502c38,
	0x00000024, 0x00502c38,
	0x00000028, 0x00502c38,
	0x0000002c, 0x00502c38,
	0x00000030, 0x00102c38, /* Q2_1 */
	0x00000034, 0x00102c38,
	0x00000038, 0x00502c38,
	0x0000003c, 0x00502c38,
	0x00000040, 0x00102c38,
	0x00000044, 0x00102c38,
	0x00000048, 0x00502c38,
	0x0000004c, 0x00502c38,
	0x00000050, 0x00102c38,
	0x00000054, 0x00102c38,
	0x00000058, 0x00502c38,
	0x0000005c, 0x00502c38,
	0x00000060, 0x00502c38, /* Q3_1 */
	0x00000064, 0x00502c38,
	0x00000068, 0x00102c38,
	0x0000006c, 0x00502c38,
	0x000000d0, 0x00502c38,
	0x000000d4, 0x00502c38,
	0x000000d8, 0x00542c38,
	0x000000dc, 0x00542c38,
	0x000000e0, 0x00502c38,
	0x000000e4, 0x00502c38,
	0x000000e8, 0x00102c38,
	0x000000ec, 0x00502c38,
	0x000000f0, 0x00502c38, /* Q4_1 */
	0x000000f4, 0x00502c38,
	0x000000f8, 0x00102c38,
	0x000000fc, 0x00502c38,
	0x00000100, 0x00502c38,
	0x00000104, 0x00502c38,
	0x00000108, 0x00102c38,
	0x0000010c, 0x00502c38,
	0x00000110, 0x00502c38,
	0x00000114, 0x00502c38,
	0x00000118, 0x00542c38,
	0x0000011c, 0x00102c38
};

const UINT32 sysmgr_pinmux_array_fpga[] = {
	0x00000000, 0x00000000,
	0x00000004, 0x00000000,
	0x00000008, 0x00000000,
	0x0000000c, 0x00000000,
	0x00000010, 0x00000000,
	0x00000014, 0x00000000,
	0x00000018, 0x00000000,
	0x0000001c, 0x00000000,
	0x00000020, 0x00000000,
	0x00000028, 0x00000000,
	0x0000002c, 0x00000000,
	0x00000030, 0x00000000,
	0x00000034, 0x00000000,
	0x00000038, 0x00000000,
	0x0000003c, 0x00000000,
	0x00000040, 0x00000000,
	0x00000044, 0x00000000,
	0x00000048, 0x00000000,
	0x00000050, 0x00000000,
	0x00000054, 0x00000000,
	0x00000058, 0x0000002a
};

const UINT32 sysmgr_pinmux_array_iodelay[] = {
	0x00000000, 0x00000000,
	0x00000004, 0x00000000,
	0x00000008, 0x00000000,
	0x0000000c, 0x00000000,
	0x00000010, 0x00000000,
	0x00000014, 0x00000000,
	0x00000018, 0x00000000,
	0x0000001c, 0x00000000,
	0x00000020, 0x00000000,
	0x00000024, 0x00000000,
	0x00000028, 0x00000000,
	0x0000002c, 0x00000000,
	0x00000030, 0x00000000,
	0x00000034, 0x00000000,
	0x00000038, 0x00000000,
	0x0000003c, 0x00000000,
	0x00000040, 0x00000000,
	0x00000044, 0x00000000,
	0x00000048, 0x00000000,
	0x0000004c, 0x00000000,
	0x00000050, 0x00000000,
	0x00000054, 0x00000000,
	0x00000058, 0x00000000,
	0x0000005c, 0x00000000,
	0x00000060, 0x00000000,
	0x00000064, 0x00000000,
	0x00000068, 0x00000000,
	0x0000006c, 0x00000000,
	0x00000070, 0x00000000,
	0x00000074, 0x00000000,
	0x00000078, 0x00000000,
	0x0000007c, 0x00000000,
	0x00000080, 0x00000000,
	0x00000084, 0x00000000,
	0x00000088, 0x00000000,
	0x0000008c, 0x00000000,
	0x00000090, 0x00000000,
	0x00000094, 0x00000000,
	0x00000098, 0x00000000,
	0x0000009c, 0x00000000,
	0x00000100, 0x00000000,
	0x00000104, 0x00000000,
	0x00000108, 0x00000000,
	0x0000010c, 0x00000000,
	0x00000110, 0x00000000,
	0x00000114, 0x00000000,
	0x00000118, 0x00000000,
	0x0000011c, 0x00000000
};

/*
 * Configure all the pin muxes
 */
VOID ConfigPinMux(VOID)
{
	UINT32 i;

	InfoPrint("Configure Pin Mux  Using Default Value\n");
	InfoPrint("\tSetup Pin Sel\n");
	/* setup the pin sel */
	for (i = 0; i < 96; i=i+2) {
		MmioWrite32( ALT_PINMUX_OFST + ALT_PINMUX_PIN0SEL_OFST + sysmgr_pinmux_array_sel[i],
		                 sysmgr_pinmux_array_sel[i+1]);
	}
	InfoPrint("\tSetup Pin Ctrl\n");
	/* setup the pin ctrl */
	for (i = 0; i < 96; i=i+2) {
		MmioWrite32( ALT_PINMUX_OFST + ALT_PINMUX_IO0CTRL_OFST + sysmgr_pinmux_array_ctrl[i],
		                 sysmgr_pinmux_array_ctrl[i+1]);
	}
	InfoPrint("\tSetup Fpga Use\n");
    /* setup the fpga use */
	for (i = 0; i < 42; i=i+2) {
		MmioWrite32( ALT_PINMUX_OFST + ALT_PINMUX_PINMUX_EMAC0_USEFPGA_OFST + sysmgr_pinmux_array_fpga[i],
		                 sysmgr_pinmux_array_fpga[i+1]);
	}
	InfoPrint("\tSetup IO Delay\n");
	/* setup the io delay */
	for (i = 0; i < 96; i=i+2) {
		MmioWrite32( ALT_PINMUX_OFST + ALT_PINMUX_IO0_DELAY_OFST + sysmgr_pinmux_array_iodelay[i],
		                 sysmgr_pinmux_array_iodelay[i+1]);
	}

}

VOID ConfigPinMuxHandoff(handoff *hoff_ptr)
{
	UINT32 i;

	InfoPrint("Configure Pin Mux From Handoff File\n");
	InfoPrint("\tSetup Pin Sel\n");
	/* setup the pin sel */
	for (i = 0; i < 96; i=i+2) {
		MmioWrite32( ALT_PINMUX_OFST + ALT_PINMUX_PIN0SEL_OFST + hoff_ptr->pinmux_sel_array[i],
		                 hoff_ptr->pinmux_sel_array[i+1]);
	}
	InfoPrint("\tSetup Pin Ctrl\n");
	/* setup the pin ctrl */
	for (i = 0; i < 96; i=i+2) {
		MmioWrite32( ALT_PINMUX_OFST + ALT_PINMUX_IO0CTRL_OFST + hoff_ptr->pinmux_io_array[i],
		                 hoff_ptr->pinmux_io_array[i+1]);
	}
	InfoPrint("\tSetup Fpga Use\n");
    /* setup the fpga use */
	for (i = 0; i < 42; i=i+2) {
		MmioWrite32( ALT_PINMUX_OFST + ALT_PINMUX_PINMUX_EMAC0_USEFPGA_OFST + hoff_ptr->pinmux_fpga_array[i],
		                 hoff_ptr->pinmux_fpga_array[i+1]);
	}
	InfoPrint("\tSetup IO Delay\n");
	/* setup the io delay */
	for (i = 0; i < 96; i=i+2) {
		MmioWrite32( ALT_PINMUX_OFST + ALT_PINMUX_IO0_DELAY_OFST + hoff_ptr->pinmux_iodelay_array[i],
		                 hoff_ptr->pinmux_iodelay_array[i+1]);
	}

}

VOID
EFIAPI
DisplayIo48PinMuxInfo (
  VOID
  )
{
#if (FixedPcdGet32(PcdDebugMsg_Pinmux) > 0)
  CHAR8 meaning_string_of_dedicated_pinmux[48][10][14] = {
    {{"sdmmc.cclk"},   {"usb0.clk"},     {"trace.d10"},  {"nand.adq0"},   {"NA"},            {"uart0.cts_n"}, {"spis0.clk"},   {"spim0.ss1_n"}, {"gpio0.io0"},  {"loan.io32"}},
    {{"sdmmc.cmd"},    {"usb0.stp"},     {"trace.d9"},   {"nand.adq1"},   {"NA"},            {"uart0.rts_n"}, {"spis0.mosi"},  {"spim1.ss1_n"}, {"gpio0.io1"},  {"loan.io0"}},
    {{"sdmmc.data0"},  {"usb0.dir"},     {"trace.d8"},   {"nand.we_n"},   {"i2c1.sda"},      {"uart0.tx"},    {"spis0.ss0_n"}, {"NA"},          {"gpio0.io2"},  {"loan.io4"}},
    {{"sdmmc.data1"},  {"usb0.data0"},   {"trace.d7"},   {"nand.re_n"},   {"i2c1.scl"},      {"uart0.rx"},    {"spis0.miso"},  {"NA"},          {"gpio0.io3"},  {"loan.io3"}},
    {{"sdmmc.data2"},  {"usb0.data1"},   {"trace.d6"},   {"nand.wp_n"},   {"i2c0.sda"},      {"uart1.cts_n"}, {"spim0.clk"},   {"NA"},          {"gpio0.io4"},  {"loan.io2"}},
    {{"sdmmc.data3"},  {"usb0.nxt"},     {"trace.d5"},   {"nand.adq2"},   {"i2c0.scl"},      {"uart1.rts_n"}, {"spim0.mosi"},  {"NA"},          {"gpio0.io5"},  {"loan.io5"}},
    {{"sdmmc.data4"},  {"usb0.data2"},   {"trace.d4"},   {"nand.adq3"},   {"i2c_emac2.sda"}, {"uart1.tx"},    {"mdio2.mdio"},  {"spim0.miso"},  {"gpio0.io6"},  {"loan.io6"}},
    {{"sdmmc.data5"},  {"usb0.data3"},   {"trace.d15"},  {"nand.cle"},    {"i2c_emac2.scl"}, {"uart1.rx"},    {"mdio2.mdc"},   {"spim0.ss0_n"}, {"gpio0.io7"},  {"loan.io8"}},
    {{"sdmmc.data6"},  {"usb0.data4"},   {"trace.d14"},  {"nand.adq4"},   {"i2c_emac1.sda"}, {"mdio1.mdio"},  {"spis1.clk"},   {"spim1.clk"},   {"gpio0.io8"},  {"loan.io7"}},
    {{"sdmmc.data7"},  {"usb0.data5"},   {"trace.d13"},  {"NA"},          {"i2c_emac1.scl"}, {"mdio1.mdc"},   {"spis1.mosi"},  {"spim1.mosi"},  {"gpio0.io9"},  {"loan.io10"}},
    {{"sdmmc.pwr_en"}, {"usb0.data6"},   {"trace.d12"},  {"nand.adq6"},   {"i2c_emac0.sda"}, {"mdio0.mdio"},  {"spis1.ss0_n"}, {"spim1.miso"},  {"gpio0.io10"}, {"loan.io9"}},
    {{"NA"},           {"usb0.data7"},   {"trace.d11"},  {"nand.adq7"},   {"i2c_emac0.scl"}, {"mdio0.mdc"},   {"spis1.miso"},  {"spim1.ss0_n"}, {"gpio0.io11"}, {"loan.io11"}},
    {{"emac0.tx_clk"}, {"usb1.clk"},     {"trace.d10"},  {"nand.ale"},    {"NA"},            {"NA"},          {"NA"},          {"NA"},          {"gpio0.io12"}, {"loan.io13"}},
    {{"emac0.tx_ctl"}, {"usb1.stp"},     {"trace.d9"},   {"nand.rb"},     {"NA"},            {"NA"},          {"NA"},          {"NA"},          {"gpio0.io13"}, {"loan.io14"}},
    {{"emac0.rx_clk"}, {"usb1.dir"},     {"trace.d8"},   {"nand.ce_n"},   {"NA"},            {"NA"},          {"NA"},          {"NA"},          {"gpio0.io14"}, {"loan.io12"}},
    {{"emac0.rx_ctl"}, {"usb1.data0"},   {"trace.d7"},   {"NA"},          {"NA"},            {"NA"},          {"NA"},          {"NA"},          {"NA"},         {"NA"}},     
	{{"emac0.txd0"},   {"usb1.data1"},   {"trace.d6"},   {"nand.adq8"},   {"NA"},            {"NA"},          {"NA"},          {"NA"},          {"gpio0.io16"}, {"loan.io16"}},
    {{"emac0.txd1"},   {"usb1.nxt"},     {"trace.d5"},   {"nand.adq9"},   {"NA"},            {"NA"},          {"NA"},          {"NA"},          {"gpio0.io17"}, {"loan.io18"}},
    {{"emac0.rxd0"},   {"usb1.data2"},   {"trace.d4"},   {"nand.adq10"},  {"NA"},            {"NA"},          {"NA"},          {"NA"},          {"gpio0.io18"}, {"loan.io17"}},
    {{"emac0.rxd1"},   {"usb1.data3"},   {"trace.clk"},  {"nand.adq11"},  {"NA"},            {"NA"},          {"NA"},          {"spim1.ss1_n"}, {"gpio0.io19"}, {"loan.io19"}}, 
    {{"emac0.txd2"},   {"usb1.data4"},   {"trace.d0"},   {"nand.adq12"},  {"i2c1.sda"},      {"uart0.cts_n"}, {"spis0.clk"},   {"spim1.clk"},   {"gpio0.io20"}, {"loan.io20"}},
    {{"emac0.txd3"},   {"usb1.data5"},   {"trace.d1"},   {"nand.adq13"},  {"i2c1.scl"},      {"uart0.rts_n"}, {"spis0.mosi"},  {"spim1.mosi"},  {"gpio0.io21"}, {"loan.io21"}},
    {{"emac0.rxd2"},   {"usb1.data6"},   {"trace.d2"},   {"nand.adq14"},  {"i2c0.sda"},      {"uart0.tx"},    {"spis0.ss0_n"}, {"spim1.miso"},  {"gpio0.io22"}, {"loan.io22"}},
    {{"emac0.rxd3"},   {"usb1.data7"},   {"trace.d3"},   {"nand.adq15"},  {"i2c0.scl"},      {"uart0.rx"},    {"spis0.miso"},  {"spim1.ss0_n"}, {"gpio0.io23"}, {"loan.io23"}},
    {{"emac1.tx_clk"}, {"NA"},           {"trace.d10"},  {"nand.adq0"},   {"NA"},            {"uart0.cts_n"}, {"cm.pll_clk0"}, {"spim1.clk"},   {"gpio1.io0"},  {"loan.io25"}},
    {{"emac1.tx_ctl"}, {"NA"},           {"trace.d9"},   {"nand.adq1"},   {"NA"},            {"uart0.rts_n"}, {"cm.pll_clk1"}, {"spim1.mosi"},  {"gpio1.io1"},  {"loan.io24"}},
    {{"emac1.rx_clk"}, {"NA"},           {"trace.d8"},   {"nand.we_n"},   {"i2c0.sda"},      {"uart0.tx"},    {"cm.pll_clk2"}, {"spim1.miso"},  {"gpio1.io2"},  {"loan.io28"}},
    {{"emac1.rx_ctl"}, {"NA"},           {"trace.d7"},   {"nand.re_n"},   {"i2c0.scl"},      {"uart0.rx"},    {"cm.pll_clk3"}, {"spim1.ss0_n"}, {"gpio1.io3"},  {"loan.io27"}},
    {{"emac1.txd0"},   {"NA"},           {"trace.d6"},   {"nand.wp_n"},   {"NA"},            {"uart1.cts_n"}, {"spis1.clk"},   {"spim1.ss1_n"}, {"gpio1.io4"},  {"loan.io26"}},
    {{"emac1.txd1"},   {"NA"},           {"trace.d5"},   {"nand.adq2"},   {"NA"},            {"uart1.rts_n"}, {"spis1.mosi"},  {"NA"},          {"gpio1.io5"},  {"loan.io29"}},
    {{"emac1.rxd0"},   {"NA"},           {"trace.d4"},   {"nand.adq3"},   {"i2c1.sda"},      {"uart1.tx"},    {"spis1.ss0_n"}, {"NA"},          {"gpio1.io6"},  {"loan.io30"}},
    {{"emac1.rxd1"},   {"NA"},           {"trace.d15"},  {"nand.cle"},    {"i2c1.scl"},      {"uart1.rx"},    {"spis1.miso"},  {"NA"},          {"gpio1.io7"},  {"loan.io32"}},
    {{"emac1.txd2"},   {"NA"},           {"trace.d14"},  {"nand.adq4"},   {"i2c_emac2.sda"}, {"mdio2.mdio"},  {"spis0.clk"},   {"jtag.tck"},    {"gpio1.io8"},  {"loan.io31"}},
    {{"emac1.txd3"},   {"NA"},           {"trace.d13"},  {"nand.adq5"},   {"i2c_emac2.scl"}, {"mdio2.mdc"},   {"spis0.mosi"},  {"jtag.tms"},    {"gpio1.io9"},  {"loan.io34"}},
    {{"emac1.rxd2"},   {"NA"},           {"trace.d12"},  {"nand.adq6"},   {"i2c_emac0.sda"}, {"mdio0.mdio"},  {"spis0.ss0_n"}, {"jtag.tdo"},    {"gpio1.io10"}, {"loan.io33"}},
    {{"emac1.rxd3"},   {"NA"},           {"trace.d11"},  {"nand.adq7"},   {"i2c_emac0.scl"}, {"mdio0.mdc"},   {"spis0.miso"},  {"jtag.tdi"},    {"gpio1.io11"}, {"loan.io35"}},
    {{"emac2.tx_clk"}, {"sdmmc.data0"},  {"trace.d10"},  {"nand.ale"},    {"i2c1.sda"},      {"NA"},          {"NA"},          {"NA"},          {"gpio1.io12"}, {"loan.io37"}}, 
    {{"emac2.rx_clk"}, {"sdmmc.cclk"},   {"trace.d8"},   {"nand.ce_n"},   {"NA"},            {"uart1.tx"},    {"NA"},          {"NA"},          {"gpio1.io14"}, {"loan.io36"}}, 
    {{"emac2.rx_ctl"}, {"sdmmc.data1"},  {"trace.d7"},   {"NA"},          {"NA"},            {"uart1.rx"},    {"NA"},          {"NA"},          {"gpio1.io15"}, {"loan.io39"}}, 
    {{"emac2.txd0"},   {"sdmmc.data2"},  {"trace.d6"},   {"nand.adq8"},   {"NA"},            {"uart1.cts_n"}, {"NA"},          {"NA"},          {"gpio1.io16"}, {"loan.io40"}}, 
    {{"emac2.txd1"},   {"sdmmc.data3"},  {"trace.d5"},   {"nand.adq9"},   {"NA"},            {"uart1.rts_n"}, {"NA"},          {"spim0.ss1_n"}, {"gpio1.io17"}, {"loan.io42"}}, 
    {{"emac2.rxd0"},   {"sdmmc.data4"},  {"trace.d4"},   {"nand.adq10"},  {"i2c_emac1.sda"}, {"mdio1.mdio"},  {"NA"},          {"spim0.miso"},  {"gpio1.io18"}, {"loan.io41"}}, 
    {{"emac2.rxd1"},   {"sdmmc.data5"},  {"trace.clk"},  {"nand.adq11"},  {"i2c_emac1.scl"}, {"mdio1.mdc"},   {"NA"},          {"spim0.ss0_n"}, {"gpio1.io19"}, {"loan.io43"}}, 
    {{"emac2.txd2"},   {"sdmmc.data6"},  {"trace.d0"},   {"nand.adq12"},  {"i2c_emac2.sda"}, {"NA"},          {"spis1.clk"},   {"spim0.clk"},   {"gpio1.io20"}, {"loan.io44"}}, 
    {{"emac2.txd3"},   {"sdmmc.data7"},  {"trace.d1"},   {"nand.adq13"},  {"i2c_emac2.scl"}, {"NA"},          {"spis1.mosi"},  {"spim0.mosi"},  {"gpio1.io21"}, {"loan.io45"}}, 
    {{"emac2.rxd2"},   {"sdmmc.pwr_en"}, {"trace.d2"},   {"nand.adq14"},  {"i2c_emac0.sda"}, {"mdio0.mdio"},  {"spis1.ss0_n"}, {"spim0.miso"},  {"gpio1.io22"}, {"loan.io46"}}, 
    {{"emac2.rxd3"},   {"NA"},           {"trace.d3"},   {"nand.adq15"},  {"i2c_emac0.scl"}, {"mdio0.mdc"},   {"spis1.miso"},  {"spim0.ss0_n"}, {"gpio1.io23"}, {"loan.io47"}}
	};

  UINTN           i;
  UINT32          Data32;

  // Char8Str is used to speed up Semihosting printing speed
  CHAR8   Char8Str[2048];
  CHAR8*  Char8Ptr = &Char8Str[0];

  // Display Pin Mux settings and Voltage settings together for dedicated IO
  InfoPrint ("Pin Mux settings for seletected IO:\r\n");
  for (i = 0; i < 48; i++)
  {
    // Pin Mux select
    Data32 = MmioRead32 (ALT_PINMUX_OFST +  ALT_PINMUX_PIN0SEL_OFST + sysmgr_pinmux_array_sel[i]);
    Char8Ptr += AsciiSPrint (Char8Ptr, 1024,
               "\t PIN%dSEL:\t%a (%d)\r\n", i,
                meaning_string_of_dedicated_pinmux[i][ALT_PINMUX_PIN1SEL_VAL_GET(Data32)],
                Data32);
    // Pix Mux Control
    Data32 = MmioRead32 (ALT_PINMUX_OFST + ALT_PINMUX_IO0CTRL_OFST + sysmgr_pinmux_array_ctrl[i]);
    Char8Ptr += AsciiSPrint (Char8Ptr, 1024,
                "\t PINMUX_IO%dCTRL   :\t0x%08x\r\n", i, Data32);
    // Control - More details
    DisplayDedicatedConfigurationDetails(Data32, &Char8Ptr);

    if ((i%3) == 0)
    {
      InfoPrint ("%a", Char8Str); Char8Ptr = &Char8Str[0];
    }
  }
  InfoPrint ("%a", Char8Str); Char8Ptr = &Char8Str[0];

 // Display Selection Between HPS Pin and FPGA Interface
  InfoPrint ("Interface Connection Settings:\r\n"
             "Legend:\r\n\t0: uses HPS IO Pins\r\n\t1: uses the FPGA Inteface\r\n");
  // EMAC(0-2)
  for (i = 0; i < 3; i++)
  {
    Char8Ptr += AsciiSPrint (Char8Ptr, 1024,
                "\t pinmux_emac%d_usefpga    :\t0x%08x\r\n", i,
                MmioRead32 (ALT_PINMUX_OFST + ALT_PINMUX_PINMUX_EMAC0_USEFPGA_OFST + (i * 4)));
  }
  // I2C(0-1)
  for (i = 0; i < 2; i++)
  {
    Char8Ptr += AsciiSPrint (Char8Ptr, 1024,
                "\t pinmux_i2c%d_usefpga     :\t0x%08x\r\n", i,
                MmioRead32 (ALT_PINMUX_OFST + ALT_PINMUX_PINMUX_I2C0_USEFPGA_OFST + (i * 4)));
  }
  // I2C_EMAC(0-2)
  for (i = 0; i < 3; i++)
  {
    Char8Ptr += AsciiSPrint (Char8Ptr, 1024,
                "\t pinmux_i2c_emac%d_usefpga:\t0x%08x\r\n", i,
                MmioRead32 (ALT_PINMUX_OFST + ALT_PINMUX_PINMUX_I2C_EMAC0_USEFPGA_OFST + (i * 4)));
  }
  // NAND
  Char8Ptr += AsciiSPrint (Char8Ptr, 1024,
              "\t pinmux_nand_usefpga     :\t0x%08x\r\n",
              MmioRead32 (ALT_PINMUX_OFST + ALT_PINMUX_PINMUX_NAND_USEFPGA_OFST));
  // SDMMC
  Char8Ptr += AsciiSPrint (Char8Ptr, 1024,
              "\t pinmux_sdmmc_usefpga    :\t0x%08x\r\n",
              MmioRead32 (ALT_PINMUX_OFST + ALT_PINMUX_PINMUX_SDMMC_USEFPGA_OFST));

  InfoPrint ("%a", Char8Str); Char8Ptr = &Char8Str[0];

  // SPIM(0-1)
  for (i = 0; i < 2; i++)
  {
    Char8Ptr += AsciiSPrint (Char8Ptr, 1024,
                "\t pinmux_spim%d_usefpga    :\t0x%08x\r\n", i,
                MmioRead32 (ALT_PINMUX_OFST + ALT_PINMUX_PINMUX_SPIM0_USEFPGA_OFST + (i * 4)));
  }
  // SPIS(0-1)
  for (i = 0; i < 2; i++)
  {
    Char8Ptr += AsciiSPrint (Char8Ptr, 1024,
                "\t pinmux_spis%d_usefpga    :\t0x%08x\r\n", i,
                MmioRead32 (ALT_PINMUX_OFST + ALT_PINMUX_PINMUX_SPIS0_USEFPGA_OFST + (i * 4)));
  }
  // UART(0-1)
  for (i = 0; i < 2; i++)
  {
    Char8Ptr += AsciiSPrint (Char8Ptr, 1024,
                "\t pinmux_uart%d_usefpga    :\t0x%08x\r\n", i,
                MmioRead32 (ALT_PINMUX_OFST + ALT_PINMUX_PINMUX_UART0_USEFPGA_OFST + (i * 4)));
  }
  InfoPrint ("%a", Char8Str); Char8Ptr = &Char8Str[0];
  
  // MDIO(0-3)
   for (i = 0; i < 2; i++)
  {
    Char8Ptr += AsciiSPrint (Char8Ptr, 1024,
                "\t pinmux_mdio%d_usefpga    :\t0x%08x\r\n", i,
                MmioRead32 (ALT_PINMUX_OFST + ALT_PINMUX_PINMUX_MDIO0_USEFPGA_OFST + (i * 4)));
  }
  InfoPrint ("%a", Char8Str); Char8Ptr = &Char8Str[0]; 
 
  // JTAG
  Char8Ptr += AsciiSPrint (Char8Ptr, 1024,
                "\t pinmux_jtag_usefpga    :\t0x%08x\r\n", i,
                MmioRead32 (ALT_PINMUX_OFST + ALT_PINMUX_PINMUX_JTAG_USEFPGA_OFST));

  InfoPrint ("%a", Char8Str); Char8Ptr = &Char8Str[0]; 
#endif
}

VOID
EFIAPI
DisplayDedicatedConfigurationDetails (
  UINT32        Data32,
  CHAR8**       Char8PtrPtr
  )
{

  *Char8PtrPtr += AsciiSPrint (*Char8PtrPtr, 1024,
             "\t\tPull down drive strength : %d\r\n"
             "\t\tPull up drive strength : %d\r\n"
             "\t\tFast Slew rate for pull down: %d\r\n"
             "\t\tFast Slew rate for pull up : %d\r\n"
             "\t\tOpen drain : %a\r\n",
			 "\t\tEnable Weak Pull %a\r\n",
    ALT_PINMUX_IO0CTRL_RPCDN_GET(Data32),
    ALT_PINMUX_IO0CTRL_RPCDP_GET(Data32),
    ALT_PINMUX_IO0CTRL_RSLEWN_GET(Data32),
    ALT_PINMUX_IO0CTRL_RSLEWP_GET(Data32),
    ALT_PINMUX_IO0CTRL_ROPDRAIN_GET(Data32) == 0 ? "disabled" : "enabled",
    ALT_PINMUX_IO0CTRL_WKPULL_UP_DN_GET(Data32) == 2 ? "up" : "down");
	
  switch (ALT_PINMUX_IO0CTRL_INPUT_SEL_GET(Data32)) {
    case ALT_PINMUX_IO0CTRL_INPUT_SEL_E_TTL:
	  *Char8PtrPtr += AsciiSPrint (*Char8PtrPtr, 1024, "\t\tInput source: TTL\r\n");
	  break;
    case ALT_PINMUX_IO0CTRL_INPUT_SEL_E_STTL:
	  *Char8PtrPtr += AsciiSPrint (*Char8PtrPtr, 1024, "\t\tInput source: STTL\r\n");
	  break;
    case ALT_PINMUX_IO0CTRL_INPUT_SEL_E_DEFAULT_SCHMITT:
	  *Char8PtrPtr += AsciiSPrint (*Char8PtrPtr, 1024, "\t\tInput source: Schmitt\r\n");
	  break;
    case ALT_PINMUX_IO0CTRL_INPUT_SEL_E_SCHMITT:
	  *Char8PtrPtr += AsciiSPrint (*Char8PtrPtr, 1024, "\t\tInput source: Schmitt\r\n");
	  break;
    default:
	  *Char8PtrPtr += AsciiSPrint (*Char8PtrPtr, 1024, "\t\tInput source: Schmitt\r\n");
	  break;
  }
}


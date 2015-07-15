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
#include "DeviceTree.h"
#include "Pinmux.h"

#if (FixedPcdGet32(PcdDebugMsg_Pinmux) == 0)
  #define ProgressPrint(FormatString, ...)    /* do nothing */
  #define InfoPrint(FormatString, ...)        /* do nothing */
#else
  #define ProgressPrint SerialPortPrint
  #define InfoPrint     SerialPortPrint
#endif


//
// Functions
//

VOID
EFIAPI
ConfigureDedicatedIoElectricalBehavior (
  IN  CONST VOID*  Fdt
  )
{
  ProgressPrint ("Config Dedicated IO\r\n");
  ConfigPinMux (Fdt, "dedicated_cfg");
}


VOID
EFIAPI
ConfigureDedicatedIoPinMux(
  IN  CONST VOID*  Fdt
  )
{
  ProgressPrint ("Set Dedicated Io Pin Mux\r\n");
  ConfigPinMux (Fdt, "dedicated");
}


VOID
EFIAPI
SelectSharedIoBetweenHpsAndFpga (
  IN  CONST VOID*  Fdt
  )
{
  ProgressPrint ("Config Shared IO\r\n");
  ConfigPinMux (Fdt, "fpga");
}


VOID
EFIAPI
ConfigureSharedIoPinMux(
  IN  CONST VOID*  Fdt
  )
{
  ProgressPrint ("Set Shared Io Pin Mux\r\n");
  ConfigPinMux (Fdt, "shared");
}


BOOLEAN
EFIAPI
UartIsUsingDedicatedIo (
  VOID
  )
{
  // HPS UART can be on UART0 or UART1
  // If UART0 then it is only possible to be using Shared IO which required FPGA side to be programmed first.
  // If UART1 then it is possible to be on Shared IO or Dedicated IO which does not have dependency on FPGA.
  // So if we are printing to UART1 and all pins of UART1 is muxed dedicated IO then it does not required FPGA to be programmed first.
  //
  // List of possible uart0.tx pins:
  // pinmux_shared_io_q1_3
  // pinmux_shared_io_q2_11
  // pinmux_shared_io_q3_3
  //
  // List of possible uart0.rx pins:
  // pinmux_shared_io_q1_4
  // pinmux_shared_io_q2_12
  // pinmux_shared_io_q3_4
  //
  // List of possible uart1.tx pins:
  // pinmux_shared_io_q1_7
  // pinmux_shared_io_q3_7
  // pinmux_shared_io_q4_3
  // pinmux_dedicated_io_12
  // pinmux_dedicated_io_16
  //
  // List of possible uart1.rx pins:
  // pinmux_shared_io_q1_8
  // pinmux_shared_io_q3_8
  // pinmux_shared_io_q4_4
  // pinmux_dedicated_io_15
  // pinmux_dedicated_io_17
  //
  // Return TRUE if UEFI is using UART1 and UART1 is using Dedicated Pin
  if ((UINT32)PcdGet64 (PcdSerialRegisterBase) == ALT_UART1_OFST)
  {
    // uart1.tx pin is Dedicated IO ?
    if ((ALT_PINMUX_DCTD_IO_12_SEL_GET(MmioRead32 (ALT_PINMUX_DCTD_IO_GRP_OFST + ALT_PINMUX_DCTD_IO_12_OFST)) == 13) ||
        (ALT_PINMUX_DCTD_IO_16_SEL_GET(MmioRead32 (ALT_PINMUX_DCTD_IO_GRP_OFST + ALT_PINMUX_DCTD_IO_16_OFST)) == 13))
    {
      // uart1.rx pin is Dedicated IO ?
      if ((ALT_PINMUX_DCTD_IO_15_SEL_GET(MmioRead32 (ALT_PINMUX_DCTD_IO_GRP_OFST + ALT_PINMUX_DCTD_IO_15_OFST)) == 13) ||
          (ALT_PINMUX_DCTD_IO_17_SEL_GET(MmioRead32 (ALT_PINMUX_DCTD_IO_GRP_OFST + ALT_PINMUX_DCTD_IO_17_OFST)) == 13))
      {
        return TRUE;
      }
    }
  }
  return FALSE;
}


VOID
EFIAPI
DisplayIo48PinMuxInfo (
  VOID
  )
{
#if (FixedPcdGet32(PcdDebugMsg_Pinmux) > 0)
  CHAR8 meaning_string_of_shared_io_pinmux[304][14] =
  {
    {"N/A"},
    {"spis0.clk"},
    {"spim0.ss1_n"},
    {"sdmmc.data0"},
    {"usb0.clk"},
    {"uart0.cts_n"},
    {"nand.adq0"},
    {"gpio0.io0"},
    {"spis0.mosi"},
    {"spim1.ss1_n"},
    {"sdmmc.cmd"},
    {"usb0.stp"},
    {"uart0.rts_n"},
    {"nand.adq1"},
    {"gpio0.io1"},
    {"i2c1.sda"},
    {"spis0.ss0_n"},
    {"sdmmc.cclk"},
    {"usb0.dir"},
    {"uart0.tx"},
    {"nand.we_n"},
    {"gpio0.io2"},
    {"i2c1.scl"},
    {"spis0.miso"},
    {"sdmmc.data1"},
    {"usb0.data0"},
    {"uart0.rx"},
    {"nand.re_n"},
    {"gpio0.io3"},
    {"i2c0.sda"},
    {"spim0.clk"},
    {"sdmmc.data2"},
    {"usb0.data1"},
    {"qspi.ss2"},
    {"uart1.cts_n"},
    {"nand.wp_n"},
    {"gpio0.io4"},
    {"i2c0.scl"},
    {"spim0.mosi"},
    {"sdmmc.data3"},
    {"usb0.nxt"},
    {"qspi.ss3"},
    {"uart1.rts_n"},
    {"nand.adq2"},
    {"gpio0.io5"},
    {"i2c_emac2.sda"},
    {"emac2.mdio"},
    {"spim0.miso"},
    {"sdmmc.data4"},
    {"usb0.data2"},
    {"uart1.tx"},
    {"nand.adq3"},
    {"gpio0.io6"},
    {"i2c_emac2.scl"},
    {"emac2.mdc"},
    {"spim0.ss0_n"},
    {"sdmmc.data5"},
    {"usb0.data3"},
    {"uart1.rx"},
    {"nand.cle"},
    {"gpio0.io7"},
    {"i2c_emac1.sda"},
    {"emac1.mdio"},
    {"spis1.clk"},
    {"spim1.clk"},
    {"sdmmc.data6"},
    {"usb0.data4"},
    {"nand.adq4"},
    {"gpio0.io8"},
    {"i2c_emac1.scl"},
    {"emac1.mdc"},
    {"spis1.mosi"},
    {"spim1.mosi"},
    {"sdmmc.data7"},
    {"usb0.data5"},
    {"nand.adq5"},
    {"gpio0.io9"},
    {"i2c_emac0.sda"},
    {"emac0.mdio"},
    {"spis1.ss0_n"},
    {"spim1.miso"},
    {"usb0.data6"},
    {"nand.adq6"},
    {"gpio0.io10"},
    {"i2c_emac0.scl"},
    {"emac0.mdc"},
    {"spis1.miso"},
    {"spim1.ss0_n"},
    {"usb0.data7"},
    {"nand.adq7"},
    {"gpio0.io11"},
    {"emac0.tx_clk"},
    {"usb1.clk"},
    {"nand.ale"},
    {"gpio0.io12"},
    {"emac0.tx_ctl"},
    {"usb1.stp"},
    {"nand.rb"},
    {"gpio0.io13"},
    {"emac0.rx_clk"},
    {"usb1.dir"},
    {"nand.ce_n"},
    {"gpio0.io14"},
    {"emac0.rx_ctl"},
    {"usb1.data0"},
    {"gpio0.io15"},
    {"emac0.txd0"},
    {"usb1.data1"},
    {"nand.adq8"},
    {"gpio0.io16"},
    {"emac0.txd1"},
    {"usb1.nxt"},
    {"nand.adq9"},
    {"gpio0.io17"},
    {"emac0.rxd0"},
    {"usb1.data2"},
    {"nand.adq10"},
    {"gpio0.io18"},
    {"spim1.ss1_n"},
    {"emac0.rxd1"},
    {"usb1.data3"},
    {"nand.adq11"},
    {"gpio0.io19"},
    {"i2c1.sda"},
    {"spis0.clk"},
    {"spim1.clk"},
    {"emac0.txd2"},
    {"usb1.data4"},
    {"uart0.cts_n"},
    {"nand.adq12"},
    {"gpio0.io20"},
    {"i2c1.scl"},
    {"spis0.mosi"},
    {"spim1.mosi"},
    {"emac0.txd3"},
    {"usb1.data5"},
    {"uart0.rts_n"},
    {"nand.adq13"},
    {"gpio0.io21"},
    {"i2c0.sda"},
    {"spis0.ss0_n"},
    {"spim1.miso"},
    {"emac0.rxd2"},
    {"usb1.data6"},
    {"uart0.tx"},
    {"nand.adq14"},
    {"gpio0.io22"},
    {"i2c0.scl"},
    {"spis0.miso"},
    {"spim1.ss0_n"},
    {"emac0.rxd3"},
    {"usb1.data7"},
    {"uart0.rx"},
    {"nand.adq15"},
    {"gpio0.io23"},
    {"spim1.clk"},
    {"emac1.tx_clk"},
    {"uart0.cts_n"},
    {"nand.adq0"},
    {"gpio1.io0"},
    {"spim1.mosi"},
    {"emac1.tx_ctl"},
    {"uart0.rts_n"},
    {"nand.adq1"},
    {"gpio1.io1"},
    {"i2c0.sda"},
    {"spim1.miso"},
    {"emac1.rx_clk"},
    {"uart0.tx"},
    {"nand.we_n"},
    {"gpio1.io2"},
    {"i2c0.scl"},
    {"spim1.ss0_n"},
    {"emac1.rx_ctl"},
    {"uart0.rx"},
    {"nand.re_n"},
    {"gpio1.io3"},
    {"spis1.clk"},
    {"spim1.ss1_n"},
    {"emac1.txd0"},
    {"uart1.cts_n"},
    {"nand.wp_n"},
    {"gpio1.io4"},
    {"spis1.mosi"},
    {"emac1.txd1"},
    {"uart1.rts_n"},
    {"nand.adq2"},
    {"gpio1.io5"},
    {"i2c1.sda"},
    {"spis1.ss0_n"},
    {"emac1.rxd0"},
    {"uart1.tx"},
    {"nand.adq3"},
    {"gpio1.io6"},
    {"i2c1.scl"},
    {"spis1.miso"},
    {"emac1.rxd1"},
    {"uart1.rx"},
    {"nand.cle"},
    {"gpio1.io7"},
    {"i2c_emac2.sda"},
    {"emac2.mdio"},
    {"spis0.clk"},
    {"emac1.txd2"},
    {"nand.adq4"},
    {"gpio1.io8"},
    {"i2c_emac2.scl"},
    {"emac2.mdc"},
    {"spis0.mosi"},
    {"emac1.txd3"},
    {"nand.adq5"},
    {"gpio1.io9"},
    {"i2c_emac0.sda"},
    {"emac0.mdio"},
    {"spis0.ss0_n"},
    {"emac1.rxd2"},
    {"nand.adq6"},
    {"gpio1.io10"},
    {"i2c_emac0.scl"},
    {"emac0.mdc"},
    {"spis0.miso"},
    {"emac1.rxd3"},
    {"nand.adq7"},
    {"gpio1.io11"},
    {"i2c1.sda"},
    {"sdmmc.data0"},
    {"emac2.tx_clk"},
    {"nand.ale"},
    {"gpio1.io12"},
    {"i2c1.scl"},
    {"sdmmc.cmd"},
    {"emac2.tx_ctl"},
    {"nand.rb"},
    {"gpio1.io13"},
    {"sdmmc.cclk"},
    {"emac2.rx_clk"},
    {"uart1.tx"},
    {"nand.ce_n"},
    {"gpio1.io14"},
    {"sdmmc.data1"},
    {"emac2.rx_ctl"},
    {"trace.clk"},
    {"uart1.rx"},
    {"gpio1.io15"},
    {"sdmmc.data2"},
    {"emac2.txd0"},
    {"qspi.ss2"},
    {"uart1.cts_n"},
    {"nand.adq8"},
    {"gpio1.io16"},
    {"spim0.ss1_n"},
    {"sdmmc.data3"},
    {"emac2.txd1"},
    {"qspi.ss3"},
    {"uart1.rts_n"},
    {"nand.adq9"},
    {"gpio1.io17"},
    {"i2c_emac1.sda"},
    {"emac1.mdio"},
    {"spim0.miso"},
    {"sdmmc.data4"},
    {"emac2.rxd0"},
    {"nand.adq10"},
    {"gpio1.io18"},
    {"i2c_emac1.scl"},
    {"emac1.mdc"},
    {"spim0.ss0_n"},
    {"sdmmc.data5"},
    {"emac2.rxd1"},
    {"trace.clk"},
    {"nand.adq11"},
    {"gpio1.io19"},
    {"i2c_emac2.sda"},
    {"spis1.clk"},
    {"spim0.clk"},
    {"sdmmc.data6"},
    {"emac2.txd2"},
    {"trace.d0"},
    {"nand.adq12"},
    {"gpio1.io20"},
    {"i2c_emac2.scl"},
    {"spis1.mosi"},
    {"spim0.mosi"},
    {"sdmmc.data7"},
    {"emac2.txd3"},
    {"trace.d1"},
    {"nand.adq13"},
    {"gpio1.io21"},
    {"i2c_emac0.sda"},
    {"emac0.mdio"},
    {"spis1.ss0_n"},
    {"spim0.miso"},
    {"emac2.rxd2"},
    {"trace.d2"},
    {"nand.adq14"},
    {"gpio1.io22"},
    {"i2c_emac0.scl"},
    {"emac0.mdc"},
    {"spis1.miso"},
    {"spim0.ss0_n"},
    {"emac2.rxd3"},
    {"trace.d3"},
    {"nand.adq15"},
    {"gpio1.io23"}
  };

  UINT8 decode_meaning_of_shared_io_q1_pinmux[12][16] =
                {
                  { //shared_io_q1_1
                    0, // 0
                    0, // 1
                    1, // 2
                    2, // 3
                    3, // 4
                    0, // 5
                    0, // 6
                    0, // 7
                    4, // 8
                    0, // 9
                    0, // 10
                    0, // 11
                    0, // 12
                    5, // 13
                    6, // 14
                    7  // 15
                  },
                  { //shared_io_q1_2
                    0, // 0
                    0, // 1
                    8, // 2
                    9, // 3
                    10, // 4
                    0, // 5
                    0, // 6
                    0, // 7
                    11, // 8
                    0, // 9
                    0, // 10
                    0, // 11
                    0, // 12
                    12, // 13
                    13, // 14
                    14  // 15
                  },
                  { //shared_io_q1_3
                    15, // 0
                    0, // 1
                    16, // 2
                    0, // 3
                    17, // 4
                    0, // 5
                    0, // 6
                    0, // 7
                    18, // 8
                    0, // 9
                    0, // 10
                    0, // 11
                    0, // 12
                    19, // 13
                    20, // 14
                    21  // 15
                  },
                  { //shared_io_q1_4
                    22, // 0
                    0, // 1
                    23, // 2
                    0, // 3
                    24, // 4
                    0, // 5
                    0, // 6
                    0, // 7
                    25, // 8
                    0, // 9
                    0, // 10
                    0, // 11
                    0, // 12
                    26, // 13
                    27, // 14
                    28  // 15
                  },
                  { //shared_io_q1_5
                    29, // 0
                    0, // 1
                    0, // 2
                    30, // 3
                    31, // 4
                    0, // 5
                    0, // 6
                    0, // 7
                    32, // 8
                    0, // 9
                    0, // 10
                    0, // 11
                    33, // 12
                    34, // 13
                    35, // 14
                    36  // 15
                  },
                  { //shared_io_q1_6
                    37, // 0
                    0, // 1
                    0, // 2
                    38, // 3
                    39, // 4
                    0, // 5
                    0, // 6
                    0, // 7
                    40, // 8
                    0, // 9
                    0, // 10
                    0, // 11
                    41, // 12
                    42, // 13
                    43, // 14
                    44  // 15
                  },
                  { //shared_io_q1_7
                    45, // 0
                    46, // 1
                    0, // 2
                    47, // 3
                    48, // 4
                    0, // 5
                    0, // 6
                    0, // 7
                    49, // 8
                    0, // 9
                    0, // 10
                    0, // 11
                    0, // 12
                    50, // 13
                    51, // 14
                    52  // 15
                  },
                  { //shared_io_q1_8
                    53, // 0
                    54, // 1
                    0, // 2
                    55, // 3
                    56, // 4
                    0, // 5
                    0, // 6
                    0, // 7
                    57, // 8
                    0, // 9
                    0, // 10
                    0, // 11
                    0, // 12
                    58, // 13
                    59, // 14
                    60  // 15
                  },
                  { //shared_io_q1_9
                    61, // 0
                    62, // 1
                    63, // 2
                    64, // 3
                    65, // 4
                    0, // 5
                    0, // 6
                    0, // 7
                    66, // 8
                    0, // 9
                    0, // 10
                    0, // 11
                    0, // 12
                    0, // 13
                    67, // 14
                    68  // 15
                  },
                  { //shared_io_q1_10
                    69, // 0
                    70, // 1
                    71, // 2
                    72, // 3
                    73, // 4
                    0, // 5
                    0, // 6
                    0, // 7
                    74, // 8
                    0, // 9
                    0, // 10
                    0, // 11
                    0, // 12
                    0, // 13
                    75, // 14
                    76  // 15
                  },
                  { //shared_io_q1_11
                    77, // 0
                    78, // 1
                    79, // 2
                    80, // 3
                    0, // 4
                    0, // 5
                    0, // 6
                    0, // 7
                    81, // 8
                    0, // 9
                    0, // 10
                    0, // 11
                    0, // 12
                    0, // 13
                    82, // 14
                    83  // 15
                  },
                  { //shared_io_q1_12
                    84, // 0
                    85, // 1
                    86, // 2
                    87, // 3
                    0, // 4
                    0, // 5
                    0, // 6
                    0, // 7
                    88, // 8
                    0, // 9
                    0, // 10
                    0, // 11
                    0, // 12
                    0, // 13
                    89, // 14
                    90  // 15
                  }
                };
  UINT8 decode_meaning_of_shared_io_q2_pinmux[12][16] =
                {
                  { //shared_io_q2_1
                    0, // 0
                    0, // 1
                    0, // 2
                    0, // 3
                    91, // 4
                    0, // 5
                    0, // 6
                    0, // 7
                    92, // 8
                    0, // 9
                    0, // 10
                    0, // 11
                    0, // 12
                    0, // 13
                    93, // 14
                    94  // 15
                  },
                  { //shared_io_q2_2
                    0, // 0
                    0, // 1
                    0, // 2
                    0, // 3
                    95, // 4
                    0, // 5
                    0, // 6
                    0, // 7
                    96, // 8
                    0, // 9
                    0, // 10
                    0, // 11
                    0, // 12
                    0, // 13
                    97, // 14
                    98  // 15
                  },
                  { //shared_io_q2_3
                    0, // 0
                    0, // 1
                    0, // 2
                    0, // 3
                    99, // 4
                    0, // 5
                    0, // 6
                    0, // 7
                    100, // 8
                    0, // 9
                    0, // 10
                    0, // 11
                    0, // 12
                    0, // 13
                    101, // 14
                    102  // 15
                  },
                  { //shared_io_q2_4
                    0, // 0
                    0, // 1
                    0, // 2
                    0, // 3
                    103, // 4
                    0, // 5
                    0, // 6
                    0, // 7
                    104, // 8
                    0, // 9
                    0, // 10
                    0, // 11
                    0, // 12
                    0, // 13
                    0, // 14
                    105  // 15
                  },
                  { //shared_io_q2_5
                    0, // 0
                    0, // 1
                    0, // 2
                    0, // 3
                    106, // 4
                    0, // 5
                    0, // 6
                    0, // 7
                    107, // 8
                    0, // 9
                    0, // 10
                    0, // 11
                    0, // 12
                    0, // 13
                    108, // 14
                    109  // 15
                  },
                  { //shared_io_q2_6
                    0, // 0
                    0, // 1
                    0, // 2
                    0, // 3
                    110, // 4
                    0, // 5
                    0, // 6
                    0, // 7
                    111, // 8
                    0, // 9
                    0, // 10
                    0, // 11
                    0, // 12
                    0, // 13
                    112, // 14
                    113  // 15
                  },
                  { //shared_io_q2_7
                    0, // 0
                    0, // 1
                    0, // 2
                    0, // 3
                    114, // 4
                    0, // 5
                    0, // 6
                    0, // 7
                    115, // 8
                    0, // 9
                    0, // 10
                    0, // 11
                    0, // 12
                    0, // 13
                    116, // 14
                    117  // 15
                  },
                  { //shared_io_q2_8
                    0, // 0
                    0, // 1
                    0, // 2
                    118, // 3
                    119, // 4
                    0, // 5
                    0, // 6
                    0, // 7
                    120, // 8
                    0, // 9
                    0, // 10
                    0, // 11
                    0, // 12
                    0, // 13
                    121, // 14
                    122  // 15
                  },
                  { //shared_io_q2_9
                    123, // 0
                    0, // 1
                    124, // 2
                    125, // 3
                    126, // 4
                    0, // 5
                    0, // 6
                    0, // 7
                    127, // 8
                    0, // 9
                    0, // 10
                    0, // 11
                    0, // 12
                    128, // 13
                    129, // 14
                    130 // 15
                  },
                  { //shared_io_q2_10
                    131, // 0
                    0, // 1
                    132, // 2
                    133, // 3
                    134, // 4
                    0, // 5
                    0, // 6
                    0, // 7
                    135, // 8
                    0, // 9
                    0, // 10
                    0, // 11
                    0, // 12
                    136, // 13
                    137, // 14
                    138 // 15
                  },
                  { //shared_io_q2_11
                    139, // 0
                    0, // 1
                    140, // 2
                    141, // 3
                    142, // 4
                    0, // 5
                    0, // 6
                    0, // 7
                    143, // 8
                    0, // 9
                    0, // 10
                    0, // 11
                    0, // 12
                    144, // 13
                    145, // 14
                    146 // 15
                  },
                  { //shared_io_q2_12
                    147, // 0
                    0, // 1
                    148, // 2
                    149, // 3
                    150, // 4
                    0, // 5
                    0, // 6
                    0, // 7
                    151, // 8
                    0, // 9
                    0, // 10
                    0, // 11
                    0, // 12
                    152, // 13
                    153, // 14
                    154 // 15
                  }
                };
  UINT8 decode_meaning_of_shared_io_q3_pinmux[12][16] =
                {
                  { //shared_io_q3_1
                    0, // 0
                    0, // 1
                    0, // 2
                    155, // 3
                    0, // 4
                    0, // 5
                    0, // 6
                    0, // 7
                    156, // 8
                    0, // 9
                    0, // 10
                    0, // 11
                    0, // 12
                    157, // 13
                    158, // 14
                    159 // 15
                  },
                  { //shared_io_q3_2
                    0, // 0
                    0, // 1
                    0, // 2
                    160, // 3
                    0, // 4
                    0, // 5
                    0, // 6
                    0, // 7
                    161, // 8
                    0, // 9
                    0, // 10
                    0, // 11
                    0, // 12
                    162, // 13
                    163, // 14
                    164 // 15
                  },
                  { //shared_io_q3_3
                    165, // 0
                    0, // 1
                    0, // 2
                    166, // 3
                    0, // 4
                    0, // 5
                    0, // 6
                    0, // 7
                    167, // 8
                    0, // 9
                    0, // 10
                    0, // 11
                    0, // 12
                    168, // 13
                    169, // 14
                    170 // 15
                  },
                  { //shared_io_q3_4
                    171, // 0
                    0, // 1
                    0, // 2
                    172, // 3
                    0, // 4
                    0, // 5
                    0, // 6
                    0, // 7
                    173, // 8
                    0, // 9
                    0, // 10
                    0, // 11
                    0, // 12
                    174, // 13
                    175, // 14
                    176 // 15
                  },
                  { //shared_io_q3_5
                    0, // 0
                    0, // 1
                    177, // 2
                    178, // 3
                    0, // 4
                    0, // 5
                    0, // 6
                    0, // 7
                    179, // 8
                    0, // 9
                    0, // 10
                    0, // 11
                    0, // 12
                    180, // 13
                    181, // 14
                    182 // 15
                  },
                  { //shared_io_q3_6
                    0, // 0
                    0, // 1
                    183, // 2
                    0, // 3
                    0, // 4
                    0, // 5
                    0, // 6
                    0, // 7
                    184, // 8
                    0, // 9
                    0, // 10
                    0, // 11
                    0, // 12
                    185, // 13
                    186, // 14
                    187 // 15
                  },
                  { //shared_io_q3_7
                    188, // 0
                    0, // 1
                    189, // 2
                    0, // 3
                    0, // 4
                    0, // 5
                    0, // 6
                    0, // 7
                    190, // 8
                    0, // 9
                    0, // 10
                    0, // 11
                    0, // 12
                    191, // 13
                    192, // 14
                    193 // 15
                  },
                  { //shared_io_q3_8
                    194, // 0
                    0, // 1
                    195, // 2
                    0, // 3
                    0, // 4
                    0, // 5
                    0, // 6
                    0, // 7
                    196, // 8
                    0, // 9
                    0, // 10
                    0, // 11
                    0, // 12
                    197, // 13
                    198, // 14
                    199 // 15
                  },
                  { //shared_io_q3_9
                    200, // 0
                    201, // 1
                    202, // 2
                    0, // 3
                    0, // 4
                    0, // 5
                    0, // 6
                    0, // 7
                    203, // 8
                    0, // 9
                    0, // 10
                    0, // 11
                    0, // 12
                    0, // 13
                    204, // 14
                    205 // 15
                  },
                  { //shared_io_q3_10
                    206, // 0
                    207, // 1
                    208, // 2
                    0, // 3
                    0, // 4
                    0, // 5
                    0, // 6
                    0, // 7
                    209, // 8
                    0, // 9
                    0, // 10
                    0, // 11
                    0, // 12
                    0, // 13
                    210, // 14
                    211 // 15
                  },
                  { //shared_io_q3_11
                    212, // 0
                    213, // 1
                    214, // 2
                    0, // 3
                    0, // 4
                    0, // 5
                    0, // 6
                    0, // 7
                    215, // 8
                    0, // 9
                    0, // 10
                    0, // 11
                    0, // 12
                    0, // 13
                    216, // 14
                    217 // 15
                  },
                  { //shared_io_q3_12
                    218, // 0
                    219, // 1
                    220, // 2
                    0, // 3
                    0, // 4
                    0, // 5
                    0, // 6
                    0, // 7
                    221, // 8
                    0, // 9
                    0, // 10
                    0, // 11
                    0, // 12
                    0, // 13
                    222, // 14
                    223 // 15
                  }
                };
  UINT16 decode_meaning_of_shared_io_q4_pinmux[12][16] =
                {
                  { //shared_io_q4_1
                    224, // 0
                    0, // 1
                    0, // 2
                    0, // 3
                    225, // 4
                    0, // 5
                    0, // 6
                    0, // 7
                    226, // 8
                    0, // 9
                    0, // 10
                    0, // 11
                    0, // 12
                    0, // 13
                    227, // 14
                    228 // 15
                  },
                  { //shared_io_q4_2
                    229, // 0
                    0, // 1
                    0, // 2
                    0, // 3
                    230, // 4
                    0, // 5
                    0, // 6
                    0, // 7
                    231, // 8
                    0, // 9
                    0, // 10
                    0, // 11
                    0, // 12
                    0, // 13
                    232, // 14
                    233 // 15
                  },
                  { //shared_io_q4_3
                    0, // 0
                    0, // 1
                    0, // 2
                    0, // 3
                    234, // 4
                    0, // 5
                    0, // 6
                    0, // 7
                    235, // 8
                    0, // 9
                    0, // 10
                    0, // 11
                    0, // 12
                    236, // 13
                    237, // 14
                    238 // 15
                  },
                  { //shared_io_q4_4
                    0, // 0
                    0, // 1
                    0, // 2
                    0, // 3
                    239, // 4
                    0, // 5
                    0, // 6
                    0, // 7
                    240, // 8
                    0, // 9
                    0, // 10
                    0, // 11
                    241, // 12
                    242, // 13
                    0, // 14
                    243 // 15
                  },
                  { //shared_io_q4_5
                    0, // 0
                    0, // 1
                    0, // 2
                    0, // 3
                    244, // 4
                    0, // 5
                    0, // 6
                    0, // 7
                    245, // 8
                    0, // 9
                    0, // 10
                    0, // 11
                    246, // 12
                    247, // 13
                    248, // 14
                    249 // 15
                  },
                  { //shared_io_q4_6
                    0, // 0
                    0, // 1
                    0, // 2
                    250, // 3
                    251, // 4
                    0, // 5
                    0, // 6
                    0, // 7
                    252, // 8
                    0, // 9
                    0, // 10
                    0, // 11
                    253, // 12
                    254, // 13
                    255, // 14
                    256 // 15
                  },
                  { //shared_io_q4_7
                    257, // 0
                    258, // 1
                    0, // 2
                    259, // 3
                    260, // 4
                    0, // 5
                    0, // 6
                    0, // 7
                    261, // 8
                    0, // 9
                    0, // 10
                    0, // 11
                    0, // 12
                    0, // 13
                    262, // 14
                    263 // 15
                  },
                  { //shared_io_q4_8
                    264, // 0
                    265, // 1
                    0, // 2
                    266, // 3
                    267, // 4
                    0, // 5
                    0, // 6
                    0, // 7
                    268, // 8
                    0, // 9
                    0, // 10
                    0, // 11
                    269, // 12
                    0, // 13
                    270, // 14
                    271 // 15
                  },
                  { //shared_io_q4_9
                    272, // 0
                    0, // 1
                    273, // 2
                    274, // 3
                    275, // 4
                    0, // 5
                    0, // 6
                    0, // 7
                    276, // 8
                    0, // 9
                    0, // 10
                    0, // 11
                    277, // 12
                    0, // 13
                    278, // 14
                    279 // 15
                  },
                  { //shared_io_q4_10
                    280, // 0
                    0, // 1
                    281, // 2
                    282, // 3
                    283, // 4
                    0, // 5
                    0, // 6
                    0, // 7
                    284, // 8
                    0, // 9
                    0, // 10
                    0, // 11
                    285, // 12
                    0, // 13
                    286, // 14
                    287 // 15
                  },
                  { //shared_io_q4_11
                    288, // 0
                    289, // 1
                    290, // 2
                    291, // 3
                    0, // 4
                    0, // 5
                    0, // 6
                    0, // 7
                    292, // 8
                    0, // 9
                    0, // 10
                    0, // 11
                    293, // 12
                    0, // 13
                    294, // 14
                    295 // 15
                  },
                  { //shared_io_q4_12
                    296, // 0
                    297, // 1
                    298, // 2
                    299, // 3
                    0, // 4
                    0, // 5
                    0, // 6
                    0, // 7
                    300, // 8
                    0, // 9
                    0, // 10
                    0, // 11
                    301, // 12
                    0, // 13
                    302, // 14
                    303 // 15
                  }
                };

  CHAR8 meaning_string_of_dedicated_pinmux[81][14] = {
    {"N/A"},
    {"qspi.clk"},
    {"sdmmc.data0"},
    {"nand.adq0"},
    {"gpio2.io0"},
    {"qspi.io0"},
    {"sdmmc.cmd"},
    {"nand.adq1"},
    {"gpio2.io1"},
    {"qspi.ss0"},
    {"sdmmc.cclk"},
    {"nand.we_n"},
    {"gpio2.io2"},
    {"qspi.io1"},
    {"sdmmc.data1"},
    {"nand.re_n"},
    {"gpio2.io3"},
    {"qspi.io2_wpn"},
    {"sdmmc.data2"},
    {"nand.adq2"},
    {"gpio2.io4"},
    {"qspi.io3_hold"},
    {"sdmmc.data3"},
    {"nand.adq3"},
    {"gpio2.io5"},
    {"spis0.miso"},
    {"spim0.ss1_n"},
    {"sdmmc.pwr_ena"},
    {"nand.cle"},
    {"gpio2.io6"},
    {"spim0.clk"},
    {"cm.pll_clk0"},
    {"qspi.ss1"},
    {"nand.ale"},
    {"gpio2.io7"},
    {"i2c_emac1.sda"},
    {"emac1.mdio"},
    {"spim0.mosi"},
    {"cm.pll_clk1"},
    {"sdmmc.data4"},
    {"uart1.tx"},
    {"nand.rb"},
    {"gpio2.io8"},
    {"i2c_emac1.scl"},
    {"emac1.mdc"},
    {"spim0.miso"},
    {"cm.pll_clk2"},
    {"sdmmc.data5"},
    {"uart1.rts_n"},
    {"nand.ce_n"},
    {"gpio2.io9"},
    {"i2c_emac2.sda"},
    {"emac2.mdio"},
    {"spim0.ss0_n"},
    {"cm.pll_clk3"},
    {"sdmmc.data6"},
    {"uart1.cts_n"},
    {"nand.adq4"},
    {"gpio2.io10"},
    {"i2c_emac2.scl"},
    {"emac2.mdc"},
    {"spis0.clk"},
    {"cm.pll_clk4"},
    {"sdmmc.data7"},
    {"uart1.rx"},
    {"nand.adq5"},
    {"gpio2.io11"},
    {"i2c_emac0.sda"},
    {"emac0.mdio"},
    {"spis0.mosi"},
    {"qspi.ss2"},
    {"uart1.tx"},
    {"nand.adq6"},
    {"gpio2.io12"},
    {"i2c_emac0.scl"},
    {"emac0.mdc"},
    {"spis0.ss0_n"},
    {"qspi.ss3"},
    {"uart1.rx"},
    {"nand.adq7"},
    {"gpio2.io13"}
  };

  UINT8 decode_meaning_of_dedicated_pinmux[14][16] =
                {
                  { //dedicated_io = 4
                    0, // 0
                    0, // 1
                    0, // 2
                    0, // 3
                    1, // 4
                    0, // 5
                    0, // 6
                    0, // 7
                    2, // 8
                    0, // 9
                    0, // 10
                    0, // 11
                    0, // 12
                    0, // 13
                    3, // 14
                    4  // 15
                  },
                  { //dedicated_io = 5
                    0, // 0
                    0, // 1
                    0, // 2
                    0, // 3
                    5, // 4
                    0, // 5
                    0, // 6
                    0, // 7
                    6, // 8
                    0, // 9
                    0, // 10
                    0, // 11
                    0, // 12
                    0, // 13
                    7, // 14
                    8  // 15
                  },
                  { //dedicated_io = 6
                    0, // 0
                    0, // 1
                    0, // 2
                    0, // 3
                    9, // 4
                    0, // 5
                    0, // 6
                    0, // 7
                    10, // 8
                    0, // 9
                    0, // 10
                    0, // 11
                    0, // 12
                    0, // 13
                    11, // 14
                    12  // 15
                  },
                  { //dedicated_io = 7
                    0, // 0
                    0, // 1
                    0, // 2
                    0, // 3
                    13, // 4
                    0, // 5
                    0, // 6
                    0, // 7
                    14, // 8
                    0, // 9
                    0, // 10
                    0, // 11
                    0, // 12
                    0, // 13
                    15, // 14
                    16  // 15
                  },
                  { //dedicated_io = 8
                    0, // 0
                    0, // 1
                    0, // 2
                    0, // 3
                    17, // 4
                    0, // 5
                    0, // 6
                    0, // 7
                    18, // 8
                    0, // 9
                    0, // 10
                    0, // 11
                    0, // 12
                    0, // 13
                    19, // 14
                    20  // 15
                  },
                  { //dedicated_io = 9
                    0, // 0
                    0, // 1
                    0, // 2
                    0, // 3
                    21, // 4
                    0, // 5
                    0, // 6
                    0, // 7
                    22, // 8
                    0, // 9
                    0, // 10
                    0, // 11
                    0, // 12
                    0, // 13
                    23, // 14
                    24  // 15
                  },
                  { //dedicated_io = 10
                    0, // 0
                    0, // 1
                    25, // 2
                    26, // 3
                    0, // 4
                    0, // 5
                    0, // 6
                    0, // 7
                    27, // 8
                    0, // 9
                    0, // 10
                    0, // 11
                    0, // 12
                    0, // 13
                    28, // 14
                    29  // 15
                  },
                  { //dedicated_io = 11
                    0, // 0
                    0, // 1
                    0, // 2
                    30, // 3
                    31, // 4
                    0, // 5
                    0, // 6
                    0, // 7
                    32, // 8
                    0, // 9
                    0, // 10
                    0, // 11
                    0, // 12
                    0, // 13
                    33, // 14
                    34  // 15
                  },
                  { //dedicated_io = 12
                    35, // 0
                    36, // 1
                    0, // 2
                    37, // 3
                    38, // 4
                    0, // 5
                    0, // 6
                    0, // 7
                    39, // 8
                    0, // 9
                    0, // 10
                    0, // 11
                    0, // 12
                    40, // 13
                    41, // 14
                    42  // 15
                  },
                  { //dedicated_io = 13
                    43, // 0
                    44, // 1
                    0, // 2
                    45, // 3
                    46, // 4
                    0, // 5
                    0, // 6
                    0, // 7
                    47, // 8
                    0, // 9
                    0, // 10
                    0, // 11
                    0, // 12
                    48, // 13
                    49, // 14
                    50  // 15
                  },
                  { //dedicated_io = 14
                    51, // 0
                    52, // 1
                    0, // 2
                    53, // 3
                    54, // 4
                    0, // 5
                    0, // 6
                    0, // 7
                    55, // 8
                    0, // 9
                    0, // 10
                    0, // 11
                    0, // 12
                    56, // 13
                    57, // 14
                    58  // 15
                  },
                  { //dedicated_io = 15
                    59, // 0
                    60, // 1
                    61, // 2
                    0, // 3
                    62, // 4
                    0, // 5
                    0, // 6
                    0, // 7
                    63, // 8
                    0, // 9
                    0, // 10
                    0, // 11
                    0, // 12
                    64, // 13
                    65, // 14
                    66  // 15
                  },
                  { //dedicated_io = 16
                    67, // 0
                    68, // 1
                    69, // 2
                    0, // 3
                    0, // 4
                    0, // 5
                    0, // 6
                    0, // 7
                    70, // 8
                    0, // 9
                    0, // 10
                    0, // 11
                    0, // 12
                    71, // 13
                    72, // 14
                    73  // 15
                  },
                  { //dedicated_io = 17
                    74, // 0
                    75, // 1
                    76, // 2
                    0, // 3
                    0, // 4
                    0, // 5
                    0, // 6
                    0, // 7
                    77, // 8
                    0, // 9
                    0, // 10
                    0, // 11
                    0, // 12
                    78, // 13
                    79, // 14
                    80  // 15
                  }
                };
  UINTN           i;
  UINT32          Data32;

  // Char8Str is use to speed up Semihosting printing speed
  CHAR8   Char8Str[2048];
  CHAR8*  Char8Ptr = &Char8Str[0];

  // Display Pin Mux settings for shared 3V IO Group
  Char8Ptr += AsciiSPrint (Char8Ptr, 1024,
    "Pin Mux settings for shared 3V IO:\r\n");
  for (i = 0; i < 12; i++)
  {
    Data32 = MmioRead32 (ALT_PINMUX_SHARED_3V_IO_GRP_OFST + ALT_PINMUX_SHARED_3V_IO_Q1_1_OFST + (i * 4));
    Char8Ptr += AsciiSPrint (Char8Ptr, 1024,
                "\t pinmux_shared_io_q1_%d (GPIO0_IO%d) :\t%a (%d)\r\n", (i + 1), (i + 0),
                meaning_string_of_shared_io_pinmux[
                  decode_meaning_of_shared_io_q1_pinmux[i][ALT_PINMUX_SHARED_3V_IO_Q1_1_SEL_GET(Data32)]
                ],
                Data32);
  }
  InfoPrint ("%a", Char8Str); Char8Ptr = &Char8Str[0];
  for (i = 0; i < 12; i++)
  {
    Data32 = MmioRead32 (ALT_PINMUX_SHARED_3V_IO_GRP_OFST + ALT_PINMUX_SHARED_3V_IO_Q2_1_OFST + (i * 4));
    Char8Ptr += AsciiSPrint (Char8Ptr, 1024,
                "\t pinmux_shared_io_q2_%d (GPIO0_IO%d) :\t%a (%d)\r\n", (i + 1), (i + 12),
                meaning_string_of_shared_io_pinmux[
                  decode_meaning_of_shared_io_q2_pinmux[i][ALT_PINMUX_SHARED_3V_IO_Q2_1_SEL_GET(Data32)]
                ],
                Data32);
  }
  InfoPrint ("%a", Char8Str); Char8Ptr = &Char8Str[0];
  for (i = 0; i < 12; i++)
  {
    Data32 = MmioRead32 (ALT_PINMUX_SHARED_3V_IO_GRP_OFST + ALT_PINMUX_SHARED_3V_IO_Q3_1_OFST + (i * 4));
    Char8Ptr += AsciiSPrint (Char8Ptr, 1024,
                "\t pinmux_shared_io_q3_%d (GPIO1_IO%d) :\t%a (%d)\r\n", (i + 1), (i + 0),
                meaning_string_of_shared_io_pinmux[
                  decode_meaning_of_shared_io_q3_pinmux[i][ALT_PINMUX_SHARED_3V_IO_Q3_1_SEL_GET(Data32)]
                ],
                Data32);
  }
  InfoPrint ("%a", Char8Str); Char8Ptr = &Char8Str[0];
  for (i = 0; i < 12; i++)
  {
    Data32 = MmioRead32 (ALT_PINMUX_SHARED_3V_IO_GRP_OFST + ALT_PINMUX_SHARED_3V_IO_Q4_1_OFST + (i * 4));
    Char8Ptr += AsciiSPrint (Char8Ptr, 1024,
                "\t pinmux_shared_io_q4_%d (GPIO1_IO%d) :\t%a (%d)\r\n", (i + 1), (i + 12),
                meaning_string_of_shared_io_pinmux[
                  decode_meaning_of_shared_io_q4_pinmux[i][ALT_PINMUX_SHARED_3V_IO_Q4_1_SEL_GET(Data32)]
                ],
                Data32);
  }
  InfoPrint ("%a", Char8Str); Char8Ptr = &Char8Str[0];

  // Display Pin Mux settings for dedicated IO
  InfoPrint ("Dedicated IO settings:\r\n");

  // Display Voltage settings for Bank
  Data32 = MmioRead32 (ALT_PINMUX_DCTD_IO_GRP_OFST + ALT_PINMUX_DCTD_IO_CFG_BANK_OFST);
  Char8Ptr += AsciiSPrint (Char8Ptr, 1024,
    "\t configuration_dedicated_io_bank:\t0x%08x\r\n", Data32);
  Char8Ptr += AsciiSPrint (Char8Ptr, 1024,
    "\t\tCLK and RST IO operation: ");
  switch (ALT_PINMUX_DCTD_IO_CFG_BANK_VOLTAGE_SEL_CLKRST_IO_GET(Data32))
  {
    case 0: Char8Ptr += AsciiSPrint (Char8Ptr, 1024, "3.0V\r\n"); break;
    case 1: Char8Ptr += AsciiSPrint (Char8Ptr, 1024, "1.8V\r\n"); break;
    case 2: Char8Ptr += AsciiSPrint (Char8Ptr, 1024, "2.5V\r\n"); break;
    case 3: Char8Ptr += AsciiSPrint (Char8Ptr, 1024, "RSVD\r\n"); break;
  }
  Char8Ptr += AsciiSPrint (Char8Ptr, 1024, "\t\tPeripheral IO operation: ");
  switch (ALT_PINMUX_DCTD_IO_CFG_BANK_VOLTAGE_SEL_PERI_IO_GET(Data32))
  {
    case 0: Char8Ptr += AsciiSPrint (Char8Ptr, 1024, "3.0V\r\n"); break;
    case 1: Char8Ptr += AsciiSPrint (Char8Ptr, 1024, "1.8V\r\n"); break;
    case 2: Char8Ptr += AsciiSPrint (Char8Ptr, 1024, "2.5V\r\n"); break;
    case 3: Char8Ptr += AsciiSPrint (Char8Ptr, 1024, "RSVD\r\n"); break;
  }

  // Display Voltage settings for pins without Pin Mux
  for (i = 0; i < 3; i++)
  {
    Data32 = MmioRead32 (ALT_PINMUX_DCTD_IO_GRP_OFST + ALT_PINMUX_DCTD_IO_CFG_1_OFST + (i * 4));
    Char8Ptr += AsciiSPrint (Char8Ptr, 1024,
                " - configuration_dedicated_io_%d   :\t0x%08x\r\n", (i + 1),
                Data32);
    switch (i)
    {
      case 0: Char8Ptr += AsciiSPrint (Char8Ptr, 1024, "\t HPS_CLK1\r\n"); break;
      case 1: Char8Ptr += AsciiSPrint (Char8Ptr, 1024, "\t HPS_nPOR\r\n"); break;
      case 2: Char8Ptr += AsciiSPrint (Char8Ptr, 1024, "\t HPS_nRST\r\n"); break;
    }

    // Config - More details
    DisplayDedicatedConfigurationDetails(Data32, &Char8Ptr);
  }
  InfoPrint ("%a", Char8Str); Char8Ptr = &Char8Str[0];

  // Display Pin Mux settings and Voltage settings together for dedicated IO
  InfoPrint ("Pin Mux settings for dedicated IO:\r\n");
  for (i = 0; i < 14; i++)
  {
    // Pin Mux
    Data32 = MmioRead32 (ALT_PINMUX_DCTD_IO_GRP_OFST + ALT_PINMUX_DCTD_IO_4_OFST + (i * 4));
    Char8Ptr += AsciiSPrint (Char8Ptr, 1024,
               "\t pinmux_dedicated_io_%d (GPIO2_IO%d) :\t%a (%d)\r\n", (i + 4), (i + 0),
                meaning_string_of_dedicated_pinmux[
                  decode_meaning_of_dedicated_pinmux[i][ALT_PINMUX_DCTD_IO_4_SEL_GET(Data32)]
                ],
                Data32);
    // Config
    Data32 = MmioRead32 (ALT_PINMUX_DCTD_IO_GRP_OFST + ALT_PINMUX_DCTD_IO_CFG_4_OFST + (i * 4));
    Char8Ptr += AsciiSPrint (Char8Ptr, 1024,
                "\t configuration_dedicated_io_%d   :\t0x%08x\r\n", (i + 4),
                Data32);
    // Config - More details
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
                MmioRead32 (ALT_PINMUX_FPGA_INTERFACE_GRP_OFST + ALT_PINMUX_FPGA_EMAC0_USEFPGA_OFST + (i * 4)));
  }
  // I2C(0-1)
  for (i = 0; i < 2; i++)
  {
    Char8Ptr += AsciiSPrint (Char8Ptr, 1024,
                "\t pinmux_i2c%d_usefpga     :\t0x%08x\r\n", i,
                MmioRead32 (ALT_PINMUX_FPGA_INTERFACE_GRP_OFST + ALT_PINMUX_FPGA_I2C0_USEFPGA_OFST + (i * 4)));
  }
  // I2C_EMAC(0-2)
  for (i = 0; i < 3; i++)
  {
    Char8Ptr += AsciiSPrint (Char8Ptr, 1024,
                "\t pinmux_i2c_emac%d_usefpga:\t0x%08x\r\n", i,
                MmioRead32 (ALT_PINMUX_FPGA_INTERFACE_GRP_OFST + ALT_PINMUX_FPGA_I2C_EMAC0_USEFPGA_OFST + (i * 4)));
  }
  // NAND
  Char8Ptr += AsciiSPrint (Char8Ptr, 1024,
              "\t pinmux_nand_usefpga     :\t0x%08x\r\n",
              MmioRead32 (ALT_PINMUX_FPGA_INTERFACE_GRP_OFST + ALT_PINMUX_FPGA_NAND_USEFPGA_OFST));
  // QSPI
  Char8Ptr += AsciiSPrint (Char8Ptr, 1024,
              "\t pinmux_qspi_usefpga     :\t0x%08x\r\n",
              MmioRead32 (ALT_PINMUX_FPGA_INTERFACE_GRP_OFST + ALT_PINMUX_FPGA_QSPI_USEFPGA_OFST));
  // SDMMC
  Char8Ptr += AsciiSPrint (Char8Ptr, 1024,
              "\t pinmux_sdmmc_usefpga    :\t0x%08x\r\n",
              MmioRead32 (ALT_PINMUX_FPGA_INTERFACE_GRP_OFST + ALT_PINMUX_FPGA_SDMMC_USEFPGA_OFST));

  InfoPrint ("%a", Char8Str); Char8Ptr = &Char8Str[0];

  // SPIM(0-1)
  for (i = 0; i < 2; i++)
  {
    Char8Ptr += AsciiSPrint (Char8Ptr, 1024,
                "\t pinmux_spim%d_usefpga    :\t0x%08x\r\n", i,
                MmioRead32 (ALT_PINMUX_FPGA_INTERFACE_GRP_OFST + ALT_PINMUX_FPGA_SPIM0_USEFPGA_OFST + (i * 4)));
  }
  // SPIS(0-1)
  for (i = 0; i < 2; i++)
  {
    Char8Ptr += AsciiSPrint (Char8Ptr, 1024,
                "\t pinmux_spis%d_usefpga    :\t0x%08x\r\n", i,
                MmioRead32 (ALT_PINMUX_FPGA_INTERFACE_GRP_OFST + ALT_PINMUX_FPGA_SPIS0_USEFPGA_OFST + (i * 4)));
  }
  // UART(0-1)
  for (i = 0; i < 2; i++)
  {
    Char8Ptr += AsciiSPrint (Char8Ptr, 1024,
                "\t pinmux_uart%d_usefpga    :\t0x%08x\r\n", i,
                MmioRead32 (ALT_PINMUX_FPGA_INTERFACE_GRP_OFST + ALT_PINMUX_FPGA_UART0_USEFPGA_OFST + (i * 4)));
  }
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
  *Char8PtrPtr += AsciiSPrint (*Char8PtrPtr, 1024, "\t\tBias Trim : ");
  switch (ALT_PINMUX_DCTD_IO_CFG_4_RTRIM_GET(Data32))
  {
     case 0: *Char8PtrPtr += AsciiSPrint (*Char8PtrPtr, 1024, "Disable\r\n"); break;
     case 1: *Char8PtrPtr += AsciiSPrint (*Char8PtrPtr, 1024, "Default\r\n"); break;
     case 2: *Char8PtrPtr += AsciiSPrint (*Char8PtrPtr, 1024, "Trim Low\r\n"); break;
     case 4: *Char8PtrPtr += AsciiSPrint (*Char8PtrPtr, 1024, "Trim High\r\n"); break;
    default: *Char8PtrPtr += AsciiSPrint (*Char8PtrPtr, 1024, "RSVD\r\n"); break;
  }
  *Char8PtrPtr += AsciiSPrint (*Char8PtrPtr, 1024, "\t\tInput Buffer : ");
  switch (ALT_PINMUX_DCTD_IO_CFG_4_INPUT_BUF_EN_GET(Data32))
  {
     case 0: *Char8PtrPtr += AsciiSPrint (*Char8PtrPtr, 1024, "Disable\r\n"); break;
     case 1: *Char8PtrPtr += AsciiSPrint (*Char8PtrPtr, 1024, "1.8V TTL\r\n"); break;
     case 2: *Char8PtrPtr += AsciiSPrint (*Char8PtrPtr, 1024, "2.5V/3.0V TTL\r\n"); break;
     case 3: *Char8PtrPtr += AsciiSPrint (*Char8PtrPtr, 1024, "1.8V TTL\r\n"); break;
  }
  *Char8PtrPtr += AsciiSPrint (*Char8PtrPtr, 1024,
             "\t\tWeak pull up : %a\r\n"
             "\t\tPull up slew rate : %a\r\n"
             "\t\tPMOS pull up drive strength : %d\r\n"
             "\t\tPull down slew rate : %a\r\n"
             "\t\tNMOS pull down drive strength : %d\r\n",
    ALT_PINMUX_DCTD_IO_CFG_4_WK_PU_EN_GET(Data32) == 0 ? "disable": "enable",
    ALT_PINMUX_DCTD_IO_CFG_4_PU_SLW_RT_GET(Data32) == 0 ? "slow": "fast",
    ALT_PINMUX_DCTD_IO_CFG_4_PU_DRV_STRG_GET(Data32),
    ALT_PINMUX_DCTD_IO_CFG_4_PD_SLW_RT_GET(Data32) == 0 ? "slow": "fast",
    ALT_PINMUX_DCTD_IO_CFG_4_PD_DRV_STRG_GET(Data32));

}


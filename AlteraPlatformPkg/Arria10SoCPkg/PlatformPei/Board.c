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
#include <Library/SerialPortPrintLib.h>
#include "Assert.h"
#include "Banner.h"
#include "Board.h"
#include "BootSource.h"
#include "ClockManager.h"
#include "DeviceTree.h"
#include "Firewall.h"
#include "FpgaManager.h"
#include "MemoryController.h"
#include "MemoryTest.h"
#include "NandBlockIo.h"
#include "Pinmux.h"
#include "PlatformInit.h"
#include "QspiBlockIo.h"
#include "ResetManager.h"
#include "SdMmc.h"
#include "SecurityManager.h"
#include "SystemManager.h"

#if (FixedPcdGet32(PcdDebugMsg_BoardInit) == 0)
  //#define ProgressPrint(FormatString, ...)  /* do nothing */
  #define ProgressPrint                       SerialPortPrint
  #define InfoPrint(FormatString, ...)        /* do nothing */
#else
  #define ProgressPrint SerialPortPrint
  #define InfoPrint     SerialPortPrint
#endif


VOID
EFIAPI
BoardSpecificInitialization (
  IN  CONST VOID*  Fdt
  )
{
#if (FixedPcdGet32(PcdIsAlteraSoCFPGADevelopmentBoards) == 1)
  UINT32  SiliconID1;
  SiliconID1 = MmioRead32 (ALT_SYSMGR_OFST + ALT_SYSMGR_SILICONID1_OFST);
  InfoPrint ("\r\n"
             "User's Code: Board Init...\r\n"
             "\r\n"
             "Print Board Info (as User's code place holder example): \r\n"
             "\r\n");
  InfoPrint ("Board SoC/FPGA/CPLD:\r\n"
             "  U23 - Altera Arria(R) 10 SoC 10AS066N2F40C2SGES\r\n"
             "  U16 - Altera MAX(R) V CPLD (5M2210ZF256) as System Controller\r\n"
             "  U21 - Altera MAX(R) V CPLD (5M2210ZF256) as IO MUX Controller\r\n"
             "\r\n");
  InfoPrint ("Ports:\r\n"
             "  J22 - USB Blaster\r\n"
             "  J10 - UART1 to USB Mini-B Port (FT232R)\r\n"
             "  J25 - UART1 to DB-9 RS-232 Port (MAX3221)\r\n"
             "  J4  - USB 2.0 OTG (PHY PN: USB3320C-EZK)\r\n"
             "  J47 - USB TI GUI Port\r\n"
             "  J24 - JTAG chain header\r\n"
             "  J46 - Display Port\r\n"
             "  J48 - SDI Tx\r\n"
             "  J49 - SDI Rx\r\n"
             "  J57 - PCIe x8 Connector\r\n"
             "  J5  - RGMII 10/100/1000 Ethernet port (PHY PN: KSZ9031RNXCA)\r\n"
             "  J2&J3-SGMII Gigabit Ethernet ports (PHY PN: 88E1111-B2-NDC2C000)\r\n"
             "  J7&J8-Optical (SFP+) Transceiver Cage & Connectors\r\n"
             "\r\n");
  InfoPrint ("J23 - Flash Card options:\r\n"
             "  1 - NAND Flash(x8) : 128MB (MT29F1G08ABBEAH4)\r\n"
             "  2 - QSPI Flash     : 128MB (MT25QU01GBBA8E12-0SIT)\r\n"
             "  3 - SD Micro card  :   4GB (Kingston)\r\n"
             "\r\n");
  InfoPrint ("J27 - FPGA memory HILO Card options:\r\n"
             "  1 - 2GB DDR4 (256Mb x 72 x 1 rank)\r\n"
             "  2 - 2GB DDR3 (256Mb x 72 x 1 rank)\r\n"
             "  3 - 4GB DDR3 (256Mb x 72 x 2 ranks)\r\n"
             "  4 - 16MB QDRV (4Mb x 36)\r\n"
             "  5 - 128MB RLDRAM3 (16Mb x 72)\r\n"
             "\r\n");
  InfoPrint ("J26 - HPS memory HILO Card options:\r\n"
             "  1 - 1GB DDR4 (256Mb x 40 x 1 Rank)\r\n"
             "  2 - 1GB DDR3 (256Mb x 40 x 1 Rank)\r\n"
             "  3 - 2GB DDR3 (256Mb x 40 x 2 Ranks)\r\n"
             "\r\n");
  if (SiliconID1 == ALT_SYSMGR_SILICONID1_ES1) {
    InfoPrint ("SW1 DIP switch\r\n"
               "  1 - I2C flag\r\n"
               "  2 - FACTORY_LOAD\r\n"
               "  3 - MSEL pins 1\r\n"
               "  4 - MSEL pins 0\r\n"
               "\r\n");
    InfoPrint ("SW2 DIP switch\r\n"
               "  1 - USER_DIPSW_HPS0\r\n"
               "  2 - USER_DIPSW_HPS1\r\n"
               "  3 - USER_DIPSW_HPS2\r\n"
               "  4 - USER_DIPSW_HPS3\r\n"
               "  5 - USER_DIPSW_FPGA0\r\n"
               "  6 - USER_DIPSW_FPGA1\r\n"
               "  7 - USER_DIPSW_FPGA2\r\n"
               "  8 - USER_DIPSW_FPGA3\r\n"
               "\r\n");
    InfoPrint ("SW3 DIP switch\r\n"
               "  1 - Board:\r\n"
               "         REV A0 - MAXV-A System Controller JTAG by pass\r\n"
               "         REV A1 - SOCHPS JTAG by pass\r\n"
               "         REV A1 - MAXV-A System Controller = Short J58, set SW3 DIP switch (on on on on on off off on)\r\n"
               "  2 - MAXV-B IO MUX Controller JTAG by pass\r\n"
               "  3 - FMC-A JTAG by pass\r\n"
               "  4 - FMC-B JTAG by pass\r\n"
               "  5 - PCIe JTAG by pass\r\n"
               "  6 - External JTAG by pass\r\n"
               "  7 - CNTR0\r\n"
               "  8 - CNTR1\r\n"
               "  Default: \r\n"
               "      USB Blaster: set SW3 DIP switch (off on on on on off off off)\r\n"
               "      Use DSTREAM: set SW3 DIP switch (off on on on on on  on  on )\r\n"
               "\r\n");
  } else {
    InfoPrint ("SW1 DIP switch\r\n"
               "  1 - I2C flag\r\n"
               "  2 - DC_POWER_CTRL\r\n"
               "  3 - FACTORY LOAD\r\n"
               "  4 - RESERVED\r\n"
               "\r\n");
    InfoPrint ("SW2 DIP switch\r\n"
               "  1 - USER_DIPSW_HPS0\r\n"
               "  2 - USER_DIPSW_HPS1\r\n"
               "  3 - USER_DIPSW_HPS2\r\n"
               "  4 - USER_DIPSW_HPS3\r\n"
               "  5 - USER_DIPSW_FPGA0\r\n"
               "  6 - USER_DIPSW_FPGA1\r\n"
               "  7 - USER_DIPSW_FPGA2\r\n"
               "  8 - USER_DIPSW_FPGA3\r\n"
               "\r\n");
    InfoPrint ("SW3 DIP switch\r\n"
               "  1 - SOCHPS JTAG by pass\r\n"
               "      MAXV-A System Controller = Short J58, set SW3 DIP switch (on on on on on off off on)\r\n"
               "  2 - MAXV-B IO MUX Controller JTAG by pass\r\n"
               "  3 - FMC-A JTAG by pass\r\n"
               "  4 - FMC-B JTAG by pass\r\n"
               "  5 - PCIe JTAG by pass\r\n"
               "  6 - External JTAG by pass\r\n"
               "  7 - CNTR0\r\n"
               "  8 - CNTR1\r\n"
               "  Default: \r\n"
               "      USB Blaster: set SW3 DIP switch (off on on on on off off off)\r\n"
               "      Use DSTREAM: set SW3 DIP switch (off on on on on on  on  on )\r\n"
               "\r\n");
    InfoPrint ("SW4 DIP switch\r\n"
               "  1 - RESERVED\r\n"
               "  2 - MSEL pins 0\r\n"
               "  3 - MSEL pins 1\r\n"
               "  4 - MSEL pins 2\r\n"
               "\r\n");
  }

#else
  InfoPrint ("User's Code: Board Init...\r\n");
#endif
}


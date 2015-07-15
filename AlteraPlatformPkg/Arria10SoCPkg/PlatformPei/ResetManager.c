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
#include "DeviceTree.h"
#include "ResetManager.h"

#if (FixedPcdGet32(PcdDebugMsg_ResetManager) == 0)
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
DeassertPeripheralsReset (
  VOID
  )
{
  UINT32 EccOcpMask;
  UINT32 AndMask;

  ProgressPrint ("Deassert Peripheral from Reset\r\n");

  //
  // De-assert Reset to Peripheral Group 1 components
  //
  AndMask = 0xFFFFFFFF;
  AndMask &= ALT_RSTMGR_PER1MODRST_WD0_CLR_MSK;
  AndMask &= ALT_RSTMGR_PER1MODRST_WD1_CLR_MSK;
  AndMask &= ALT_RSTMGR_PER1MODRST_L4SYSTMR0_CLR_MSK;
  AndMask &= ALT_RSTMGR_PER1MODRST_L4SYSTMR1_CLR_MSK;
  AndMask &= ALT_RSTMGR_PER1MODRST_SPTMR0_CLR_MSK;
  AndMask &= ALT_RSTMGR_PER1MODRST_SPTMR1_CLR_MSK;
  AndMask &= ALT_RSTMGR_PER1MODRST_I2C0_CLR_MSK;
  AndMask &= ALT_RSTMGR_PER1MODRST_I2C1_CLR_MSK;
  AndMask &= ALT_RSTMGR_PER1MODRST_I2C2_CLR_MSK;
  AndMask &= ALT_RSTMGR_PER1MODRST_I2C3_CLR_MSK;
  AndMask &= ALT_RSTMGR_PER1MODRST_I2C4_CLR_MSK;
  AndMask &= ALT_RSTMGR_PER1MODRST_UART0_CLR_MSK;
  AndMask &= ALT_RSTMGR_PER1MODRST_UART1_CLR_MSK;
  AndMask &= ALT_RSTMGR_PER1MODRST_GPIO0_CLR_MSK;
  AndMask &= ALT_RSTMGR_PER1MODRST_GPIO1_CLR_MSK;
  AndMask &= ALT_RSTMGR_PER1MODRST_GPIO2_CLR_MSK;
  MmioAnd32 (ALT_RSTMGR_OFST +
             ALT_RSTMGR_PER1MODRST_OFST,
             AndMask);

  // De-assert Reset to Peripheral Group 0 components two step:
  // First, all none ECC_OCP components, followed by ECC_OCP components
  // NOTE:
  // When asserting reset, we have to set the ECC OCP last
  // When deasserting reset, we have to set the ECC OCP first
  EccOcpMask = ALT_RSTMGR_PER0MODRST_EMAC0OCP_CLR_MSK &
               ALT_RSTMGR_PER0MODRST_EMAC1OCP_CLR_MSK &
               ALT_RSTMGR_PER0MODRST_EMAC2OCP_CLR_MSK &
               ALT_RSTMGR_PER0MODRST_USB0OCP_CLR_MSK &
               ALT_RSTMGR_PER0MODRST_USB1OCP_CLR_MSK &
               ALT_RSTMGR_PER0MODRST_NANDOCP_CLR_MSK &
               ALT_RSTMGR_PER0MODRST_QSPIOCP_CLR_MSK &
               ALT_RSTMGR_PER0MODRST_SDMMCOCP_CLR_MSK &
               ALT_RSTMGR_PER0MODRST_DMAOCP_CLR_MSK;
  // De-assert reset of ECC_OCP components
  MmioAnd32 (ALT_RSTMGR_OFST +
             ALT_RSTMGR_PER0MODRST_OFST, EccOcpMask);
  // De-assert reset of none ECC_OCP components
  AndMask = 0xFFFFFFFF;
  AndMask &= ALT_RSTMGR_PER0MODRST_EMAC0_CLR_MSK;
  AndMask &= ALT_RSTMGR_PER0MODRST_EMAC1_CLR_MSK;
  AndMask &= ALT_RSTMGR_PER0MODRST_EMAC2_CLR_MSK;
  AndMask &= ALT_RSTMGR_PER0MODRST_USB0_CLR_MSK;
  AndMask &= ALT_RSTMGR_PER0MODRST_USB1_CLR_MSK;
  AndMask &= ALT_RSTMGR_PER0MODRST_NAND_CLR_MSK;
  AndMask &= ALT_RSTMGR_PER0MODRST_QSPI_CLR_MSK;
  AndMask &= ALT_RSTMGR_PER0MODRST_SDMMC_CLR_MSK;
  AndMask &= ALT_RSTMGR_PER0MODRST_DMA_CLR_MSK;
  AndMask &= ALT_RSTMGR_PER0MODRST_SPIM0_CLR_MSK;
  AndMask &= ALT_RSTMGR_PER0MODRST_SPIM1_CLR_MSK;
  AndMask &= ALT_RSTMGR_PER0MODRST_SPIS0_CLR_MSK;
  AndMask &= ALT_RSTMGR_PER0MODRST_SPIS1_CLR_MSK;
  AndMask &= ALT_RSTMGR_PER0MODRST_EMACPTP_CLR_MSK;
  AndMask &= ALT_RSTMGR_PER0MODRST_DMAIF0_CLR_MSK;
  AndMask &= ALT_RSTMGR_PER0MODRST_DMAIF1_CLR_MSK;
  AndMask &= ALT_RSTMGR_PER0MODRST_DMAIF2_CLR_MSK;
  AndMask &= ALT_RSTMGR_PER0MODRST_DMAIF3_CLR_MSK;
  AndMask &= ALT_RSTMGR_PER0MODRST_DMAIF4_CLR_MSK;
  AndMask &= ALT_RSTMGR_PER0MODRST_DMAIF5_CLR_MSK;
  AndMask &= ALT_RSTMGR_PER0MODRST_DMAIF6_CLR_MSK;
  AndMask &= ALT_RSTMGR_PER0MODRST_DMAIF7_CLR_MSK;
  MmioAnd32 (ALT_RSTMGR_OFST +
             ALT_RSTMGR_PER0MODRST_OFST,
             AndMask);
}


VOID
EFIAPI
ConfigureHpsSubsystemHandshakeBehaviorBeforeWarmReset (
  VOID
  )
{
  UINT32 OrMask;

  ProgressPrint ("Config HPS Handshake\r\n");

  // Enable hardware handshake with other modules before warm reset.
  OrMask = 0;
  OrMask |= ALT_RSTMGR_HDSKEN_SDRSELFREFEN_SET_MSK;
  OrMask |= ALT_RSTMGR_HDSKEN_FPGAMGRHSEN_SET_MSK;
  OrMask |= ALT_RSTMGR_HDSKEN_FPGAHSEN_SET_MSK;
  OrMask |= ALT_RSTMGR_HDSKEN_ETRSTALLEN_SET_MSK;
  MmioOr32 (ALT_RSTMGR_OFST +
            ALT_RSTMGR_HDSKEN_OFST,
            OrMask);
}


VOID
EFIAPI
DisplayResetManagerInfo (
  VOID
  )
{
  UINT32          Data32;

  // Display logged reset request source(s) de-assert the request in the same cycle
  Data32 = MmioRead32 (ALT_RSTMGR_OFST + ALT_RSTMGR_STAT_OFST);
  InfoPrint ( "Reset Request Source(s): 0x%08x\r\n",
              Data32);
  if (ALT_RSTMGR_STAT_PORHPSVOLTRST_GET(Data32) == 1)
    InfoPrint ("\t HPS Power-On Voltage Detector Cold Reset \r\n");
  if (ALT_RSTMGR_STAT_PORFPGAVOLTRST_GET(Data32) == 1)
    InfoPrint ("\t Power-On FPGA Voltage Detector Cold Reset \r\n");
  if (ALT_RSTMGR_STAT_NPORPINRST_GET(Data32) == 1)
    InfoPrint ("\t nPOR Pin Cold Reset \r\n");
  if (ALT_RSTMGR_STAT_FPGACOLDRST_GET(Data32) == 1)
    InfoPrint ("\t FPGA Core Cold Reset \r\n");
  if (ALT_RSTMGR_STAT_CFGIOCOLDRST_GET(Data32) == 1)
    InfoPrint ("\t CONFIG_IO Cold Reset \r\n");
  if (ALT_RSTMGR_STAT_SWCOLDRST_GET(Data32) == 1)
    InfoPrint ("\t Software Cold Reset \r\n");
  if (ALT_RSTMGR_STAT_NRSTPINRST_GET(Data32) == 1)
    InfoPrint ("\t nRST Pin Warm Reset \r\n");
  if (ALT_RSTMGR_STAT_FPGAWARMRST_GET(Data32) == 1)
    InfoPrint ("\t FPGA Core Warm Reset \r\n");
  if (ALT_RSTMGR_STAT_SWWARMRST_GET(Data32) == 1)
    InfoPrint ("\t Software Warm Reset \r\n");
  if (ALT_RSTMGR_STAT_MPUWD0RST_GET(Data32) == 1)
    InfoPrint ("\t MPU Watchdog 0 Warm Reset \r\n");
  if (ALT_RSTMGR_STAT_MPUWD1RST_GET(Data32) == 1)
    InfoPrint ("\t MPU Watchdog 1 Warm Reset \r\n");
  if (ALT_RSTMGR_STAT_L4WD0RST_GET(Data32) == 1)
    InfoPrint ("\t L4 Watchdog 0 Warm Reset \r\n");
  if (ALT_RSTMGR_STAT_L4WD1RST_GET(Data32) == 1)
    InfoPrint ("\t L4 Watchdog 1 Warm Reset \r\n");
  if (ALT_RSTMGR_STAT_FPGADBGRST_GET(Data32) == 1)
    InfoPrint ("\t FPGA Core Debug Reset \r\n");
  if (ALT_RSTMGR_STAT_CDBGREQRST_GET(Data32) == 1)
    InfoPrint ("\t DAP Debug Reset \r\n");

  // Display Reset Manager RAM Status Register - ramstat
  // The RAMSTAT register contains bits that indicate the security RAM clearing event
  // during cold or warm reset for each RAM
  InfoPrint ("RST_MGR RAM Status : 0x%08x\r\n"
             "RST_MGR MISC Status: 0x%08x\r\n"
             "RST_MGR HdSkEn reg : 0x%08x\r\n"
             "RST_MGR HdSkAck reg: 0x%08x\r\n"
             "RST_MGR Peripherals Reset State:\r\n"
             "\tPER0MODRST: 0x%08x\r\n\tPER0M1DRST: 0x%08x\r\n",
              MmioRead32 (ALT_RSTMGR_OFST + ALT_RSTMGR_RAMSTAT_OFST),
              MmioRead32 (ALT_RSTMGR_OFST + ALT_RSTMGR_MISCSTAT_OFST),
              MmioRead32 (ALT_RSTMGR_OFST + ALT_RSTMGR_HDSKEN_OFST),
              MmioRead32 (ALT_RSTMGR_OFST + ALT_RSTMGR_HDSKACK_OFST),
              MmioRead32 (ALT_RSTMGR_OFST + ALT_RSTMGR_PER0MODRST_OFST),
              MmioRead32 (ALT_RSTMGR_OFST + ALT_RSTMGR_PER1MODRST_OFST));

}



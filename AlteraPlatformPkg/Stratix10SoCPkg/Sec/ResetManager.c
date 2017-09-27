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

  //ProgressPrint ("Deassert Peripheral from Reset\r\n");

  //
  // De-assert Reset to Peripheral Group 1 components
  //
  AndMask = 0xFFFFFFFF;
  //AndMask &= ALT_RSTMGR_PER1MODRST_WATCHDOG0_CLR_MSK;
  //AndMask &= ALT_RSTMGR_PER1MODRST_WATCHDOG1_CLR_MSK;
  //AndMask &= ALT_RSTMGR_PER1MODRST_WATCHDOG2_CLR_MSK;
  //AndMask &= ALT_RSTMGR_PER1MODRST_WATCHDOG3_CLR_MSK;
  AndMask &= ALT_RSTMGR_PER1MODRST_L4SYSTIMER0_CLR_MSK;
  AndMask &= ALT_RSTMGR_PER1MODRST_L4SYSTIMER1_CLR_MSK;
  AndMask &= ALT_RSTMGR_PER1MODRST_SPTIMER0_CLR_MSK;
  AndMask &= ALT_RSTMGR_PER1MODRST_SPTIMER1_CLR_MSK;
  AndMask &= ALT_RSTMGR_PER1MODRST_I2C0_CLR_MSK;
  AndMask &= ALT_RSTMGR_PER1MODRST_I2C1_CLR_MSK;
  AndMask &= ALT_RSTMGR_PER1MODRST_I2C2_CLR_MSK;
  AndMask &= ALT_RSTMGR_PER1MODRST_I2C3_CLR_MSK;
  AndMask &= ALT_RSTMGR_PER1MODRST_I2C4_CLR_MSK;
  AndMask &= ALT_RSTMGR_PER1MODRST_UART0_CLR_MSK;
  AndMask &= ALT_RSTMGR_PER1MODRST_UART1_CLR_MSK;
  AndMask &= ALT_RSTMGR_PER1MODRST_GPIO0_CLR_MSK;
  AndMask &= ALT_RSTMGR_PER1MODRST_GPIO1_CLR_MSK;
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

  //ProgressPrint ("Config HPS Handshake\r\n");

  // Enable hardware handshake with other modules before warm reset.
  OrMask = 0;
  OrMask |= ALT_RSTMGR_HDSKEN_SDRSELFREFEN_SET_MSK;
  OrMask |= ALT_RSTMGR_HDSKEN_FPGAHSEN_SET_MSK;
  OrMask |= ALT_RSTMGR_HDSKEN_ETRSTALLEN_SET_MSK;
  OrMask |= ALT_RSTMGR_HDSKEN_L2FLUSHEN_SET_MSK;
  OrMask |= ALT_RSTMGR_HDSKEN_L3NOC_DBG_SET_MSK;
  OrMask |= ALT_RSTMGR_HDSKEN_DEBUG_L3NOC_SET_MSK;

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
  if (ALT_RSTMGR_STAT_SDMCOLDRST_GET(Data32) == 1)
    InfoPrint ("\t SDM Cold Reset \r\n");
  if (ALT_RSTMGR_STAT_SDMWARMRST_GET(Data32) == 1)
    InfoPrint ("\t SDM Warm Reset \r\n");
  if (ALT_RSTMGR_STAT_SDMLASTPORRST_GET(Data32) == 1)
    InfoPrint ("\t nSDM last POR Reset \r\n");
  if (ALT_RSTMGR_STAT_MPU0RST_GET(Data32) == 1)
    InfoPrint ("\t MPU0 Reset \r\n");
  if (ALT_RSTMGR_STAT_MPU1RST_GET(Data32) == 1)
    InfoPrint ("\t MPU1 Reset \r\n");
  if (ALT_RSTMGR_STAT_MPU2RST_GET(Data32) == 1)
    InfoPrint ("\t MPU2 Resett \r\n");
  if (ALT_RSTMGR_STAT_MPU3RST_GET(Data32) == 1)
    InfoPrint ("\t MPU3 Reset \r\n");
  if (ALT_RSTMGR_STAT_L4WD0RST_GET(Data32) == 1)
    InfoPrint ("\t MPU Watchdog 0 Warm Reset \r\n");
  if (ALT_RSTMGR_STAT_L4WD1RST_GET(Data32) == 1)
    InfoPrint ("\t MPU Watchdog 1 Warm Reset \r\n");
  if (ALT_RSTMGR_STAT_L4WD2RST_GET(Data32) == 1)
    InfoPrint ("\t MPU Watchdog 2 Warm Reset \r\n");
  if (ALT_RSTMGR_STAT_L4WD3RST_GET(Data32) == 1)
    InfoPrint ("\t MPU Watchdog 3 Warm Reset \r\n");
  if (ALT_RSTMGR_STAT_DEBUGRST_GET(Data32) == 1)
    InfoPrint ("\t  FPGA Core Debug Reset \r\n");
  if (ALT_RSTMGR_STAT_CSDAPRST_GET(Data32) == 1)
    InfoPrint ("\t CS DAP Reset \r\n");


  // Display Reset Manager Register
  InfoPrint ("RST_MGR MPU Status : 0x%08x\r\n"
             "RST_MGR MISC Status: 0x%08x\r\n"
             "RST_MGR HdSkEn reg : 0x%08x\r\n"
             "RST_MGR HdSkAck reg: 0x%08x\r\n"
             "RST_MGR Peripherals Reset State:\r\n"
             "\tPER0MODRST: 0x%08x\r\n\tPER0M1DRST: 0x%08x\r\n",
              MmioRead32 (ALT_RSTMGR_OFST + ALT_RSTMGR_MPURSTSTAT_OFST),
              MmioRead32 (ALT_RSTMGR_OFST + ALT_RSTMGR_MISCSTAT_OFST),
              MmioRead32 (ALT_RSTMGR_OFST + ALT_RSTMGR_HDSKEN_OFST),
              MmioRead32 (ALT_RSTMGR_OFST + ALT_RSTMGR_HDSKACK_OFST),
              MmioRead32 (ALT_RSTMGR_OFST + ALT_RSTMGR_PER0MODRST_OFST),
              MmioRead32 (ALT_RSTMGR_OFST + ALT_RSTMGR_PER1MODRST_OFST));

}

VOID
EFIAPI
EnableHpsAndFpgaBridges (
  VOID
  )
{
  // Clear NOC idle request
  MmioWrite32 (ALT_SYSMGR_CORE_OFST +
              ALT_SYSMGR_CORE_NOC_IDLEREQ_CLR_OFST,
			  ~ALT_SYSMGR_CORE_NOC_IDLEREQ_CLR_RESET);

  // De-assert all the bridge reset
  MmioWrite32 (ALT_SYSMGR_CORE_OFST +
              ALT_RSTMGR_BRGMODRST_OFST,
			  ALT_RSTMGR_BRGMODRST_RESET);

  // Poll until all noc idle request ack to 0
  while (MmioRead32(ALT_SYSMGR_CORE_OFST +
                     ALT_SYSMGR_CORE_NOC_IDLEACK_OFST));
}

VOID
EFIAPI
DisableHpsAndFpgaBridges (
  VOID
  )
{
  // Set NOC idle request
  MmioWrite32 (ALT_SYSMGR_CORE_OFST +
              ALT_SYSMGR_CORE_NOC_IDLEREQ_SET_OFST,
			  ~ALT_SYSMGR_CORE_NOC_IDLEREQ_SET_RESET);
  // Enable the NOC timeout
  MmioOr32(ALT_SYSMGR_CORE_OFST +
             ALT_SYSMGR_CORE_NOC_TIMEOUT_OFST,
			 ALT_SYSMGR_CORE_NOC_TIMEOUT_EN_SET_MSK);


  // Poll until all noc idle request ack to 1
  while (MmioRead32(ALT_SYSMGR_CORE_OFST +
         ALT_SYSMGR_CORE_NOC_IDLEACK_OFST) !=
        (ALT_SYSMGR_CORE_NOC_IDLEACK_SOC2FPGA_SET_MSK |
		 ALT_SYSMGR_CORE_NOC_IDLEACK_LWSOC2FPGA_SET_MSK));

  // Poll until all noc idle status to 1
  while (MmioRead32(ALT_SYSMGR_CORE_OFST +
                     ALT_SYSMGR_CORE_NOC_IDLESTATUS_OFST) !=
                    (ALT_SYSMGR_CORE_NOC_IDLESTATUS_SOC2FPGA_SET_MSK |
 		             ALT_SYSMGR_CORE_NOC_IDLESTATUS_LWSOC2FPGA_SET_MSK));

  // Assert all bridges reset except DDR schdule
  MmioOr32 (ALT_SYSMGR_CORE_OFST +
              ALT_RSTMGR_BRGMODRST_OFST,
			  ALT_RSTMGR_BRGMODRST_SOC2FPGA_SET_MSK |
			  ALT_RSTMGR_BRGMODRST_LWHPS2FPGA_SET_MSK |
			  ALT_RSTMGR_BRGMODRST_FPGA2SOC_SET_MSK |
			  ALT_RSTMGR_BRGMODRST_F2SSDRAM0_SET_MSK |
			  ALT_RSTMGR_BRGMODRST_F2SSDRAM1_SET_MSK |
			  ALT_RSTMGR_BRGMODRST_F2SSDRAM2_SET_MSK
			  );
}

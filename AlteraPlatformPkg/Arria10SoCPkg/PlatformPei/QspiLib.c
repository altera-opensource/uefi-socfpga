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

#include <AlteraPlatform.h>
#include <libfdt.h>
#include <Library/BaseMemoryLib.h>
#include <Library/DebugLib.h>
#include <Library/IoLib.h>
#include <Library/MemoryAllocationLib.h>
#include <Library/PcdLib.h>
#include <Library/PrintLib.h>
#include <Library/SerialPortPrintLib.h>
#include <Library/TimerLib.h>
#include "Assert.h"
#include "DeviceTree.h"
#include "QspiLib.h"

#if (FixedPcdGet32(PcdDebugMsg_Qspi) == 0)
  #define ProgressPrint(FormatString, ...)    /* do nothing */
  #define InfoPrint(FormatString, ...)        /* do nothing */
  #define MmioHexDump(BaseAddr, Data32Size)   /* do nothing */
#else
  #define ProgressPrint SerialPortPrint
  #define InfoPrint     SerialPortPrint
  #define MmioHexDump SerialPortMmioHexDump
#endif


// qspi_clk operating frequency range.
#define ALT_QSPI_CLK_FREQ_MIN (0)
#define ALT_QSPI_CLK_FREQ_MAX (432000000)

// The set of all valid QSPI controller interrupt Status Mask values.
#define ALT_QSPI_INT_STATUS_ALL  (ALT_QSPI_INT_STATUS_MODE_FAIL | \
        ALT_QSPI_INT_STATUS_UFL       | \
        ALT_QSPI_INT_STATUS_IDAC_OP_COMPLETE  | \
        ALT_QSPI_INT_STATUS_IDAC_OP_REJECT    | \
        ALT_QSPI_INT_STATUS_WR_PROT_VIOL      | \
        ALT_QSPI_INT_STATUS_ILL_AHB_ACCESS    | \
        ALT_QSPI_INT_STATUS_IDAC_WTRMK_TRIG   | \
        ALT_QSPI_INT_STATUS_RX_OVF            | \
        ALT_QSPI_INT_STATUS_TX_FIFO_NOT_FULL  | \
        ALT_QSPI_INT_STATUS_TX_FIFO_FULL      | \
        ALT_QSPI_INT_STATUS_RX_FIFO_NOT_EMPTY | \
        ALT_QSPI_INT_STATUS_RX_FIFO_FULL      | \
        ALT_QSPI_INT_STATUS_IDAC_RD_FULL \
        )

#define QSPI_SUBSECTOR_COUNT      (QspiDeviceSize / ALT_QSPI_SUBSECTOR_SIZE)

UINT32 QspiDeviceSize = 0;
//device tree
UINT32 Osc1ClkFreq = 0;
UINT32 IntOscClkFreq = 0;
UINT32 PerClkFreq = 0;

///////////////////////////////////////////////////////////////////////////////////////////
// Function Definition
////////////////////////////////////////////////////////////////////////////////////////////

EFI_STATUS
EFIAPI
AltClkFregGetFdt (
  IN VOID*  Fdt
  )
{
  INT32    Node;
  CONST UINT32*  Prop32Ptr;

  Node = fdt_node_offset_by_compatible(Fdt, -1, "fixed-clock");
  if (Node == -FDT_ERR_NOTFOUND) {
    InfoPrint ("FDT: fixed-clock not found \r\n");
    ASSERT_PLATFORM_INIT(Node >= 0);
    return EFI_DEVICE_ERROR;
  }

  Prop32Ptr = fdt_getprop (Fdt, Node, "clock-frequency", NULL);
  if (Prop32Ptr == NULL) {
    InfoPrint ("FDT: clock-frequency not found\r\n");
    ASSERT_PLATFORM_INIT(Prop32Ptr != NULL);
    return EFI_DEVICE_ERROR;
  }

  Osc1ClkFreq = fdt32_to_cpu (ReadUnaligned32(Prop32Ptr));
  return EFI_SUCCESS;

}

EFI_STATUS
EFIAPI
AltClkPllVcoFreqGet (
  IN UINT32 Pll,
  IN UINT32 *Freq
  )
{
  UINT64            Temp1 = 0;
  UINT32            Temp, Vco0;
  UINT32            Numer =0;
  UINT32            Denom =0;
  EFI_STATUS     Status = EFI_DEVICE_ERROR;

  if (Freq == NULL) {
        return Status;
  }

  if (Pll == ALT_CLK_MAIN_PLL) {
    Temp = MmioRead32(ALT_CLKMGR_MAINPLL_OFST + ALT_CLKMGR_MAINPLL_VCO1_OFST);
    Numer = ALT_CLKMGR_MAINPLL_VCO1_NUMER_GET(Temp);
    Denom = ALT_CLKMGR_MAINPLL_VCO1_DENOM_GET(Temp);

    Vco0 = MmioRead32(ALT_CLKMGR_MAINPLL_OFST + ALT_CLKMGR_MAINPLL_VCO0_OFST);
    Temp = ALT_CLKMGR_MAINPLL_VCO0_PSRC_GET(Vco0);

    if (Temp == ALT_CLKMGR_MAINPLL_VCO0_PSRC_E_EOSC1) {
      Temp1 = (UINT64) Osc1ClkFreq;

    } else if (Temp == ALT_CLKMGR_MAINPLL_VCO0_PSRC_E_INTOSC) {
      Temp1 = (UINT64) IntOscClkFreq;

    } else if (Temp == ALT_CLKMGR_MAINPLL_VCO0_PSRC_E_F2S) {
      Temp1 = (UINT64) PerClkFreq;
    }

    if (Temp1 != 0) {
      Temp1 *= (UINT64)(Numer + 1);
      Temp1 /= (UINT64)(Denom + 1);

      if (Temp1 <= 0xFFFFFFFF) {
        *Freq = (UINT32) Temp1;;
        Status = EFI_SUCCESS;
      }

    }
  } else if (Pll == ALT_CLK_PERIPHERAL_PLL) {
      Temp = MmioRead32(ALT_CLKMGR_PERPLL_OFST + ALT_CLKMGR_PERPLL_VCO1_OFST);
      Numer = ALT_CLKMGR_PERPLL_VCO1_NUMER_GET(Temp);
      Denom = ALT_CLKMGR_PERPLL_VCO1_DENOM_GET(Temp);

      Temp = MmioRead32(ALT_CLKMGR_PERPLL_OFST + ALT_CLKMGR_PERPLL_VCO0_OFST);
      Temp = ALT_CLKMGR_PERPLL_VCO0_PSRC_GET(Temp);

    if (Temp == ALT_CLKMGR_PERPLL_VCO0_PSRC_E_EOSC1) {
      Temp1 = (UINT64) Osc1ClkFreq;
    } else if (Temp == ALT_CLKMGR_PERPLL_VCO0_PSRC_E_INTOSC) {
      Temp1 = (UINT64) IntOscClkFreq;
    } else if (Temp == ALT_CLKMGR_PERPLL_VCO0_PSRC_E_F2S) {
      Temp1 = (UINT64) PerClkFreq;
    }
    ASSERT_PLATFORM_INIT(Temp1 != 0);
    if (Temp1 != 0) {
      Temp1 *= (Numer + 1);
      Temp1 /= (Denom + 1);
      if (Temp1 <= UINT32_MAX) {
          Temp = (UINT32) Temp1;
          *Freq = Temp;
          Status = EFI_SUCCESS;
      }

    }
  }
  return Status;
}

UINT32
EFIAPI
QspiGetClockFreq (
  VOID
  )
{
  EFI_STATUS Status = EFI_DEVICE_ERROR;
  UINT32 ClkSrc, Dividor, NocClk, SrcHz;
  UINT32 Dividor2;

  NocClk = MmioRead32(ALT_CLKMGR_MAINPLL_OFST + ALT_CLKMGR_MAINPLL_NOCCLK_OFST);

  Dividor = 1 + (NocClk & ALT_CLKMGR_MAINPLL_NOCCLK_CNT_SET_MSK);

  Dividor2 = 1 << ALT_CLKMGR_MAINPLL_NOCDIV_L4MAINCLK_GET (MmioRead32(ALT_CLKMGR_MAINPLL_OFST + ALT_CLKMGR_MAINPLL_NOCDIV_OFST));

  ClkSrc = (NocClk >> ALT_CLKMGR_MAINPLL_NOCCLK_SRC_LSB) &
    ALT_CLKMGR_MAINPLL_NOCCLK_SRC_SET_MSK;

  if (ClkSrc == ALT_CLKMGR_MAINPLL_NOCCLK_SRC_E_MAIN) {
    Status = AltClkPllVcoFreqGet(ALT_CLK_MAIN_PLL, &SrcHz);
    if (Status != EFI_SUCCESS)
      return Status;

    SrcHz /= 1 + ALT_CLKMGR_MAINPLL_NOCCLK_CNT_GET (MmioRead32(ALT_CLKMGR_ALTERA_OFST + ALT_CLKMGR_NOCCLK_OFST));

  } else if (ClkSrc == ALT_CLKMGR_MAINPLL_NOCCLK_SRC_E_PERI) {
    Status = AltClkPllVcoFreqGet(ALT_CLK_PERIPHERAL_PLL, &SrcHz);
    if (Status != EFI_SUCCESS)
      return Status;

     SrcHz /= 1 + ALT_CLKMGR_MAINPLL_NOCCLK_CNT_GET (MmioRead32(ALT_CLKMGR_ALTERA_OFST + ALT_CLKMGR_NOCCLK_OFST));

  } else if (ClkSrc == ALT_CLKMGR_MAINPLL_NOCCLK_SRC_E_OSC1) {
    SrcHz = Osc1ClkFreq;

  } else if (ClkSrc == ALT_CLKMGR_MAINPLL_NOCCLK_SRC_E_INTOSC) {
    SrcHz = IntOscClkFreq;

  } else if (ClkSrc == ALT_CLKMGR_MAINPLL_NOCCLK_SRC_E_FPGA) {
    SrcHz = PerClkFreq;

  } else {
    SrcHz = 0;
  }
  ASSERT_PLATFORM_INIT(SrcHz != 0);
  return SrcHz/Dividor/Dividor2;
}

BOOLEAN
EFIAPI
QspiIsIdle (
  VOID
  )
{
  // If the idle field of the QSPI configuration register is 1 then the serial
  // interface and QSPI pipeline is idle.
  return ALT_QSPI_CFG_IDLE_GET (MmioRead32 (ALT_QSPI_OFST + ALT_QSPI_CFG_OFST)) == 1;
}

////
//
// A helper function which converts a ns interval into a delay interval for a given MHz.
// The +999 is there to round up the result.
//
UINT32
EFIAPI
QspiNs2Multiplier (
  UINT32 Ns,
  UINT32 Mhz
  )
{
  return ((Ns * Mhz) + 999) / 1000;
}

EFI_STATUS
EFIAPI
QspiBaudRateDivSet (
  IN ALT_QSPI_BAUD_DIV_T BaudRateDiv
  )
{
  if (0xf < (UINT32)BaudRateDiv) {
      // Invalid baud rate divisor value.
      return EFI_INVALID_PARAMETER;
  }
   // Set the Master Mode Baud Rate Divisor Field of the QSPI Configuration Register.
  MmioAndThenOr32(ALT_QSPI_OFST +
                  ALT_QSPI_CFG_OFST,
                  ALT_QSPI_CFG_BAUDDIV_CLR_MSK,
                  ALT_QSPI_CFG_BAUDDIV_SET(BaudRateDiv));

  return EFI_SUCCESS;
}

EFI_STATUS
EFIAPI
QspiDeviceSizeConfigSet (
  OUT ALT_QSPI_DEV_SIZE_CONFIG_T *Cfg
  )
{
  // Although not required, it is recommended that the write protect feature
  // be enabled prior to enabling the QSPI controller. This will block any AHB
  // writes from taking effect. This also means the write protection registers
  // (Lower Write Protection, Upper Write Protection, and Write Protection)
  // should be setup and the number of bytes per device block in the device
  // Size configuration register should be setup prior to enabling the QSPI
  // controller.

  // Read Device Size Register and get the Number of Bytes per Block, Number
  // of Bytes per Device, and Number of Address Bytes Fields.

  UINT32 DevSz = MmioRead32(ALT_QSPI_OFST + ALT_QSPI_DEVSZ_OFST);

  Cfg->BlockSize = ALT_QSPI_DEVSZ_BYTESPERSUBSECTOR_GET(DevSz);
  Cfg->PageSize  = ALT_QSPI_DEVSZ_BYTESPERDEVICEPAGE_GET(DevSz);
  Cfg->AddrSize  = ALT_QSPI_DEVSZ_NUMADDRBYTES_GET(DevSz);

  // Read Lower Write Protection, Upper Write Protection, and Write Protection
  // Registers.

  Cfg->LowerWrProtBlock = ALT_QSPI_LOWWRPROT_SUBSECTOR_GET(MmioRead32(ALT_QSPI_OFST + ALT_QSPI_LOWWRPROT_OFST));
  Cfg->UpperWrProtBlock = ALT_QSPI_UPPWRPROT_SUBSECTOR_GET(MmioRead32(ALT_QSPI_OFST + ALT_QSPI_UPPWRPROT_OFST));
  Cfg->WrProtEnable     = ALT_QSPI_WRPROT_EN_GET(MmioRead32(ALT_QSPI_OFST + ALT_QSPI_WRPROT_OFST));

  return EFI_SUCCESS;
}

EFI_STATUS
EFIAPI
QspiDeviceReadConfigSet (
  IN ALT_QSPI_DEV_INST_CONFIG_T * Cfg
  )
{
  UINT32 DevRd;

  // Validate input
  if (Cfg->OpCode > ((1 << ALT_QSPI_DEVRD_RDOPCODE_WIDTH) - 1))  {
        return EFI_INVALID_PARAMETER;
  }

  switch (Cfg->InstType) {
    case ALT_QSPI_MODE_SINGLE:
    case ALT_QSPI_MODE_DUAL:
    case ALT_QSPI_MODE_QUAD:
        break;
    default:
        return EFI_INVALID_PARAMETER;
  }

  switch (Cfg->AddrXferType) {
    case ALT_QSPI_MODE_SINGLE:
    case ALT_QSPI_MODE_DUAL:
    case ALT_QSPI_MODE_QUAD:
        break;
    default:
        return EFI_INVALID_PARAMETER;
  }

  switch (Cfg->DataXferType){
    case ALT_QSPI_MODE_SINGLE:
    case ALT_QSPI_MODE_DUAL:
    case ALT_QSPI_MODE_QUAD:
        break;
    default:
        return EFI_INVALID_PARAMETER;
  }

  if (Cfg->DummyCycles > ((1 << ALT_QSPI_DEVRD_DUMMYRDCLKS_WIDTH) - 1)) {
        return EFI_INVALID_PARAMETER;
 }

  // Read the Device Read Instruction Register - devrd.
  DevRd = MmioRead32(ALT_QSPI_OFST + ALT_QSPI_DEVRD_OFST);

  DevRd &= ALT_QSPI_DEVRD_RDOPCODE_CLR_MSK &
           ALT_QSPI_DEVRD_INSTWIDTH_CLR_MSK &
           ALT_QSPI_DEVRD_ADDRWIDTH_CLR_MSK &
           ALT_QSPI_DEVRD_DATAWIDTH_CLR_MSK &
           ALT_QSPI_DEVRD_DUMMYRDCLKS_CLR_MSK;

  DevRd |= ALT_QSPI_DEVRD_RDOPCODE_SET(Cfg->OpCode) |
           ALT_QSPI_DEVRD_INSTWIDTH_SET(Cfg->InstType) |
           ALT_QSPI_DEVRD_ADDRWIDTH_SET(Cfg->AddrXferType) |
           ALT_QSPI_DEVRD_DATAWIDTH_SET(Cfg->DataXferType) |
           ALT_QSPI_DEVRD_DUMMYRDCLKS_SET(Cfg->DummyCycles);

  MmioWrite32(ALT_QSPI_OFST + ALT_QSPI_DEVRD_OFST, DevRd);

  return EFI_SUCCESS;
}

EFI_STATUS
EFIAPI
QspiDeviceWriteConfigSet (
  OUT ALT_QSPI_DEV_INST_CONFIG_T * Cfg
  )
{
  UINT32 DevWr;

  // Validate input

  if (Cfg->OpCode > ((1 << ALT_QSPI_DEVWR_WROPCODE_WIDTH) - 1)) {
     return EFI_INVALID_PARAMETER;
  }

  switch (Cfg->InstType) {
    case ALT_QSPI_MODE_SINGLE:
    case ALT_QSPI_MODE_DUAL:
    case ALT_QSPI_MODE_QUAD:
        break;
    default:
        return EFI_DEVICE_ERROR;
  }

  switch (Cfg->AddrXferType) {
    case ALT_QSPI_MODE_SINGLE:
    case ALT_QSPI_MODE_DUAL:
    case ALT_QSPI_MODE_QUAD:
        break;
    default:
        return EFI_DEVICE_ERROR;
  }

  switch (Cfg->DataXferType) {
    case ALT_QSPI_MODE_SINGLE:
    case ALT_QSPI_MODE_DUAL:
    case ALT_QSPI_MODE_QUAD:
        break;
    default:
        return EFI_DEVICE_ERROR;
  }

  if (Cfg->DummyCycles > ((1 << ALT_QSPI_DEVWR_DUMMYWRCLKS_WIDTH) - 1)) {
        return EFI_INVALID_PARAMETER;
  }

  // Read the Device Write Instruction Register - devwr.
  DevWr = MmioRead32(ALT_QSPI_OFST + ALT_QSPI_DEVWR_OFST);

  DevWr &= ALT_QSPI_DEVWR_WROPCODE_CLR_MSK &
           ALT_QSPI_DEVWR_ADDRWIDTH_CLR_MSK &
           ALT_QSPI_DEVWR_DATAWIDTH_CLR_MSK &
           ALT_QSPI_DEVWR_DUMMYWRCLKS_CLR_MSK;

  DevWr |= ALT_QSPI_DEVWR_WROPCODE_SET(Cfg->OpCode) |
           ALT_QSPI_DEVWR_ADDRWIDTH_SET(Cfg->AddrXferType) |
           ALT_QSPI_DEVWR_DATAWIDTH_SET(Cfg->DataXferType) |
           ALT_QSPI_DEVWR_DUMMYWRCLKS_SET(Cfg->DummyCycles);

  MmioWrite32(ALT_QSPI_OFST + ALT_QSPI_DEVWR_OFST, DevWr);

  // The Instruction Type field in the Device READ Instruction Register only appears
  // once and applies to both READ and WRITE operations - it is not included in the
  // Device WRITE Instruction Register. Therefore, modify the Instruction Type
  // Field in the Device Read Register.
  MmioAndThenOr32 (ALT_QSPI_OFST +
                   ALT_QSPI_DEVRD_OFST,
                   ALT_QSPI_DEVRD_INSTWIDTH_CLR_MSK,
                   ALT_QSPI_DEVRD_INSTWIDTH_SET((UINT32) Cfg->InstType));

  return EFI_SUCCESS;
}

EFI_STATUS
EFIAPI
QspiTimingConfigSet (
  OUT ALT_QSPI_TIMING_CONFIG_T * Cfg
  )
{
  UINT32 Cfgreg;

  // Validate parameter(s)

  switch (Cfg->ClkPhase) {
    case ALT_QSPI_CLK_PHASE_ACTIVE:
    case ALT_QSPI_CLK_PHASE_INACTIVE:
        break;
    default:
        return EFI_DEVICE_ERROR;
  }

  switch (Cfg->ClkPol) {
    case ALT_QSPI_CLK_POLARITY_LOW:
    case ALT_QSPI_CLK_POLARITY_HIGH:
        break;
    default:
        return EFI_DEVICE_ERROR;
  }

  if( (Cfg->CsDa > ((1 << ALT_QSPI_DELAY_NSS_WIDTH) - 1)) ||
     (Cfg->CsDads > ((1 << ALT_QSPI_DELAY_BTWN_WIDTH) - 1)) ||
     (Cfg->CsEot > ((1 << ALT_QSPI_DELAY_AFTER_WIDTH) - 1)) ||
     (Cfg->CsSot > ((1 << ALT_QSPI_DELAY_INIT_WIDTH) - 1)) ||
     (Cfg->RdDatacap > ((1 << ALT_QSPI_RDDATACAP_DELAY_WIDTH) - 1))) {
    return EFI_DEVICE_ERROR;
  }

   // QSPI Configuration Register - Cfg
  Cfgreg = MmioRead32(ALT_QSPI_OFST + ALT_QSPI_CFG_OFST);
  Cfgreg &= ALT_QSPI_CFG_SELCLKPHASE_CLR_MSK &
            ALT_QSPI_CFG_SELCLKPOL_CLR_MSK;
  Cfgreg |= ALT_QSPI_CFG_SELCLKPHASE_SET(Cfg->ClkPhase) |
            ALT_QSPI_CFG_SELCLKPOL_SET(Cfg->ClkPol);
  MmioWrite32(ALT_QSPI_OFST + ALT_QSPI_CFG_OFST, Cfgreg);

  // QSPI Device Delay Register
  UINT32 DelayReg = ALT_QSPI_DELAY_INIT_SET(Cfg->CsSot)  |
                    ALT_QSPI_DELAY_AFTER_SET(Cfg->CsEot) |
                    ALT_QSPI_DELAY_BTWN_SET(Cfg->CsDads) |
                    ALT_QSPI_DELAY_NSS_SET(Cfg->CsDa);
  MmioWrite32(ALT_QSPI_OFST + ALT_QSPI_DELAY_OFST, DelayReg);

  // Read Data Capture Register

  MmioWrite32(ALT_QSPI_OFST + ALT_QSPI_RDDATACAP_OFST,
              ALT_QSPI_RDDATACAP_BYP_SET(1) |
              ALT_QSPI_RDDATACAP_DELAY_SET(Cfg->RdDatacap));

  return EFI_SUCCESS;
}

//
// Private STIG and device commands
//

EFI_STATUS
EFIAPI
QspiStigCmdHelper (
  IN UINT32 RegValue
  )
{
  EFI_STATUS Status = EFI_SUCCESS;
  UINTN Count;

  MmioWrite32(ALT_QSPI_OFST + ALT_QSPI_FLSHCMD_OFST, RegValue);
  MmioWrite32(ALT_QSPI_OFST + ALT_QSPI_FLSHCMD_OFST, RegValue | ALT_QSPI_FLSHCMD_EXECCMD_E_EXECUTE);

  Count = 0;
  do {
    RegValue = MmioRead32(ALT_QSPI_OFST + ALT_QSPI_FLSHCMD_OFST);
    if (!(RegValue & ALT_QSPI_FLSHCMD_CMDEXECSTAT_SET_MSK)) {
      break;
    }
	Count++;
  } while (Count < ALT_QSPI_COMMAND_TIMEOUT);

  if (Count >= ALT_QSPI_COMMAND_TIMEOUT) {
     Status = EFI_TIMEOUT;
  }
  return Status;
}

EFI_STATUS
EFIAPI
QspiStigCmd (
  IN UINT32 Opcode,
  IN UINT32 Dummy
  )
{
  UINT32 Reg;
  if (Dummy > ((1 << ALT_QSPI_FLSHCMD_NUMDUMMYBYTES_WIDTH) - 1)) {
    return EFI_DEVICE_ERROR;
  }
  Reg = ALT_QSPI_FLSHCMD_CMDOPCODE_SET(Opcode) |
        ALT_QSPI_FLSHCMD_NUMDUMMYBYTES_SET(Dummy);

  return QspiStigCmdHelper(Reg);
}

EFI_STATUS
EFIAPI
QspiStigRdCmd (
  IN UINT8 Opcode,
  IN UINT32 Dummy,
  IN UINT32 NumBytes,
  IN UINT32 *Output
  )
{
  EFI_STATUS Status = EFI_SUCCESS;
  UINT32 RegValue;

  if (Dummy > ((1 << ALT_QSPI_FLSHCMD_NUMDUMMYBYTES_WIDTH) - 1)) {
     return EFI_DEVICE_ERROR;
  }

  // STIG read can only return up to 8 bytes.
  if ((NumBytes > 8) || (NumBytes == 0)) {
    return EFI_UNSUPPORTED;
  }

  RegValue =
        ALT_QSPI_FLSHCMD_CMDOPCODE_SET(Opcode)                              |
        ALT_QSPI_FLSHCMD_ENRDDATA_SET(ALT_QSPI_FLSHCMD_ENRDDATA_E_EN)       |
        ALT_QSPI_FLSHCMD_NUMRDDATABYTES_SET(NumBytes - 1)                   |
        ALT_QSPI_FLSHCMD_ENCMDADDR_SET(ALT_QSPI_FLSHCMD_ENCMDADDR_E_DISD)   |
        ALT_QSPI_FLSHCMD_ENMODBIT_SET(ALT_QSPI_FLSHCMD_ENMODBIT_E_DISD)     |
        ALT_QSPI_FLSHCMD_NUMADDRBYTES_SET(0)                                |
        ALT_QSPI_FLSHCMD_ENWRDATA_SET(ALT_QSPI_FLSHCMD_ENWRDATA_E_NOACTION) |
        ALT_QSPI_FLSHCMD_NUMWRDATABYTES_SET(0)                              |
        ALT_QSPI_FLSHCMD_NUMDUMMYBYTES_SET(Dummy);


  Status = QspiStigCmdHelper(RegValue);
  if (EFI_ERROR(Status))
    return Status;
  Output[0] = MmioRead32(ALT_QSPI_OFST + ALT_QSPI_FLSHCMDRDDATALO_OFST);

  if (NumBytes > 4) {
    Output[1] = MmioRead32(ALT_QSPI_OFST + ALT_QSPI_FLSHCMDRDDATAUP_OFST);
  }

  return EFI_SUCCESS;
}

EFI_STATUS
EFIAPI
QspiStigWrCmd (
  IN UINT8 Opcode,
  IN UINT32 Dummy,
  IN UINT32 NumBytes,
  IN UINT32 *Input
  )
{
  UINT32 RegValue;
  if (Dummy > ((1 << ALT_QSPI_FLSHCMD_NUMDUMMYBYTES_WIDTH) - 1)) {
    return EFI_DEVICE_ERROR;
  }

  // STIG read can only return up to 8 bytes.
  if ((NumBytes > 8) || (NumBytes == 0)) {
        return EFI_UNSUPPORTED;
    }

  RegValue =
        ALT_QSPI_FLSHCMD_CMDOPCODE_SET(Opcode)                                 |
        ALT_QSPI_FLSHCMD_ENRDDATA_SET(ALT_QSPI_FLSHCMD_ENRDDATA_E_NOACTION)    |
        ALT_QSPI_FLSHCMD_NUMRDDATABYTES_SET(0)                                 |
        ALT_QSPI_FLSHCMD_ENCMDADDR_SET(ALT_QSPI_FLSHCMD_ENCMDADDR_E_DISD)      |
        ALT_QSPI_FLSHCMD_ENMODBIT_SET(ALT_QSPI_FLSHCMD_ENMODBIT_E_DISD)        |
        ALT_QSPI_FLSHCMD_NUMADDRBYTES_SET(0)                                   |
        ALT_QSPI_FLSHCMD_ENWRDATA_SET(ALT_QSPI_FLSHCMD_ENWRDATA_E_WRDATABYTES) |
        ALT_QSPI_FLSHCMD_NUMWRDATABYTES_SET(NumBytes - 1)                     |
        ALT_QSPI_FLSHCMD_NUMDUMMYBYTES_SET(Dummy);

  MmioWrite32(ALT_QSPI_OFST + ALT_QSPI_FLSHCMDWRDATALO_OFST, Input[0]);

  if (NumBytes > 4) {
    MmioWrite32(ALT_QSPI_OFST + ALT_QSPI_FLSHCMDWRDATAUP_OFST, Input[1]);
  }

  return QspiStigCmdHelper(RegValue);
}

EFI_STATUS
EFIAPI
QspiStigAddrCmd (
  IN UINT8 Opcode,
  IN UINT32 Dummy,
  IN UINT32 Address
  )
{
  UINT32 Reg;
  if (Dummy > ((1 << ALT_QSPI_FLSHCMD_NUMDUMMYBYTES_WIDTH) - 1)) {
    return EFI_DEVICE_ERROR;
  }

  Reg = ALT_QSPI_FLSHCMD_CMDOPCODE_SET(Opcode) |
        ALT_QSPI_FLSHCMD_NUMDUMMYBYTES_SET(Dummy);

  Reg |= ALT_QSPI_FLSHCMD_ENCMDADDR_SET(ALT_QSPI_FLSHCMD_ENCMDADDR_E_END);
  Reg |= ALT_QSPI_FLSHCMD_NUMADDRBYTES_SET(ALT_QSPI_FLSHCMD_NUMADDRBYTES_E_ADDRBYTE3);

  MmioWrite32(ALT_QSPI_OFST + ALT_QSPI_FLSHCMDADDR_OFST, Address);

  return QspiStigCmdHelper (Reg);
}

EFI_STATUS
EFIAPI
QspiDeviceWren (
  VOID
  )
{
  // Write enable through STIG (not required, auto send by controller during write)
  return QspiStigCmd(ALT_QSPI_STIG_OPCODE_WREN, 0);
}

EFI_STATUS QspiDeviceRdId (
  IN UINT32 *Rdid
  )
{
  // Read flash device ID through STIG
  return QspiStigRdCmd(ALT_QSPI_STIG_OPCODE_RDID, 0, 4, Rdid);
}

EFI_STATUS
EFIAPI
QspiDeviceBankSelect (
  IN UINT32 Bank
  )
{
  EFI_STATUS Status = EFI_SUCCESS;
  // InfoPrint("DEBUG[QSPI]: bank_select(): switching to bank 0x%d.\n", Bank);

  Status = QspiDeviceWren();
  if (EFI_ERROR(Status))
    return Status;

  Status = QspiStigWrCmd(ALT_QSPI_STIG_OPCODE_WR_EXT_REG, 0, 1, &Bank);
  if (EFI_ERROR(Status))
    return Status;

  return QspiStigCmd(ALT_QSPI_STIG_OPCODE_WRDIS, 0);
}

EFI_STATUS
EFIAPI
QspiDeviceStatus (
  IN UINT32 *Status)
{
  // Read flag status register through STIG
  return QspiStigRdCmd(ALT_QSPI_STIG_OPCODE_RDSR, 0, 1, Status);
}

#if ALT_QSPI_PROVISION_MICRON_N25Q_SUPPORT
EFI_STATUS
EFIAPI
QspiN25QDeviceFlag (
  IN UINT32 *FlagSr
  )
{
  // Read flag status register through STIG
  return QspiStigRdCmd(ALT_QSPI_STIG_OPCODE_RDFLGSR, 0, 1, FlagSr);
}

// NOTE: This must be called after QSPI has been enabled. Communications with
//   the device will not happen until QSPI is enabled.
EFI_STATUS
EFIAPI
QspiN25QEnable (
  VOID
  )
{
  ALT_QSPI_DEV_INST_CONFIG_T  Cfg;
  ALT_QSPI_MODE_T             TransferMode;
  UINT32                      Opcode;
  UINT32                      DummyCycles;
  EFI_STATUS                  Status;

  Status = EFI_SUCCESS;

  switch (PcdGet32 (PcdQspiMode)) {
    case ALT_QSPI_MODE_SINGLE:
      TransferMode = ALT_QSPI_MODE_SINGLE;
      Opcode = ALT_QSPI_STIG_OPCODE_FASTREAD;
      DummyCycles = 8;
      break;
    case ALT_QSPI_MODE_DUAL:
      TransferMode = ALT_QSPI_MODE_DUAL;
      Opcode = ALT_QSPI_STIG_OPCODE_FASTREAD_DUAL_IO;
      DummyCycles = 8;
      break;
    case ALT_QSPI_MODE_QUAD:
      TransferMode = ALT_QSPI_MODE_QUAD;
      Opcode = ALT_QSPI_STIG_OPCODE_FASTREAD_QUAD_IO;
      DummyCycles = 10;
      break;
    default:
      //ProgressPrint ("Invalid PcdQspiMode \r\n");
	    ASSERT_PLATFORM_INIT (0);
      return EFI_INVALID_PARAMETER;
  }
  // Reset the volatile memory on the N25Q
  Status = QspiStigCmd(ALT_QSPI_STIG_OPCODE_RESET_EN, 0);

  Status = QspiStigCmd(ALT_QSPI_STIG_OPCODE_RESET_MEM, 0);

  Cfg.OpCode        = Opcode;
  Cfg.InstType      = ALT_QSPI_MODE_SINGLE; // RDID does not support QUAD.
  Cfg.AddrXferType  = TransferMode;
  Cfg.DataXferType  = TransferMode;
  Cfg.DummyCycles   = DummyCycles;

  return QspiDeviceReadConfigSet(&Cfg);
}

EFI_STATUS
EFIAPI
QspiN25QFlagWaitForProgramOrErase (
  IN BOOLEAN ProgramNotErase
  )
{
  EFI_STATUS Status;
  UINT32 Count = 0;
  UINT32 Stat = 0;
  UINT32 FlagSr;

  // poll for device no long busy
  do {
    Status = QspiDeviceStatus(&Stat);
    if (EFI_ERROR(Status))
      break;
    if (!ALT_QSPI_STIG_SR_BUSY_GET(Stat))
      break;
    Count++;
  } while (Count < ALT_QSPI_COMMAND_TIMEOUT);

  if (Count == ALT_QSPI_COMMAND_TIMEOUT)
    return EFI_TIMEOUT;

 // poll for program ready
  Count = 0;
  FlagSr = 0;
  do  {
    Status = QspiN25QDeviceFlag(&FlagSr);
    if (Status != EFI_SUCCESS)
      return Status;
    if ((ProgramNotErase && ALT_QSPI_STIG_FLAGSR_PROGRAMREADY_GET(FlagSr)) ||
	   (!ProgramNotErase && ALT_QSPI_STIG_FLAGSR_ERASEREADY_GET(FlagSr)))
      break;

  } while (Count < ALT_QSPI_COMMAND_TIMEOUT);

  if (Count == ALT_QSPI_COMMAND_TIMEOUT)
    return EFI_TIMEOUT;

  if ((ProgramNotErase && ALT_QSPI_STIG_FLAGSR_PROGRAMERROR_GET(FlagSr)) ||
      (!ProgramNotErase && ALT_QSPI_STIG_FLAGSR_ERASEERROR_GET(FlagSr)))
    return EFI_DEVICE_ERROR;

  return EFI_SUCCESS;
}

#endif

EFI_STATUS
EFIAPI
QspiIndirectReadStartBank  (
  IN UINT32 FlashAddr,
  IN UINT32 NumBytes
  )
{
  MmioWrite32(ALT_QSPI_OFST + ALT_QSPI_INDRDSTADDR_OFST, FlashAddr);
  MmioWrite32(ALT_QSPI_OFST + ALT_QSPI_INDRDCNT_OFST, NumBytes);
  MmioWrite32(ALT_QSPI_OFST + ALT_QSPI_INDRD_OFST,
              ALT_QSPI_INDRD_START_SET_MSK |
              ALT_QSPI_INDRD_IND_OPS_DONE_STAT_SET_MSK);

  return EFI_SUCCESS;
}

EFI_STATUS
EFIAPI
QspiIndirectReadStart(
  IN UINT32 FlashAddr,
  IN UINT32 NumBytes
  )
{
  EFI_STATUS Status;

  // FlashAddr and NumBytes restriction is to prevent possible unaligned
  // exceptions.

  if ((FlashAddr & 0x3) ||
      (NumBytes & 0x3) ||
      (NumBytes == 0)  ||
      (FlashAddr > QspiDeviceSize) ||
      (FlashAddr + NumBytes > QspiDeviceSize)) {
    return EFI_DEVICE_ERROR;
  }

  // Verify request does not cross bank boundary.
  // This limitation is due to the 3-byte addressing limitation.
  if ((FlashAddr & ALT_QSPI_BANK_ADDR_MSK) != ((FlashAddr + NumBytes - 1) &
      ALT_QSPI_BANK_ADDR_MSK)) {
    return EFI_DEVICE_ERROR;
  }

  // Verify that there is not already a read in progress.
  if (ALT_QSPI_INDRD_RD_STAT_GET(MmioRead32(ALT_QSPI_OFST + ALT_QSPI_INDRD_OFST))) {
    return EFI_DEVICE_ERROR;
  }

  Status = QspiDeviceBankSelect(ALT_QSPI_BANK_ADDR_GET(FlashAddr));
  if (EFI_ERROR(Status)) {
    return Status;
  }

  return QspiIndirectReadStartBank(FlashAddr, NumBytes);

}

EFI_STATUS
EFIAPI
QspiIndirectWriteStartBank (
  IN UINT32 FlashAddr,
  IN UINT32 NumBytes
  )
{
  MmioWrite32(ALT_QSPI_OFST + ALT_QSPI_INDWRSTADDR_OFST, FlashAddr);
  MmioWrite32(ALT_QSPI_OFST + ALT_QSPI_INDWRCNT_OFST, NumBytes);
  MmioWrite32(ALT_QSPI_OFST + ALT_QSPI_INDWR_OFST,
              ALT_QSPI_INDWR_START_SET_MSK |
              ALT_QSPI_INDWR_INDDONE_SET_MSK);

  return EFI_SUCCESS;
}

EFI_STATUS
EFIAPI
QspiIndirectWriteStart (
  IN UINT32 FlashAddr,
  IN UINT32 NumBytes
  )
{
  EFI_STATUS Status = EFI_SUCCESS;

  // FlashAddr and NumBytes restriction is to prevent possible unaligned
   // exceptions.
  if ((FlashAddr & 0x3) ||
      (NumBytes & 0x3) ||
      (NumBytes == 0)  ||
      (NumBytes > 256) ||
      (FlashAddr > QspiDeviceSize) ||
      (FlashAddr + NumBytes > QspiDeviceSize)) {
    return EFI_DEVICE_ERROR;
  }

  // Verify request does not cross bank boundary.
  // This limitation is due to the 3-byte addressing limitation.
  //if ((FlashAddr & ALT_QSPI_BANK_OFST_MSK) != ((FlashAddr + NumBytes - 1) & ALT_QSPI_BANK_OFST_MSK))
  //{
  //    return EFI_DEVICE_ERROR;
  //}
  // Verify request does not cross page boundary.
  // This limitation is in place for the Micron part used.
  if ((FlashAddr & ALT_QSPI_PAGE_ADDR_MSK) != ((FlashAddr + NumBytes - 1) &
       ALT_QSPI_PAGE_ADDR_MSK)) {
      return EFI_DEVICE_ERROR;
  }

  // Verify that there is not already a write in progress.
  if (ALT_QSPI_INDWR_RDSTAT_GET(MmioRead32(ALT_QSPI_OFST + ALT_QSPI_INDWR_OFST))) {
    return EFI_NOT_READY;
  }
  Status = QspiDeviceBankSelect (ALT_QSPI_BANK_ADDR_GET(FlashAddr));
  if (EFI_ERROR(Status)) {
    return Status;
  }

  return QspiIndirectWriteStartBank (FlashAddr, NumBytes);
}

EFI_STATUS
EFIAPI
QspiIndirectWriteFinish (
  VOID
  )
{
 #if ALT_QSPI_PROVISION_MICRON_N25Q_SUPPORT
    return QspiN25QFlagWaitForProgramOrErase (1);
#else
    return EFI_SUCCESS;
#endif

}


EFI_STATUS
EFIAPI
QspiEnable (
  VOID
  )
{
  EFI_STATUS Status;
  MmioOr32(ALT_QSPI_OFST +
           ALT_QSPI_CFG_OFST,
           ALT_QSPI_CFG_EN_SET_MSK);

  // Device specific configuration
#if ALT_QSPI_PROVISION_MICRON_N25Q_SUPPORT
  Status = QspiN25QEnable();
  if (EFI_ERROR(Status))
    return Status;
#endif
  return EFI_SUCCESS;
}

EFI_STATUS
EFIAPI
QspiEraseSubsectorBank (
  IN UINT32 Addr
  )
{
  EFI_STATUS Status = EFI_SUCCESS;

  Status = QspiDeviceWren();
  if (EFI_ERROR(Status)) {
    return Status;
  }
  Status = QspiStigAddrCmd (ALT_QSPI_STIG_OPCODE_SUBSEC_ERASE, 0, Addr);
  if (EFI_ERROR(Status)) {
    return Status;
  }

#if ALT_QSPI_PROVISION_MICRON_N25Q_SUPPORT
  Status = QspiN25QFlagWaitForProgramOrErase (0);
#endif
  return Status;
}

EFI_STATUS
EFIAPI
QspiEraseSubsector (
  IN UINT32 Addr
  )
{
  EFI_STATUS Status = EFI_SUCCESS;

  Status = QspiDeviceBankSelect(Addr >> 24);
  if (EFI_ERROR(Status)) {
    return Status;
  }
  return QspiEraseSubsectorBank(Addr);
}

EFI_STATUS
EFIAPI
QspiEraseSector (
  IN UINT32 Addr
  )
{
  EFI_STATUS Status = EFI_SUCCESS;

  Status = QspiDeviceBankSelect (Addr >> 24);
  if (EFI_ERROR(Status)) {
    return Status;
  }
  Status = QspiDeviceWren ();
  if (EFI_ERROR(Status)) {
    return Status;
  }
  Status = QspiStigAddrCmd (ALT_QSPI_STIG_OPCODE_SEC_ERASE, 0, Addr);
  if (EFI_ERROR(Status)) {
    return Status;
  }
#if ALT_QSPI_PROVISION_MICRON_N25Q_SUPPORT
  Status = QspiN25QFlagWaitForProgramOrErase (0);
#endif
  return Status;
}

EFI_STATUS
EFIAPI
QspiEraseChip (
  VOID
  )
{
  EFI_STATUS Status = EFI_SUCCESS;
  UINTN i;
  UINT32 DieCount;

  if (QspiDeviceSize >= (2 * ALT_QSPI_N25Q_DIE_SIZE)) {
    // NOTE: This path is specifically for 512 Mib and 1 Gib Micron N25Q
    //   chips only.
    // InfoPrint("DEBUG[QSPI]: erase[chip]: FYI, wait time is ~800s for 128 MiB.\n");
    DieCount = QspiDeviceSize / ALT_QSPI_N25Q_DIE_SIZE;
    //DieCount = 1; // for fast test only
    for (i = 0; i < DieCount; ++i)  {
      if (Status != EFI_SUCCESS) {
        break;
      }

      // InfoPrint("DEBUG[QSPI]: Erase chip: die = %d, total = %d.\n", i, DieCount);

      Status = QspiDeviceBankSelect(i * (ALT_QSPI_N25Q_DIE_SIZE / ALT_QSPI_BANK_SIZE));
      if (EFI_ERROR(Status))
        return Status;
      Status = QspiDeviceWren();
      if (EFI_ERROR(Status))
        return Status;
      Status = QspiStigAddrCmd(ALT_QSPI_STIG_OPCODE_DIE_ERASE, 0,
                                  i * ALT_QSPI_N25Q_DIE_SIZE);
      if (EFI_ERROR(Status))
        return Status;
      #if ALT_QSPI_PROVISION_MICRON_N25Q_SUPPORT
      Status = QspiN25QFlagWaitForProgramOrErase (0);
       if (EFI_ERROR(Status))
        return Status;
      #endif
    }
  } else {
    // NOTE: Untested path.

    // InfoPrint("DEBUG[QSPI]: Bulk erase.\n");
    Status = QspiDeviceBankSelect(0);
    if (EFI_ERROR(Status))
      return Status;
    Status = QspiDeviceWren();
    if (EFI_ERROR(Status))
      return Status;
    // If BULK_ERASE is like other ERASE, it needs the address command.
    Status = QspiStigAddrCmd(ALT_QSPI_STIG_OPCODE_BULK_ERASE, 0, 0);
    if (EFI_ERROR(Status))
      return Status;
    #if ALT_QSPI_PROVISION_MICRON_N25Q_SUPPORT
      Status = QspiN25QFlagWaitForProgramOrErase (0);
      if (EFI_ERROR(Status))
        return Status;
    #endif
  }

  return Status;
}

VOID
QspiCalibration (
  IN UINT32 DeviceClk,
  IN UINT32 QspiClkMhz
  )
{
  EFI_STATUS Status;
  UINT32 DeviceSClkMhz = 27; // minum value to get biggest 0xF div factor
  UINT32 DataCapDelay;
  UINT32 SampleRdId;
  UINT32 RdId;
  UINT32 DivActual;
  ALT_QSPI_BAUD_DIV_T DivBits;
  UINT32 Temp;

  // 1.  Set divider to bigger value (slowest SCLK)
  // 2.  RDID and save the value

  DivActual = (QspiClkMhz + (DeviceSClkMhz - 1)) / DeviceSClkMhz;
  DivBits = (ALT_QSPI_BAUD_DIV_T)(((DivActual + 1) / 2) - 1);
  Status = QspiBaudRateDivSet(DivBits);

  Status = QspiDeviceRdId(&SampleRdId);
  if (EFI_ERROR (Status))
    return;

  //3. Set divider to the intended frequency
  //4.  Set the read delay = 0
  //5.  RDID and check whether the value is same as item 2
  //6.  Increase read delay and compared the value against item 2
  //7.  Find the range of read delay that have same as item 2 and divide it to 2
  DivActual = (QspiClkMhz + (DeviceClk - 1)) / DeviceClk;
  DivBits = (ALT_QSPI_BAUD_DIV_T)(((DivActual + 1) / 2) - 1);
  Status = QspiBaudRateDivSet(DivBits);
  if (EFI_ERROR (Status))
    return;

  DataCapDelay = 0;
  Temp = 0;

  do {
    if (EFI_ERROR (Status))
      break;
    Status = QspiDeviceRdId(&RdId);
    if (EFI_ERROR (Status))
      break;
    if (RdId == SampleRdId) {
      Temp = DataCapDelay;
    }
    DataCapDelay ++;
    // Set Read Data Capture Register
    MmioWrite32(ALT_QSPI_OFST + ALT_QSPI_RDDATACAP_OFST,
                   ALT_QSPI_RDDATACAP_BYP_SET(1) |
                   ALT_QSPI_RDDATACAP_DELAY_SET(DataCapDelay));

  } while (DataCapDelay < 0x10);

  if (Temp > 0)
    DataCapDelay = Temp/2;
  // Set Read Data Capture Register
  // InfoPrint("DEBUG[QSPI]: Finish Calibration\n");
  // InfoPrint("DEBUG[QSPI]: DataCap Delay = %x.\n", DataCapDelay);
  MmioWrite32(ALT_QSPI_OFST + ALT_QSPI_RDDATACAP_OFST,
                   ALT_QSPI_RDDATACAP_BYP_SET(1) |
                   ALT_QSPI_RDDATACAP_DELAY_SET(DataCapDelay));
  Status = QspiDeviceRdId(&RdId);
  if (EFI_ERROR (Status))
    return;
  // InfoPrint("DEBUG[QSPI]: RdId = %x.\n", RdId);

}

EFI_STATUS
EFIAPI
QspiIntDisable (
  IN UINT32 Mask
  )
{
  if (QspiIsIdle() == FALSE) {
    return EFI_DEVICE_ERROR;
  }

  // Check that the [Mask] contains valid interrupt Status conditions values.
  if ((ALT_QSPI_INT_STATUS_ALL & Mask) == 0) {
    return EFI_DEVICE_ERROR;
  }

  // Write 0's to disable the desired interrupt Status condition(s).
  MmioAnd32(ALT_QSPI_OFST +
            ALT_QSPI_IRQMSK_OFST,
            Mask);
  return EFI_SUCCESS;
}


EFI_STATUS
EFIAPI
QspiInit (
  VOID
  )
{
  ALT_QSPI_DEV_SIZE_CONFIG_T SizeCfg;
  ALT_QSPI_TIMING_CONFIG_T TimingCfg;
  EFI_STATUS               Status;
  UINT32                   QspiClkFreq;
  UINT32                   QspiClkMhz;
  UINT32                   QspiDesiredClkFreq;
  UINT32                   RdId;
  VOID*                    Fdt;
  UINT32                   CapCode;
  UINT32				   AddrSize;

  RdId = 0;
  Status = EFI_SUCCESS;

  ProgressPrint ("Initializing QSPI\r\n");

  Status = GetFlattenedDeviceTreePtr (&Fdt);
  ASSERT_PLATFORM_INIT (!EFI_ERROR(Status));

  Status = AltClkFregGetFdt (Fdt);
  if (EFI_ERROR(Status)) return Status;

  QspiClkFreq = QspiGetClockFreq();
  QspiClkMhz = QspiClkFreq / 1000000;
  if (QspiClkFreq > ALT_QSPI_CLK_FREQ_MAX) {
    InfoPrint ("Unsupported QSPI clock freq %d Mhz\n", QspiClkMhz);
    return EFI_DEVICE_ERROR;
  }

  // Take QSPI controller out of reset.
  MmioAnd32(ALT_RSTMGR_OFST +
            ALT_RSTMGR_PER0MODRST_OFST,
            ALT_RSTMGR_PER0MODRST_QSPI_CLR_MSK);

  if (QspiIsIdle() == FALSE) {
    // InfoPrint("QSPI is not idle\n");
    return EFI_DEVICE_ERROR;
  }

  // Configure the delay/device timing
  TimingCfg.ClkPhase  = (ALT_QSPI_CLK_PHASE_T)ALT_QSPI_CFG_SELCLKPHASE_RESET;
  TimingCfg.ClkPol    = (ALT_QSPI_CLK_POLARITY_T)ALT_QSPI_CFG_SELCLKPOL_RESET;
  TimingCfg.CsDa      = QspiNs2Multiplier(ALT_QSPI_TSHSL_NS_DEF, QspiClkMhz);
  TimingCfg.CsDads    = QspiNs2Multiplier(ALT_QSPI_TSD2D_NS_DEF, QspiClkMhz);
  TimingCfg.CsEot     = QspiNs2Multiplier(ALT_QSPI_TCHSH_NS_DEF, QspiClkMhz);
  TimingCfg.CsSot     = QspiNs2Multiplier(ALT_QSPI_TSLCH_NS_DEF, QspiClkMhz);
  TimingCfg.RdDatacap = 1;

  // InfoPrint("DEBUG[QSPI]: CsDa   = %x.\n", QspiNs2Multiplier(ALT_QSPI_TSHSL_NS_DEF, QspiClkMhz));
  // InfoPrint("DEBUG[QSPI]: CsDads = %x.\n", TimingCfg.CsDads);
  // InfoPrint("DEBUG[QSPI]: CsEot  = %x.\n", TimingCfg.CsEot);
  // InfoPrint("DEBUG[QSPI]: CsSot  = %x.\n", TimingCfg.CsSot);

  Status = QspiTimingConfigSet(&TimingCfg);
  if (EFI_ERROR (Status))
    return Status;

  // InfoPrint ("Configure the remap address\n");
  // Configure the remap address register, no remap
   MmioWrite32(ALT_QSPI_OFST + ALT_QSPI_REMAPADDR_OFST, ALT_QSPI_REMAPADDR_VALUE_SET(0));

  // InfoPrint ("Configure the interrupt Mask\n");
  // Configure the interrupt Mask register, disabled all first
  Status = QspiIntDisable(ALT_QSPI_INT_STATUS_ALL);
  if (EFI_ERROR (Status))
    return Status;

  // InfoPrint("DEBUG[QSPI]: alt_qspi_enable()\n");
  Status = QspiEnable();
  if (EFI_ERROR (Status))
    return Status;

  // InfoPrint("DEBUG[QSPI]: qspi calibration\n");
  QspiDesiredClkFreq = PcdGet32 (PcdQspiClkFreq);
  QspiCalibration  (QspiDesiredClkFreq, QspiClkMhz);

  // InfoPrint("DEBUG[QSPI]: Query device capabilities\n");
  Status = QspiDeviceRdId(&RdId);
  if (EFI_ERROR(Status))
    return Status;

  // NOTE: The Size code seems to be a form of BCD (binary coded decimal).
  //   The first nibble is the 10's digit and the second nibble is the 1's
  //   digit in the number of bytes.

        // Capacity ID samples:
        //  0x15 :   16 Mb =>   2 MiB => 1 << 21 ; BCD=15
        //  0x16 :   32 Mb =>   4 MiB => 1 << 22 ; BCD=16
        //  0x17 :   64 Mb =>   8 MiB => 1 << 23 ; BCD=17
        //  0x18 :  128 Mb =>  16 MiB => 1 << 24 ; BCD=18
        //  0x19 :  256 Mb =>  32 MiB => 1 << 25 ; BCD=19
        //  0x1a
        //  0x1b
        //  0x1c
        //  0x1d
        //  0x1e
        //  0x1f
        //  0x20 :  512 Mb =>  64 MiB => 1 << 26 ; BCD=20
        //  0x21 : 1024 Mb => 128 MiB => 1 << 27 ; BCD=21

  CapCode = ALT_QSPI_STIG_RDID_CAPACITYID_GET(RdId);
  if ( ((CapCode >> 4) > 0x9) || ((CapCode & 0xf) > 0x9)) {
    // If a non-valid BCD value is detected at the top or bottom nibble, chances
    // are that the chip has a problem.
    InfoPrint("DEBUG[QSPI]: Invalid CapacityID encountered: 0x%02x\n", CapCode);
    return EFI_DEVICE_ERROR;
  } else {
    UINT32 CapDecoded = ((CapCode >> 4) * 10) + (CapCode & 0xf);
    QspiDeviceSize = 1 << (CapDecoded + 6);
    // InfoPrint("DEBUG[QSPI]: Device Size = 0x%x\n", QspiDeviceSize);
  }
  // If DeviceSize > 128M
  AddrSize = (CapCode > 0x18) ?
             ALT_QSPI_DEVSZ_4BYTE_ADDR:
			 ALT_QSPI_DEVSZ_3BYTE_ADDR;
  // InfoPrint("DEBUG[QSPI]: Configure the device Size and address bytes\n");
  SizeCfg.BlockSize         = ALT_QSPI_DEVSZ_BYTESPERSUBSECTOR_RESET;  // 0x10  => 2^16 = 64 KiB
  SizeCfg.PageSize          = ALT_QSPI_DEVSZ_BYTESPERDEVICEPAGE_RESET; // 0x100 => 256 B
  SizeCfg.AddrSize          = AddrSize - 1;
  SizeCfg.LowerWrProtBlock  = 0;
  SizeCfg.UpperWrProtBlock  = (QspiDeviceSize - 1) >> 16;
  SizeCfg.WrProtEnable      = ALT_QSPI_WRPROT_EN_RESET;

  Status = QspiDeviceSizeConfigSet(&SizeCfg);
  if (EFI_ERROR(Status))
    return Status;

  // InfoPrint("EVENT: Configure the DMA parameters\n");
  // Configure the DMA parameters
  // This will allow DMA to work well without much intervention by users.
  // FUTURE: Restore to {4, 32}; see case:240657.
  // Status = Qspidma_config_set(4, 32);
  //Status = QspiDmaConfigSet(4, 4);
  //disable DMA
  MmioAnd32(ALT_QSPI_OFST + ALT_QSPI_CFG_OFST, ALT_QSPI_CFG_ENDMA_CLR_MSK);
  // Print QSPI Info
  ProgressPrint("QSPI Flash Size = ", QspiDeviceSize);
  if (QspiDeviceSize >= (1024*1024*10)) {
    ProgressPrint ( "%d MiB\r\n", QspiDeviceSize / 1024 / 1024);
  } else {
    ProgressPrint ( "%d Bytes\r\n", QspiDeviceSize);
  }

  return Status;
}

EFI_STATUS
EFIAPI
QspiIndirectPageBoundWrite (
  IN UINT32  Offset,
  IN UINT8*  Buffer,
  IN UINT32  Length
  )
{
  EFI_STATUS Status = EFI_SUCCESS;
  UINTN i;
  UINT32 WriteCount;
  UINT32 WriteCapacity;
  UINT32 *WriteData;
  UINT32 Space;
  UINT32 WriteFillLever;
  UINT32 SramPartition;

  Status = QspiIndirectWriteStartBank(Offset, Length);
  if (EFI_ERROR(Status))
    return Status;

  WriteCount = 0;
  SramPartition = ALT_QSPI_SRAMPART_ADDR_GET(MmioRead32(ALT_QSPI_OFST + ALT_QSPI_SRAMPART_OFST));
  WriteCapacity = (UINT32) ALT_QSPI_SRAM_FIFO_ENTRY_COUNT - SramPartition;
  while (WriteCount < Length) {
    WriteFillLever = ALT_QSPI_SRAMFILL_INDWRPART_GET(MmioRead32(ALT_QSPI_OFST + ALT_QSPI_SRAMFILL_OFST));
	Space = WriteCapacity - WriteFillLever;
    WriteData = (UINT32 *)(Buffer + WriteCount);
    for (i= 0; i < Space; ++i) {
      MmioWrite32(ALT_QSPIDATA_OFST, *WriteData++);
    }
    WriteCount += Space * sizeof(UINT32);
  }
  return QspiIndirectWriteFinish();
}

//
// This helper function reads a segment of Data, which is limited to 1 bank
// (24 bits of addressing).
//
EFI_STATUS
QspiReadBank (
  IN UINT8*  Buffer,
  IN UINT32  Offset,
  IN UINT32  Size
  )
{
  EFI_STATUS Status;
  UINT32 ReadCount;
  UINT32 *ReadData;
  UINT32 i;
  UINT32 Level;

  Status = QspiIndirectReadStartBank(Offset, Size);
  if (EFI_ERROR(Status))
    return Status;

  ReadCount = 0;
  UINT32 Count = 0;
  while (!ALT_QSPI_INDRD_IND_OPS_DONE_STAT_GET(MmioRead32(ALT_QSPI_OFST + ALT_QSPI_INDRD_OFST))) {
    Level = ALT_QSPI_SRAMFILL_INDRDPART_GET(MmioRead32(ALT_QSPI_OFST + ALT_QSPI_SRAMFILL_OFST));
    ReadData = (UINT32 *)(Buffer + ReadCount);
    for (i = 0; i < Level; ++i) {
       *ReadData++ = MmioRead32(ALT_QSPIDATA_OFST);
    }
    ReadCount += Level * sizeof(UINT32);
    Count++;
  }
  return EFI_SUCCESS;
}

EFI_STATUS
EFIAPI
QspiWriteBank (
  IN UINT32  Offset,
  IN UINT8*  Buffer,
  IN UINT32  Size
  )
{
  EFI_STATUS Status = EFI_SUCCESS;
  UINT32 PageOfst  = Offset & (ALT_QSPI_PAGE_SIZE - 1);
  UINT32 WriteSize = MIN(Size, ALT_QSPI_PAGE_SIZE - PageOfst);

  while (Size) {
    // InfoPrint("DEBUG[QSPI]: write(): flash Offset = 0x%x, "
    //           "mem Buffer = %p, write Size = 0x%x, Size left = 0x%x.\n",
    //           Offset, Buffer, WriteSize, Size);

    Status = QspiIndirectPageBoundWrite(Offset, Buffer, WriteSize);
    if (Status != EFI_SUCCESS) {
      break;
    }

    Offset  += WriteSize;
    Buffer  += WriteSize;
    Size -= WriteSize;
    WriteSize = MIN(Size, ALT_QSPI_PAGE_SIZE);
  }
  return Status;
}

EFI_STATUS
EFIAPI
QspiRead (
  OUT VOID*  Buffer,
  IN UINT32  Offset,
  IN UINT32  Size
  )
{
  UINT32     BankCount;
  UINT32     BankAddr;
  UINT32     BankOfst;
  UINT8*     ReadData;
  UINT32     CopyLength;
  UINT32     i;
  EFI_STATUS Status;

  Status = EFI_SUCCESS;

  if ((Offset >= QspiDeviceSize) ||
      (Offset + Size - 1 >= QspiDeviceSize) ||
      (Size == 0) ||
      ((INT32)Buffer & 0x3)  ||
      (Offset & 0x3) ||
      (Size & 0x3)) {
    return EFI_INVALID_PARAMETER;
  }
    // Verify that there is not already a read in progress.
  if (ALT_QSPI_INDRD_RD_STAT_GET(MmioRead32(ALT_QSPI_OFST + ALT_QSPI_INDRD_OFST))) {
    return EFI_NOT_READY;
  }

  //
  // bank_count : The number of bank(s) affected, including partial banks.
  // bank_addr  : The aligned address of the first affected bank, including partial bank(s).
  // bank_ofst  : The offset of the bank to read. Only used when reading the first bank.
  //
  BankCount = ALT_QSPI_BANK_ADDR_GET (Offset + Size - 1) - ALT_QSPI_BANK_ADDR_GET (Offset) + 1;
  BankAddr  = Offset & ALT_QSPI_BANK_ADDR_MSK;
  BankOfst  = Offset & (ALT_QSPI_BANK_SIZE - 1);

  ReadData = (UINT8 *)Buffer;

  CopyLength = MIN(Size, ALT_QSPI_BANK_SIZE - BankOfst);

  //InfoPrint("DEBUG[QSPI]: read(): bulk: mem_addr = %p; FlashAddr = 0x%x.\n", ReadData, Offset);
  //InfoPrint("DEBUG[QSPI]: read(): bulk: bank_count = 0x%x, bank_ofst = 0x%x.\n", BankCount, BankOfst);

  for (i= 0; i < BankCount; ++i) {
    // InfoPrint("DEBUG[QSPI]: read(): bank 0x%x; copy_length = 0x%x.\n", BankAddr >> 24, CopyLength);

    Status = QspiDeviceBankSelect(ALT_QSPI_BANK_ADDR_GET (BankAddr));
    if (Status != EFI_SUCCESS) {
      break;
    }
    Status = QspiReadBank(ReadData, BankOfst, CopyLength);
    if (Status != EFI_SUCCESS) {
      break;
    }

    BankAddr += ALT_QSPI_BANK_SIZE;
    ReadData += CopyLength;
    Size -= CopyLength;
    BankOfst = 0;
    CopyLength = MIN(Size, ALT_QSPI_BANK_SIZE);
  }

  return Status;
}

EFI_STATUS
EFIAPI
QspiErase (
  IN UINT32 Offset,
  IN UINT32 Size
  )
{
  EFI_STATUS Status = EFI_SUCCESS;
  UINT32 SubsectorOfst  = Offset & (ALT_QSPI_SUBSECTOR_SIZE - 1);
  UINT32 EraseSize = MIN(Size, ALT_QSPI_SUBSECTOR_SIZE - SubsectorOfst);

  while (Size) {
    Status = QspiEraseSubsector(Offset);
    if (Status != EFI_SUCCESS) {
      break;
    }

    Offset  += EraseSize;
    Size -= EraseSize;
    EraseSize = MIN(Size, ALT_QSPI_SUBSECTOR_SIZE);
  }
  return Status;
}

EFI_STATUS
EFIAPI
QspiWrite (
  IN VOID*   Buffer,
  IN UINT32  Offset,
  IN UINT32  Size
  )
{
  EFI_STATUS Status;
  UINT32     BankCount;
  UINT32     BankAddr;
  UINT32     BankOfst;
  UINT8*     WriteData;
  UINT32     CopyLength;
  UINT32     i;

  Status = EFI_SUCCESS;

  if ((Offset >= QspiDeviceSize) ||
      (Offset + Size - 1 >= QspiDeviceSize) ||
      (Size == 0) ||
      ((INT32)Buffer & 0x3)  ||
      (Offset & 0x3) ||
      (Size & 0x3)) {
    return EFI_INVALID_PARAMETER;
  }

  // Verify that there is not already a write in progress.
  if (ALT_QSPI_INDWR_RDSTAT_GET(MmioRead32(ALT_QSPI_OFST + ALT_QSPI_INDWR_OFST))) {
    return EFI_DEVICE_ERROR;
  }

  BankCount = ALT_QSPI_BANK_ADDR_GET (Offset + Size - 1) - ALT_QSPI_BANK_ADDR_GET (Offset) + 1;
  BankAddr  = Offset & ALT_QSPI_BANK_ADDR_MSK;
  BankOfst  = Offset & (ALT_QSPI_BANK_SIZE - 1);

  WriteData  = Buffer;

  CopyLength = MIN(Size, ALT_QSPI_BANK_SIZE - BankOfst);

  // InfoPrint("DEBUG[QSPI]: write(): bulk: FlashAddr = 0x%x; mem_addr = %p.\n", Offset, WriteData);
  // InfoPrint("DEBUG[QSPI]: write(): bulk: bank_count = 0x%x, bank_ofst = 0x%x.\n", BankCount, BankOfst);

  for (i = 0; i < BankCount; ++i) {
    // InfoPrint("DEBUG[QSPI]: write(): bank 0x%x; copy_length = 0x%x.\n",  BankAddr >> 24, CopyLength);

    Status = QspiDeviceBankSelect(ALT_QSPI_BANK_ADDR_GET (BankAddr));
    if (Status != EFI_SUCCESS) {
      break;
    }

    Status = QspiWriteBank(BankOfst, WriteData, CopyLength);
    if (Status != EFI_SUCCESS){
      break;
    }

    BankAddr += ALT_QSPI_BANK_SIZE;
    WriteData += CopyLength;
    Size -= CopyLength;
    BankOfst = 0;

    CopyLength = MIN(Size, ALT_QSPI_BANK_SIZE);
  }
  return Status;
}

EFI_STATUS
EFIAPI
QspiUpdate (
  IN VOID*   Buffer,
  IN UINT32  Offset,
  IN UINT32  Size
  )
{
  EFI_STATUS Status = EFI_SUCCESS;;
  Status = QspiErase (Offset, Size);
  if (Status != EFI_SUCCESS) {
    return Status;
  }
  return QspiWrite (Buffer, Offset, Size);
}


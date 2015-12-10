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
#include <Library/TimerLib.h>
#include "Assert.h"
#include "DeviceTree.h"
#include "FpgaManager.h"
#include "RawBinaryFile.h"

#if (FixedPcdGet32(PcdDebugMsg_FpgaManager) == 0)
  //#define ProgressPrint(FormatString, ...)    /* do nothing */
  #define ProgressPrint                       SerialPortPrint
  #define InfoPrint(FormatString, ...)        /* do nothing */
#else
  #define ProgressPrint SerialPortPrint
  #define InfoPrint     SerialPortPrint
#endif

#define RBF_BUFFER_SIZE    4096

//
// Clock-to-data ratio (CDRATIO) is dependent on cfg width
// and whether the bitstream is encrypted and/or compressed.
//
// | width | encr. | compr. | cd ratio |
// |  16   |   0   |   0    |     1    |
// |  16   |   0   |   1    |     4    |
// |  16   |   1   |   0    |     2    |
// |  16   |   1   |   1    |     4    |
// |  32   |   0   |   0    |     1    |
// |  32   |   0   |   1    |     8    |
// |  32   |   1   |   0    |     4    |
// |  32   |   1   |   1    |     8    |
//
#define CRLUT_CFGWIDTH   2
#define CRLUT_ENCRYPTED  2
#define CRLUT_COMPRESSED 2
static const UINT32 CdRatioLookUpTable[CRLUT_CFGWIDTH][CRLUT_ENCRYPTED][CRLUT_COMPRESSED] =
{
  { // 16-bit configuration data width
    { // 0 - unencrypted
        // 0 - uncompressed
        ALT_FPGAMGR_IMGCFG_CTL_02_CDRATIO_E_X1,
        // 1 - compressed
        ALT_FPGAMGR_IMGCFG_CTL_02_CDRATIO_E_X4
    },
    { // 1 - encrypted
        // 0 - uncompressed
        ALT_FPGAMGR_IMGCFG_CTL_02_CDRATIO_E_X2,
        // 1 - compressed
        ALT_FPGAMGR_IMGCFG_CTL_02_CDRATIO_E_X4
    }
  },
  { // 32-bit configuration data width
    { // 0 - unencrypted
        // 0 - uncompressed
        ALT_FPGAMGR_IMGCFG_CTL_02_CDRATIO_E_X1,
        // 1 - compressed
        ALT_FPGAMGR_IMGCFG_CTL_02_CDRATIO_E_X8
    },
    { // 1 - encrypted
        // 0 - uncompressed
        ALT_FPGAMGR_IMGCFG_CTL_02_CDRATIO_E_X4,
        // 0 - compressed
        ALT_FPGAMGR_IMGCFG_CTL_02_CDRATIO_E_X8
    }
  }
};


VOID
EFIAPI
GetCdRatio (
  IN  UINT32    CfgWidth,
  OUT UINT32*   CdRatioPtr
  )
{
  EFI_STATUS  Status;
  UINT32      Compressed;
  UINT32      Encrypted;
  UINT32      Data32;

  #define RBF_ENC_OFST   ( 69 * 4 )
  #define RBF_ENC_LSB       2
  #define RBF_ENC_MSK       3

  #define RBF_CMP_OFST  ( 229 * 4 )
  #define RBF_CMP_LSB       1
  #define RBF_CMP_MSK       1

  Status = ReadRawBinaryFile (RBF_ENC_OFST, sizeof(Data32), &Data32);
  ASSERT_PLATFORM_INIT(!EFI_ERROR(Status));
  Encrypted   = (((Data32 >> RBF_ENC_LSB) & RBF_ENC_MSK) != 0) ? 1:0;
  InfoPrint("RbfData32[%3d]=0x%08x\tEncrypted =%d\r\n", RBF_ENC_OFST/4, Data32, Encrypted);

  Status = ReadRawBinaryFile (RBF_CMP_OFST, sizeof(Data32), &Data32);
  ASSERT_PLATFORM_INIT(!EFI_ERROR(Status));
  Compressed  = (((Data32 >> RBF_CMP_LSB) & RBF_CMP_MSK) == 0) ? 1:0;
  InfoPrint("RbfData32[%3d]=0x%08x\tCompressed=%d\r\n", RBF_CMP_OFST/4, Data32, Compressed);

  *CdRatioPtr = CdRatioLookUpTable[CfgWidth][Encrypted][Compressed];
}


BOOLEAN
EFIAPI
FpgaIsInUserMode (
  VOID
  )
{
  if (GET_F2S_USERMODE == 1)
  {
    //InfoPrint ("FPGA is in User Mode\r\n");
    return TRUE;
  } else {
    //InfoPrint ("FPGA is not in User Mode\r\n");
    return FALSE;
  }
}


EFI_STATUS
EFIAPI
WaitForUserMode (
  VOID
  )
{
  UINTN i;

  for (i = 0; i < FPGA_TIMEOUT_CNT; i++) {
    if (FpgaIsInUserMode() == TRUE)
      break;
  }
  if (i >= FPGA_TIMEOUT_CNT)
  {
    DisplayFpgaManagerInfo ();
    InfoPrint ("Timeout waiting for FPGA to enter User Mode\r\n");
    ASSERT_PLATFORM_INIT(0);
    return EFI_TIMEOUT;
  } else {
    return EFI_SUCCESS;
  }
}


EFI_STATUS
EFIAPI
WaitForNconfigAndNstatusToGoesHigh (
  VOID
  )
{
  UINT32 i;
  for (i = 0; i < FPGA_TIMEOUT_CNT; i++) {
    if (GET_F2S_NCONFIG_PIN == 1 &&
        GET_F2S_NSTATUS_PIN == 1)
      break;
  }
  if (i >= FPGA_TIMEOUT_CNT)
  {
    DisplayFpgaManagerInfo ();
    InfoPrint ("Timeout waiting for nCONFIG and nSTATUS to goes high\r\n");
    ASSERT_PLATFORM_INIT(0);
    return EFI_TIMEOUT;
  } else {
    return EFI_SUCCESS;
  }
}


EFI_STATUS
EFIAPI
WaitForF2sNstatus (
  UINT32 Value
  )
{
  UINT32 i;
  for (i = 0; i < FPGA_TIMEOUT_CNT; i++) {
    if (GET_F2S_NSTATUS_PIN == Value)
      break;
  }
  if (i >= FPGA_TIMEOUT_CNT)
  {
    InfoPrint ("Timeout waiting for nStatus pin to goes %d\r\n", Value);
    DisplayFpgaManagerInfo ();
    ASSERT_PLATFORM_INIT(0);
    return EFI_TIMEOUT;
  } else {
    return EFI_SUCCESS;
  }
}


EFI_STATUS
EFIAPI
WaitForFpgaProgrammingCompletion (
  VOID
  )
{
  // Wait and loop till you read F2S_CONDONE_PIN=1 or F2S_NSTATUS_PIN=0
  // If you exit the loop with F2S_NSTATUS_PIN = 0, config FAILED.
  // If you exit the loop with F2S_CONEDONE_PIN =1, config PASSED.
  UINT32 i;
  for (i = 0; i < FPGA_TIMEOUT_CNT; i++) {
    if (GET_F2S_CONDONE_PIN == 1)
      return EFI_SUCCESS;

    if (GET_F2S_NSTATUS_PIN == 0) {
      DisplayFpgaManagerInfo ();
      InfoPrint ("FPGA Config failed\r\n");
      ASSERT_PLATFORM_INIT(0);
      return EFI_DEVICE_ERROR;
    }
  }
  DisplayFpgaManagerInfo ();
  InfoPrint ("Timeout waiting for FPGA CONDONE=1 or nSTATUS=0\r\n");
  ProgressPrint  ("FPGA Programming failed!\r\n");
  ASSERT_PLATFORM_INIT(0);
  return EFI_TIMEOUT;
}


EFI_STATUS
EFIAPI
GeneratesDclkPulses (
  IN UINT32 NumberOfDclkPulses
  )
{
  UINT32 i;

  // Generates the specified number of DCLK pulses

  // Clear any existing done status
  if (MmioRead32(ALT_FPGAMGR_OFST +
                 ALT_FPGAMGR_DCLKSTAT_OFST))
  {
    // This is a write one to clear bit
    MmioWrite32(ALT_FPGAMGR_OFST +
                ALT_FPGAMGR_DCLKSTAT_OFST,
                ALT_FPGAMGR_DCLKSTAT_DCNTDONE_SET_MSK);
  }

  // Write the dclkcnt
  MmioWrite32(ALT_FPGAMGR_OFST +
              ALT_FPGAMGR_DCLKCNT_OFST,
              NumberOfDclkPulses);

  // Wait untill the dclkcnt done
  for (i = 0; i < FPGA_TIMEOUT_CNT; i++) {
    if (MmioRead32(ALT_FPGAMGR_OFST +
                   ALT_FPGAMGR_DCLKSTAT_OFST) ==
                   ALT_FPGAMGR_DCLKSTAT_DCNTDONE_E_DONE) {
      // Clear the dclkcnt done and then return success
      MmioWrite32(ALT_FPGAMGR_OFST +
                  ALT_FPGAMGR_DCLKSTAT_OFST,
                  ALT_FPGAMGR_DCLKSTAT_DCNTDONE_SET_MSK);
      return EFI_SUCCESS;
    }
  }

  DisplayFpgaManagerInfo ();
  InfoPrint ("Timeout waiting for DCNT DONE bit.\r\n");
  ASSERT_PLATFORM_INIT(0);
  return EFI_TIMEOUT;
}


EFI_STATUS
EFIAPI
FpgaProgramWrite (
  IN UINTN  RbfSize
  )
{
  UINTN       i;
  UINT32*     SrcDataPtr;
  UINT32*     DstDataPtr;
  UINT32      TotalLength;
  UINTN       Offset;
  UINT32      NumOfBytesToRead;
  EFI_STATUS  Status;
  UINT8       RbfData[RBF_BUFFER_SIZE];
  UINTN       Percentage;
  UINTN       BytesReadCounter;

  BytesReadCounter = 0;
  Percentage = 0;
  TotalLength = RbfSize;
  Offset = 0;
  DstDataPtr = (UINT32 *)ALT_FPGAMGRDATA_OFST;

  while (TotalLength) {
    // Reading a block of Data
    NumOfBytesToRead = MIN (TotalLength, RBF_BUFFER_SIZE);
    Status = ReadRawBinaryFile (Offset, NumOfBytesToRead, &RbfData[0]);
    if (EFI_ERROR(Status))
    {
      InfoPrint ("Error reading RBF!\r\n");
      ASSERT_PLATFORM_INIT(0);
      return Status;
    }

    // Sending block of Data to FPGA
    SrcDataPtr = (UINT32 *)(&RbfData[0]);
    for (i = 0; i < (NumOfBytesToRead/4); i++)
    {
      *DstDataPtr = *SrcDataPtr++;
    }

    // Point to next block of Data
    TotalLength -= NumOfBytesToRead;
    Offset += NumOfBytesToRead;

    // Count progress
    BytesReadCounter += NumOfBytesToRead;
    // Update progress once every 10%
    if (((BytesReadCounter * 100 / RbfSize) - Percentage) >= 10)
    {
      Percentage = (BytesReadCounter * 100 / RbfSize);
      ProgressPrint ("\r%2d%% ", Percentage);
    }
  }
  ProgressPrint ("\rDone.\r\n");
  return EFI_SUCCESS;
}

EFI_STATUS
EFIAPI
FpgaFullConfiguration (
  IN  VOID*             Fdt,
  IN  BOOT_SOURCE_TYPE  BootSourceType
  )
{
  EFI_STATUS  Status;
  UINT32      Msel;
  UINT32      CdRatio;
  UINT32      CfgWidth;
  UINT32      RbfSize;


  //
  // DEBUG NOTE:
  // You are advised to use Semihosting to debug this function,
  // because if the UART are using shared IO pins
  // debug message may not appear on UART during FPGA programming steps 7..11.
  //

  // Display FPGA Manager Info
  InfoPrint ( "FPGA Manager Info before configuration begin:\r\n");
  DisplayFpgaManagerInfo ();

  Status = OpenRawBinaryFile(Fdt, BootSourceType, &RbfSize);
  if ((Status != EFI_SUCCESS) ||
      (RbfSize < RBF_BUFFER_SIZE)) {
    InfoPrint ("Error opening RBF file\r\n");
    return EFI_BAD_BUFFER_SIZE;
  }

  //
  // Arria 10 SoC FPGA Full Configuration Flow:
  //

  // InfoPrint ("Step 1\r\n");
  //
  // Step 1:
  // Read the f2s_msel[2:0] field in the imgcfg_stat register
  // to verify that the configuration mode is the passive fast or passive slow.
  Msel = GET_MSEL;
  if ((Msel != 0) && (Msel != 1)) {
    InfoPrint ("ERROR! Incorrect MSEL=%d\r\n", Msel);
    ASSERT_PLATFORM_INIT((Msel == 0) || (Msel == 1));
    return EFI_DEVICE_ERROR;
  }

  // InfoPrint ("Step 2\r\n");
  //
  // Step 2:
  // Set the cdratio and cfgwidth bits of the imgcfg_ctrl_02 register in the FPGA manager
  // to match the characteristics of the configuration image.

  // Set configuration data width (CFGWDTH)
  // When HPS does Normal Configuration,should use 32-bit Passive Parallel Mode.
  CfgWidth = ALT_FPGAMGR_IMGCFG_CTL_02_CFGWIDTH_E_PPX32;
  MmioAndThenOr32 (ALT_FPGAMGR_OFST +
                   ALT_FPGAMGR_IMGCFG_CTL_02_OFST,
                   ALT_FPGAMGR_IMGCFG_CTL_02_CFGWIDTH_CLR_MSK,
                   ALT_FPGAMGR_IMGCFG_CTL_02_CFGWIDTH_SET(CfgWidth));

  // Set clock-to-data ratio (CDRATIO)
  GetCdRatio (CfgWidth, &CdRatio);
  MmioAndThenOr32 (ALT_FPGAMGR_OFST +
                   ALT_FPGAMGR_IMGCFG_CTL_02_OFST,
                   ALT_FPGAMGR_IMGCFG_CTL_02_CDRATIO_CLR_MSK,
                   ALT_FPGAMGR_IMGCFG_CTL_02_CDRATIO_SET(CdRatio));

  ProgressPrint ("Programming FPGA......\r\n");
  // The following InfoPrint matter only if UART is using shared IO and on Warm reset flow
  // Flush the UART FIFO, fill the UART 128-byte transmit and receive FIFO buffers
  // So that any remaining message get printed before Pin Mux get interfere during programming
  ProgressPrint ("\r\r\r\r\r\r\r\r\r\r\r\r\r\r\r\r\r\r\r\r\r\r\r\r\r\r\r\r\r\r\r\r"
                 "\r\r\r\r\r\r\r\r\r\r\r\r\r\r\r\r\r\r\r\r\r\r\r\r\r\r\r\r\r\r\r\r"
                 "\r\r\r\r\r\r\r\r\r\r\r\r\r\r\r\r\r\r\r\r\r\r\r\r\r\r\r\r\r\r\r\r"
                 "\r\r\r\r\r\r\r\r\r\r\r\r\r\r\r\r\r\r\r\r\r\r\r\r\r\r\r\r\r\r\r\r"
                 "\r\r\r\r\r\r\r\r\r\r\r\r\r\r\r\r\r\r\r\r\r\r\r\r\r\r\r\r\r\r\r\r");

  // InfoPrint ("Step 3\r\n");
  //
  // Step 3:
  // Make sure no other external devices are trying to interfere with programming
  // To begin configuration, nCONFIG and nSTATUS must be at a logic high level.
  //
  Status = WaitForNconfigAndNstatusToGoesHigh ();
  if (EFI_ERROR(Status)) return Status;

  // InfoPrint ("Step 4\r\n");
  //
  // Step 4:
  // Drives the following signal from HPS
  //
  // S2F_NCE = 1
  // S2F_PR_REQUEST = 0
  // EN_CFG_CTRL = 0
  // EN_CFG_DATA = 0
  // S2F_NCONFIG = 1
  // S2F_NSTATUS_OE = 0
  // S2F_CONDONE_OE = 0
  //

  // S2F_NCE = 1
  MmioOr32(ALT_FPGAMGR_OFST +
           ALT_FPGAMGR_IMGCFG_CTL_01_OFST,
           ALT_FPGAMGR_IMGCFG_CTL_01_S2F_NCE_SET_MSK);

  // S2F_PR_REQUEST = 0
  MmioAnd32(ALT_FPGAMGR_OFST +
            ALT_FPGAMGR_IMGCFG_CTL_01_OFST,
            ALT_FPGAMGR_IMGCFG_CTL_01_S2F_PR_REQUEST_CLR_MSK);

  // EN_CFG_CTRL = 0
  // EN_CFG_DATA = 0
  MmioAnd32(ALT_FPGAMGR_OFST +
            ALT_FPGAMGR_IMGCFG_CTL_02_OFST,
            ALT_FPGAMGR_IMGCFG_CTL_02_EN_CFG_CTL_CLR_MSK &
            ALT_FPGAMGR_IMGCFG_CTL_02_EN_CFG_DATA_CLR_MSK);

  // S2F_NCONFIG = 1
  MmioOr32(ALT_FPGAMGR_OFST +
           ALT_FPGAMGR_IMGCFG_CTL_00_OFST,
           ALT_FPGAMGR_IMGCFG_CTL_00_S2F_NCFG_SET_MSK);

  // S2F_NSTATUS_OE = 0
  // S2F_CONDONE_OE = 0
  MmioAnd32(ALT_FPGAMGR_OFST +
             ALT_FPGAMGR_IMGCFG_CTL_00_OFST,
             ALT_FPGAMGR_IMGCFG_CTL_00_S2F_NSTAT_OE_CLR_MSK &
             ALT_FPGAMGR_IMGCFG_CTL_00_S2F_CONDONE_OE_CLR_MSK);

  // InfoPrint ("Step 5\r\n");
  //
  // Step 5:
  // Enable overrides
  // S2F_NENABLE_CONFIG = 0
  // S2F_NENABLE_NCONFIG = 0
  //
  MmioAnd32(ALT_FPGAMGR_OFST +
            ALT_FPGAMGR_IMGCFG_CTL_01_OFST,
            ALT_FPGAMGR_IMGCFG_CTL_01_S2F_NEN_CFG_CLR_MSK);
  MmioAnd32(ALT_FPGAMGR_OFST +
            ALT_FPGAMGR_IMGCFG_CTL_00_OFST,
            ALT_FPGAMGR_IMGCFG_CTL_00_S2F_NEN_NCFG_CLR_MSK);
  //
  // FB:285090 - Disable driving signals that HPS doesn't need to drive.
  // S2F_NENABLE_NSTATUS = 1
  // S2F_NENABLE_CONDONE = 1
  //
  MmioOr32(ALT_FPGAMGR_OFST +
           ALT_FPGAMGR_IMGCFG_CTL_00_OFST,
           ALT_FPGAMGR_IMGCFG_CTL_00_S2F_NEN_NSTAT_SET_MSK |
           ALT_FPGAMGR_IMGCFG_CTL_00_S2F_NEN_CONDONE_SET_MSK);

  // InfoPrint ("Step 6\r\n");
  //
  // Step 6:
  // Drive chip select S2F_NCE = 0
  //
  MmioAnd32(ALT_FPGAMGR_OFST +
            ALT_FPGAMGR_IMGCFG_CTL_01_OFST,
            ALT_FPGAMGR_IMGCFG_CTL_01_S2F_NCE_CLR_MSK);

  // InfoPrint ("Step 7\r\n");
  //
  // Step 7:
  // Make sure no other external devices are trying to interfere with programming
  // To begin configuration, nCONFIG and nSTATUS must be at a logic high level.
  //
  Status = WaitForNconfigAndNstatusToGoesHigh ();
  if (EFI_ERROR(Status)) return Status;

  // InfoPrint ("Step 8\r\n");
  //
  // Step 8
  // Reset configuration
  // Write S2F_NCONFIG=0
  // Wait till you read F2S_NSTATUS_PIN=0
  // Write S2F_NCONFIG=1
  // Wait till you read F2S_NSTATUS_PIN=1 and
  // Read and confirm F2S_CONDONE_PIN=0, F2S_CONDONE_OE=1
  //
  MmioAnd32 (ALT_FPGAMGR_OFST +
             ALT_FPGAMGR_IMGCFG_CTL_00_OFST,
             ALT_FPGAMGR_IMGCFG_CTL_00_S2F_NCFG_CLR_MSK);

  Status = WaitForF2sNstatus(0);
  if (EFI_ERROR(Status)) return Status;

  MmioOr32(ALT_FPGAMGR_OFST +
           ALT_FPGAMGR_IMGCFG_CTL_00_OFST,
           ALT_FPGAMGR_IMGCFG_CTL_00_S2F_NCFG_SET_MSK);

  Status = WaitForF2sNstatus(1);
  if (EFI_ERROR(Status)) return Status;

  if (GET_F2S_CONDONE_PIN != 0)
    return EFI_DEVICE_ERROR;

  if (GET_F2S_CONDONE_OE == 0)
    return EFI_DEVICE_ERROR;

  // InfoPrint ("Step 9\r\n");
  //
  // Step 9:
  // EN_CFG_CTRL and EN_CFG_DATA = 1
  //
  MmioOr32(ALT_FPGAMGR_OFST +
           ALT_FPGAMGR_IMGCFG_CTL_02_OFST,
           ALT_FPGAMGR_IMGCFG_CTL_02_EN_CFG_DATA_SET_MSK |
           ALT_FPGAMGR_IMGCFG_CTL_02_EN_CFG_CTL_SET_MSK);

  // InfoPrint ("Step 10\r\n");
  //
  // Step 10:
  // When the f2s_nstatus_pin is high,
  // write the SOF/POF data to the img_data_w register in the FPGA Manager
  Status = WaitForNconfigAndNstatusToGoesHigh ();
  if (EFI_ERROR(Status)) return Status;

  Status = FpgaProgramWrite (RbfSize);
  if (EFI_ERROR(Status)) return Status;

  // InfoPrint ("Step 11\r\n");
  //
  // Step 11:
  // After sending bitstream, Wait for ConfigCompletion.
  //
  Status = WaitForFpgaProgrammingCompletion();
  if (EFI_ERROR(Status)) return Status;

  // InfoPrint ("Step 12\r\n");
  //
  // Step 12:
  // Write dclkcnt=0xf. Loop until dclkcnt reaches 0.
  //
  Status = GeneratesDclkPulses(0x0F);
  if (EFI_ERROR(Status)) return Status;

  // InfoPrint ("Step 13\r\n");
  //
  // Step 13:
  // Wait for Initialization Sequence Completion.
  // Loop until F2S_USERMODE=1
  //
  Status = WaitForUserMode();
  if (EFI_ERROR(Status)) return Status;

  // InfoPrint ("Step 14\r\n");
  //
  // Step 14:
  // Stop DATA path and Dclk
  // EN_CFG_CTRL and EN_CFG_DATA = 0
  //
  MmioAnd32(ALT_FPGAMGR_OFST +
            ALT_FPGAMGR_IMGCFG_CTL_02_OFST,
            ALT_FPGAMGR_IMGCFG_CTL_02_EN_CFG_DATA_CLR_MSK &
            ALT_FPGAMGR_IMGCFG_CTL_02_EN_CFG_CTL_CLR_MSK);

  // InfoPrint ("Step 15\r\n");
  //
  // Step 15:
  // Disable chip select S2F_NCE = 1
  //
  MmioOr32(ALT_FPGAMGR_OFST +
           ALT_FPGAMGR_IMGCFG_CTL_01_OFST,
           ALT_FPGAMGR_IMGCFG_CTL_01_S2F_NCE_SET_MSK);

  // InfoPrint ("Step 16\r\n");
  //
  // Step 16:
  // Disable overrides
  // S2F_NENABLE_CONFIG = 1
  // S2F_NENABLE_NCONFIG = 1
  //
  MmioOr32(ALT_FPGAMGR_OFST +
           ALT_FPGAMGR_IMGCFG_CTL_01_OFST,
           ALT_FPGAMGR_IMGCFG_CTL_01_S2F_NEN_CFG_SET_MSK);
  MmioOr32(ALT_FPGAMGR_OFST +
           ALT_FPGAMGR_IMGCFG_CTL_00_OFST,
           ALT_FPGAMGR_IMGCFG_CTL_00_S2F_NEN_NCFG_SET_MSK);

  // InfoPrint ("Step 17\r\n");
  //
  // Step 17:
  //Final check
  //
  if (GET_F2S_USERMODE == 0 ||
      GET_F2S_CONDONE_PIN == 0 ||
      GET_F2S_NSTATUS_PIN == 0)
  return EFI_DEVICE_ERROR;

  ProgressPrint ("FPGA programming Successful\r\n");
  return EFI_SUCCESS;
}


VOID
EFIAPI
EnableHpsAndFpgaBridges (
  IN  VOID*             Fdt
  )
{
  FPGA_BRIDGE_CONFIG Cfg;
  UINT32 NocMask;
  UINT32 AndMask;
  UINT32 Data32;
  UINTN  TimeCount;
  #define TIMEOUT_BRIDGES  1000000

  ProgressPrint ("Enable Hps And Fpga Bridges\r\n");

  // Get Device Tree Settings
  GetFpgaBridgeCfg (Fdt, &Cfg);

  NocMask = 0;
  AndMask = 0xFFFFFFFF;
  if (Cfg.h2f.enable == 1) {
    NocMask |= ALT_SYSMGR_NOC_IDLEREQ_CLR_H2F_SET_MSK;
    AndMask &= ALT_RSTMGR_BRGMODRST_H2F_CLR_MSK;
  }
  if (Cfg.lwh2f.enable == 1) {
    NocMask |= ALT_SYSMGR_NOC_IDLEREQ_CLR_LWH2F_SET_MSK;
    AndMask &= ALT_RSTMGR_BRGMODRST_LWH2F_CLR_MSK;
  }
  if (Cfg.f2h.enable == 1) {
    NocMask |= ALT_SYSMGR_NOC_IDLEREQ_CLR_F2H_SET_MSK;
    AndMask &= ALT_RSTMGR_BRGMODRST_F2H_CLR_MSK;
  }
  if (Cfg.f2sdr0.enable == 1) {
    NocMask |= ALT_SYSMGR_NOC_IDLEREQ_CLR_F2SDR0_SET_MSK;
    AndMask &= ALT_RSTMGR_BRGMODRST_F2SSDRAM0_CLR_MSK;
  }
  if (Cfg.f2sdr1.enable == 1) {
    NocMask |= ALT_SYSMGR_NOC_IDLEREQ_CLR_F2SDR1_SET_MSK;
    AndMask &= ALT_RSTMGR_BRGMODRST_F2SSDRAM1_CLR_MSK;
  }
  if (Cfg.f2sdr2.enable == 1) {
    NocMask |= ALT_SYSMGR_NOC_IDLEREQ_CLR_F2SDR2_SET_MSK;
    AndMask &= ALT_RSTMGR_BRGMODRST_F2SSDRAM2_CLR_MSK;
  }

  // Clear IDLE request to each NOC master.
  MmioWrite32 (ALT_SYSMGR_OFST +
               ALT_SYSMGR_NOC_IDLEREQ_CLR_OFST,
               NocMask);

  // De-assert reset of Bridges
  MmioAnd32 (ALT_RSTMGR_OFST +
             ALT_RSTMGR_BRGMODRST_OFST,
             AndMask);

  // Poll until ZERO each IDLE request bit for each NOC master.
  TimeCount = 0;
  do {
    Data32 = MmioRead32(ALT_SYSMGR_OFST + ALT_SYSMGR_NOC_IDLEACK_OFST);
    if ((Data32 & NocMask) == 0)
      break;
    MicroSecondDelay (1);
  } while (++TimeCount < TIMEOUT_BRIDGES);
  if (TimeCount >= TIMEOUT_BRIDGES) {
    ProgressPrint ("\t ERROR! Timeout waiting for IDLE request bit to become 0.\r\n");
    InfoPrint (
      "\t\t IDLEACK_H2F\t: %d\r\n"
      "\t\t IDLEACK_LWH2F\t: %d\r\n"
      "\t\t IDLEACK_F2H\t: %d\r\n"
      "\t\t IDLEACK_F2SDR0\t: %d\r\n"
      "\t\t IDLEACK_F2SDR1\t: %d\r\n"
      "\t\t IDLEACK_F2SDR2\t: %d\r\n",
      ALT_SYSMGR_NOC_IDLEACK_H2F_GET(Data32),
      ALT_SYSMGR_NOC_IDLEACK_LWH2F_GET(Data32),
      ALT_SYSMGR_NOC_IDLEACK_F2H_GET(Data32),
      ALT_SYSMGR_NOC_IDLEACK_F2SDR0_GET(Data32),
      ALT_SYSMGR_NOC_IDLEACK_F2SDR1_GET(Data32),
      ALT_SYSMGR_NOC_IDLEACK_F2SDR2_GET(Data32));
    ASSERT_PLATFORM_INIT(TimeCount < TIMEOUT_BRIDGES);
  }
}


VOID
EFIAPI
DisplayFpgaManagerInfo (
  VOID
  )
{
  UINT32          Data32;

  InfoPrint (
    "FPGA Manager dclk cnt: 0x%08x\r\n"
    "FPGA Manager dclk stat: 0x%08x\r\n"
    "FPGA Manager img cfg ctrl 00: 0x%08x\r\n"
    "FPGA Manager img cfg ctrl 01: 0x%08x\r\n"
    "FPGA Manager img cfg ctrl 02: 0x%08x\r\n"
    "FPGA Manager img cfg fifo status: 0x%08x\r\n",
    MmioRead32 (ALT_FPGAMGR_OFST + ALT_FPGAMGR_DCLKCNT_OFST),
    MmioRead32 (ALT_FPGAMGR_OFST + ALT_FPGAMGR_DCLKSTAT_OFST),
    MmioRead32 (ALT_FPGAMGR_OFST + ALT_FPGAMGR_IMGCFG_CTL_00_OFST),
    MmioRead32 (ALT_FPGAMGR_OFST + ALT_FPGAMGR_IMGCFG_CTL_01_OFST),
    MmioRead32 (ALT_FPGAMGR_OFST + ALT_FPGAMGR_IMGCFG_CTL_02_OFST),
    MmioRead32 (ALT_FPGAMGR_OFST + ALT_FPGAMGR_IMGCFG_FIFO_STAT_OFST));

  Data32 = MmioRead32 (ALT_FPGAMGR_OFST + ALT_FPGAMGR_IMGCFG_STAT_OFST);
  InfoPrint (
    "FPGA Manager imgcfg_stat: 0x%08x\r\n"
    "\temr :\t\t\t%d\r\n"
    "\tjtagm :\t\t\t%d\r\n"
    "\timgcfg_FifoFull :\t%d\r\n"
    "\timgcfg_FifoEmpty :\t%d\r\n"
    "\tf2s_msel2 :\t\t%d\r\n"
    "\tf2s_msel1 :\t\t%d\r\n"
    "\tf2s_msel0 :\t\t%d\r\n"
    "\tf2s_nceo_oe :\t\t%d\r\n"
    "\tf2s_nconfig_pin :\t%d\r\n"
    "\tf2s_pr_error :\t\t%d\r\n"
    "\tf2s_pr_done :\t\t%d\r\n"
    "\tf2s_pr_ready :\t\t%d\r\n"
    "\tf2s_cvp_conf_done :\t%d\r\n"
    "\tf2s_initdone_oe :\t%d\r\n"
    "\tf2s_condone_pin :\t%d\r\n"
    "\tf2s_nstatus_oe :\t%d\r\n"
    "\tf2s_nstatus_pin :\t%d\r\n"
    "\tf2s_initdone_oe :\t%d\r\n"
    "\tf2s_usermode :\t\t%d\r\n"
    "\tf2s_early_usermode :\t%d\r\n"
    "\tf2s_crc_error :\t\t%d\r\n",
     Data32,
     ALT_FPGAMGR_IMGCFG_STAT_EMR_GET(Data32),
     ALT_FPGAMGR_IMGCFG_STAT_JTAGM_GET(Data32),
     ALT_FPGAMGR_IMGCFG_STAT_IMGCFG_FIFOFULL_GET(Data32),
     ALT_FPGAMGR_IMGCFG_STAT_IMGCFG_FIFOEMPTY_GET(Data32),
     ALT_FPGAMGR_IMGCFG_STAT_F2S_MSEL2_GET(Data32),
     ALT_FPGAMGR_IMGCFG_STAT_F2S_MSEL1_GET(Data32),
     ALT_FPGAMGR_IMGCFG_STAT_F2S_MSEL0_GET(Data32),
     ALT_FPGAMGR_IMGCFG_STAT_F2S_NCEO_OE_GET(Data32),
     ALT_FPGAMGR_IMGCFG_STAT_F2S_NCFG_PIN_GET(Data32),
     ALT_FPGAMGR_IMGCFG_STAT_F2S_PR_ERROR_GET(Data32),
     ALT_FPGAMGR_IMGCFG_STAT_F2S_PR_DONE_GET(Data32),
     ALT_FPGAMGR_IMGCFG_STAT_F2S_PR_RDY_GET(Data32),
     ALT_FPGAMGR_IMGCFG_STAT_F2S_CVP_CONF_DONE_GET(Data32),
     ALT_FPGAMGR_IMGCFG_STAT_F2S_CONDONE_OE_GET(Data32),
     ALT_FPGAMGR_IMGCFG_STAT_F2S_CONDONE_PIN_GET(Data32),
     ALT_FPGAMGR_IMGCFG_STAT_F2S_NSTAT_OE_GET(Data32),
     ALT_FPGAMGR_IMGCFG_STAT_F2S_NSTAT_PIN_GET(Data32),
     ALT_FPGAMGR_IMGCFG_STAT_F2S_INITDONE_OE_GET(Data32),
     ALT_FPGAMGR_IMGCFG_STAT_F2S_USERMOD_GET(Data32),
     ALT_FPGAMGR_IMGCFG_STAT_F2S_EARLY_USERMOD_GET(Data32),
     ALT_FPGAMGR_IMGCFG_STAT_F2S_CRC_ERROR_GET(Data32));
}





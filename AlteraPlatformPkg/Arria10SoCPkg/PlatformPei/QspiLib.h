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

#ifndef __QSPI_LIB_H__
#define __QSPI_LIB_H__

//
// This section provisions support for various flash devices.
//


#define ALT_QSPI_PROVISION_MICRON_N25Q_SUPPORT 1

/////

#define ALT_QSPI_PAGE_ADDR_MSK          0xFFFFFF00
#define ALT_QSPI_PAGE_SIZE              0x00000100 // 256 B
#define ALT_QSPI_SUBSECTOR_ADDR_MSK     0xFFFFF000
#define ALT_QSPI_SUBSECTOR_SIZE         0x00001000 // 4096 B
#define ALT_QSPI_SECTOR_ADDR_MSK        0xFFFF0000
#define ALT_QSPI_SECTOR_SIZE            0x00010000 // 64 KiB
#define ALT_QSPI_BANK_ADDR_MSK          0xFF000000
#define ALT_QSPI_BANK_SIZE              0x01000000 // 16 MiB

#if ALT_QSPI_PROVISION_MICRON_N25Q_SUPPORT
#define ALT_QSPI_N25Q_DIE_ADDR_MSK      0xFE000000
#define ALT_QSPI_N25Q_DIE_SIZE          0x02000000 // 32 MiB
#endif

/////

// Default delay timing (in ns) for N25Q.
// These values are from the N25Q handbook. The timing correctness is difficult
// to test because the test setup does not feature mutliple chips.
#define ALT_QSPI_TSHSL_NS_DEF       (50)
#define ALT_QSPI_TSD2D_NS_DEF       (0)
#define ALT_QSPI_TCHSH_NS_DEF       (4)
#define ALT_QSPI_TSLCH_NS_DEF       (4)

/*
// Default delay timing (in ns)
#define ALT_QSPI_TSHSL_NS_DEF       (200)
#define ALT_QSPI_TSD2D_NS_DEF       (255)
#define ALT_QSPI_TCHSH_NS_DEF       (20)
#define ALT_QSPI_TSLCH_NS_DEF       (20)
*/

// Flash commands
#define ALT_QSPI_STIG_OPCODE_READ                 (0x03)
#define ALT_QSPI_STIG_OPCODE_4BYTE_READ           (0x13)
#define ALT_QSPI_STIG_OPCODE_FASTREAD             (0x0B)
#define ALT_QSPI_STIG_OPCODE_FASTREAD_DUAL_OUTPUT (0x3B)
#define ALT_QSPI_STIG_OPCODE_FASTREAD_QUAD_OUTPUT (0x6B)
#define ALT_QSPI_STIG_OPCODE_FASTREAD_DUAL_IO     (0xBB)
#define ALT_QSPI_STIG_OPCODE_FASTREAD_QUAD_IO     (0xEB)
#define ALT_QSPI_STIG_OPCODE_PP                   (0x02)
#define ALT_QSPI_STIG_OPCODE_DUAL_PP              (0xA2)
#define ALT_QSPI_STIG_OPCODE_QUAD_PP              (0x32)
#define ALT_QSPI_STIG_OPCODE_RDID                 (0x9F)
#define ALT_QSPI_STIG_OPCODE_WREN                 (0x06)
#define ALT_QSPI_STIG_OPCODE_WRDIS                (0x04)
#define ALT_QSPI_STIG_OPCODE_RDSR                 (0x05)
#define ALT_QSPI_STIG_OPCODE_WRSR                 (0x01)
#define ALT_QSPI_STIG_OPCODE_SUBSEC_ERASE         (0x20)
#define ALT_QSPI_STIG_OPCODE_SEC_ERASE            (0xD8)
#define ALT_QSPI_STIG_OPCODE_BULK_ERASE           (0xC7)
#define ALT_QSPI_STIG_OPCODE_DIE_ERASE            (0xC4)
#define ALT_QSPI_STIG_OPCODE_CHIP_ERASE           (0x60)
#define ALT_QSPI_STIG_OPCODE_RD_EXT_REG           (0xC8)
#define ALT_QSPI_STIG_OPCODE_WR_EXT_REG           (0xC5)
#define ALT_QSPI_STIG_OPCODE_RD_STAT_REG          (0x05)
#define ALT_QSPI_STIG_OPCODE_WR_STAT_REG          (0x01)
#define ALT_QSPI_STIG_OPCODE_ENTER_4BYTE_MODE     (0xB7)
#define ALT_QSPI_STIG_OPCODE_EXIT_4BYTE_MODE      (0xE9)

// Micron commands, for 512 Mib, 1 Gib (64 MiB, 128 MiB) parts.
#if ALT_QSPI_PROVISION_MICRON_N25Q_SUPPORT
#define ALT_QSPI_STIG_OPCODE_RESET_EN             (0x66)
#define ALT_QSPI_STIG_OPCODE_RESET_MEM            (0x99)
#define ALT_QSPI_STIG_OPCODE_RDFLGSR              (0x70)
#define ALT_QSPI_STIG_OPCODE_CLRFLGSR             (0x50)
#define ALT_QSPI_STIG_OPCODE_DISCVR_PARAM         (0x5A)
#endif

// Spansion commands
// #define OPCODE_ECRM                 (0xFF) // Exit continuous read mode

#define QSPI_READ_CLK_MHZ           (50)
#define QSPI_FASTREAD_CLK_MHZ       (100)

// Manufacturer ID
#define ALT_QSPI_STIG_RDID_JEDECID_MICRON      (0x20)
#define ALT_QSPI_STIG_RDID_JEDECID_NUMONYX     (0x20) // Same as Micron
#define ALT_QSPI_STIG_RDID_JEDECID_SPANSION    (0xEF)
#define ALT_QSPI_STIG_RDID_JEDECID_WINBOND     (0xEF) // Same as Spansion
#define ALT_QSPI_STIG_RDID_JEDECID_MACRONIC    (0xC2)
#define ALT_QSPI_STIG_RDID_JEDECID_ATMEL       (0x1F)

#define ALT_QSPI_STIG_RDID_JEDECID_GET(value)    ((value >>  0) & 0xff)
#define ALT_QSPI_STIG_RDID_CAPACITYID_GET(value) ((value >> 16) & 0xff)

#define ALT_QSPI_STIG_FLAGSR_ERASEPROGRAMREADY_GET(value) ((value >> 7) & 0x1)
#define ALT_QSPI_STIG_FLAGSR_ERASEREADY_GET(value)        ((value >> 7) & 0x1)
#define ALT_QSPI_STIG_FLAGSR_PROGRAMREADY_GET(value)      ((value >> 7) & 0x1)
#define ALT_QSPI_STIG_FLAGSR_ERASEERROR_GET(value)        ((value >> 5) & 0x1)
#define ALT_QSPI_STIG_FLAGSR_PROGRAMERROR_GET(value)      ((value >> 4) & 0x1)
#define ALT_QSPI_STIG_FLAGSR_ADDRESSINGMODE_GET(value)    ((value >> 1) & 0x1)
#define ALT_QSPI_STIG_FLAGSR_PROTECTIONERROR_GET(value)   ((value >> 0) & 0x1)

#define ALT_QSPI_STIG_SR_BUSY_GET(value)                 ((value >> 0) & 0x1)
#define ALT_QSPI_BANK_ADDR_GET(value)					((value) >> 24)

#define ALT_QSPI_COMMAND_TIMEOUT (0x10000)

#define ALT_QSPI_DEVSZ_4BYTE_ADDR 0x3
#define ALT_QSPI_DEVSZ_3BYTE_ADDR 0x2

#define ALT_QSPI_SRAM_FIFO_SIZE           (512)

/*
 * The size of the onboard SRAM in entries. Each entry is word (32-bit) sized.
 */
#define ALT_QSPI_SRAM_FIFO_ENTRY_COUNT    (512 / sizeof(UINT32))

typedef enum ALT_QSPI_INT_STATUS_E
{
    /*!
     * Mode fail M - indicates the voltage on pin n_ss_in is inconsistent with
     * the SPI mode. Set = 1 if n_ss_in is low in master mode (multi-master
     * contention). These conditions will clear the spi_enable bit and disable
     * the SPI.
     *  * 0 = no mode fault has been detected.
     *  * 1 = a mode fault has occurred.
     */
    ALT_QSPI_INT_STATUS_MODE_FAIL         = (0x1 << 0),

    /*!
     * Underflow Detected.
     *  * 0 = no underflow has been detected.
     *  * 1 = underflow is detected and an attempt to transfer data is made
     *        when the small TX FIFO is empty. This may occur when AHB write
     *        data is being supplied too slowly to keep up with the requested
     *        write operation.
     */
    ALT_QSPI_INT_STATUS_UFL               = (0x1 << 1),

    /*!
     * Controller has completed last triggered indirect operation.
     */
    ALT_QSPI_INT_STATUS_IDAC_OP_COMPLETE  = (0x1 << 2),

    /*!
     * Indirect operation was requested but could not be accepted. Two indirect
     * operations already in storage.
     */
    ALT_QSPI_INT_STATUS_IDAC_OP_REJECT    = (0x1 << 3),

    /*!
     * Write to protected area was attempted and rejected.
     */
    ALT_QSPI_INT_STATUS_WR_PROT_VIOL      = (0x1 << 4),

    /*!
     * Illegal AHB Access Detected. AHB write wrapping bursts and the use of
     * SPLIT/RETRY accesses will cause this interrupt to trigger.
     */
    ALT_QSPI_INT_STATUS_ILL_AHB_ACCESS    = (0x1 << 5),

    /*!
     * Indirect Transfer Watermark Level Breached.
     */
    ALT_QSPI_INT_STATUS_IDAC_WTRMK_TRIG   = (0x1 << 6),

    /*!
     * Receive Overflow. This should only occur in Legacy SPI mode.
     *
     * Set if an attempt is made to push the RX FIFO when it is full. This bit
     * is reset only by a system reset and cleared only when this register is
     * read. If a new push to the RX FIFO occurs coincident with a register read
     * this flag will remain set.
     *  * 0 = no overflow has been detected.
     *  * 1 = an overflow has occurred.
     */
    ALT_QSPI_INT_STATUS_RX_OVF            = (0x1 << 7),

    /*!
     * Small TX FIFO not full (current FIFO status). Can be ignored in non-SPI
     * legacy mode.
     *  * 0 = FIFO has >= THRESHOLD entries.
     *  * 1 = FIFO has < THRESHOLD entries.
     */
    ALT_QSPI_INT_STATUS_TX_FIFO_NOT_FULL  = (0x1 << 8),

    /*!
     * Small TX FIFO full (current FIFO status). Can be ignored in non-SPI
     * legacy mode.
     *  * 0 = FIFO is not full.
     *  * 1 = FIFO is full.
     */
    ALT_QSPI_INT_STATUS_TX_FIFO_FULL      = (0x1 << 9),

    /*!
     * Small RX FIFO not empty (current FIFO status). Can be ignored in non-SPI
     * legacy mode.
     *  * 0 = FIFO has < RX THRESHOLD entries.
     *  * 1 = FIFO has >= THRESHOLD entries.
     */
    ALT_QSPI_INT_STATUS_RX_FIFO_NOT_EMPTY = (0x1 << 10),

    /*!
     * Small RX FIFO full (current FIFO status). Can be ignored in non-SPI
     * legacy mode.
     *  * 0 = FIFO is not full.
     *  * 1 = FIFO is full.
     */
    ALT_QSPI_INT_STATUS_RX_FIFO_FULL      = (0x1 << 11),

    /*!
     * Indirect Read partition of SRAM is full and unable to immediately
     * complete indirect operation.
     */
    ALT_QSPI_INT_STATUS_IDAC_RD_FULL      = (0x1 << 12)

} ALT_QSPI_INT_STATUS_T;

typedef enum ALT_CLK_E {
  ALT_CLK_MAIN_PLL,
  ALT_CLK_PERIPHERAL_PLL,
 } ALT_CLK_T;

typedef enum ALT_QSPI_MODE_E
{
  ALT_QSPI_MODE_SINGLE = 0,     /*!< Use Standard Single SPI (SIO-SPI) mode (bits
                                 *   always transferred into the device on DQ0
                                 *   only). Supported by all SPI flash devices.
                                 */
  ALT_QSPI_MODE_DUAL   = 1,     /*!< Use Dual SPI (DIO-SPI) SPI mode where bits are
                                 *   transferred on DQ0 and DQ1.
                                 */
  ALT_QSPI_MODE_QUAD   = 2      /*!< Use Quad SPI (QIO-SPI) SPI mode where bits are
                                 *   transferred on DQ0, DQ1, DQ2, and DQ3.
                                 */
} ALT_QSPI_MODE_T;

/******************************************************************************/
/*!
 * This type enumerates the mode configurations available for driving the
 * ss_n[3:0] device chip selects.  The chip selects may be controlled as either
 * in a '1 of 4' or '4 to 16 decode' mode.
 */
typedef enum ALT_QSPI_CS_MODE_E
{
  ALT_QSPI_CS_MODE_SINGLE_SELECT = 0,   /*!< Select 1 of 4 chip select ss_n[3:0]
                                         */
  ALT_QSPI_CS_MODE_DECODE        = 1    /*!< Select external 4 to 16 decode of
                                         *   ss_n[3:0].
                                         */
} ALT_QSPI_CS_MODE_T;

/******************************************************************************/
/*!
 * This type enumerates the QSPI controller master baud rate divisor selections.
 */
typedef enum ALT_QSPI_BAUD_DIV_E
{
  ALT_QSPI_BAUD_DIV_2            = 0x0, /*!< Divide by 2 */
  ALT_QSPI_BAUD_DIV_4            = 0x1, /*!< Divide by 4 */
  ALT_QSPI_BAUD_DIV_6            = 0x2, /*!< Divide by 6 */
  ALT_QSPI_BAUD_DIV_8            = 0x3, /*!< Divide by 8 */
  ALT_QSPI_BAUD_DIV_10           = 0x4, /*!< Divide by 10 */
  ALT_QSPI_BAUD_DIV_12           = 0x5, /*!< Divide by 12 */
  ALT_QSPI_BAUD_DIV_14           = 0x6, /*!< Divide by 14 */
  ALT_QSPI_BAUD_DIV_16           = 0x7, /*!< Divide by 16 */
  ALT_QSPI_BAUD_DIV_18           = 0x8, /*!< Divide by 18 */
  ALT_QSPI_BAUD_DIV_20           = 0x9, /*!< Divide by 20 */
  ALT_QSPI_BAUD_DIV_22           = 0xA, /*!< Divide by 22 */
  ALT_QSPI_BAUD_DIV_24           = 0xB, /*!< Divide by 24 */
  ALT_QSPI_BAUD_DIV_26           = 0xC, /*!< Divide by 26 */
  ALT_QSPI_BAUD_DIV_28           = 0xD, /*!< Divide by 28 */
  ALT_QSPI_BAUD_DIV_30           = 0xE, /*!< Divide by 30 */
  ALT_QSPI_BAUD_DIV_32           = 0xF  /*!< Divide by 32 */
} ALT_QSPI_BAUD_DIV_T;

/******************************************************************************/
/*!
 * Device Size Configuration
 *
 * This type defines the structure used to specify flash device size and write
 * protect regions.
 */
typedef struct ALT_QSPI_DEV_SIZE_CONFIG_S
{
  UINT32      BlockSize;         /*!< Number of bytes per device block. The
                                     *   number is specified as a power of 2.
                                     *   That is 0 = 1 byte, 1 = 2 bytes, ...
                                     *   16 = 65535 bytes, etc.
                                     */
  UINT32      PageSize;          /*!< Number of bytes per device page.  This
                                     *   is required by the controller for
                                     *   performing flash writes up to and
                                     *   across page boundaries.
                                     */
  UINT32      AddrSize;          /*!< Number of bytes used for the flash
                                     *   address. The value is \e n + 1
                                     *   based. That is 0 = 1 byte, 1 = 2 bytes,
                                     *   2 = 3 bytes, 3 = 4 bytes.
                                     */
  UINT32      LowerWrProtBlock; /*!< The block number that defines the lower
                                     *   block in the range of blocks that is
                                     *   protected from writing. This field
                                     *   is ignored it write protection is
                                     *   disabled.
                                     */
  UINT32      UpperWrProtBlock; /*!< The block number that defines the upper
                                     *   block in the range of blocks that is
                                     *   protected from writing. This field
                                     *   is ignored it write protection is
                                     *   disabled.
                                     */
  BOOLEAN     WrProtEnable;      /*!< The write region enable value. A value
                                     *   of \b true enables write protection
                                     *   on the region specified by the
                                     *   \e lower_wrprot_block and
                                     *   \e upper_wrprot_block range.
                                     */
} ALT_QSPI_DEV_SIZE_CONFIG_T;

/******************************************************************************/
/*!
 * This type enumerates the QSPI clock phase activity options outside the SPI
 * word.
 */
typedef enum ALT_QSPI_CLK_PHASE_E
{
  ALT_QSPI_CLK_PHASE_ACTIVE     = 0,    /*!< The SPI clock is active outside the
                                         *   word
                                         */
  ALT_QSPI_CLK_PHASE_INACTIVE   = 1     /*!< The SPI clock is inactive outside the
                                         *   word
                                         */
} ALT_QSPI_CLK_PHASE_T;

/******************************************************************************/
/*!
 * This type enumerates the QSPI clock polarity options outside the SPI word.
 */
typedef enum ALT_QSPI_CLK_POLARITY_E
{
  ALT_QSPI_CLK_POLARITY_LOW     = 0,    /*!< SPI clock is quiescent low outside the
                                         *   word.
                                         */
  ALT_QSPI_CLK_POLARITY_HIGH    = 1     /*!< SPI clock is quiescent high outside the
                                         *   word.
                                         */
} ALT_QSPI_CLK_POLARITY_T;

/******************************************************************************/
/*!
 * QSPI Controller Timing Configuration
 *
 * This type defines the structure used to configure timing paramaters used by
 * the QSPI controller to communicate with a target flash device.
 *
 * All timing values are defined in cycles of the SPI master ref clock.
 */
typedef struct ALT_QSPI_TIMING_CONFIG_S
{
  ALT_QSPI_CLK_PHASE_T      ClkPhase;  /*!< Selects whether the clock is in an
                                         *   active or inactive phase outside the
                                         *   SPI word.
                                         */

  ALT_QSPI_CLK_POLARITY_T   ClkPol;    /*!< Selects whether the clock is quiescent
                                         *   low or high outside the SPI word.
                                         */

  UINT32                  CsDa;      /*!< Chip Select De-Assert. Added delay in
                                         *   master reference clocks for the length
                                         *   that the master mode chip select
                                         *   outputs are de-asserted between
                                         *   transactions. If CSDA = \e X, then the
                                         *   chip select de-assert time will be: 1
                                         *   sclk_out + 1 ref_clk + \e X ref_clks.
                                         */
  UINT32                  CsDads;    /*!< Chip Select De-Assert Different
                                         *   Slaves. Delay in master reference
                                         *   clocks between one chip select being
                                         *   de-activated and the activation of
                                         *   another. This is used to ensure a quiet
                                         *   period between the selection of two
                                         *   different slaves.  CSDADS is only
                                         *   relevant when switching between 2
                                         *   different external flash devices. If
                                         *   CSDADS = \e X, then the delay will be:
                                         *   1 sclk_out + 3 ref_clks + \e X
                                         *   ref_clks.
                                         */
  UINT32                  CsEot;     /*!< Chip Select End Of Transfer. Delay in
                                         *   master reference clocks between last
                                         *   bit of current transaction and
                                         *   de-asserting the device chip select
                                         *   (n_Ss_out). By default (when CSEOT=0),
                                         *   the chip select will be de-asserted on
                                         *   the last falling edge of sclk_out at
                                         *   the completion of the current
                                         *   transaction. If CSEOT = \e X, then chip
                                         *   selected will de-assert \e X ref_clks
                                         *   after the last falling edge of
                                         *   sclk_out.
                                         */
  UINT32                  CsSot;     /*!< Chip Select Start Of Transfer. Delay in
                                         *   master reference clocks between setting
                                         *   n_Ss_out low and first bit transfer. By
                                         *   default (CSSOT=0), chip select will be
                                         *   asserted half a SCLK period before the
                                         *   first rising edge of sclk_out. If CSSOT
                                         *   = \e X, chip select will be asserted
                                         *   half an sclk_out period before the
                                         *   first rising edge of sclk_out + \e X
                                         *   ref_clks.
                                         */

  UINT32                  RdDatacap; /*!< The additional number of read data
                                         *   capture cycles (ref_clk) that should be
                                         *   applied to the internal read data
                                         *   capture circuit.  The large
                                         *   clock-to-out delay of the flash memory
                                         *   together with trace delays as well as
                                         *   other device delays may impose a
                                         *   maximum flash clock frequency which is
                                         *   less than the flash memory device
                                         *   itself can operate at. To compensate,
                                         *   software should set this register to a
                                         *   value that guarantees robust data
                                         *   captures.
                                         */
} ALT_QSPI_TIMING_CONFIG_T;

/******************************************************************************/
/*!
 * Device Instruction Configuration
 *
 * This type defines a structure for specifying the optimal instruction set
 * configuration to use with a target flash device.
 */
typedef struct ALT_QSPI_DEV_INST_CONFIG_S
{
  UINT32              OpCode;            /*!< The read or write op code to use
                                             *   for the device transaction.
                                             */
  ALT_QSPI_MODE_T       InstType;          /*!< Instruction mode type for the
                                             *   controller to use with the
                                             *   device. The instruction type
                                             *   applies to all instructions
                                             *   (reads and writes) issued from
                                             *   either the Direct Access
                                             *   Controller or the Indirect
                                             *   Acces Controller.
                                             */
  ALT_QSPI_MODE_T       AddrXferType;     /*!< Address transfer mode type. The
                                             *   value of this field is ignored
                                             *   if the \e inst_Type data member
                                             *   is set to anything other than
                                             *   ALT_QSPI_MODE_SINGLE. In that
                                             *   case, the addr_xfer_Type
                                             *   assumes the same mode as the \e
                                             *   inst_Type.
                                             */
  ALT_QSPI_MODE_T       DataXferType;     /*!< Data transfer mode type. The
                                             *   value of this field is ignored
                                             *   if the \e inst_Type data member
                                             *   is set to anything other than
                                             *   ALT_QSPI_MODE_SINGLE. In that
                                             *   case, the data_xfer_Type
                                             *   assumes the same mode as the \e
                                             *   inst_Type.
                                             */
  UINT32              DummyCycles;       /*!< Number of dummy clock cycles
                                             *   required by device for a read
                                             *   or write instruction.
                                             */

} ALT_QSPI_DEV_INST_CONFIG_T;


EFI_STATUS
EFIAPI
QspiInit (
  VOID
  );

EFI_STATUS
EFIAPI
QspiErase (
  IN UINT32 Offset,
  IN UINT32 Size
  );

EFI_STATUS
EFIAPI
QspiWrite (
  IN VOID *Buffer,
  IN UINT32 Offset,
  IN UINT32 Size
  );

EFI_STATUS
EFIAPI
QspiRead (
  OUT VOID *Buffer,
  IN  UINT32 Offset,
  IN  UINT32 Size
  );

EFI_STATUS
EFIAPI
QspiUpdate (
  IN VOID *Buffer,
  IN UINT32 Offset,
  IN UINT32 Size
  );
#endif


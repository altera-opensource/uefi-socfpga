/** @file
  A shell application that triggers I2c controller.

  Copyright (c) 2016 - 2018, Intel Corporation. All rights reserved.<BR>
  This program and the accompanying materials
  are licensed and made available under the terms and conditions of the BSD License
  which accompanies this distribution.  The full text of the license may be found at
  http://opensource.org/licenses/bsd-license.php

  THE PROGRAM IS DISTRIBUTED UNDER THE BSD LICENSE ON AN "AS IS" BASIS,
  WITHOUT WARRANTIES OR REPRESENTATIONS OF ANY KIND, EITHER EXPRESS OR IMPLIED.

**/

#ifndef __FPGA_I2C_H__
#define __FPGA_I2C_H__

#include <AlteraPlatform.h>
//
/*!
 * This type enumerates the HPS I2C controller locations.
 */
typedef enum I2C_CTLR_e
{
    ALT_I2C_I2C0        = (int)ALT_I2C0_OFST,       /*!< I2C0 location. */
    ALT_I2C_I2C1        = (int)ALT_I2C1_OFST,       /*!< I2C1 location. */
    /* I2C{2-4] are for general purpose or as control interfaces for connectors
     * with embedded I2C channels such as SFP. */
    ALT_I2C_I2C2        = (int)ALT_I2C_EMAC0_OFST,  /*!< I2C2 location. */
    ALT_I2C_I2C3        = (int)ALT_I2C_EMAC1_OFST,  /*!< I2C3 location. */
    ALT_I2C_I2C4        = (int)ALT_I2C_EMAC2_OFST,  /*!< I2C4 location. */
    ALT_I2C_MAX
}I2C_CTLR_t;

#define DEFAULT_ADDR_LENGTH 2

#endif  /* __FPGA_I2C_H__ */
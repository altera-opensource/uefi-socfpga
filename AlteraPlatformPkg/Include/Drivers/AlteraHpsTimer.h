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

#ifndef _ALTERA_HPS_TIMER_H__
#define _ALTERA_HPS_TIMER_H__

// Registers Definition for an instances of the Synopsys(R) DesignWare(R) APB Timers
#define TIMER1LOADCOUNT                 0x00     // RW  Timer1 Load Count Register
#define TIMER1CURRENTVAL                0x04     // RO  Timer1 Current Value Register
#define TIMER1CONTROLREG                0x08     // RW  Timer1 Control Register
#define TIMER1EOI                       0x0C     // RO  Timer1 End-of-Interrupt Register
#define TIMER1INTSTAT                   0x10     // RO  Timer1 Interrupt Status Register

// timer1controlreg Fields
#define TIMER1_ENABLE                   BIT0
#define TIMER1_DISABLED                          0x00
#define TIMER1_ENABLED                           BIT0

#define TIMER1_MODE                     BIT1
#define TIMER1_MODE_FREE_RUNNING                 0x00      
#define TIMER1_MODE_USER_DEFINED_COUNT_MODE      BIT1

#define TIMER1_INTERRUPT_MASK           BIT2
#define TIMER1_INTERRUPT_NOT_MASKED              0x00
#define TIMER1_INTERRUPT_MASKED                  BIT2

// timer1intstat Fields
#define TIMER1INTSTAT_INT_ACTIVE        BIT0
#define TIMER1INTSTAT_INT_IS_NOT_ACTIVE          0x00
#define TIMER1INTSTAT_INT_IS_ACTIVE              BIT0

#define TIMERSINTSTAT                   0xA0     // RO  Timers Interrupt Status Register
#define TIMERSEOI                       0xA4     // RO  Timers End-of-Interrupt Register
#define TIMERSRAWINTSTAT                0xA8     // RO  Timers Raw Interrupt Status Register
#define TIMERSCOMPVERSION               0xAC     // RO  Timers Component Version Register

#define MAX_TICKS_FOR_32BIT_TIMER                0xFFFFFFFF

#endif

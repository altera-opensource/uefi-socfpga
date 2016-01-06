/******************************************************************************
*
* Copyright 2013-2014 Altera Corporation. All Rights Reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* 1. Redistributions of source code must retain the above copyright notice,
* this list of conditions and the following disclaimer.
*
* 2. Redistributions in binary form must reproduce the above copyright notice,
* this list of conditions and the following disclaimer in the documentation
* and/or other materials provided with the distribution.
*
* 3. Neither the name of the copyright holder nor the names of its contributors
* may be used to endorse or promote products derived from this software without
* specific prior written permission.
* 
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
* ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
* LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
* CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
* SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
* INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
* CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
* ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
*
******************************************************************************/

#if !defined(ALT_PRINTF_H)
#define ALT_PRINTF_H
#include <stdio.h>
#include <stdarg.h>
#include <inttypes.h>

#if !defined(NULL)
#define NULL    ((void*)0)
#endif

extern FILE *term0;
extern FILE *term1;

static __inline int null_printf(const char *format, ...){return 0;}
static __inline int null_vfprintf(FILE *stream, const char *format, va_list args){return 0;}

#if defined (PRINTF_HOST) 
  #define ALT_PRINTF printf
  int snprintf(char *to, size_t n, const char *format, ...);
#else 
  int alt_snprintf(char *to, size_t n, const char *format, ...);
  #define snprintf alt_snprintf
  #define ALT_SNPRINTF alt_snprintf

  int alt_sprintf(char *to, const char *format, ...);
  #define sprintf alt_sprintf
  #define ALT_SPRINTF alt_sprintf

  #if defined (PRINTF_UART)
    int alt_printf(const char *format, ...);
    int alt_fprintf(FILE *stream, const char *format, ...);
    int alt_vfprintf(FILE *stream, const char *format, va_list args);

    #define ALT_PRINTF alt_printf
    #define printf alt_printf
  #else
    #define ALT_PRINTF null_printf
    #define printf null_printf
    #define vprintf null_vfprintf
  #endif /* PRINTF_UART */
#endif /* PRINTF_HOST */

typedef struct _FILEOP
{
  void (*print_function)(struct _FILEOP *context, char toprint);
} FILEOP;

void alt_log_done(FILE *op);

#endif /* ALT_PRINTF_H */

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

#ifdef __cplusplus
#include <cstddef>
#include <cstdbool>
#include <cstdint>
#else   /* __cplusplus */
#include <stddef.h>
#include <stdbool.h>
#include <stdint.h>
#endif  /* __cplusplus */

#include <stdarg.h>
#include "alt_printf.h"

#ifdef soc_a10
  #define DEFAULT_TERM  term1
#else
  #define DEFAULT_TERM  term0
#endif

#define USE_DIVIDE
#define PAD_RIGHT       1
#define PAD_ZERO        2
#define PAD_0x          4
#define PAD_PLUS        8

void null_print_function(FILEOP *context, char toprint)
{}

/* If you are printing to uart then you need to redefine these elsewhere */
static FILEOP term0_st = {null_print_function};
static FILEOP term1_st = {null_print_function};

__attribute__((weak)) FILE *term0 = (FILE *) &term0_st;
__attribute__((weak)) FILE *term1 = (FILE *) &term1_st;

static int prints(FILEOP *stream, const char *string, int width, int pad);
static int printi(FILEOP *p_op, int64_t i, int b, int sg, int width, int pad, int letbase);
int alt_vfprintf(FILE *stream, const char *format, va_list args);

int puts(const char *str)
{
    return prints((FILEOP *)DEFAULT_TERM, str, 0, 0);
}

size_t fwrite(const void *ptr, size_t size, size_t count, FILE *stream)
{
    size_t a;
    FILEOP *p_op = (FILEOP *)stream;
    for(a=0; a < size*count; a++)
    	p_op->print_function(p_op, ((char *) ptr)[a]);
    return a;
}


/** @Function Description:  Writes to the standard output (STDOUT) a sequence of 
  *                         data format argument specifies. Supported format are:
  *                         %x, %s, %c, %%, %X, %d or %i and %u.
  * @API Type:              External
  * @param str              String that contains the text to be written to STDOUT.
  * @return                 None 
  */
typedef struct _sprintf_context
{
  FILEOP  fop;
  char    *location;
  size_t  numchars;
} sprintf_context;

void sprintf_char(struct _FILEOP *context, char toprint)
{
  sprintf_context *spcont = (sprintf_context *) context;
  if(spcont->numchars)
  {
    *spcont->location = toprint;
    spcont->location++;
    spcont->numchars--;
  }
}

int alt_snprintf(char *to, size_t n, const char *format, ...)
{
    sprintf_context cont;
    int retval;
    va_list args;
    va_start( args, format );
    cont.location = to;
    cont.numchars = n;
    cont.fop.print_function = sprintf_char;
    retval = alt_vfprintf((FILE *) &cont, format, args); 
    va_end(args);
    return retval;
}
int alt_sprintf(char *to, const char *format, ...)
{
    int retval;
    va_list args;
    sprintf_context cont;
    va_start( args, format );
    cont.fop.print_function = sprintf_char;
    cont.location = to;
    cont.numchars = 0xFFFFFFFF;
    retval = alt_vfprintf((FILE *) &cont, format, args); 
    va_end(args);
    return retval;
}

int alt_printf(const char *format, ...)
{
    int retval;
    va_list args;
    va_start( args, format );
    retval = alt_vfprintf(DEFAULT_TERM, format, args);
    va_end(args);
    return retval;
}

int fprintf(FILE *stream, const char *format, ...)
{
    int retval;
    va_list args;
    va_start( args, format );
    retval = alt_vfprintf(stream, format, args);
    va_end(args);
    return retval;
}

int alt_vfprintf(FILE *stream, const char *format, va_list args)
{
    int     width, pad;
    char    scr[3];
    int     printed = 0;
    int     bLong;
    FILEOP *p_op = (FILEOP *)stream;

    for (; *format != 0; ++format)
    {
        bLong = 0;
        if (*format == '%')
        {
            ++format;
            width = pad = 0;
            if( *format == '\0') break;
            if( *format == '%') goto out;
            if( *format == '-')
            {
                ++format;
                pad = PAD_RIGHT;
            }
            if( *format == '.')
            {
                ++format;
                pad |= PAD_ZERO;
            }/* proceed with 0's */
            if( *format == '*')
            {
                ++format;
                width = va_arg( args, int );
            }/* width is a paramter */
            if( *format == '#')
            {
                ++format;
                pad |= PAD_0x;
            }/* preceed with 0x */
            if( *format == '+')
            {
                ++format;
                pad |= PAD_PLUS;
            }/* proceed with + if positive */

            while( *format == '0')
            {
                ++format;
                pad |= PAD_ZERO;
            }

            for ( ; *format >= '0' && *format <= '9'; ++format)
            {
                width *= 10;
                width += *format - '0';
            }

            if( *format == 's' )
            {
                char *s = (char *)va_arg( args, void * );
                printed += prints(p_op, s ? s:"(null)", width, pad);
                continue;
            }
            while( *format == 'l')
            {
                format++;
                bLong++;
            }/* proceed to 'd','i', or 'x' */
            if( *format == 'd' || *format == 'i' )
            {
                if(bLong == 2)
                  printed += printi(p_op, va_arg( args, uint64_t ), 10, 1, width, pad, 'a');
                else
                  printed += printi(p_op, va_arg( args, int ), 10, 1, width, pad, 'a');
                continue;
            }
            if( *format == 'p' )
            {
                pad |= PAD_0x;
                printed += printi(p_op, va_arg( args, int ), 16, 0, width, pad, 'a');
            }
            if( *format == 'x' )
            {
                if(bLong == 2)
                  printed += printi(p_op, va_arg( args, uint64_t ), 16, 0, width, pad, 'a');
                else
                  printed += printi(p_op, va_arg( args, int ), 16, 0, width, pad, 'a');
                continue;
            }

            if( *format == 'X' )
            {
                if(bLong == 2)
                  printed += printi(p_op, va_arg( args, uint64_t ), 16, 0, width, pad, 'A');
                else
                  printed += printi(p_op, va_arg( args, int ), 16, 0, width, pad, 'A');
                continue;
            }

            if( *format == 'u' )
            {
                if(bLong == 2)
                  printed += printi(p_op, va_arg( args, uint64_t ), 10, 0, width, pad, 'a');
                else
                  printed += printi(p_op, va_arg( args, int ), 10, 0, width, pad, 'a');
                continue;
            }

            if( *format == 'c' )
            {
                /* char are converted to int then pushed on the stack */
                scr[0] = (char)va_arg( args, int );
                scr[1] = '\0';
                printed += prints(p_op, scr, width, pad);
                continue;
            }
        }
        else
        {
            out:
    	    p_op->print_function(p_op, *format);
            printed ++;
        }
    }

    return printed;
}


static int prints(FILEOP *p_op, const char *string, int width, int pad)
{
    int padchar = ' ';
    int printed = 0;

    if (width > 0)
    {
        int len = 0;
        const char *ptr;
        
        for (ptr = string; *ptr; ++ptr) 
            ++len;
        
        if (len >= width) 
            width = 0;
        else 
            width -= len;
        
        if (pad & PAD_ZERO) 
            padchar = '0';
    }

    if (!(pad & PAD_RIGHT))
    {
        for ( ; width > 0; --width)
        {
    	    p_op->print_function(p_op, padchar);
            printed++;
        }
    }

    for ( ; *string ; ++string)
    {
    	p_op->print_function(p_op, *string);
        printed++;
    }

    for ( ; width > 0; --width)
    {
    	p_op->print_function(p_op, padchar);
        printed++;
    }

    return printed;
}

/*
        p_op:    structure that indicates where the new characters will go
        i:       value to print
        b:       base (10 or 16 usually)
        sg:      signed (1) or unsigned (0)
        width:
        pad:     Set of flags for padding:
                pad with 0's or spaces (1) or no (0)
                  PAD_RIGHT - add spaces
                  PAD_ZERO - add 0's
                  PAD_0x - prepend with 0x
                  PAD_PLUS - add + to positive numbers
        letbase: letter base ('a' or 'A') for hex
*/


/* the following should be enough for 64 bit int */
#define PRINT_BUF_LEN 22

static int printi(FILEOP *p_op, int64_t i, int b, int sg, int width, int pad, int letbase)
{
    char           print_buf[PRINT_BUF_LEN];
    char*          s;
    int            t, neg = 0;
    uint64_t       u = i;
    int            printed=0;
    char           scr[3];

    if(pad & PAD_0x)
    {
        scr[0] = '0';
        scr[1] = 'x';
        scr[2] = 0;
        printed += prints(p_op, scr, width, pad);
    }
    if (i == 0)
    {
        print_buf[0] = '0';
        print_buf[1] = '\0';
        return prints(p_op, print_buf, width, pad);
    }

    if (sg && b == 10 && i < 0)
    {
        neg = 1;
        u = -i;
    }

    s  = print_buf + PRINT_BUF_LEN-1;
    *s = '\0';

#ifdef USE_DIVIDE
    while (u)
    {
        t = u % b;
        if( t >= 10 )
            t += letbase - '0' - 10;
        *--s = t + '0';
        u /= b;
    }
#else
    if(b == 16)
    {
	while(u)
        {
            int count=0;
            t = u & 0xF;
            if( t >= 10 )
                t += letbase - '0' - 10;
            *--s = t + '0';
            u >>= 4;
        }
    }
    else
    {
	unsigned uint_64 int_tables[19][9] = 
	{
          {  9000000000000000000ul, 8000000000000000000ul, 7000000000000000000ul, 
             6000000000000000000ul, 5000000000000000000ul, 4000000000000000000ul, 
             3000000000000000000ul, 2000000000000000000ul, 1000000000000000000ul}, 
          {  900000000000000000ul, 800000000000000000ul, 700000000000000000ul, 
             600000000000000000ul, 500000000000000000ul, 400000000000000000ul, 
             300000000000000000ul, 200000000000000000ul, 100000000000000000ul}, 
          {  90000000000000000ul, 80000000000000000ul, 70000000000000000ul, 
             60000000000000000ul, 50000000000000000ul, 40000000000000000ul, 
             30000000000000000ul, 20000000000000000ul, 10000000000000000ul}, 
          {  9000000000000000ul, 8000000000000000ul, 7000000000000000ul, 
             6000000000000000ul, 5000000000000000ul, 4000000000000000ul, 
             3000000000000000ul, 2000000000000000ul, 1000000000000000ul}, 
          {  900000000000000ul, 800000000000000ul, 700000000000000ul, 
             600000000000000ul, 500000000000000ul, 400000000000000ul, 
             300000000000000ul, 200000000000000ul, 100000000000000ul}, 
          {  90000000000000ul, 80000000000000ul, 70000000000000ul, 
             60000000000000ul, 50000000000000ul, 40000000000000ul, 
             30000000000000ul, 20000000000000ul, 10000000000000ul}, 
          {  9000000000000ul, 8000000000000ul, 7000000000000ul, 
             6000000000000ul, 5000000000000ul, 4000000000000ul,
             3000000000000ul, 2000000000000ul, 1000000000000ul},
          {  900000000000ul, 800000000000ul, 700000000000ul, 600000000000ul,
             500000000000ul, 400000000000ul, 300000000000ul, 200000000000ul,
             100000000000ul}, 
          {  90000000000ul, 80000000000ul, 70000000000ul, 60000000000ul,
             50000000000ul, 40000000000ul, 30000000000ul, 20000000000ul,
             10000000000ul}, 
          {  9000000000u, 8000000000u, 7000000000, 6000000000, 5000000000, 
             4000000000, 3000000000, 2000000000, 1000000000}, 
          {  900000000, 800000000, 700000000, 600000000, 
             500000000, 400000000, 300000000, 200000000, 100000000},
          {  90000000, 80000000, 70000000, 60000000, 
             50000000, 40000000, 30000000, 20000000, 10000000},
          {  9000000, 8000000, 7000000, 6000000, 
             5000000, 4000000, 3000000, 2000000, 1000000},
          {  900000, 800000, 700000, 600000, 
             500000, 400000, 300000, 200000, 100000},
          {90000, 80000, 70000, 60000, 50000, 40000, 30000, 20000, 10000},
          {9000, 8000, 7000, 6000, 5000, 4000, 3000, 2000, 1000},
          {900, 800, 700, 600, 500, 400, 300, 200, 100},
          {90, 80, 70, 60, 50, 40, 30, 20, 10},
          {9, 8, 7, 6, 5, 4, 3, 2, 1}
        };

        int count;
        int offset;
        int hasprinted=0;

        s = &print_buf[0];
        for(offset = 0; offset < 19; offset++)
        {
            t = 0;
            for(count=0; count < 9; count++)
            {
                if(int_tables[offset][count] <= u)
                {
                    u -= int_tables[offset][count];
                    t = 9-count;
                    if(offset == 0)
                        t = 4-count;
                }
            }
            if(hasprinted || t != 0)
            {
                *(++s) = t + '0';
                hasprinted = 1;
            }
        }
        *++s = '\0';
        s = &print_buf[1];
    }
#endif
    if (pad & PAD_PLUS)
    {
        printed++;
        p_op->print_function(p_op, '+');
        --width;
    }
    if (neg)
    {
        if( width && (pad & PAD_ZERO) )
        {
            printed++;
            p_op->print_function(p_op, '-');
            --width;
        }
        else
        {
            *--s = '-';
        }
    }

    return printed + prints(p_op, s, width, pad);
}

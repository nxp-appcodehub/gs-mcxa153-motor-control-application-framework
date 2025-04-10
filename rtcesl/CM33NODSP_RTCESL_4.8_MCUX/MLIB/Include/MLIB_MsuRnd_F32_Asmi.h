/*******************************************************************************
*
* Copyright (c) 2013 - 2016, Freescale Semiconductor, Inc.
* Copyright 2016-2021, 2024 NXP
*
* NXP Proprietary. This software is owned or controlled by NXP and may
* only be used strictly in accordance with the applicable license terms. 
* By expressly accepting such terms or by downloading, installing,
* activating and/or otherwise using the software, you are agreeing that
* you have read, and that you agree to comply with and are bound by,
* such license terms.  If you do not agree to be bound by the applicable
* license terms, then you may not retain, install, activate or otherwise
* use the software.
* 
*
****************************************************************************//*!
*
* @brief  Multiply subtract with rounding
* 
*******************************************************************************/
#ifndef _MLIB_MSURND_F32_ASM_H_
#define _MLIB_MSURND_F32_ASM_H_

#if defined(__cplusplus)
extern "C" {
#endif

/*******************************************************************************
* Includes
*******************************************************************************/
#include "mlib_types.h"

/*******************************************************************************
* Macros
*******************************************************************************/ 
#define MLIB_MsuRnd_F32_Asmi( f32Accum, f32Mult1, f32Mult2)                    \
        MLIB_MsuRnd_F32_FAsmi( f32Accum, f32Mult1, f32Mult2)                   
#define MLIB_MsuRndSat_F32_Asm( f32Accum, f32Mult1, f32Mult2)                  \
        MLIB_MsuRndSat_F32_FAsm( f32Accum, f32Mult1, f32Mult2)                 
#define MLIB_MsuRndSat_F32lls_Asmi( f32Accum, f32Mult1, f16Mult2)              \
        MLIB_MsuRndSat_F32lls_FAsmi( f32Accum, f32Mult1, f16Mult2)
  
/*******************************************************************************
* Exported function prototypes
*******************************************************************************/
extern frac32_t MLIB_MsuRndSat_F32_FAsm(register frac32_t f32Accum,
                                        register frac32_t f32Mult1,register frac32_t f32Mult2);

/***************************************************************************//*!
*
* f32Out = f32Accum - ( f32Mult1 * f32Mult2)
* Without saturation
*******************************************************************************/  
#if defined(__IAR_SYSTEMS_ICC__)           /* IAR compiler */
#pragma diag_suppress=Pe549  /* Suppresses the Pe549 warning for IAR compiler*/
#endif
/* inline function without any optimization (compilation issue) */ 
RTCESL_INLINE_OPTIM_SAVE
RTCESL_INLINE_OPTIM_SET
static inline frac32_t MLIB_MsuRnd_F32_FAsmi(register frac32_t f32Accum,
                                             register frac32_t f32Mult1,register frac32_t f32Mult2)
{
    register frac32_t f32Val1=0, f32Val2=0, f32Val3=0;
    #if defined(__CC_ARM)                                   /* For ARM Compiler */
        __asm volatile{ uxth f32Val1, f32Mult1              /* f32Val1 = f32Mult1.L */
                        uxth f32Val2, f32Mult2              /* f32Val2 = f32Mult2.L */

                        asrs f32Mult1, f32Mult1, #16        /* f32Mult1 = f32Mult1.H */
                        asrs f32Mult2, f32Mult2, #16        /* f32Mult2 = f32Mult2.H */

                        movs f32Val3, f32Val1               /* f32Val3 = f32Mult1.L */
                        muls f32Val3, f32Val3, f32Val2      /* f32Val3 = f32Mult1.L * f32Mult2.L */
                        lsrs f32Val3, f32Val3, #16          /* f32Val3 >> 16 */

                        muls f32Val1, f32Val1, f32Mult2     /* f32Val1 = f32Mult1.L * f32Mult2.H */
                        adds f32Val1, f32Val1, f32Val3      /* f32Val1 = f32Val1 + f32Val3 */
                        asrs f32Val1, f32Val1, #1           /* f32Val1 >> 1 */

                        muls f32Val2, f32Val2, f32Mult1     /* f32Val2 = f32Mult2.L * f32Mult1.H */
                        asrs f32Val2, f32Val2, #1           /* f32Val2 >> 1 */
                        adds f32Val2, f32Val2, f32Val1      /* f32Val2 = f32Val2 + f32Val1 */
                        asrs f32Val2, f32Val2, #13          /* f32Val2 >> 13 */
                        adds f32Val2, f32Val2, #1           /* Rounding */
                        asrs f32Val2, f32Val2, #1           /* f32Val2 >> 1 */

                        muls f32Mult1, f32Mult1, f32Mult2   /* f32Mult1 = f32Mult1.H * f32Mult2.H */
                        lsls f32Mult1, f32Mult1, #1         /* f32Mult1 << 1 */
                        adds f32Mult1, f32Mult1, f32Val2    /* f32Mult1 = f32Mult1 + f32Val2 */
                        subs f32Accum, f32Accum, f32Mult1 };/* f32Accum - f16Mult1 * f16Mult2 */
    #elif defined(__GNUC__) && defined(__ARMCC_VERSION) 
        __asm volatile(
                        "uxth %3, %1 \n\t"                  /* f32Val1 = f32Mult1.L */
                        "uxth %4, %2 \n\t"                  /* f32Val2 = f32Mult2.L */

                        "asrs %1, %1, #16 \n\t"             /* f32Mult1 = f32Mult1.H */
                        "asrs %2, %2, #16 \n\t"             /* f32Mult2 = f32Mult2.H */

                        "movs %5, %3 \n\t"                  /* f32Val3 = f32Mult1.L */
                        "muls %5, %5, %4 \n\t"              /* f32Val3 = f32Mult1.L * f32Mult2.L */
                        "lsrs %5, %5, #16 \n\t"             /* f32Val3 >> 16 */

                        "muls %3, %3, %2 \n\t"              /* f32Val1 = f32Mult1.L * f32Mult2.H */
                        "adds %3, %3, %5 \n\t"              /* f32Val1 = f32Val1 + f32Val3 */
                        "asrs %3, %3, #1 \n\t"              /* f32Val1 >> 1 */

                        "muls %4, %4, %1 \n\t"              /* f32Val2 = f32Mult2.L * f32Mult1.H */
                        "asrs %4, %4, #1 \n\t"              /* f32Val2 >> 1 */
                        "adds %4, %4, %3 \n\t"              /* f32Val2 = f32Val2 + f32Val1 */
                        "asrs %4, %4, #13 \n\t"             /* f32Val2 >> 13 */
                        "adds %4, %4, #1 \n\t"              /* Rounding */
                        "asrs %4, %4, #1 \n\t"              /* f32Val2 >> 1 */

                        "muls %1, %1, %2 \n\t"              /* f32Mult1 = f32Mult1.H * f32Mult2.H */
                        "lsls %1, %1, #1 \n\t"              /* f32Mult1 << 1 */
                        "adds %1, %1, %4 \n\t"              /* f32Mult1 = f32Mult1 + f32Val2 */
                        "subs %0, %0, %1 \n\t"              /* f32Accum - f16Mult1 * f16Mult2 */
                        : "+l"(f32Accum), "+l"(f32Mult1), "+l"(f32Mult2), "+l"(f32Val1), "+l"(f32Val2), "+l"(f32Val3):);
    #else
        __asm volatile(
                        #if defined(__GNUC__)               /* For GCC compiler */
                            ".syntax unified \n"            /* Using unified asm syntax */
                        #endif
                        "uxth %3, %1 \n"                    /* f32Val1 = f32Mult1.L */
                        "uxth %4, %2 \n"                    /* f32Val2 = f32Mult2.L */

                        "asrs %1, %1, #16 \n"               /* f32Mult1 = f32Mult1.H */
                        "asrs %2, %2, #16 \n"               /* f32Mult2 = f32Mult2.H */

                        "movs %5, %3 \n"                    /* f32Val3 = f32Mult1.L */
                        "muls %5, %5, %4 \n"                /* f32Val3 = f32Mult1.L * f32Mult2.L */
                        "lsrs %5, %5, #16 \n"               /* f32Val3 >> 16 */

                        "muls %3, %3, %2 \n"                /* f32Val1 = f32Mult1.L * f32Mult2.H */
                        "adds %3, %3, %5 \n"                /* f32Val1 = f32Val1 + f32Val3 */
                        "asrs %3, %3, #1 \n"                /* f32Val1 >> 1 */

                        "muls %4, %4, %1 \n"                /* f32Val2 = f32Mult2.L * f32Mult1.H */
                        "asrs %4, %4, #1 \n"                /* f32Val2 >> 1 */
                        "adds %4, %4, %3 \n"                /* f32Val2 = f32Val2 + f32Val1 */
                        "asrs %4, %4, #13 \n"               /* f32Val2 >> 13 */
                        "adds %4, %4, #1 \n"                /* Rounding */
                        "asrs %4, %4, #1 \n"                /* f32Val2 >> 1 */

                        "muls %1, %1, %2 \n"                /* f32Mult1 = f32Mult1.H * f32Mult2.H */
                        "lsls %1, %1, #1 \n"                /* f32Mult1 << 1 */
                        "adds %1, %1, %4 \n"                /* f32Mult1 = f32Mult1 + f32Val2 */
                        "subs %0, %0, %1 \n"                /* f32Accum - f16Mult1 * f16Mult2 */
                        #if defined(__GNUC__)               /* For GCC compiler */
                            ".syntax divided \n"
                        #endif
                        : "+l"(f32Accum), "+l"(f32Mult1), "+l"(f32Mult2), "+l"(f32Val1), "+l"(f32Val2), "+l"(f32Val3):);
    #endif

    return f32Accum;
}
/* inline function without any optimization (compilation issue) */ 
RTCESL_INLINE_OPTIM_RESTORE 

#if defined(__IAR_SYSTEMS_ICC__)           /* IAR compiler */
#pragma diag_default=Pe549
#endif

/***************************************************************************//*!
*
* f32Out = f32Accum - ( f32Mult1 * f16Mult2)
* With saturation
*******************************************************************************/
#if defined(__IAR_SYSTEMS_ICC__)           /* IAR compiler */
#pragma diag_suppress=Pe549                /* Suppresses the Pe549 warning for IAR compiler*/
#endif
/* inline function without any optimization (compilation issue) */ 
RTCESL_INLINE_OPTIM_SAVE
RTCESL_INLINE_OPTIM_SET
static inline frac32_t MLIB_MsuRndSat_F32lls_FAsmi(register frac32_t f32Accum,
                                                   register frac32_t f32Mult1,register frac16_t f16Mult2)
{
    register frac32_t f32Val1=0, f32Val2=0;
    #if defined(__CC_ARM)                                       /* For ARM Compiler */
        __asm volatile{ sxth f16Mult2, f16Mult2                 /* Converts 16-bit input to 32-bit */
                        rsbs f16Mult2, f16Mult2, #0             /* f16Mult2 = - f16Mult2 */
                        uxth f32Val2, f32Mult1                  /* f32Val2 = f32Mult1.L */
                        asrs f32Mult1, f32Mult1, #16            /* f32Mult1 = f32Mult1.H */
    
                        muls f32Val2, f32Val2, f16Mult2         /* f32Val2 = f32Mult1.L * f16Mult2 */
                        asrs f32Val2, f32Val2, #7               /* f32Val2 >> 7 */
                        adds f32Val2, f32Val2, #128             /* Rounding */
                        asrs f32Val2, f32Val2, #8               /* f32Val2 >> 8 */
    
                        movs f32Val1, #0                        /* f32Val1 = 0 */
                        asrs f32Val2, f32Val2, #1               /* f32Val2 >> 1 */
                        adcs f32Val1, f32Val1, f32Val1          /* Stores the last bit of multiplication to f32Val1 */
                        muls f32Mult1, f32Mult1, f16Mult2       /* f32Mult1 = f32Mult1.H * f16Mult2 */
                        adds f32Mult1, f32Mult1, f32Val2        /* f32Mult1 = f32Mult1 + f32Val2 */
    
                        movs f32Val2, #0                        /* f32Val1 = 0 */
                        asrs f32Accum, f32Accum, #1             /* f32Accum >> 1 */
                        adcs f32Val2, f32Val2, f32Val2          /* Stores the last bit of f32Accum to f32Val2 */
    
                        adds f32Val1, f32Val1, f32Val2          /* f32Val1 = f32Val1 + f32Val2 */
                        lsls f32Val1, f32Val1, #31              /* f32Val1 << 31 (Carry = the first bit of f32Val1) */
                        adcs f32Accum, f32Accum, f32Mult1       /* (result / 2) + Carry */
    
                        mov f32Mult1, f32Accum                  /* f32Mult1 = result / 2 */
                        lsls f32Val2, f32Accum, #1              /* f32Val2 = result */
                        lsrs f32Val1, f32Val1, #31              /* f32Val1 >> 31 (the last bit of result) */
                        adds f32Val2, f32Val2, f32Val1          /* f32Val2 = result + last bit */
    
                        eors f32Accum, f32Accum, f32Val2        /* f32Accum = f32Accum ^ f32Val2 */
                        bpl SatEnd                              /* If f32Accum >= 0, then goes to SatEnd */
                        movs f32Val2, #128                      /* f32Val2 = 0x80 */
                        lsls f32Val2, f32Val2, #24              /* result = 0x80000000 */
                        cmp f32Mult1, #0                        /* Compares input value with 0 */
                        blt SatEnd                              /* If f32Mult1 < 0, then goes to SatEnd */
                        subs f32Val2, f32Val2, #1               /* result = 0x7FFFFFFF*/
                    SatEnd:    
                        movs f32Accum, f32Val2 };               /* f32Accum = result*/
    #elif defined(__GNUC__) && defined(__ARMCC_VERSION) 
        __asm volatile(
                        "sxth %2, %2 \n\t"                      /* Converts 16-bit input to 32-bit */
                        "rsbs %2, %2, #0 \n\t"                  /* f16Mult2 = - f16Mult2 */
                        "uxth %4, %1 \n\t"                      /* f32Val2 = f32Mult1.L */
                        "asrs %1, %1, #16 \n\t"                 /* f32Mult1 = f32Mult1.H */
   
                        "muls %4, %4, %2 \n\t"                  /* f32Val2 = f32Mult1.L * f16Mult2 */
                        "asrs %4, %4, #7 \n\t"                  /* f32Val2 >> 7 */
                        "adds %4, %4, #128 \n\t"                /* Rounding */
                        "asrs %4, %4, #8 \n\t"                  /* f32Val2 >> 8 */
   
                        "movs %3, #0 \n\t"                      /* f32Val1 = 0 */
                        "asrs %4, %4, #1 \n\t"                  /* f32Val2 >> 1 */
                        "adcs %3, %3, %3 \n\t"                  /* Stores the last bit of multiplication to f32Val1 */
                        "muls %1, %1, %2 \n\t"                  /* f32Mult1 = f32Mult1.H * f16Mult2 */
                        "adds %1, %1, %4 \n\t"                  /* f32Mult1 = f32Mult1 + f32Val2 */
   
                        "movs %4, #0 \n\t"                      /* f32Val1 = 0 */
                        "asrs %0, %0, #1 \n\t"                  /* f32Accum >> 1 */
                        "adcs %4, %4, %4 \n\t"                  /* Stores the last bit of f32Accum to f32Val2 */
   
                        "adds %3, %3, %4 \n\t"                  /* f32Val1 = f32Val1 + f32Val2 */
                        "lsls %3, %3, #31 \n\t"                 /* f32Val1 << 31 (Carry = the first bit of f32Val1) */
                        "adcs %0, %0, %1 \n\t"                  /* (result / 2) + Carry */
   
                        "mov %1, %0 \n\t"                       /* f32Mult1 = result / 2 */
                        "lsls %4, %0, #1 \n\t"                  /* f32Val2 = result */
                        "lsrs %3, %3, #31 \n\t"                 /* f32Val1 >> 31 (the last bit of result) */
                        "adds %4, %4, %3 \n\t"                  /* f32Val2 = result + last bit */
   
                        "eors %0, %0, %4 \n\t"                  /* f32Accum = f32Accum ^ f32Val2 */
                        "bpl MLIB_MsuRndSat_F32lls_SatEnd%= \n\t" /* If f32Accum >= 0, then jumps to the end of function*/
                        "movs %4, #128 \n\t"                    /* f32Val2 = 0x80 */
                        "lsls %4, %4, #24 \n\t"                 /* result = 0x80000000 */
                        "cmp %1, #0 \n\t"                       /* Compares input value with 0 */
                        "blt MLIB_MsuRndSat_F32lls_SatEnd%= \n\t" /* If f32Mult1 < 0, then jumps through next command */
                        "subs %4, %4, #1 \n\t"                  /* result = 0x7FFFFFFF*/
                    "MLIB_MsuRndSat_F32lls_SatEnd%=: \n\t"
                        "movs %0, %4 \n\t"                      /* f32Accum = result*/
                        : "+l"(f32Accum), "+l"(f32Mult1), "+l"(f16Mult2), "+l"(f32Val1), "+l"(f32Val2):);
    #else
        __asm volatile(
                        #if defined(__GNUC__)                   /* For GCC compiler */
                            ".syntax unified \n"                /* Using unified asm syntax */
                        #endif
                        "sxth %2, %2 \n"                        /* Converts 16-bit input to 32-bit */
                        "rsbs %2, %2, #0 \n"                    /* f16Mult2 = - f16Mult2 */
                        "uxth %4, %1 \n"                        /* f32Val2 = f32Mult1.L */
                        "asrs %1, %1, #16 \n"                   /* f32Mult1 = f32Mult1.H */

                        "muls %4, %4, %2 \n"                    /* f32Val2 = f32Mult1.L * f16Mult2 */
                        "asrs %4, %4, #7 \n"                    /* f32Val2 >> 7 */
                        "adds %4, %4, #128 \n"                  /* Rounding */
                        "asrs %4, %4, #8 \n"                    /* f32Val2 >> 8 */

                        "movs %3, #0 \n"                        /* f32Val1 = 0 */
                        "asrs %4, %4, #1 \n"                    /* f32Val2 >> 1 */
                        "adcs %3, %3, %3 \n"                    /* Stores the last bit of multiplication to f32Val1 */
                        "muls %1, %1, %2 \n"                    /* f32Mult1 = f32Mult1.H * f16Mult2 */
                        "adds %1, %1, %4 \n"                    /* f32Mult1 = f32Mult1 + f32Val2 */

                        "movs %4, #0 \n"                        /* f32Val1 = 0 */
                        "asrs %0, %0, #1 \n"                    /* f32Accum >> 1 */
                        "adcs %4, %4, %4 \n"                    /* Stores the last bit of f32Accum to f32Val2 */

                        "adds %3, %3, %4 \n"                    /* f32Val1 = f32Val1 + f32Val2 */
                        "lsls %3, %3, #31 \n"                   /* f32Val1 << 31 (Carry = the first bit of f32Val1) */
                        "adcs %0, %0, %1 \n"                    /* (result / 2) + Carry */

                        "mov %1, %0 \n"                         /* f32Mult1 = result / 2 */
                        "lsls %4, %0, #1 \n"                    /* f32Val2 = result */
                        "lsrs %3, %3, #31 \n"                   /* f32Val1 >> 31 (the last bit of result) */
                        "adds %4, %4, %3 \n"                    /* f32Val2 = result + last bit */

                        "eors %0, %0, %4 \n"                    /* f32Accum = f32Accum ^ f32Val2 */
                        "bpl .+12 \n"                           /* If f32Accum >= 0, then jumps to the end of function*/
                        "movs %4, #128 \n"                      /* f32Val2 = 0x80 */
                        "lsls %4, %4, #24 \n"                   /* result = 0x80000000 */
                        "cmp %1, #0 \n"                         /* Compares input value with 0 */
                        "blt .+4 \n"                            /* If f32Mult1 < 0, then jumps through next command */
                        "subs %4, %4, #1 \n"                    /* result = 0x7FFFFFFF*/
                        "movs %0, %4 \n"                        /* f32Accum = result*/
                        #if defined(__GNUC__)                   /* For GCC compiler */
                            ".syntax divided \n"
                        #endif
                        : "+l"(f32Accum), "+l"(f32Mult1), "+l"(f16Mult2), "+l"(f32Val1), "+l"(f32Val2):);
    #endif

    return f32Accum;
}
/* inline function without any optimization (compilation issue) */ 
RTCESL_INLINE_OPTIM_RESTORE 

#if defined(__IAR_SYSTEMS_ICC__)           /* IAR compiler */
#pragma diag_default=Pe549
#endif

#if defined(__cplusplus)
}
#endif

#endif /* _MLIB_MSURND_F32_ASM_H_ */

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
* @brief  Multiply
* 
*******************************************************************************/
#ifndef _MLIB_MUL_F16_ASM_H_
#define _MLIB_MUL_F16_ASM_H_

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
#define MLIB_MulSat_F16_Asmi(f16Mult1, f16Mult2) MLIB_MulSat_F16_FAsmi(f16Mult1, f16Mult2)

/***************************************************************************//*!
*
* Output = f16Mult1 * f16Mult2
* With saturation
*******************************************************************************/
/* inline function without any optimization (compilation issue) */ 
RTCESL_INLINE_OPTIM_SAVE
RTCESL_INLINE_OPTIM_SET
static inline frac16_t MLIB_MulSat_F16_FAsmi(register frac16_t f16Mult1, register frac16_t f16Mult2)
{
    register frac32_t f32SatVal = 0x8000;
    #if defined(__CC_ARM)                                     /* For ARM Compiler */
        __asm volatile{ sxth f16Mult1, f16Mult1               /* Converts 16-bit input to 32-bit */
                        sxth f16Mult2, f16Mult2               /* Converts 16-bit input to 32-bit */
                        muls f16Mult1, f16Mult1, f16Mult2     /* f16Mult1 * f16Mult2 */
                        asrs f16Mult1, f16Mult1, #15          /* f16Mult1 >> 15  */
                        cmp f16Mult1, f32SatVal               /* Compares f16Mult1 with 0x8000*/
                        bne NotSat                            /* If f16Mult1 <> 0x8000, then jumps through next command */
                        subs f16Mult1, f16Mult1, #1           /* If f16Mult1 = 0x8000, then f16Mult1 = 0x7FFF */
                    NotSat: };
    #elif defined(__GNUC__) && defined(__ARMCC_VERSION) 
        __asm volatile(
                        "sxth %0, %0 \n\t"                    /* Converts 16-bit input to 32-bit */
                        "sxth %1, %1 \n\t"                    /* Converts 16-bit input to 32-bit */
                        "muls %0, %0, %1 \n\t"                /* f16Mult1 * f16Mult2 */
                        "asrs %0, %0, #15 \n\t"               /* f16Mult1 >> 15  */
                        "cmp %0, %2 \n\t"                     /* Compares f16Mult1 with 0x8000*/
                        "bne MLIB_MulSat_F16_NotSat%= \n\t"     /* If f16Mult1 <> 0x8000, then jumps through next command */
                        "subs %0, %0, #1 \n\t"                /* If f16Mult1 = 0x8000, then f16Mult1 = 0x7FFF */
					"MLIB_MulSat_F16_NotSat%=: \n\t"
                        : "+l"(f16Mult1), "+l"(f16Mult2): "l"(f32SatVal));
    #else
        __asm volatile(
                        #if defined(__GNUC__)                 /* For GCC compiler */
                            ".syntax unified \n"              /* Using unified asm syntax */
                        #endif  
                        "sxth %0, %0 \n"                      /* Converts 16-bit input to 32-bit */
                        "sxth %1, %1 \n"                      /* Converts 16-bit input to 32-bit */
                        "muls %0, %0, %1 \n"                  /* f16Mult1 * f16Mult2 */
                        "asrs %0, %0, #15 \n"                 /* f16Mult1 >> 15  */
                        "cmp %0, %2 \n"                       /* Compares f16Mult1 with 0x8000*/
                        "bne .+4 \n"                          /* If f16Mult1 <> 0x8000, then jumps through next command */
                        "subs %0, %0, #1 \n"                  /* If f16Mult1 = 0x8000, then f16Mult1 = 0x7FFF */
                        #if defined(__GNUC__)                 /* For GCC compiler */
                            ".syntax divided \n"
                        #endif
                        : "+l"(f16Mult1), "+l"(f16Mult2): "l"(f32SatVal));
    #endif

    return f16Mult1;
}
/* inline function without any optimization (compilation issue) */ 
RTCESL_INLINE_OPTIM_RESTORE 

#if defined(__cplusplus)
}
#endif

#endif /* _MLIB_MUL_F16_ASM_H_ */

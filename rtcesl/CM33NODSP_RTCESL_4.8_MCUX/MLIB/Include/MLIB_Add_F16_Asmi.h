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
* @brief  Addition
* 
*******************************************************************************/
#ifndef _MLIB_ADD_F16_ASM_H_
#define _MLIB_ADD_F16_ASM_H_

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
#define MLIB_AddSat_F16_Asmi(f16Add1, f16Add2) MLIB_AddSat_F16_FAsmi(f16Add1, f16Add2)

/***************************************************************************//*!
*  Saturated Addition      
*  Output = f16Add1 + f16Add2
*******************************************************************************/
/* inline function without any optimization (compilation issue) */ 
RTCESL_INLINE_OPTIM_SAVE
RTCESL_INLINE_OPTIM_SET
static inline frac16_t MLIB_AddSat_F16_FAsmi(register frac16_t f16Add1, register frac16_t f16Add2)
{
    register frac32_t f32Val = 0x8000;

    #if defined(__CC_ARM)                                  /* For ARM Compiler */
        __asm volatile{ sxth f16Add1, f16Add1              /* Transforms 16-bit input f16Add1 to 32-bit */
                        sxth f16Add2, f16Add2              /* Transforms 16-bit input f16Add2 to 32-bit */
                        adds f16Add1, f16Add2, f16Add1     /* f16Add1 = f16Add1 + f16Add2 */
                        cmp f32Val, f16Add1                /* Compares f16Add1 with 0x8000 */
                        bgt NegTest                        /* If f16Add1 < 0x8000, then goes to NegTest */
                        subs f16Add1, f32Val, #1           /* If f16Add1 >= 0x8000, then f16Add1 = 0x7FFF */
                        b SatEnd                           /* Goes to SatEnd */
                    NegTest:   
                        rsbs f32Val, f32Val, #0            /* f32Val = 0xFFFF8000 */
                        cmp f32Val, f16Add1                /* Compares f16Add1 with 0xFFFF8000 */
                        ble SatEnd                         /* If f16Add1 >= 0xFFFF8000, then goes to SatEnd */
                        mov f16Add1, f32Val};              /* If f16Add1 < 0xFFFF8000, then f16Add1 = 0xFFFF8000 */
                    SatEnd:
    #elif defined(__GNUC__) && defined(__ARMCC_VERSION) 
        __asm volatile(
                        "sxth %1, %1 \n\t"                 /* Transforms 16-bit input f16Add1 to 32-bit */
                        "sxth %2, %2 \n\t"                 /* Transforms 16-bit input f16Add2 to 32-bit */
                        "adds %1, %2, %1 \n\t"             /* f16Add1 = f16Add1 + f16Add2 */
                        "cmp %0, %1 \n\t"                  /* Compares f16Add1 with 0x8000 */
                        "bgt MLIB_AddSat_F16_NegTest%= \n\t" /* If f16Add1 <= 0x8000, then jumps through two commands */
				         "subs %1, %0, #1 \n\t"            /* If f16Add1 > 0x8000, then f16Add1 = 0x7FFF */			
                        "b MLIB_AddSat_SatEnd%= \n\t"        /* Jumps through four commands */
                    "MLIB_AddSat_F16_NegTest%=: \n\t"	
				        "rsbs %0, %0, #0 \n\t"             /* f32Val = 0xFFFF8000 */
                        "cmp %0, %1 \n\t"                  /* Compares f16Add1 with 0xFFFF8000 */
                        "ble MLIB_AddSat_SatEnd%= \n\t"      /* If f16Add1 >= 0xFFFF8000, then jumps through next commands */
                        "mov %1, %0 \n\t"                  /* If f16Add1 < 0xFFFF8000, then f16Add1 = 0xFFFF8000 */
                    "MLIB_AddSat_SatEnd%=: \n\t"
                        : "+l"(f32Val), "+l"(f16Add1), "+l"(f16Add2):);
    #else
        __asm volatile(
                        #if defined(__GNUC__)              /* For GCC compiler */
                            ".syntax unified \n"           /* Using unified asm syntax */
                        #endif   
                        "sxth %1, %1 \n"                   /* Transforms 16-bit input f16Add1 to 32-bit */
                        "sxth %2, %2 \n"                   /* Transforms 16-bit input f16Add2 to 32-bit */
                        "adds %1, %2, %1 \n"               /* f16Add1 = f16Add1 + f16Add2 */
                        "cmp %0, %1 \n"                    /* Compares f16Add1 with 0x8000 */
                        "bgt .+6 \n"                       /* If f16Add1 <= 0x8000, then jumps through two commands */
                        "subs %1, %0, #1 \n"               /* If f16Add1 > 0x8000, then f16Add1 = 0x7FFF */
                        "b .+10 \n"                        /* Jumps through four commands */
                        "rsbs %0, %0, #0 \n"               /* f32Val = 0xFFFF8000 */
                        "cmp %0, %1 \n"                    /* Compares f16Add1 with 0xFFFF8000 */
                        "ble .+4 \n"                       /* If f16Add1 >= 0xFFFF8000, then jumps through next commands */
                        "mov %1, %0 \n"                    /* If f16Add1 < 0xFFFF8000, then f16Add1 = 0xFFFF8000 */
                        #if defined(__GNUC__)              /* For GCC compiler */
                            ".syntax divided \n"
                        #endif
                        : "+l"(f32Val), "+l"(f16Add1), "+l"(f16Add2):);
    #endif

    return f16Add1;
}
/* inline function without any optimization (compilation issue) */ 
RTCESL_INLINE_OPTIM_RESTORE 

#if defined(__cplusplus)
}
#endif

#endif  /*_MLIB_ADD_F16_ASM_H_*/

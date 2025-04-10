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
* @brief  MLIB_PQ.h
* 
*******************************************************************************/
#ifndef _MLIB_PQ_H_
#define _MLIB_PQ_H_

#if defined(__cplusplus)
extern "C" {
#endif

/******************************************************************************
* Includes
******************************************************************************/   
#if defined(__CC_ARM)

#elif defined(__ICCARM__)
#include <intrinsics.h>
#elif defined(__GNUC__)
#include <arm_acle.h>
#endif /* defined(__CC_ARM) */

#include "mlib_types.h"  

/*******************************************************************************
* Macros
*******************************************************************************/    
/* Base address of Power Quad peripheral module(PQ) */ 
/* LPC Family disabled as default */
#if 0
#define RTCESL_SYSCON_BASE_PTR 0x40000000U
#define RTCESL_PQ_BASE_PTR     0x400A6000U
#define RTCESL_AHBCLKCTRLSET2 *(volatile unsigned int*)(RTCESL_SYSCON_BASE_PTR + 0x228U) */
#else
/* MCXN Family */
#define RTCESL_SYSCON_BASE_PTR 0x40000000U
#define RTCESL_PQ_BASE_PTR     0x400BF000U
#define RTCESL_AHBCLKCTRLSET2 *(volatile unsigned int*)(RTCESL_SYSCON_BASE_PTR + 0x208U) 
/* End of Family spefific section */
#endif

/* Power Quad register access macros */ 
#define RTCESL_PQ_CONTROL     *(volatile unsigned int*)(RTCESL_PQ_BASE_PTR + 0x100U)
#define RTCESL_PQ_CPPRE       *(volatile unsigned int*)(RTCESL_PQ_BASE_PTR + 0x108U)

#define RTCESL_PQ_CORDIC_X    *(volatile unsigned int*)(RTCESL_PQ_BASE_PTR + 0x180U)
#define RTCESL_PQ_CORDIC_Y    *(volatile unsigned int*)(RTCESL_PQ_BASE_PTR + 0x184U)
#define RTCESL_PQ_CORDIC_Z    *(volatile unsigned int*)(RTCESL_PQ_BASE_PTR + 0x188U)

#define RTCESL_PQ_GPREG0      *(volatile unsigned int*)(RTCESL_PQ_BASE_PTR + 0x200U)
#define RTCESL_PQ_GPREG1      *(volatile unsigned int*)(RTCESL_PQ_BASE_PTR + 0x204U)
#define RTCESL_PQ_GPREG2      *(volatile unsigned int*)(RTCESL_PQ_BASE_PTR + 0x208U)
#define RTCESL_PQ_GPREG3      *(volatile unsigned int*)(RTCESL_PQ_BASE_PTR + 0x20CU)
#define RTCESL_PQ_GPREG4      *(volatile unsigned int*)(RTCESL_PQ_BASE_PTR + 0x210U)
#define RTCESL_PQ_GPREG5      *(volatile unsigned int*)(RTCESL_PQ_BASE_PTR + 0x214U)
#define RTCESL_PQ_GPREG6      *(volatile unsigned int*)(RTCESL_PQ_BASE_PTR + 0x218U)

/* Power Quad register bit fields macros */  
#define RTCESL_PQ_CPPRE_CPPRE_OUT_MASK   (0xFF00U)
#define RTCESL_PQ_CPPRE_CPPRE_OUT_SHIFT  (8U)
#define RTCESL_PQ_CPPRE_CPPRE_OUT(x)     (((uint32_t)(((uint32_t)(x)) << RTCESL_PQ_CPPRE_CPPRE_OUT_SHIFT)) & RTCESL_PQ_CPPRE_CPPRE_OUT_MASK)

/* Power Quad coprocessor based macros */  
#define RTCESL_PQ_FLT (0U)
#define RTCESL_PQ_FIX (1U)
#define RTCESL_PQ_CP (0U)
#define RTCESL_PQ_CP_CORDIC (5U)
#define RTCESL_PQ_TRIG (1U)
#define RTCESL_PQ_TRANS_FIX (4U)
#define RTCESL_PQ_TRIG_FIX (5U)
#define RTCESL_PQ_BIQUAD_FIX (6U)
#define RTCESL_PQ_SQRT (2U)
#define RTCESL_PQ_SIN (0U)
#define RTCESL_PQ_COS (1U)
#define RTCESL_PQ_BIQ0_CALC (1U)
#define RTCESL_PQ_COMP0 (0U << 1)
#define RTCESL_PQ_CORDIC_ITER(x) ((x) << 2U)
#define RTCESL_PQ_CORDIC_MIU(x) (x << 1U)
#define RTCESL_PQ_CORDIC_T(x) (x << 0U)
#define RTCESL_PQ_CORDIC_ARCTAN (1U)
#define RTCESL_PQ_INST_BUSY (0x80000000U)
  
/* Power Quad coprocessor  access based macros */  
#define RTCESL_PQ_SIN0(x)          __arm_mcr(RTCESL_PQ_CP, RTCESL_PQ_SIN, x, RTCESL_PQ_FLT | RTCESL_PQ_COMP0, 0, RTCESL_PQ_TRIG)
#define RTCESL_PQ_COS0(x)          __arm_mcr(RTCESL_PQ_CP, RTCESL_PQ_COS, x, RTCESL_PQ_FLT | RTCESL_PQ_COMP0, 0, RTCESL_PQ_TRIG)
#define RTCESL_PQ_SIN0_FIX(x)      __arm_mcr(RTCESL_PQ_CP, RTCESL_PQ_SIN, x, RTCESL_PQ_FIX | RTCESL_PQ_COMP0, 0, RTCESL_PQ_TRIG_FIX)
#define RTCESL_PQ_COS0_FIX(x)      __arm_mcr(RTCESL_PQ_CP, RTCESL_PQ_COS, x, RTCESL_PQ_FIX | RTCESL_PQ_COMP0, 0, RTCESL_PQ_TRIG_FIX)

#define RTCESL_PQ_SQRT_FLT0(x)     __arm_mcr(RTCESL_PQ_CP, RTCESL_PQ_SQRT,x, RTCESL_PQ_FIX | RTCESL_PQ_COMP0, 0, RTCESL_PQ_TRANS_FIX)
#define RTCESL_PQ_BIQUAD0_FLT(x)   __arm_mcr(RTCESL_PQ_CP, RTCESL_PQ_BIQ0_CALC, x, RTCESL_PQ_FIX | RTCESL_PQ_COMP0, 0,RTCESL_PQ_BIQUAD_FIX)
#define RTCESL_PQ_READ_ADD0()      __arm_mrc(RTCESL_PQ_CP, 1U, RTCESL_PQ_FLT | RTCESL_PQ_COMP0, 0U, 0U)
#define RTCESL_PQ_READ_MULT0_FLT() __arm_mrc(RTCESL_PQ_CP, 0U, RTCESL_PQ_FIX | RTCESL_PQ_COMP0, 0U, 0U)
#define RTCESL_PQ_READ_ADD0_FLT()  __arm_mrc(RTCESL_PQ_CP, 1U, RTCESL_PQ_FIX | RTCESL_PQ_COMP0, 0U, 0U)

/*******************************************************************************
* Types
*******************************************************************************/
/*! @brief Conversion between integer and float type */
typedef union
{
    float_t  flt; /*!< Float type.*/
    uint32_t u32; /*!< Iterger type.*/
} rtcesl_pq_convert_t; 

/****************************************************************************
* Inline functions 
****************************************************************************/ 

/***************************************************************************//*!
* @brief  The function set the clock for proper PQ operation. Must be called before first PQ using.
*
* @param  void
*
*******************************************************************************/
static inline void RTCESL_PQ_Init(void)
{ 
    RTCESL_AHBCLKCTRLSET2 = 0x80000;
}

/***************************************************************************//*!
* @brief  The function execute waiting for PQ completion the operation. 
*
* @param  void
*
*******************************************************************************/
static inline void RTCESL_PQ_WaitDone(void)
{
    while ((RTCESL_PQ_CONTROL & RTCESL_PQ_INST_BUSY) == RTCESL_PQ_INST_BUSY)
    {
        #if defined(__IAR_SYSTEMS_ICC__)                         /* For IAR compiler   */
        __iar_builtin_WFE();
        #elif defined(__GNUC__) && defined(__ARMCC_VERSION)      /* For ARM(KEIL) version >= 6 compiler */
        __builtin_arm_wfe();
        #elif defined(__GNUC__)                                  /* For GCC compiler */
        __asm volatile(
                        #if defined(__GNUC__) && !defined(__ARMCC_VERSION)        /* For GCC compiler */
                            ".syntax unified \n"                 /* Using unified asm syntax */
                        #endif
        		        "wfe \n"
                        #if defined(__GNUC__) && !defined(__ARMCC_VERSION)        /* For GCC compiler */
                            ".syntax divided \n"
                        #endif
                        :);
        #else                                                    /* Other compiler used */
            #warning "Unsupported compiler/IDE used !"    
        #endif
    }
}

#if defined(__cplusplus)
}
#endif

#endif /* _MLIB_PQ_H_ */

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
* @brief  Multiply subtract of four inputs
* 
*******************************************************************************/
#ifndef _MLIB_MSU4_F32_H_
#define _MLIB_MSU4_F32_H_

#if defined(__cplusplus)
extern "C" {
#endif

/*******************************************************************************
* Includes
*******************************************************************************/
#include "mlib_types.h"
#include "MLIB_Shift_F32.h"

/*******************************************************************************
* Macros
*******************************************************************************/
#define MLIB_Msu4_F32ssss_Ci(f16MinMul1, f16MinMul2, f16SubMul1, f16SubMul2)    \
        MLIB_Msu4_F32ssss_FCi(f16MinMul1, f16MinMul2, f16SubMul1, f16SubMul2) 
#define MLIB_Msu4Sat_F32ssss_Ci(f16MinMul1, f16MinMul2, f16SubMul1, f16SubMul2) \
        MLIB_Msu4Sat_F32ssss_FCi(f16MinMul1, f16MinMul2, f16SubMul1, f16SubMul2) 
/***************************************************************************//*!
*
* f32Out = (f16MinMul1 * f16MinMul2) - (f16SubMul1 * f16SubMul2)
* Without saturation
*******************************************************************************/
static inline frac32_t MLIB_Msu4_F32ssss_FCi(register frac16_t f16MinMul1, register frac16_t f16MinMul2,
                                             register frac16_t f16SubMul1, register frac16_t f16SubMul2)
{
    return  (MLIB_Sh1L_F32_Ci(((int32_t)f16MinMul1 * (int32_t)f16MinMul2) -
                              ((int32_t)f16SubMul1 * (int32_t)f16SubMul2)));
}

/***************************************************************************//*!
*
* f32Out = (f16MinMul1 * f16MinMul2) - (f16SubMul1 * f16SubMul2)
* With saturation
*******************************************************************************/
static inline frac32_t MLIB_Msu4Sat_F32ssss_FCi(register frac16_t f16MinMul1, register frac16_t f16MinMul2,
                                                register frac16_t f16SubMul1, register frac16_t f16SubMul2)
{
    int32_t i32Temp;
    
    i32Temp = ((frac32_t)((int32_t)(f16MinMul1)*(int32_t)(f16MinMul2))) - 
              ((frac32_t)((int32_t)(f16SubMul1)*(int32_t)(f16SubMul2)));
    if (i32Temp >= 1073741824)
    {
        i32Temp = 2147483647;
    }
    else
    {
        if (i32Temp <= -1073741824)
        {
            i32Temp = INT32_MIN;
        }
        else
        {
            i32Temp = MLIB_Sh1L_F32_Ci(i32Temp);
        }
    }
    return (i32Temp); 
}
#if defined(__cplusplus)
}
#endif

#endif  /* _MLIB_MSU4_F32_H_ */

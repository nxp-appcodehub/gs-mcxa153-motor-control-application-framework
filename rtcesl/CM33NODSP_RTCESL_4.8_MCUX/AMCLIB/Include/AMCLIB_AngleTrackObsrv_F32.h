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
* @brief  Angle tracking observer
* 
*******************************************************************************/
#ifndef _AMCLIB_ANGLE_TRACK_OBSRV_F32_H_
#define _AMCLIB_ANGLE_TRACK_OBSRV_F32_H_

#if defined(__cplusplus) 
extern "C" { 
#endif 
/******************************************************************************
* Includes
******************************************************************************/
#include "gflib.h"
#include "gmclib.h"

/******************************************************************************
* Macros 
******************************************************************************/
#define AMCLIB_AngleTrackObsrv_F16_C(psSinCos, psCtrl)                        \
        AMCLIB_AngleTrackObsrv_F16_FC(psSinCos, psCtrl)                       
#define AMCLIB_AngleTrackObsrvInit_F16_C(f16ThetaInit, psCtrl)                \
        AMCLIB_AngleTrackObsrvInit_F16_FC(f16ThetaInit, psCtrl)
        
/******************************************************************************
* Types
******************************************************************************/
typedef struct
{
    frac32_t   f32Speed;       
    frac32_t   f32A2;          
    frac16_t   f16Theta;       
    frac16_t   f16SinEstim;    
    frac16_t   f16CosEstim;    
    frac16_t   f16K1Gain;        
    int16_t    i16K1GainSh;
    frac16_t   f16K2Gain;        
    int16_t    i16K2GainSh;
    frac16_t   f16A2Gain;        
    int16_t    i16A2GainSh;
} AMCLIB_ANGLE_TRACK_OBSRV_T_F32;
   
/******************************************************************************
* Exported function prototypes
******************************************************************************/
extern frac16_t AMCLIB_AngleTrackObsrv_F16_FC(const GMCLIB_2COOR_SINCOS_T_F16 *psSinCos,
                                              AMCLIB_ANGLE_TRACK_OBSRV_T_F32 *psCtrl);
extern void AMCLIB_AngleTrackObsrvInit_F16_FC(frac16_t f16ThetaInit, 
                                              AMCLIB_ANGLE_TRACK_OBSRV_T_F32 *psCtrl);

#if defined(__cplusplus) 
} 
#endif 

#endif /* _AMCLIB_ANGLE_TRACK_OBSRV_F32_H_ */

/*
 * Copyright 2021, 2025 NXP
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef _MC_PERIPH_INIT_H_
#define _MC_PERIPH_INIT_H_

#include "fsl_device_registers.h"
#include "pin_mux.h"
#include "app_config.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/* Independent register definitions for baremetal port init */

#define ADC0_CLK_FRO_12M 		    0U
#define ADC0_CLK_FRO_HF_GATED 		1U
#define ADC0_CLK_CLK_IN 		    3U
#define ADC0_CLK_CLK_1M 		    5U

#define CMP0_CLK_FRO_12M 		    0U
#define CMP0_CLK_FRO_HF_GATED 		2U
#define CMP0_CLK_CLK_IN 		    3U
#define CMP0_CLK_CLK_1M 		    5U

#define CTIMER0_CLK_FRO_12M 	    0U
#define CTIMER0_CLK_FRO_HF_GATED 	1U
#define CTIMER0_CLK_CLK_IN 	    	3U
#define CTIMER0_CLK_CLK_16K 	    4U
#define CTIMER0_CLK_CLK_1M 	    	5U

#define OVERCURRENT(x)      (uint8_t)((x / I_MAX_SCALE) * (float)255U)

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

void InitADC0(void);
void InitLPCMP0(void);
void InitPWM0(void);
void InitSlowLoop(void);


/******************************************************************************
 * Timing
 ******************************************************************************/
/* MCU core clock in Hz */
#define MCU_CLOCK_FREQ          (96000000U)
/* PWM frequency in Hz*/
#define M1_PWM_FREQ             (10000U)
/* PWM modulo = FTM_input_clock / M1_PWM_FREQ */
#define M1_PWM_MODULO           (MCU_CLOCK_FREQ / M1_PWM_FREQ)
/* Output PWM deadtime value in nanoseconds */
#define M1_PWM_DEADTIME (500)
/* PWM vs. Fast control loop ratio */
#define M1_FOC_FREQ_VS_PWM_FREQ (1U)
/* Slow control loop frequency */
#define M1_SLOW_LOOP_FREQ       (1000U)

#endif /* _MC_PERIPH_INIT_H_  */

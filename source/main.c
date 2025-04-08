/*
 * Copyright 2016, 2025 NXP
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/**
 * @file    main.c
 * @brief   Application entry point.
 */
#include "main.h"
#include "motor_control.h"
#include "board.h"
#include "peripherals.h"
#include "pin_mux.h"
#include "fsl_lpuart.h"
#include "mc_periph_init.h"
#include "motor_control.h"
#include "freemaster.h"
#include "freemaster_serial_lpuart.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*******************************************************************************
 * Variables
 ******************************************************************************/
uint32_t ui32Ctimer0IsrCnt = 0;
uint32_t ui32ADC0IsrCnt = 0;

static FMSTR_U8 FreeMASTER_RecBuffer0[2048];

FMSTR_REC_BUFF FreeMASTER_Recorder_0 =
{
  .name = "Description of recorder 0",
  .addr = (FMSTR_ADDR)FreeMASTER_RecBuffer0,
  .size = sizeof(FreeMASTER_RecBuffer0),
  .basePeriod_ns = 100000UL
};

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
static void InitLPUART0(void);
static void InitFreeMASTER(void);
static void SyncAdcPwm(void);
static tPointerFcn AppStateMachineFast[] = {AppInitFast,AppStopFast,AppStartFast,AppRunFast,AppErrorFast};
static tPointerFcn AppStateMachineSlow[] = {AppInitSlow,AppStopSlow,AppStartSlow,AppRunSlow,AppErrorSlow};
/*******************************************************************************
 * Code
 ******************************************************************************/

int main(void)
{
    uint32_t ui32PrimaskReg;
    ui32PrimaskReg = DisableGlobalIRQ(); 	/* Disable interrupts */
    BOARD_BootClockFRO96M();   				/* Set core clock */
    BOARD_InitPins();
	InitSlowLoop();     	                /* Init slow loop timer */
	InitADC0();         	                /* Init ADC0 */
	InitLPCMP0();       	                /* Init Low Power Comparator 0 */
	InitPWM0();         	                /* 6-channel PWM0 peripheral init */
	InitLPUART0();       	                /* FreeMASTER communication layer initialization */
	InitFreeMASTER();   	                /* FreeMASTER middleware initialization */
    ui8MotorState = APP_ST_INIT;			/* Initial state */
    SyncAdcPwm();							/* Re-sync ADC-PWM */
    EnableGlobalIRQ(ui32PrimaskReg);		/* Enable interrupts  */

    /* Infinite loop */
    while (1)
    {
        FMSTR_Poll();						/* FreeMASTER Polling function */
    }
}

/*****************************************************************************
*
* Function: static void InitLPUART0(void)
*
* Description: LPUART Module initialization for FreeMASTER
*
* Returns: None
*
* Global Data:
*
* Arguments: None
*
*****************************************************************************/
static void InitLPUART0(void)
{
    lpuart_config_t config;

    /* Attach main clock divide to LPUART0 */
    CLOCK_SetClockDiv(kCLOCK_DivLPUART0, 1u);
    CLOCK_AttachClk(kFRO12M_to_LPUART0);

    LPUART_GetDefaultConfig(&config);
    /* Override the Default configuration to satisfy FreeMASTER needs */
    config.baudRate_Bps = 115200U;
    config.enableTx = true;
    config.enableRx = true;
    /* Clock for LPUART0 peripheral is 12 MHz */
    LPUART_Init((LPUART_Type*)LPUART0, &config, 12000000U);
    /* Register communication module used by FreeMASTER driver. */
    FMSTR_SerialSetBaseAddress((LPUART_Type*)LPUART0);

}

/*****************************************************************************
*
* Function: static void InitFreeMASTER(void)
*
* Description: FreeMASTER middleware initialization
*
* Returns: None
*
* Global Data:
*
* Arguments: None
*
*****************************************************************************/
static void InitFreeMASTER(void)
{
	FMSTR_Init();									/* FreeMASTER middleware initialization */
	FMSTR_RecorderCreate(0, &FreeMASTER_Recorder_0);/* FreeMASTER recorder 0 configuration initialization  */
}

/*****************************************************************************
*
* Function: void CTIMER0_IRQHandler(void)
*
* Description: Ctimer Slow loop interrupt handler (1ms period)
*
* Returns: None
*
* Global Data:
*
* Arguments: None
*
*****************************************************************************/
void CTIMER0_IRQHandler(void)
{
	ui32Ctimer0IsrCnt++;    				/* Isr check counter */
    CTIMER0->IR |= CTIMER_IR_MR0INT(1U);    /* Clear the match interrupt flag. */
	AppStateMachineSlow[ui8MotorState]();	/* Call state machine */
    M1_END_OF_ISR;							/* Add empty instructions for correct interrupt flag clearing */
}

/*****************************************************************************
*
* Function: void ADC0_IRQHandler(void)
*
* Description: ADC Fast loop interrupt handler
*
* Returns: None
*
* Global Data:
*
* Arguments: None
*
*****************************************************************************/
void ADC0_IRQHandler(void)
{
    ui32ADC0IsrCnt++;						/* Isr check counter */
    ADC0->STAT |= ADC_STAT_FOF0_MASK;		/* Clear the FIFO INT flag */
    AdcGetResults();    					/* Get ADC0 results */
	AppStateMachineFast[ui8MotorState]();	/* Call state machine */
    FMSTR_Recorder(0);						/* Call FreeMASTER recorder */
    M1_END_OF_ISR;							/* Add empty instructions for correct interrupt flag clearing */
}

/*****************************************************************************
*
* Function: static void SyncAdcPwm(void)
*
* Description: PWM to ADC sync function
*
* Returns: None
*
* Global Data:
*
* Arguments: None
*
*****************************************************************************/
static void SyncAdcPwm(void)
{
	ADC0->CTRL |= ADC_CTRL_RSTFIFO0_MASK;	/* Reset FIFO 0 */
    FLEXPWM0->MCTRL|= PWM_MCTRL_RUN(0x7);	/* Run PWM */
}

/*
 * Copyright 2021, 2025 NXP
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "motor_control.h"
#include "app_config.h"
#include "gdflib.h"


static bool_t OverCurrentFault(PWM_Type *pPWMBase);
static void UpdatePWM(frac16_t f16PhA, frac16_t f16PhB, frac16_t f16PhC, PWM_Type *pPWMBase);

volatile uint8_t ui8MotorState;
volatile float fltDcBusVoltageScale;
volatile float fltCurrentScale;
uint16_t ui16CalibCnt;
uint16_t ui16Run;
volatile uint32_t ui32Fcount0;

GDFLIB_FILTER_MA_T_A32 sFilterParamOffsetIa;
GDFLIB_FILTER_MA_T_A32 sFilterParamOffsetIb;
GDFLIB_FILTER_MA_T_A32 sFilterParamOffsetIc;
GDFLIB_FILTER_MA_T_A32 sFilterParamOffsetIdcb;
sAdcRawResult sAdcRaw;
sAdcResult sAdcResRaw;
sAdcResult sAdcOffset;
sAdcResult sAdcRes;
sMotorDuty sMotorPWM;

/*****************************************************************************
*
* Function: void AppInitFast(void)
*
* Description: State function of application state machine
*
* Returns: None
*
* Global Data:
*
* Arguments: None
*
*****************************************************************************/
void AppInitFast(void)
{
	ui16CalibCnt = 0;
	ui16Run = 0;
    fltDcBusVoltageScale = U_DCB_MAX_SCALE;
    fltCurrentScale      = I_MAX_SCALE;
    sAdcOffset.f16Ia     = FRAC16(0.5);
    sAdcOffset.f16Ib     = FRAC16(0.5);
    sAdcOffset.f16Ic     = FRAC16(0.5);
    sAdcOffset.f16Idcb   = FRAC16(0.5);
    sMotorPWM.f16DutyA   = FRAC16(0.5);
    sMotorPWM.f16DutyB   = FRAC16(0.5);
    sMotorPWM.f16DutyC   = FRAC16(0.5);
	/* Set filter size*/
    sFilterParamOffsetIa.u16Sh = 3;
	/* Init filter*/
	GDFLIB_FilterMAInit_F16(FRAC16(0.0), &sFilterParamOffsetIa);
	/* Set filter size*/
	sFilterParamOffsetIb.u16Sh = 3;
	/* Init filter*/
	GDFLIB_FilterMAInit_F16(FRAC16(0.0), &sFilterParamOffsetIb);
	/* Set filter size*/
	sFilterParamOffsetIc.u16Sh = 3;
	/* Init filter*/
	GDFLIB_FilterMAInit_F16(FRAC16(0.0), &sFilterParamOffsetIc);
	/* Set filter size*/
	sFilterParamOffsetIdcb.u16Sh = 3;
	/* Init filter*/
	GDFLIB_FilterMAInit_F16(FRAC16(0.0), &sFilterParamOffsetIdcb);
	/* Transition to next state */
	AppTransInitToStopFast();
}
/*****************************************************************************
*
* Function: void AppRunFast(void)
*
* Description: State function of application state machine
*
* Returns: None
*
* Global Data:
*
* Arguments: None
*
*****************************************************************************/
void AppRunFast(void)
{
    /* Update PWM register values */
    UpdatePWM(sMotorPWM.f16DutyA, sMotorPWM.f16DutyB, sMotorPWM.f16DutyC, FLEXPWM0);

    /* Run condition */
	if (ui16Run == 0)
	{
		AppTransRunToStopFast();	/* Transition to next state */
	}

    /* Over current fault detection */
    /*
    if(OverCurrentFault(PWM0))
    {
    	AppTransRunToErrorFast();
    }
	*/
}
/*****************************************************************************
*
* Function: void AppStopFast(void)
*
* Description: State function of application state machine
*
* Returns: None
*
* Global Data:
*
* Arguments: None
*
*****************************************************************************/
void AppStopFast(void)
{
    /* Update PWM register values */
    UpdatePWM(sMotorPWM.f16DutyA, sMotorPWM.f16DutyB, sMotorPWM.f16DutyC, FLEXPWM0);
    /* Run condition */
	if (ui16Run > 0)
	{
		/* Make calibration before start */
		if(Calibration(&sAdcResRaw, &sAdcOffset, &ui16CalibCnt, CALIB_LOOP_CNT))
		{
		    PWM_EN(FLEXPWM0);				/* Enable PWM outputs */
			AppTransStopToStartFast();	/* Transition to next state */
		}
	}

    /* Over current fault detection */
	/*
    if(OverCurrentFault(PWM0))
    {
    	AppTransStopToErrorFast();
    }
    */
}
/*****************************************************************************
*
* Function: void AppStartFast(void)
*
* Description: State function of application state machine
*
* Returns: None
*
* Global Data:
*
* Arguments: None
*
*****************************************************************************/
void AppStartFast(void)
{
	AppTransStartToRunFast();
}
/*****************************************************************************
*
* Function: void AppErrorFast(void)
*
* Description: State function of application state machine
*
* Returns: None
*
* Global Data:
*
* Arguments: None
*
*****************************************************************************/
void AppErrorFast(void)
{
	AppTransErrorToStopFast();
}
/*****************************************************************************
*
* Function: void AppTransInitToStopFast(void)
*
* Description: Transition function of application state machine
*
* Returns: None
*
* Global Data:
*
* Arguments: None
*
* Range Issues: None
*
* Special Issues: None
*
*****************************************************************************/
void AppTransInitToStopFast(void)
{
	ui8MotorState = APP_ST_STOP;
}
/*****************************************************************************
*
* Function: void AppTransStopToStartFast(void)
*
* Description: Transition function of application state machine
*
* Returns: None
*
* Global Data:
*
* Arguments: None
*
*****************************************************************************/
void AppTransStopToStartFast(void)
{
	ui8MotorState = APP_ST_START;
}
/*****************************************************************************
*
* Function: void AppTransStartToRunFast(void)
*
* Description: Transition function of application state machine
*
* Returns: None
*
* Global Data:
*
* Arguments: None
*
*****************************************************************************/
void AppTransStartToRunFast(void)
{
	ui8MotorState = APP_ST_RUN;
}
/*****************************************************************************
*
* Function: void AppTransRunToStopFast(void)
*
* Description: Transition function of application state machine
*
* Returns: None
*
* Global Data:
*
* Arguments: None
*
*****************************************************************************/
void AppTransRunToStopFast(void)
{
	ui8MotorState = APP_ST_STOP;
}
/*****************************************************************************
*
* Function: void AppTransStopToErrorFast(void)
*
* Description: Transition function of application state machine
*
* Returns: None
*
* Global Data:
*
* Arguments: None
*
*****************************************************************************/
void AppTransStopToErrorFast(void)
{
	ui8MotorState = APP_ST_ERROR;
}
/*****************************************************************************
*
* Function: void AppTransRunToErrorFast(void)
*
* Description: Transition function of application state machine
*
* Returns: None
*
* Global Data:
*
* Arguments: None
*
*****************************************************************************/
void AppTransRunToErrorFast(void)
{
	ui8MotorState = APP_ST_ERROR;
}
/*****************************************************************************
*
* Function: void AppTransErrorToStopFast(void)
*
* Description: Transition function of application state machine
*
* Returns: None
*
* Global Data:
*
* Arguments: None
*
*****************************************************************************/
void AppTransErrorToStopFast(void)
{
	ui8MotorState = APP_ST_STOP;
}

/*****************************************************************************
*
* Function: void AppInitSlow(void)
*
* Description: State function of application state machine
*
* Returns: None
*
* Global Data:
*
* Arguments: None
*
*****************************************************************************/
void AppInitSlow(void)
{

}
/*****************************************************************************
*
* Function: void AppRunSlow(void)
*
* Description: State function of application state machine
*
* Returns: None
*
* Global Data:
*
* Arguments: None
*
*****************************************************************************/
void AppRunSlow(void)
{

}
/*****************************************************************************
*
* Function: void AppStopSlow(void)
*
* Description: State function of application state machine
*
* Returns: None
*
* Global Data:
*
* Arguments: None
*
*****************************************************************************/
void AppStopSlow(void)
{

}
/*****************************************************************************
*
* Function: void AppStartSlow(void)
*
* Description: State function of application state machine
*
* Returns: None
*
* Global Data:
*
* Arguments: None
*
*****************************************************************************/
void AppStartSlow(void)
{

}
/*****************************************************************************
*
* Function: void AppErrorSlow(void)
*
* Description: State function of application state machine
*
* Returns: None
*
* Global Data:
*
* Arguments: None
*
*****************************************************************************/
void AppErrorSlow(void)
{

}

/*****************************************************************************
*
* Function: bool_t Calibration(sAdcResult *f16InputRawValue, sAdcResult *f16OutputOffsetValue, uint16_t *ui16CountInput, uint16_t ui16CalibLoop)
*
* Description: Offset calibration function
*
* Returns:
* True 	- calibration done
* False	- calibration in progress
*
* Global Data:
* sFilterParamOffsetIa				- structure of parameters for GDFLIB_FilterMA_F16
* sFilterParamOffsetIb				- structure of parameters for GDFLIB_FilterMA_F16
* sFilterParamOffsetIc				- structure of parameters for GDFLIB_FilterMA_F16
* sFilterParamOffsetIdcb			- structure of parameters for GDFLIB_FilterMA_F16
*
* Arguments:
* sAdcResult *f16InputRawValue		- pointer to raw adc result structure
* sAdcResult *f16OutputOffsetValue	- pointer to offset structure
* uint16_t *ui16CountInput			- counter variable
* uint16_t ui16CalibLoop			- counter limit value
*
*****************************************************************************/
bool_t Calibration(sAdcResult *f16InputRawValue, sAdcResult *f16OutputOffsetValue, uint16_t *ui16CountInput, uint16_t ui16CalibLoop)
{

	/* Filter offset values */
	if (*ui16CountInput < ui16CalibLoop )
	{
		/* Average Idcb offset */
		f16OutputOffsetValue->f16Ia   = GDFLIB_FilterMA_F16(f16InputRawValue->f16Ia, &sFilterParamOffsetIa);
		f16OutputOffsetValue->f16Ib   = GDFLIB_FilterMA_F16(f16InputRawValue->f16Ib, &sFilterParamOffsetIb);
		f16OutputOffsetValue->f16Ic   = GDFLIB_FilterMA_F16(f16InputRawValue->f16Ic, &sFilterParamOffsetIc);
		f16OutputOffsetValue->f16Idcb = GDFLIB_FilterMA_F16(f16InputRawValue->f16Idcb, &sFilterParamOffsetIdcb);
		/* Inc calibration counter */
		(*ui16CountInput)++;
        return FALSE;
	}
	else
	{
		/* SET DONE FLAG */
        return TRUE;
	}
}

/*****************************************************************************
*
* Function: void AdcGetResults(void)
*
* Description: State function of application state machine
*
* Returns: None
*
* Global Data:
* ui32Fcount0		    - number of results in FIFO0
* sAdcRaw.ui32Ia	    - raw 32-bit result with extended data
* sAdcRaw.ui32Ib	    - raw 32-bit result with extended data
* sAdcRaw.ui32Ic	    - raw 32-bit result with extended data
* sAdcRaw.ui32Vdcb	    - raw 32-bit result with extended data
* sAdcRaw.ui32Idcb	    - raw 32-bit result with extended data
* sAdcResRaw.f16Ia	    - raw 16-bit fractional data without extended data
* sAdcResRaw.f16Ib	    - raw 16-bit fractional data without extended data
* sAdcResRaw.f16Ic	    - raw 16-bit fractional data without extended data
* sAdcResRaw.f16Vdcb    - raw 16-bit fractional data without extended data
* sAdcResRaw.f16Idcb    - raw 16-bit fractional data without extended data
* sAdcOffset.f16Ia	    - external offset calculated in calibration function
* sAdcOffset.f16Ib	    - external offset calculated in calibration function
* sAdcOffset.f16Ic	    - external offset calculated in calibration function
* sAdcOffset.f16Idcb    - external offset calculated in calibration function
* sAdcRes.f16Ia		    - final value without offset
* sAdcRes.f16Ib		    - final value without offset
* sAdcRes.f16Ic		    - final value without offset
* sAdcRes.f16Idcb	    - final value without offset
* sAdcRes.f16Vdcb	    - final value
*
* Arguments: None
*
*****************************************************************************/
void AdcGetResults(void)
{
	/* Get number of results in FIFOs */
	ui32Fcount0 = ADC0->FCTRL & ADC_FCTRL_FCOUNT_MASK;

	/* Read and store dual conversion A command 1 */
	sAdcRaw.ui32Ia = ADC0->RESFIFO;
	sAdcResRaw.f16Ia = (frac16_t)(sAdcRaw.ui32Ia & ADC_RESFIFO_D_MASK);
	/* Remove DC offset (around 1/2 scale) and shift to full scale */
	sAdcRes.f16Ia = MLIB_Sh1L_F16(sAdcResRaw.f16Ia - sAdcOffset.f16Ia);

	/* Read and store dual conversion A command 2 */
	sAdcRaw.ui32Ib = ADC0->RESFIFO;
	sAdcResRaw.f16Ib = (frac16_t)(sAdcRaw.ui32Ib & ADC_RESFIFO_D_MASK);
	/* Remove DC offset (around 1/2 scale) and shift to full scale */
    sAdcRes.f16Ib = MLIB_Sh1L_F16(sAdcResRaw.f16Ib - sAdcOffset.f16Ib);

	/* Read and store single conversion A command 3 */
	sAdcRaw.ui32Ic = ADC0->RESFIFO;
	sAdcResRaw.f16Ic = (frac16_t)(sAdcRaw.ui32Ic & ADC_RESFIFO_D_MASK);
	/* Remove DC offset (around 1/2 scale) and shift to full scale */
	sAdcRes.f16Ic = MLIB_Sh1L_F16(sAdcResRaw.f16Ic - sAdcOffset.f16Ic);

	/* Read and store single conversion A command 4 */
	sAdcRaw.ui32Vdcb = ADC0->RESFIFO;
	sAdcResRaw.f16Vdcb = (frac16_t)(sAdcRaw.ui32Vdcb & ADC_RESFIFO_D_MASK);
	sAdcRes.f16Vdcb = sAdcResRaw.f16Vdcb;

	/* Read and store single conversion A command 5 */
	sAdcRaw.ui32Idcb = ADC0->RESFIFO;
	sAdcResRaw.f16Idcb = (frac16_t)(sAdcRaw.ui32Idcb & ADC_RESFIFO_D_MASK);
	/* Remove DC offset (around 1/2 scale) and shift to full scale */
	sAdcRes.f16Idcb = MLIB_Sh1L_F16(sAdcResRaw.f16Idcb - sAdcOffset.f16Idcb);
}

/*****************************************************************************
*
* Function: bool_t OverCurrentFault(PWM_Type *pPWMBase)
*
* Fault state reading from PWM module
*
* Returns:
* True 	- Fault
* False	- No fault
*
* Global Data:
*
* Arguments:
* PWM_Type *pPWMBase	- pointer to PWM module base address
*
*****************************************************************************/
static bool_t OverCurrentFault(PWM_Type *pPWMBase)
{
    bool_t bOCFaultRet;

    /* Read over-current flags */
    bOCFaultRet = (((pPWMBase->FSTS & PWM_FSTS_FFPIN_MASK) >> 8) & (1 << 0U));
    /* Clear faults flag */
    pPWMBase->FSTS = ((pPWMBase->FSTS & ~(uint16_t)(PWM_FSTS_FFLAG_MASK)) | (1 << 0U));

    return bOCFaultRet;
}

/*****************************************************************************
*
* Function: void UpdatePWM(frac16_t f16PhA, frac16_t f16PhB, frac16_t f16PhC, PWM_Type *pPWMBase)
*
* PWM update function
*
* Returns: None
*
* Global Data: None
*
* Arguments:
* frac16_t f16PhA		- duty cycle A
* frac16_t f16PhB		- duty cycle B
* frac16_t f16PhC		- duty cycle C
* PWM_Type *pPWMBase	- pointer to PWM module base address
*
*****************************************************************************/
static void UpdatePWM(frac16_t f16PhA, frac16_t f16PhB, frac16_t f16PhC, PWM_Type *pPWMBase)
{
    /* PWM duty cycles calculation */
    uint16_t ui16Modulo;
    frac16_t f16DutyCycle;

    /* Modulo read from PWM val1 register */
    ui16Modulo = pPWMBase->SM[0].VAL1 + 1;

    /* Phase A - duty cycle calculation */
    f16DutyCycle = MLIB_Mul_F16(ui16Modulo, f16PhA);
    pPWMBase->SM[0].VAL2 = (uint16_t)MLIB_Neg_F16(f16DutyCycle);
    pPWMBase->SM[0].VAL3 = (uint16_t)f16DutyCycle;

    /* Phase B - duty cycle calculation */
    f16DutyCycle = MLIB_Mul_F16(ui16Modulo, f16PhB);
    pPWMBase->SM[1].VAL2 = (uint16_t)MLIB_Neg_F16(f16DutyCycle);
    pPWMBase->SM[1].VAL3 = (uint16_t)f16DutyCycle;

    /* Phase C - duty cycle calculation */
    f16DutyCycle = MLIB_Mul_F16(ui16Modulo, f16PhC);
    pPWMBase->SM[2].VAL2 = (uint16_t)MLIB_Neg_F16(f16DutyCycle);
    pPWMBase->SM[2].VAL3 = (uint16_t)f16DutyCycle;

    /* Set LDOK bit for all sub0, sub1, sub2 and sub3 */
    pPWMBase->MCTRL |= PWM_MCTRL_LDOK(15);
}

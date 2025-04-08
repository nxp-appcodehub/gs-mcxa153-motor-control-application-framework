/*
 * Copyright 2021, 2025 NXP
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef MCXA153_LOWLEVEL_ADC_H_
#define MCXA153_LOWLEVEL_ADC_H_

#include "fsl_device_registers.h"


#define  CMDL_CTYPE_SINGLE_ENDED_A		0b00	/* Single-Ended mode. Only A-side channel is converted.*/

#define  CMDL_MODE_STANDARD				0b0		/* Standard resolution. Single-ended 12-bit conversion; differential 13-bit conversion with 2's complement output.*/
#define  CMDL_MODE_HIRES				0b1     /* High resolution. Single-ended 16-bit conversion; differential 16-bit conversion with 2's complement output.*/
#define  CMDL_MODE						CMDL_MODE_STANDARD


#define  CMDH_SAMPLETIME_3_5				0U
#define  CMDH_SAMPLETIME_5_5				1U
#define  CMDH_SAMPLETIME_7_5				2U
#define  CMDH_SAMPLETIME_11_5				3U
#define  CMDH_SAMPLETIME_19_5				4U
#define  CMDH_SAMPLETIME_35_5				5U
#define  CMDH_SAMPLETIME_67_5				6U
#define  CMDH_SAMPLETIME_131_5				7U

#define  CMDH_AVERAGE_1						0U
#define  CMDH_AVERAGE_2						1U
#define  CMDH_AVERAGE_4						2U
#define  CMDH_AVERAGE_8						3U
#define  CMDH_AVERAGE_16					4U
#define  CMDH_AVERAGE_32					5U
#define  CMDH_AVERAGE_64					6U
#define  CMDH_AVERAGE_128					7U
#define  CMDH_AVERAGE_256					8U
#define  CMDH_AVERAGE_512					9U
#define  CMDH_AVERAGE_1024					10U

#define  ADC_FIFO_RESULTS				5U		/* Set expected number of results in FIFO */

#define  ACT_CMD1 		0U
#define  ACT_CMD2 		1U
#define  ACT_CMD3 		2U
#define  ACT_CMD4 		3U
#define  ACT_CMD5 		4U
#define  ACT_CMD6 		5U
#define  ACT_CMD7 		6U

#define  NEXT_CMD_NONE 	0U
#define  NEXT_CMD1 		1U
#define  NEXT_CMD2 		2U
#define  NEXT_CMD3 		3U
#define  NEXT_CMD4 		4U
#define  NEXT_CMD5 		5U
#define  NEXT_CMD6 		6U
#define  NEXT_CMD7 		7U

#endif /* MCXA153_LOWLEVEL_ADC_H_ */

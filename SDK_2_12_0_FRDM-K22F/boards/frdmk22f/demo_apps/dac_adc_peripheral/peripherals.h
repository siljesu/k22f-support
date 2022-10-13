/*
 * Copyright 2019 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/***********************************************************************************************************************
 * This file was generated by the MCUXpresso Config Tools. Any manual edits made to this file
 * will be overwritten if the respective MCUXpresso Config Tools is used to update this file.
 **********************************************************************************************************************/

#ifndef _PERIPHERALS_H_
#define _PERIPHERALS_H_

/***********************************************************************************************************************
 * Included files
 **********************************************************************************************************************/
#include "fsl_common.h"
#include "fsl_adc16.h"
#include "fsl_dac.h"

#if defined(__cplusplus)
extern "C" {
#endif /* __cplusplus */

/***********************************************************************************************************************
 * Definitions
 **********************************************************************************************************************/
/* Definitions for BOARD_InitPeripherals functional group */
/* Alias for ADC0 peripheral */
#define BOARD_ADC16_PERIPHERAL ADC0
/* ADC16 interrupt vector ID (number). */
#define BOARD_ADC16_IRQN ADC0_IRQn
/* ADC16 interrupt handler identifier. */
#define BOARD_ADC16_IRQ_HANDLER_FUNC ADC0_IRQHandler
/* Alias for DAC0 peripheral */
#define BOARD_DAC_PERIPHERAL DAC0

/***********************************************************************************************************************
 * Global variables
 **********************************************************************************************************************/
extern adc16_channel_config_t BOARD_ADC16_channelsConfig[1];
extern const adc16_config_t BOARD_ADC16_config;
extern const adc16_channel_mux_mode_t BOARD_ADC16_muxMode;
extern const adc16_hardware_average_mode_t BOARD_ADC16_hardwareAverageMode;
extern const dac_config_t BOARD_DAC_config;

/***********************************************************************************************************************
 * Initialization functions
 **********************************************************************************************************************/
void BOARD_InitPeripherals(void);

/***********************************************************************************************************************
 * BOARD_InitBootPeripherals function
 **********************************************************************************************************************/
void BOARD_InitBootPeripherals(void);

#if defined(__cplusplus)
}
#endif

#endif /* _PERIPHERALS_H_ */

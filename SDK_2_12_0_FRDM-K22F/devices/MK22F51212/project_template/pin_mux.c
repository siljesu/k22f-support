/*
 * Copyright 2019 ,2021 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/***********************************************************************************************************************
 * This file was generated by the MCUXpresso Config Tools. Any manual edits made to this file
 * will be overwritten if the respective MCUXpresso Config Tools is used to update this file.
 **********************************************************************************************************************/

/* clang-format off */
/*
 * TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
!!GlobalInfo
product: Pins v9.0
processor: MK22FN512xxx12
package_id: MK22FN512VLH12
mcu_data: ksdk2_0
processor_version: 9.0.0
board: FRDM-K22F
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS ***********
 */
/* clang-format on */

#include "fsl_common.h"
#include "fsl_port.h"
#include "fsl_gpio.h"
#include "pin_mux.h"

/* FUNCTION ************************************************************************************************************
 *
 * Function Name : BOARD_InitBootPins
 * Description   : Calls initialization functions.
 *
 * END ****************************************************************************************************************/
void BOARD_InitBootPins(void)
{
    BOARD_InitPins();
}

/* clang-format off */
/*
 * TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
BOARD_InitPins:
- options: {callFromInitBoot: 'true', prefix: BOARD_, coreID: core0, enableClock: 'true'}
- pin_list:
  - {pin_num: '23', peripheral: GPIOA, signal: 'GPIO, 1', pin_signal: PTA1/UART0_RX/FTM0_CH6/JTAG_TDI/EZP_DI, direction: OUTPUT}
  - {pin_num: '24', peripheral: TPIU, signal: SWO, pin_signal: PTA2/UART0_TX/FTM0_CH7/JTAG_TDO/TRACE_SWO/EZP_DO, pull_select: down, pull_enable: disable}
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS ***********
 */
/* clang-format on */

/* FUNCTION ************************************************************************************************************
 *
 * Function Name : BOARD_InitPins
 * Description   : Configures pin routing and optionally pin electrical features.
 *
 * END ****************************************************************************************************************/
void BOARD_InitPins(void)
{
    /* Port A Clock Gate Control: Clock enabled */
    CLOCK_EnableClock(kCLOCK_PortA);
    CLOCK_EnableClock(kCLOCK_PortD);

    gpio_pin_config_t LEDRGB_RED_config = {
        .pinDirection = kGPIO_DigitalOutput,
        .outputLogic = 0U
    };
    gpio_pin_config_t LEDRGB_BLUE_config = {
        .pinDirection = kGPIO_DigitalOutput,
        .outputLogic = 0U
    };
    gpio_pin_config_t LEDRGB_GREEN_config = {
        .pinDirection = kGPIO_DigitalOutput,
        .outputLogic = 0U
    };
    /* Initialize GPIO functionality on pin PTA1 (pin 23)  */
    GPIO_PinInit(BOARD_LEDRGB_RED_GPIO, BOARD_LEDRGB_RED_PIN, &LEDRGB_RED_config);

    /* PORTA1 (pin 23) is configured as PTA1 */
    PORT_SetPinMux(BOARD_LEDRGB_RED_PORT, BOARD_LEDRGB_RED_PIN, kPORT_MuxAsGpio);

    /* Initialize GPIO functionality on pin PTD5 (number 62) */
    GPIO_PinInit(BOARD_LEDRGB_BLUE_GPIO, BOARD_LEDRGB_BLUE_PIN, &LEDRGB_BLUE_config);

    /* PORTD5 (number 62) is configured as PTD5 */
    PORT_SetPinMux(BOARD_LEDRGB_BLUE_PORT, BOARD_LEDRGB_BLUE_PIN, kPORT_MuxAsGpio);

    /* Initialize GPIO functionality on pin PTA2 */
    GPIO_PinInit(BOARD_LEDRGB_GREEN_GPIO, BOARD_LEDRGB_GREEN_PIN, &LEDRGB_GREEN_config);

    /* PORTA2  is configured as PTA2 */
    PORT_SetPinMux(BOARD_LEDRGB_GREEN_PORT, BOARD_LEDRGB_GREEN_PIN, kPORT_MuxAsGpio);

    PORTA->PCR[2] = ((PORTA->PCR[2] &
                      /* Mask bits to zero which are setting */
                      (~(PORT_PCR_PS_MASK | PORT_PCR_PE_MASK | PORT_PCR_ISF_MASK)))

                     /* Pull Select: Internal pulldown resistor is enabled on the corresponding pin, if the
                      * corresponding PE field is set. */
                     | PORT_PCR_PS(kPORT_PullDown)

                     /* Pull Enable: Internal pullup or pulldown resistor is not enabled on the corresponding pin. */
                     | PORT_PCR_PE(kPORT_PullDisable));
}
/***********************************************************************************************************************
 * EOF
 **********************************************************************************************************************/

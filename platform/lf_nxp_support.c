#include <stdlib.h> // Defines malloc.
#include <string.h> // Defines memcpy.
#include <assert.h>
#include <stdarg.h>
#include <stdio.h>

#include "lf_nxp_support.h"
#include "../platform.h"
#include "../utils/util.h"

#include "../../../../SDK_2_12_0_FRDM-K22F/devices/MK22F51212/project_template/pin_mux.h"
#include "../../../../SDK_2_12_0_FRDM-K22F/devices/MK22F51212/project_template/clock_config.h"
#include "../../../../SDK_2_12_0_FRDM-K22F/devices/MK22F51212/project_template/board.h"
#include "../../../../SDK_2_12_0_FRDM-K22F/devices/MK22F51212/drivers/fsl_pit.h"
#include "../../../../SDK_2_12_0_FRDM-K22F/devices/MK22F51212/drivers/fsl_port.h"
#include "../../../../SDK_2_12_0_FRDM-K22F/devices/MK22F51212/drivers/fsl_smc.h"

#define PIT_BASEADDR PIT
#define SMC_BASEADDR SMC
#define PIT0_CHANNEL  kPIT_Chnl_0
#define PIT1_CHANNEL  kPIT_Chnl_1
#define PIT0_OVERFLOW_HANDLER   PIT0_IRQHandler
#define PIT1_TIMER_HANDLER   PIT1_IRQHandler
#define PIT0_IRQ_ID        PIT0_IRQn
#define PIT1_IRQ_ID        PIT1_IRQn

/* Get source clock for PIT driver */
#define PIT_SOURCE_CLOCK CLOCK_GetFreq(kCLOCK_BusClk)

#define ACTION_NOTIFY_GPIO             GPIOA
#define ACTION_NOTIFY_GPIO_PORT        PORTA
#define ACTION_NOTIFY_GPIO_PIN         10
#define ACTION_NOTIFY_GPIO_IRQ         PORTA_IRQn
#define ACTION_NOTIFY_GPIO_IRQ_HANDLER PORTA_IRQHandler
#define ACTION_NOTIFY_GPIO_IRQ_TYPE    kPORT_InterruptEitherEdge

#define MAX_TICKS UINT32_MAX
#define COMBINE_HI_LO(hi,lo) ((((uint64_t) hi) << 32) | ((uint64_t) lo))

static volatile uint32_t _lf_time_us_high = 0;
static volatile uint32_t regPrimask;
volatile bool _lf_sleep_completed = false;

void PIT0_OVERFLOW_HANDLER(void)
{
    /* Clear interrupt flag.*/
    PIT_ClearStatusFlags(PIT_BASEADDR, PIT0_CHANNEL, kPIT_TimerFlag);

    /* Increment highest 32 bit clock value */
    _lf_time_us_high += 1;
    
    /* Added for, and affects, all PIT handlers. For CPU clock which is much larger than the IP bus clock,
     * CPU can run out of the interrupt handler before the interrupt flag being cleared, resulting in the
     * CPU's entering the handler again and again. Adding DSB can prevent the issue from happening.
     */
    __DSB();
}

void PIT1_TIMER_HANDLER(void)
{
    /* Clear interrupt flag.*/
    PIT_ClearStatusFlags(PIT_BASEADDR, PIT1_CHANNEL, kPIT_TimerFlag);

    /* Set sleep completed flag */
    _lf_sleep_completed = 1;
    
    /* Added for, and affects, all PIT handlers. For CPU clock which is much larger than the IP bus clock,
     * CPU can run out of the interrupt handler before the interrupt flag being cleared, resulting in the
     * CPU's entering the handler again and again. Adding DSB can prevent the issue from happening.
     */
    __DSB();
}

void ACTION_NOTIFY_GPIO_IRQ_HANDLER(void)
{
    if ((1U << ACTION_NOTIFY_GPIO_PIN) & PORT_GetPinsInterruptFlags(ACTION_NOTIFY_GPIO_PORT))
    {
        PORT_ClearPinsInterruptFlags(ACTION_NOTIFY_GPIO_PORT, (1U << ACTION_NOTIFY_GPIO_PIN));
    }
    /* Add for ARM errata 838869, affects Cortex-M4, Cortex-M4F Store immediate overlapping
    exception return operation might vector to incorrect interrupt */
    __DSB();
}

int lf_critical_section_enter(){
    regPrimask = DisableGlobalIRQ();
    return 0;
}

int lf_critical_section_exit(){
    EnableGlobalIRQ(regPrimask);
    return 0;
}

int lf_notify_of_event(){
    
    /* Generate action notification interrupt by toggling GPIO pin*/
    GPIO_PortToggle(ACTION_NOTIFY_GPIO, 1U << ACTION_NOTIFY_GPIO_PIN);

    return 0;
}

void lf_initialize_clock(void){
    pit_config_t pitConfig;
    pitConfig.enableRunInDebug = true;

    /* Init board hardware. */
    BOARD_InitBootPins();
    BOARD_InitBootClocks();
    BOARD_InitDebugConsole();

    /* Setup LF clock countdown timer */
    PIT_Init(PIT_BASEADDR, &pitConfig);
    PIT_SetTimerPeriod(PIT_BASEADDR, PIT0_CHANNEL, MAX_TICKS);
    PIT_EnableInterrupts(PIT_BASEADDR, PIT0_CHANNEL, kPIT_TimerInterruptEnable);
    EnableIRQ(PIT0_IRQ_ID);

    /* Setup sleep countdown timer */
    PIT_EnableInterrupts(PIT_BASEADDR, PIT1_CHANNEL, kPIT_TimerInterruptEnable);
    EnableIRQ(PIT1_IRQ_ID);

    /* Setup action notification interrupt */
    NVIC_EnableIRQ(ACTION_NOTIFY_GPIO_IRQ);
    PORT_SetPinInterruptConfig(ACTION_NOTIFY_GPIO_PORT, ACTION_NOTIFY_GPIO_PIN, ACTION_NOTIFY_GPIO_IRQ_TYPE);

    /* Start counter for LF clock */
    PIT_StartTimer(PIT_BASEADDR, PIT0_CHANNEL);
}

int lf_clock_gettime(instant_t* t){

    /* Does the argument reference invalid memory? */
    if (t == NULL) {
        return -1;
    }

    /* Read tick value, ans substract since PIT counts down*/
    uint32_t now_us_hi_pre = _lf_time_us_high;
    uint32_t ticks = MAX_TICKS - PIT_GetCurrentTimerCount(PIT_BASEADDR, PIT0_CHANNEL);
    uint32_t now_us_hi_post = _lf_time_us_high;

    if (now_us_hi_pre != now_us_hi_post) {
        /* Count overflowed while reading, read new value */
        ticks = MAX_TICKS - PIT_GetCurrentTimerCount(PIT_BASEADDR, PIT0_CHANNEL);
    }

    /* Combine the two counter values */
    uint64_t time_us = COMBINE_HI_LO(_lf_time_us_high, COUNT_TO_USEC(ticks, PIT_SOURCE_CLOCK));

    /* Convert to nanoseconds */
    *t = ((instant_t)time_us) * 1000U;

    return 0;
}

int lf_sleep(interval_t sleep_duration){

    /* Initialize sleep completed to false */
    _lf_sleep_completed = false;

    lf_critical_section_exit();

    /* Setup and start sleep countdown timer */
    PIT_SetTimerPeriod(PIT_BASEADDR, PIT1_CHANNEL, USEC_TO_COUNT(sleep_duration/1000LL, PIT_SOURCE_CLOCK));
    PIT_StartTimer(PIT_BASEADDR, PIT1_CHANNEL);

    /* Enter low power mode (Wait mode), exit upon any interrupt */
    SMC_PreEnterWaitModes();
    SMC_SetPowerModeWait(SMC_BASEADDR);
    SMC_PostExitWaitModes();

    /* Recover normal power mode (Run mode) */
    SMC_SetPowerModeRun(SMC_BASEADDR);
    while (kSMC_PowerStateRun != SMC_GetPowerModeState(SMC))
    {
    }

    /* Disable timer */
    PIT_StopTimer(PIT_BASEADDR, PIT1_CHANNEL);

    lf_critical_section_enter();

    if (_lf_sleep_completed) {
        /* Sleep was completed */
        return 0;
    } else {
        /* Sleep was interrupted by some interrupt */
        return -1;
    }
}

int lf_sleep_until(instant_t wakeup_time) {

    instant_t now;
    lf_clock_gettime(&now);
    interval_t duration = wakeup_time - now;

    return lf_sleep(duration);    
}


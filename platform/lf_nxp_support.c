#include <stdlib.h> // Defines malloc.
#include <string.h> // Defines memcpy.
#include <assert.h>
#include <stdarg.h>
#include <stdio.h>

#include "lf_nxp_support.h"
#include "../platform.h"
#include "../utils/util.h"

#include "../../../../../SDK_2_12_0_FRDM-K22F/devices/MK22F51212/project_template/pin_mux.h"
#include "../../../../../SDK_2_12_0_FRDM-K22F/devices/MK22F51212/project_template/clock_config.h"
#include "../../../../../SDK_2_12_0_FRDM-K22F/devices/MK22F51212/project_template/board.h"
#include "../../../../../SDK_2_12_0_FRDM-K22F/devices/MK22F51212/drivers/fsl_pit.h"
#include "../../../../../SDK_2_12_0_FRDM-K22F/devices/MK22F51212/drivers/fsl_port.h"
#include "../../../../../SDK_2_12_0_FRDM-K22F/devices/MK22F51212/drivers/fsl_smc.h"

#define PIT_BASEADDR PIT
#define SMC_BASEADDR SMC
#define PIT0_CHANNEL  kPIT_Chnl_0
#define PIT1_CHANNEL  kPIT_Chnl_1
#define PIT2_CHANNEL  kPIT_Chnl_2
#define PIT0_TIMER_HANDLER   PIT0_IRQHandler
#define PIT0_IRQ_ID        PIT0_IRQn

/* Get source clock for PIT driver */
#define PIT_SOURCE_CLOCK CLOCK_GetFreq(kCLOCK_BusClk)

#define MAX_TICKS UINT32_MAX
#define COMBINE_HI_LO(hi,lo) ((((uint64_t) hi) << 32) | ((uint64_t) lo))

static volatile uint32_t regPrimask;
volatile bool _lf_sleep_completed = false;

void PIT0_TIMER_HANDLER(void)
{
    /* Clear interrupt flag.*/
    PIT_ClearStatusFlags(PIT_BASEADDR, PIT0_CHANNEL, kPIT_TimerFlag);

    /* Set sleep completed flag */
    _lf_sleep_completed = 1;
    
    /* Added for, and affects, all PIT handlers. For CPU clock which is much larger than the IP bus clock,
     * CPU can run out of the interrupt handler before the interrupt flag being cleared, resulting in the
     * CPU's entering the handler again and again. Adding DSB can prevent the issue from happening.
     */
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
    return 0;
}

void lf_initialize_clock(void){
    pit_config_t pitConfig;
    pitConfig.enableRunInDebug = true;
    PIT_Init(PIT_BASEADDR, &pitConfig);  

    /* Init board hardware. */
    BOARD_InitBootPins();
    BOARD_InitBootClocks();
    BOARD_InitDebugConsole();

    /* Setup sleep countdown timer */  
    PIT_EnableInterrupts(PIT_BASEADDR, PIT0_CHANNEL, kPIT_TimerInterruptEnable);
    EnableIRQ(PIT0_IRQ_ID);

    /* Initialize two chained 32-bit counters on channel 1 and 2 */
    PIT_SetTimerPeriod(PIT_BASEADDR, PIT1_CHANNEL, MAX_TICKS);
    PIT_SetTimerPeriod(PIT_BASEADDR, PIT2_CHANNEL, MAX_TICKS);
    PIT_SetTimerChainMode(PIT_BASEADDR, PIT2_CHANNEL, true);

    /* Start counters for LF clock */
    PIT_StartTimer(PIT_BASEADDR, PIT2_CHANNEL);
    PIT_StartTimer(PIT_BASEADDR, PIT1_CHANNEL);
    
}

int lf_clock_gettime(instant_t* t){

    /* Does the argument reference invalid memory? */
    if (t == NULL) {
        return -1;
    }

    /* Read tick value, ans substract since PIT counts down*/
    uint32_t ticks_high_pre = (MAX_TICKS - 1) - PIT_GetCurrentTimerCount(PIT_BASEADDR, PIT2_CHANNEL);
    uint32_t ticks_low = (MAX_TICKS - 1) - PIT_GetCurrentTimerCount(PIT_BASEADDR, PIT1_CHANNEL);
    uint32_t ticks_high = (MAX_TICKS - 1) - PIT_GetCurrentTimerCount(PIT_BASEADDR, PIT2_CHANNEL);

    if (ticks_high_pre != ticks_high) {
        /* Count overflowed while reading, read new value */
        ticks_low = MAX_TICKS - PIT_GetCurrentTimerCount(PIT_BASEADDR, PIT1_CHANNEL);
    }

    /* Combine the two counter values */
    uint64_t total_ticks = COMBINE_HI_LO(ticks_high, ticks_low);

    /* Convert to nanoseconds */
    *t = ((instant_t)COUNT_TO_USEC(total_ticks, PIT_SOURCE_CLOCK)) * 1000U;

    return 0;
}

int lf_sleep(interval_t sleep_duration){

    /* Initialize sleep completed to false */
    _lf_sleep_completed = false;

    lf_critical_section_exit();

    /* Setup and start sleep countdown timer */
    PIT_SetTimerPeriod(PIT_BASEADDR, PIT0_CHANNEL, USEC_TO_COUNT(sleep_duration/1000LL, PIT_SOURCE_CLOCK));
    PIT_StartTimer(PIT_BASEADDR, PIT0_CHANNEL);

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
    PIT_StopTimer(PIT_BASEADDR, PIT0_CHANNEL);

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


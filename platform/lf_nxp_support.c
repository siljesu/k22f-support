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

#define PIT_BASEADDR PIT
#define PIT_CHANNEL  kPIT_Chnl_0
#define PIT_OVERFLOW_HANDLER   PIT0_IRQHandler
#define PIT_IRQ_ID        PIT0_IRQn
/* Get source clock for PIT driver */
#define PIT_SOURCE_CLOCK CLOCK_GetFreq(kCLOCK_BusClk)

#define MAX_TICKS UINT32_MAX
#define COMBINE_HI_LO(hi,lo) ((((uint64_t) hi) << 32) | ((uint64_t) lo))

static volatile uint32_t _lf_time_us_high = 0;
static volatile uint32_t regPrimask;
// physical_action_occured_flag

void PIT_OVERFLOW_HANDLER(void)
{
    /* Clear interrupt flag.*/
    PIT_ClearStatusFlags(PIT_BASEADDR, PIT_CHANNEL, kPIT_TimerFlag);
    _lf_time_us_high += 1;
    
    /* Added for, and affects, all PIT handlers. For CPU clock which is much larger than the IP bus clock,
     * CPU can run out of the interrupt handler before the interrupt flag being cleared, resulting in the
     * CPU's entering the handler again and again. Adding DSB can prevent the issue from happening.
     */
    __DSB();
}

/**
 * Enter a critical section where logical time and the event queue are guaranteed
 * to not change unless they are changed within the critical section.
 * In platforms with threading support, this normally will be implemented
 * by acquiring a mutex lock. In platforms without threading support,
 * this can be implemented by disabling interrupts.
 * Users of this function must ensure that lf_init_critical_sections() is
 * called first and that lf_critical_section_exit() is called later.
 * @return 0 on success, platform-specific error number otherwise.
 */
int lf_critical_section_enter(){
    regPrimask = DisableGlobalIRQ();
    return 0;
}

/**
 * Exit the critical section entered with lf_lock_time().
 * @return 0 on success, platform-specific error number otherwise.
 */
int lf_critical_section_exit(){
    EnableGlobalIRQ(regPrimask);
    return 0;
}

/**
 * Notify any listeners that an event has been created.
 * The caller should call lf_critical_section_enter() before calling this function.
 * @return 0 on success, platform-specific error number otherwise.
 */
int lf_notify_of_event(){
    return 0;
}

/**
 * Initialize the LF clock. Must be called before using other clock-related APIs.
 */
void lf_initialize_clock(void){
    pit_config_t pitConfig;
    pitConfig.enableRunInDebug = true;

    /* Init board hardware. */
    BOARD_InitBootPins();
    BOARD_InitBootClocks();
    BOARD_InitDebugConsole();

    //PIT_GetDefaultConfig(&pitConfig);
    PIT_Init(PIT_BASEADDR, &pitConfig);
    PIT_SetTimerPeriod(PIT_BASEADDR, PIT_CHANNEL, MAX_TICKS);
    PIT_EnableInterrupts(PIT_BASEADDR, PIT_CHANNEL, kPIT_TimerInterruptEnable);
    EnableIRQ(PIT_IRQ_ID);
    LF_PRINT_DEBUG("\r\nStarting channel No.0 ...");
    PIT_StartTimer(PIT_BASEADDR, PIT_CHANNEL);
}

/**
 * Fetch the value of an internal (and platform-specific) physical clock and 
 * store it in `t`.
 * 
 * Ideally, the underlying platform clock should be monotonic. However, the
 * core lib tries to enforce monotonicity at higher level APIs (see tag.h).
 * 
 * @return 0 for success, or -1 for failure
 */
int lf_clock_gettime(instant_t* t){
    if (t == NULL) {
        // The t argument address references invalid memory
        return -1;
    }
    uint32_t now_us_hi_pre = _lf_time_us_high;
    uint32_t ticks = MAX_TICKS - PIT_GetCurrentTimerCount(PIT_BASEADDR, PIT_CHANNEL); //timer is counting down
    uint32_t now_us_hi_post = _lf_time_us_high;

    if (now_us_hi_pre != now_us_hi_post) {
        //overflow occured, read new value
        ticks = MAX_TICKS - PIT_GetCurrentTimerCount(PIT_BASEADDR, PIT_CHANNEL); //timer is counting down
    }

    // ticks to us: macro from fsl_common_arm.h
    // us to ns by multiplying with 1000.
    // t is an instant, which is int64_t. How to convert?
    uint64_t time_us = COMBINE_HI_LO(_lf_time_us_high, COUNT_TO_USEC(ticks, PIT_SOURCE_CLOCK));
    *t = (instant_t)(time_us * 1000);
    return 0;
}

/**
 * Pause execution for a given number duration.
 * @return 0 if sleep was completed, or -1 if it was interrupted.
 */
int lf_sleep(interval_t sleep_duration){
    instant_t target_time;
    instant_t current_time;
    lf_clock_gettime(&current_time);
    target_time = current_time + sleep_duration;
    while (current_time <= target_time) {
        lf_clock_gettime(&current_time);
    }
    return 0;
}


/**
 * @brief Sleep until the given wakeup time.
 * 
 * @param wakeup_time The time instant at which to wake up.
 * @return int 0 if sleep completed, or -1 if it was interrupted.
 */
int lf_sleep_until(instant_t wakeup_time) {
    instant_t* t;
    lf_clock_gettime(t);
    interval_t duration = wakeup_time - *t;
    lf_sleep(duration);
    return 0;
}


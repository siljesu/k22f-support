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

// PTA10 as wakeup GPIO pin
#define WAKEUP_GPIO             GPIOA
#define WAKEUP_GPIO_PORT        PORTA
#define WAKEUP_GPIO_PIN         10
#define WAKEUP_GPIO_IRQ         PORTA_IRQn
#define WAKEUP_GPIO_IRQ_HANDLER PORTA_IRQHandler
#define WAKEUP_GPIO_IRQ_TYPE    kPORT_InterruptEitherEdge


#define MAX_TICKS UINT32_MAX
#define COMBINE_HI_LO(hi,lo) ((((uint64_t) hi) << 32) | ((uint64_t) lo))

static volatile uint32_t _lf_time_us_high = 0;
static volatile uint32_t regPrimask;
interval_t _lf_time_epoch_offset = 0LL;
volatile bool _lf_sleep_completed = false;

void PIT0_OVERFLOW_HANDLER(void)
{
    PRINTF("\r\nPIT0 overflow \r\n");
    /* Clear interrupt flag.*/
    PIT_ClearStatusFlags(PIT_BASEADDR, PIT0_CHANNEL, kPIT_TimerFlag);
    _lf_time_us_high += 1;
    
    /* Added for, and affects, all PIT handlers. For CPU clock which is much larger than the IP bus clock,
     * CPU can run out of the interrupt handler before the interrupt flag being cleared, resulting in the
     * CPU's entering the handler again and again. Adding DSB can prevent the issue from happening.
     */
    __DSB();
}

void PIT1_TIMER_HANDLER(void)
{
    PRINTF("\r\nPIT1 overflow \r\n");
    /* Clear interrupt flag.*/
    PIT_ClearStatusFlags(PIT_BASEADDR, PIT1_CHANNEL, kPIT_TimerFlag);
    _lf_sleep_completed = 1;
    
    /* Added for, and affects, all PIT handlers. For CPU clock which is much larger than the IP bus clock,
     * CPU can run out of the interrupt handler before the interrupt flag being cleared, resulting in the
     * CPU's entering the handler again and again. Adding DSB can prevent the issue from happening.
     */
    __DSB();
}

/*!
 * @brief PTA10 pin interrupt handler.
 */
void WAKEUP_GPIO_IRQ_HANDLER(void)
{
    if ((1U << WAKEUP_GPIO_PIN) & PORT_GetPinsInterruptFlags(WAKEUP_GPIO_PORT))
    {
        /* Disable interrupt. */
        //PORT_SetPinInterruptConfig(WAKEUP_GPIO_PORT, WAKEUP_GPIO_PIN, kPORT_InterruptOrDMADisabled);
        PORT_ClearPinsInterruptFlags(WAKEUP_GPIO_PORT, (1U << WAKEUP_GPIO_PIN));
    }
    /* Add for ARM errata 838869, affects Cortex-M4, Cortex-M4F Store immediate overlapping
    exception return operation might vector to incorrect interrupt */
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
    PRINTF("\r\nPhysical action \r\n");
    GPIO_PortToggle(WAKEUP_GPIO, 1U << WAKEUP_GPIO_PIN);
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

    //Init PIT0 which is used as a free running counter
    PIT_Init(PIT_BASEADDR, &pitConfig);
    PIT_SetTimerPeriod(PIT_BASEADDR, PIT0_CHANNEL, MAX_TICKS);
    PIT_EnableInterrupts(PIT_BASEADDR, PIT0_CHANNEL, kPIT_TimerInterruptEnable);
    EnableIRQ(PIT0_IRQ_ID);

    //Init PIT1 which is used for handling sleep interrupts
    PIT_SetTimerPeriod(PIT_BASEADDR, PIT1_CHANNEL, MAX_TICKS);
    PIT_EnableInterrupts(PIT_BASEADDR, PIT1_CHANNEL, kPIT_TimerInterruptEnable);
    EnableIRQ(PIT1_IRQ_ID);

    //Init GPIO pin interrupt
    NVIC_EnableIRQ(WAKEUP_GPIO_IRQ);
    PORT_SetPinInterruptConfig(WAKEUP_GPIO_PORT, WAKEUP_GPIO_PIN, WAKEUP_GPIO_IRQ_TYPE);

    PRINTF("\r\nStarting PIT channel with frequency: %u\r\n", PIT_SOURCE_CLOCK);
    PIT_StartTimer(PIT_BASEADDR, PIT0_CHANNEL);
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
    uint32_t ticks = MAX_TICKS - PIT_GetCurrentTimerCount(PIT_BASEADDR, PIT0_CHANNEL); //timer is counting down
    uint32_t now_us_hi_post = _lf_time_us_high;

    if (now_us_hi_pre != now_us_hi_post) {
        //overflow occured, read new value
        ticks = MAX_TICKS - PIT_GetCurrentTimerCount(PIT_BASEADDR, PIT0_CHANNEL); //timer is counting down
    }

    // ticks to us: macro from fsl_common_arm.h
    // us to ns by multiplying with 1000.
    // t is an instant, which is int64_t. How to convert?
    uint64_t time_us = COMBINE_HI_LO(_lf_time_us_high, COUNT_TO_USEC(ticks, PIT_SOURCE_CLOCK));
    *t = ((instant_t)time_us) * 1000U;
    //PRINTF("\r\nConverted finally to " PRINTF_TIME " ns (instant) \r\n", *t);

    return 0;
}

/**
 * Pause execution for a given number duration.
 * @return 0 if sleep was completed, or -1 if it was interrupted.
 */
int lf_sleep(interval_t sleep_duration){
    //PRINTF("Going to sleep...\r\n");
    instant_t target_time;
    instant_t current_time;
    lf_clock_gettime(&current_time);
    target_time = current_time + sleep_duration;
    return lf_sleep_until(target_time);
}


/**
 * @brief Sleep until the given wakeup time.
 * 
 * @param wakeup_time The time instant at which to wake up.
 * @return int 0 if sleep completed, or -1 if it was interrupted.
 */
int lf_sleep_until(instant_t wakeup_time) {
    _lf_sleep_completed = false;
    instant_t now;
    lf_clock_gettime(&now);
    interval_t duration = wakeup_time - now;

    lf_critical_section_exit();

    PIT_SetTimerPeriod(PIT_BASEADDR, PIT1_CHANNEL, USEC_TO_COUNT(duration/1000LL, PIT_SOURCE_CLOCK));
    PIT_StartTimer(PIT_BASEADDR, PIT1_CHANNEL);

    //low power mode - how does it exit? Any interrupt? Exit upon timer interrupt, which can originate from several sources.
    PRINTF("\r\nEntering wait mode \r\n");
    SMC_PreEnterWaitModes();
    SMC_SetPowerModeWait(SMC_BASEADDR);
    SMC_PostExitWaitModes();

    //while(!_lf_sleep_completed OR _lf_sleep_interrupted) {}

    SMC_SetPowerModeRun(SMC_BASEADDR);
    while (kSMC_PowerStateRun != SMC_GetPowerModeState(SMC))
    {
    }

    //disable timer
    PIT_StopTimer(PIT_BASEADDR, PIT1_CHANNEL);
    PRINTF("\r\nExited wait mode \r\n");
    lf_critical_section_enter();

    if (_lf_sleep_completed) {
        PRINTF("\r\nSleep completed \r\n");
        return 0;
    } else {
        PRINTF("\r\nSleep interrupted \r\n");
        return -1;
    }
}


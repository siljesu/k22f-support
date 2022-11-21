
#include <stdint.h> // For fixed-width integral types
#include <time.h>   // For CLOCK_MONOTONIC
#include <stdbool.h>
#include <inttypes.h>
#include "../../../SDK_2_12_0_FRDM-K22F/devices/MK22F51212/utilities/debug_console_lite/fsl_debug_console.h"

#define PRINTF_TIME "%" PRId64
#define PRINTF_MICROSTEP "%" PRIu32
#define PRINTF_TAG "(" PRINTF_TIME ", " PRINTF_MICROSTEP ")"
#define printf PRINTF
#define PLATFORM_NXP 1

/**
 * Time instant. Both physical and logical times are represented
 * using this typedef.
 */
typedef int64_t _instant_t;

/**
 * Interval of time.
 */
typedef int64_t _interval_t;

/**
 * Microstep instant.
 */
typedef uint32_t _microstep_t;


#define _LF_CLOCK CLOCK_MONOTONIC
# CROSS COMPILER SETTING
SET(CMAKE_SYSTEM_NAME Generic)
CMAKE_MINIMUM_REQUIRED (VERSION 3.10.0)

# THE VERSION NUMBER
SET (Tutorial_VERSION_MAJOR 1)
SET (Tutorial_VERSION_MINOR 0)

# ENABLE ASM
ENABLE_LANGUAGE(ASM)

SET(CMAKE_STATIC_LIBRARY_PREFIX)
SET(CMAKE_STATIC_LIBRARY_SUFFIX)

SET(CMAKE_EXECUTABLE_LIBRARY_PREFIX)
SET(CMAKE_EXECUTABLE_LIBRARY_SUFFIX)

# CURRENT DIRECTORY
SET(ProjDirPath ${CMAKE_CURRENT_SOURCE_DIR})

SET(EXECUTABLE_OUTPUT_PATH ${ProjDirPath}/${CMAKE_BUILD_TYPE})
SET(LIBRARY_OUTPUT_PATH ${ProjDirPath}/${CMAKE_BUILD_TYPE})


project(log_bm)

set(MCUX_SDK_PROJECT_NAME log_bm.elf)

include(${ProjDirPath}/flags.cmake)

include(${ProjDirPath}/config.cmake)

add_executable(${MCUX_SDK_PROJECT_NAME} 
"${ProjDirPath}/../log_main.c"
"${ProjDirPath}/../pin_mux.c"
"${ProjDirPath}/../pin_mux.h"
"${ProjDirPath}/../board.c"
"${ProjDirPath}/../board.h"
"${ProjDirPath}/../clock_config.c"
"${ProjDirPath}/../clock_config.h"
)

target_include_directories(${MCUX_SDK_PROJECT_NAME} PRIVATE
    ${ProjDirPath}/..
)

set(CMAKE_MODULE_PATH
    ${ProjDirPath}/../../../../../../devices/MK22F51212/drivers
    ${ProjDirPath}/../../../../../../devices/MK22F51212/utilities
    ${ProjDirPath}/../../../../../../components/lists
    ${ProjDirPath}/../../../../../../components/panic
    ${ProjDirPath}/../../../../../../components/serial_manager
    ${ProjDirPath}/../../../../../../components/osa
    ${ProjDirPath}/../../../../../../components/log
    ${ProjDirPath}/../../../../../../devices/MK22F51212
    ${ProjDirPath}/../../../../../../components/uart
    ${ProjDirPath}/../../../../../../CMSIS/Core/Include
)

# include modules
include(driver_gpio_MK22F51212)

include(utility_shell_MK22F51212)

include(component_lists_MK22F51212)

include(component_panic_MK22F51212)

include(component_serial_manager_MK22F51212)

include(driver_common_MK22F51212)

include(component_serial_manager_uart_MK22F51212)

include(component_osa_bm_MK22F51212)

include(component_log_MK22F51212)

include(component_log_backend_debugconsole_MK22F51212)

include(component_log_backend_ringbuffer_MK22F51212)

include(driver_clock_MK22F51212)

include(device_MK22F51212_CMSIS_MK22F51212)

include(utility_debug_console_MK22F51212)

include(component_uart_adapter_MK22F51212)

include(driver_uart_MK22F51212)

include(device_MK22F51212_startup_MK22F51212)

include(driver_lpuart_MK22F51212)

include(driver_port_MK22F51212)

include(driver_smc_MK22F51212)

include(utility_assert_MK22F51212)

include(CMSIS_Include_core_cm_MK22F51212)

include(utilities_misc_utilities_MK22F51212)

include(device_MK22F51212_system_MK22F51212)


TARGET_LINK_LIBRARIES(${MCUX_SDK_PROJECT_NAME} PRIVATE -Wl,--start-group)

target_link_libraries(${MCUX_SDK_PROJECT_NAME} PRIVATE m)

target_link_libraries(${MCUX_SDK_PROJECT_NAME} PRIVATE c)

target_link_libraries(${MCUX_SDK_PROJECT_NAME} PRIVATE gcc)

target_link_libraries(${MCUX_SDK_PROJECT_NAME} PRIVATE nosys)

TARGET_LINK_LIBRARIES(${MCUX_SDK_PROJECT_NAME} PRIVATE -Wl,--end-group)



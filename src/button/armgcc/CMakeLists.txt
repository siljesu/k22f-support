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
SET(ProjSDKPath ${ProjDirPath}/../../../SDK_2_12_0_FRDM-K22F)
SET(ProjName button)

SET(EXECUTABLE_OUTPUT_PATH ${ProjDirPath}/${CMAKE_BUILD_TYPE})
SET(LIBRARY_OUTPUT_PATH ${ProjDirPath}/${CMAKE_BUILD_TYPE})


project(${ProjName})

set(MCUX_SDK_PROJECT_NAME ${ProjName}.elf)

include(${ProjDirPath}/flags.cmake)

include(${ProjDirPath}/config.cmake)

include_directories(${ProjDirPath}/../../../src-gen/${ProjName}/${ProjName}/core)
include_directories(${ProjDirPath}/../../../src-gen/${ProjName}/${ProjName}/core/platform)
include_directories(${ProjDirPath}/../../../src-gen/${ProjName}/${ProjName}/core/federated)
set(LF_MAIN_TARGET ${ProjName})
# Declare a new executable target and list all its sources

add_executable(${MCUX_SDK_PROJECT_NAME}
"${ProjDirPath}/../../../src-gen/${ProjName}/${ProjName}/board.c"
"${ProjDirPath}/../../../src-gen/${ProjName}/${ProjName}/clock_config.c"
"${ProjDirPath}/../../../src-gen/${ProjName}/${ProjName}/pin_mux.c"
"${ProjDirPath}/../../../src-gen/${ProjName}/${ProjName}/board.h"
"${ProjDirPath}/../../../src-gen/${ProjName}/${ProjName}/clock_config.h"
"${ProjDirPath}/../../../src-gen/${ProjName}/${ProjName}/pin_mux.h"
"${ProjDirPath}/../../../src-gen/${ProjName}/${ProjName}/${ProjName}.c"
"${ProjDirPath}/../../../src-gen/${ProjName}/${ProjName}/core/platform/lf_nxp_support.c"
"${ProjDirPath}/../../../src-gen/${ProjName}/${ProjName}/lib/schedule.c"
"${ProjDirPath}/../../../src-gen/${ProjName}/${ProjName}/lib/util.c"
"${ProjDirPath}/../../../src-gen/${ProjName}/${ProjName}/lib/tag.c"
"${ProjDirPath}/../../../src-gen/${ProjName}/${ProjName}/lib/time.c"
"${ProjDirPath}/../../../src-gen/${ProjName}/${ProjName}/core/mixed_radix.c"
)

target_include_directories(${MCUX_SDK_PROJECT_NAME} PRIVATE
${ProjDirPath}/../../../src-gen/${ProjName}/${ProjName}/core
)

set(CMAKE_MODULE_PATH
${ProjSDKPath}/
${ProjSDKPath}/CMSIS/Core/Include
${ProjSDKPath}/components/lists
${ProjSDKPath}/components/uart
${ProjSDKPath}/devices/MK22F51212
${ProjSDKPath}/devices/MK22F51212/drivers
${ProjSDKPath}/devices/MK22F51212/startup
${ProjSDKPath}/devices/MK22F51212/utilities
${ProjSDKPath}/devices/MK22F51212/utilities/debug_console_lite
)

# include modules
include(utility_debug_console_lite_MK22F51212)

include(utility_assert_lite_MK22F51212)

include(driver_gpio_MK22F51212)

include(driver_clock_MK22F51212)

include(driver_common_MK22F51212)

include(device_MK22F51212_CMSIS_MK22F51212)

include(component_uart_adapter_MK22F51212)

include(component_lists_MK22F51212)

include(driver_uart_MK22F51212)

include(device_MK22F51212_startup_MK22F51212)

include(driver_lpuart_MK22F51212)

include(driver_port_MK22F51212)

include(driver_smc_MK22F51212)

include(CMSIS_Include_core_cm_MK22F51212)

include(utilities_misc_utilities_MK22F51212)

include(device_MK22F51212_system_MK22F51212)

include(driver_pit_MK22F51212)


TARGET_LINK_LIBRARIES(${MCUX_SDK_PROJECT_NAME} PRIVATE -Wl,--start-group)

target_link_libraries(${MCUX_SDK_PROJECT_NAME} PRIVATE m)

target_link_libraries(${MCUX_SDK_PROJECT_NAME} PRIVATE c)

target_link_libraries(${MCUX_SDK_PROJECT_NAME} PRIVATE gcc)

target_link_libraries(${MCUX_SDK_PROJECT_NAME} PRIVATE nosys)

TARGET_LINK_LIBRARIES(${MCUX_SDK_PROJECT_NAME} PRIVATE -Wl,--end-group)



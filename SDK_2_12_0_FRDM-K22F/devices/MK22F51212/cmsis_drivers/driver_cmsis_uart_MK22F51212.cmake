include_guard()
message("driver_cmsis_uart component is included.")

target_sources(${MCUX_SDK_PROJECT_NAME} PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}/fsl_uart_cmsis.c
)


target_include_directories(${MCUX_SDK_PROJECT_NAME} PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}/.
)


include(driver_uart_edma_MK22F51212)

include(driver_uart_MK22F51212)

include(CMSIS_Driver_Include_USART_MK22F51212)


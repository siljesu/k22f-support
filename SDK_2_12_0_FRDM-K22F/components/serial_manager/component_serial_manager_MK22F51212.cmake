include_guard()
message("component_serial_manager component is included.")

target_sources(${MCUX_SDK_PROJECT_NAME} PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}/fsl_component_serial_manager.c
)


target_include_directories(${MCUX_SDK_PROJECT_NAME} PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}/.
)


#OR Logic component
if(CONFIG_USE_component_serial_manager_uart_MK22F51212)
     include(component_serial_manager_uart_MK22F51212)
endif()
if(CONFIG_USE_component_serial_manager_usb_cdc_MK22F51212)
     include(component_serial_manager_usb_cdc_MK22F51212)
endif()
if(CONFIG_USE_component_serial_manager_virtual_MK22F51212)
     include(component_serial_manager_virtual_MK22F51212)
endif()
if(CONFIG_USE_component_serial_manager_swo_MK22F51212)
     include(component_serial_manager_swo_MK22F51212)
endif()
if(CONFIG_USE_component_serial_manager_rpmsg_MK22F51212)
     include(component_serial_manager_rpmsg_MK22F51212)
endif()
if(CONFIG_USE_component_serial_manager_spi_MK22F51212)
     include(component_serial_manager_spi_MK22F51212)
endif()
if(NOT (CONFIG_USE_component_serial_manager_uart_MK22F51212 OR CONFIG_USE_component_serial_manager_usb_cdc_MK22F51212 OR CONFIG_USE_component_serial_manager_virtual_MK22F51212 OR CONFIG_USE_component_serial_manager_swo_MK22F51212 OR CONFIG_USE_component_serial_manager_rpmsg_MK22F51212 OR CONFIG_USE_component_serial_manager_spi_MK22F51212))
    message(WARNING "Since component_serial_manager_uart_MK22F51212/component_serial_manager_usb_cdc_MK22F51212/component_serial_manager_virtual_MK22F51212/component_serial_manager_swo_MK22F51212/component_serial_manager_rpmsg_MK22F51212/component_serial_manager_spi_MK22F51212 is not included at first or config in config.cmake file, use component_serial_manager_uart_MK22F51212 by default.")
    include(component_serial_manager_uart_MK22F51212)
endif()

include(driver_common_MK22F51212)

include(component_lists_MK22F51212)

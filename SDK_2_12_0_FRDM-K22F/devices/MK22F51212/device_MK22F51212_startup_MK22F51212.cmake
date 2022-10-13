include_guard()
message("device_MK22F51212_startup component is included.")

target_sources(${MCUX_SDK_PROJECT_NAME} PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}/gcc/startup_MK22F51212.S
)


include(device_MK22F51212_system_MK22F51212)


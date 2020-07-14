###############################################################################
# USB On-the-Go Driver Library
###############################################################################
set(TARGET_NAME usb-stm32f4)
set(TARGET_SRC
    InEndpointViaSTM32F4.cpp
    OutEndpointViaSTM32F4.cpp
    UsbCoreViaSTM32F4.cpp
    UsbDeviceViaSTM32F4.cpp
    # FIXME This doesn't work for some reason.
    # UsbIrqViaSTM32F4.cpp 
)

if(${STM32F4_HAVE_USB_OTG})
    message(STATUS "STM32F4 CPU ${STM32F4_CPU_TYPE} supports USB On-the-Go (OTG) Macro")

    add_library(${TARGET_NAME} ${TARGET_SRC})
    target_link_libraries(${TARGET_NAME}
        stm32f4
        usb
    )
    target_include_directories(${TARGET_NAME} PRIVATE
    )
    target_include_directories(${TARGET_NAME} PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
    )
else()
    add_library(${TARGET_NAME} INTERFACE)
    target_include_directories(${TARGET_NAME} INTERFACE
        ${CMAKE_CURRENT_SOURCE_DIR}/include
    )
    target_link_libraries(${TARGET_NAME} INTERFACE
        stm32f4
        usb
    )
endif()

#*******************************************************************************
# Testcases
#*******************************************************************************
if (UNITTEST)
    MESSAGE(STATUS "Building ${TARGET_NAME} Testcases")
    add_subdirectory(testcases)
endif(UNITTEST)

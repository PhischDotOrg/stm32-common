###############################################################################
# USB Library
###############################################################################
set(TARGET_NAME usb-stm32f4)
set(TARGET_SRC
    InEndpointViaSTM32F4.cpp
    OutEndpointViaSTM32F4.cpp
    UsbCoreViaSTM32F4.cpp
    UsbDeviceViaSTM32F4.cpp
)
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

#*******************************************************************************
# Testcases
#*******************************************************************************
if (UNITTEST)
    MESSAGE(STATUS "Building ${TARGET_NAME} Testcases")
    add_subdirectory(testcases)
endif(UNITTEST)

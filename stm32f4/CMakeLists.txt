###############################################################################
# stm32f4
###############################################################################
set(TARGET_NAME stm32f4)
set(TARGET_SRC
    # AdcViaSTM32F4.cpp
    DmaChannelViaSTM32F4.cpp
    DmaEngineViaSTM32F4.cpp
    DmaStreamViaSTM32F4.cpp
    FlashViaSTM32F4.cpp
    GpioAccessViaSTM32F4.cpp
    NvicViaSTM32F4.cpp
    PwrViaSTM32F4.cpp
    RccViaSTM32F4.cpp
    ScbViaSTM32F4.cpp
    # SpiAccessViaSTM32F4.cpp
    TimerChannelViaSTM32F4.cpp
    TimerViaSTM32F4.cpp
    UartAccessViaSTM32F4.cpp
)
add_library(${TARGET_NAME} ${TARGET_SRC})
target_link_libraries(${TARGET_NAME}
    cmsis
    FreeRTOS
    gpio
    uart
    usb-stm32f4
)
target_include_directories(${TARGET_NAME} PRIVATE
)
target_include_directories(${TARGET_NAME} PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/include/stm32f4
)

###############################################################################
# usb
###############################################################################
add_subdirectory(usb)

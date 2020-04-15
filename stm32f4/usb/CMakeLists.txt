###############################################################################
# USB Library
###############################################################################
set(TARGET_NAME usb)
set(TARGET_SRC
    UsbCore.cpp
    UsbDevice.cpp
)
add_library(${TARGET_NAME} ${TARGET_SRC})
target_link_libraries(${TARGET_NAME}
    stm32f4
)
target_include_directories(${TARGET_NAME} PRIVATE
)
target_include_directories(${TARGET_NAME} PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

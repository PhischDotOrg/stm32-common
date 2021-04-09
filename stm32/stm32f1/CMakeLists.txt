###############################################################################
# stm32f1
###############################################################################
set(TARGET_NAME stm32f1)
set(TARGET_SRC
)
add_library(${TARGET_NAME} INTERFACE
    ${TARGET_SRC}
)
target_link_libraries(${TARGET_NAME} INTERFACE
    cmsis
    FreeRTOS
    gpio
    uart
    usb-stm32f1
)
target_include_directories(${TARGET_NAME} PRIVATE
)
target_include_directories(${TARGET_NAME} INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/include/stm32f1
)

###############################################################################
# usb
###############################################################################
add_subdirectory(usb)

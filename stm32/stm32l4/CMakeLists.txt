###############################################################################
# stm32l4
###############################################################################
set(TARGET_NAME stm32l4)
set(TARGET_SRC
)
add_library(${TARGET_NAME} INTERFACE
    ${TARGET_SRC}
)
target_link_libraries(${TARGET_NAME} INTERFACE
    cmsis
    FreeRTOS
    gpio
    stm32f4
    uart
#    usb-stm32f4
)
target_include_directories(${TARGET_NAME} PRIVATE
)
target_include_directories(${TARGET_NAME} INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/include/stm32l4
)

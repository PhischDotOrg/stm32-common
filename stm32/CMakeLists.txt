###############################################################################
# stm32
###############################################################################
string(TOLOWER ${STM32_CPU_FAMILY} STM32_CPU_LIBRARY)
add_subdirectory(${STM32_CPU_LIBRARY})

set(TARGET_NAME stm32)
add_library(${TARGET_NAME} INTERFACE)
target_include_directories(${TARGET_NAME} INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)
target_link_libraries(${TARGET_NAME} INTERFACE
    ${STM32_CPU_LIBRARY}
)

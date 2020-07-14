###############################################################################
# Start-up Code
###############################################################################
set(TARGET_NAME     startup)

enable_language(ASM)

if (CMAKE_CROSSCOMPILING)
    add_library(${TARGET_NAME} 
        ${STM32F4_STARTUP_CODE}    
    )
    target_compile_options(${TARGET_NAME} PRIVATE
        -mcpu=cortex-m4
        -mthumb
    )
    target_link_options(${TARGET_NAME} INTERFACE
        -nostartfiles
    )
else()
    add_library(${TARGET_NAME} 
        UnitTestDummyStartUp.cpp
    )
endif()

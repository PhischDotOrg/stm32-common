###############################################################################
# Start-up Code
###############################################################################
set(TARGET_NAME     startup)
set(STARTUP_OBJ     ${TARGET_NAME}.o)

add_custom_command(OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/${STARTUP_OBJ}
    COMMAND ${CMAKE_C_COMPILER} ARGS
      -c -o ${CMAKE_CURRENT_BINARY_DIR}/${STARTUP_OBJ}
      -nostartfiles -mcpu=cortex-m4 -mthumb
      ${CMAKE_CURRENT_SOURCE_DIR}/${STM32F4_STARTUP_CODE}
    DEPENDS
     ${CMAKE_CURRENT_SOURCE_DIR}/${STM32F4_STARTUP_CODE}
     ${CMAKE_CURRENT_SOURCE_DIR}/${STM32F4_LDSCRIPT}
)

add_custom_target(${TARGET_NAME}
    COMMAND true
    DEPENDS ${CMAKE_CURRENT_BINARY_DIR}/${STARTUP_OBJ}
)

###############################################################################
# Firmware image
###############################################################################
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${STARTUP_OBJ} -T ${CMAKE_CURRENT_SOURCE_DIR}/${STM32F4_LDSCRIPT}")

target_link_libraries(${TARGET_ELF}
    cmsis
)
add_dependencies(${TARGET_ELF}
    startup
)

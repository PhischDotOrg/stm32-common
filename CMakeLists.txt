#-
# $Copyright$
#
project(${PROJECT_NAME})

###############################################################################
# Print some generic information on the Project.
###############################################################################
set(PROJECT_ROOT "${CMAKE_SOURCE_DIR}")
message(STATUS "Project Name: ${PROJECT_NAME}")
message(STATUS "Project Root: ${PROJECT_ROOT}")
message(STATUS "Target System Type: ${CMAKE_SYSTEM_NAME}")
message(STATUS "Target System Processor: ${CMAKE_SYSTEM_PROCESSOR}")
message(STATUS "Host System: ${CMAKE_HOST_SYSTEM} @ ${CMAKE_HOST_SYSTEM_PROCESSOR}")

###############################################################################
# Set up generic compile flags. If we're on a 'Generic', i.e. embedded system,
# we need the CMAKE_SYSTEM_PROCESSOR variable set via the Toolchain file.
###############################################################################
if ("${CMAKE_SYSTEM_NAME}" STREQUAL "Generic")
    if (NOT DEFINED CMAKE_SYSTEM_PROCESSOR)
        message(FATAL_ERROR "CMAKE_SYSTEM_PROCESSOR not defined!")
    endif()
endif()

if ("${CMAKE_SYSTEM_NAME}" STREQUAL "Generic")
    set(ARCH_NAME "${CMAKE_SYSTEM_PROCESSOR}")
else()
    set(ARCH_NAME "${CMAKE_SYSTEM_NAME}")
endif()
set_property(GLOBAL PROPERTY ARCH_NAME ${ARCH_NAME})

###############################################################################
# STM32F4 Supported Boards
###############################################################################
list(APPEND STM32F4_SUPPORTED_BOARDS "STM32F4_Discovery")
list(APPEND STM32F4_SUPPORTED_BOARDS "STM32F4_Nucleo_F411RE")

if(NOT DEFINED STM32F4_BOARD)
    set(STM32F4_BOARD "" CACHE STRING "STM32F4 Board Type. Must be set to one of ${STM32F4_SUPPORTED_BOARDS}")
endif()

if(("${STM32F4_BOARD}" STREQUAL "")
  OR NOT ("${STM32F4_BOARD}" IN_LIST STM32F4_SUPPORTED_BOARDS))
    message(FATAL_ERROR "STM32F4_BOARD is set to \"${STM32F4_BOARD}\" which is "
      "not in the list of supported Boards. Please configure STM32F4 Board "
      "(Cache Variable \"STM32F4_BOARD\") to one of [${STM32F4_SUPPORTED_BOARDS}]")
endif()

# Configure Board-specific Options
include(boards/${STM32F4_BOARD}/stm32f4-board.cmake)

###############################################################################
# If we're on an embedded system, include a CMake File that sets up the
# architecture specific flags for the Compiler.
###############################################################################
if ("${CMAKE_SYSTEM_NAME}" STREQUAL "Generic")
    include(${CMAKE_SYSTEM_NAME}_${CMAKE_SYSTEM_PROCESSOR}.cflags)
endif()

add_definitions("-Wall")
add_definitions("-Wextra")
add_definitions("-Werror")

if("${CMAKE_BUILD_TYPE}" STREQUAL "Release")
    add_definitions("-O3")
elseif("${CMAKE_BUILD_TYPE}" STREQUAL "RelWithDebInfo")
    add_definitions("-O3")
    add_definitions("-g")
elseif("${CMAKE_BUILD_TYPE}" STREQUAL "MinSizeRel")
    add_definitions("-Os")
else()
    add_definitions("-O0")
    add_definitions("-g")
endif()

if ((NOT "${CMAKE_BUILD_TYPE}" STREQUAL "")
  AND NOT ("${CMAKE_BUILD_TYPE}" STREQUAL "Debug"))
    # For some reason, -Os and -O3 optimizes things in a way that cause the
    # compiler to throw a bunch of warnings about unused parameters and/or
    # variables.  Degrade those to a warning here so we can still compile.
    add_definitions("-Wno-error=unused-parameter")
    add_definitions("-Wno-error=unused-variable")
endif()

###############################################################################
# Set up the environment for the Phisch.Org libraries
###############################################################################
if ("${CMAKE_SYSTEM_NAME}" STREQUAL "Generic")
    if ("${CMAKE_SYSTEM_PROCESSOR}" STREQUAL "STM32F4")
        add_definitions("-DGPIO_ENV=${CMAKE_SYSTEM_PROCESSOR}")
        add_definitions("-DUART_ENV=${CMAKE_SYSTEM_PROCESSOR}")

        add_definitions("-DCORTEX_M4")
        add_definitions("-DUSE_FULL_ASSERT")
    endif()
endif()

###############################################################################
# Include Search Paths for project
###############################################################################
if ("${CMAKE_SYSTEM_NAME}" STREQUAL "Generic")
    include_directories(${CMAKE_CURRENT_SOURCE_DIR}/lib/compat/include)
endif()

###############################################################################
# Actually build the target binary
###############################################################################
set(TARGET_ELF ${TARGET_NAME}.elf)
set(TARGET_BIN ${TARGET_NAME}.bin)
add_executable(${TARGET_ELF}
    ${TARGET_SRC}
    FreeRTOSCallback.cpp
    CompatStdCpp.cpp
)
target_link_libraries(${TARGET_ELF}
    build_version
    cmsis
    compatc
    FreeRTOS
    stm32f4
    tasks
    uart
)

###############################################################################
# Include Library subdirectories
###############################################################################
add_subdirectory(contrib)
add_subdirectory(lib)
add_subdirectory(stm32f4)
add_subdirectory(tasks)

###############################################################################
# Include Build Version Support
###############################################################################
include(STM32F4_Archives.cmake)
include(STM32F4_BuildVersion.cmake)
include(STM32F4_OpenOCD.cmake)
include(STM32F4_Startup.cmake)

###############################################################################
# If we're on an embedded system, include a CMake File that defines how the
# flashable binary image is built
###############################################################################
if ("${CMAKE_SYSTEM_NAME}" STREQUAL "Generic")
    set(GDBINIT_FILE ${CMAKE_CURRENT_SOURCE_DIR}/gdbinit-${CMAKE_SYSTEM_PROCESSOR}.tpl)
else()
    find_package(Threads REQUIRED)
    target_link_libraries(${TARGET_ELF}
        ${CMAKE_THREAD_LIBS_INIT}
    )
    set(GDBINIT_FILE ${CMAKE_CURRENT_SOURCE_DIR}/gdbinit-native.tpl)
endif()
configure_file(${GDBINIT_FILE} ${CMAKE_CURRENT_BINARY_DIR}/.gdbinit)

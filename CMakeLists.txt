#-
# $Copyright$
#

###############################################################################
# Print some generic information on the Project.
###############################################################################
set(PROJECT_ROOT "${CMAKE_SOURCE_DIR}")
message(STATUS "Project Name: ${CMAKE_PROJECT_NAME}")
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

###############################################################################
# STM32 Supported Boards
#
# A board is supported if there is a directory with the board's name which
# contains a file "stm32-board.cmake".
###############################################################################
file(GLOB STM32_SUPPORTED_BOARDS
  LIST_DIRECTORIES TRUE
  RELATIVE ${CMAKE_CURRENT_SOURCE_DIR}/boards
  "boards/STM32*"
)

if(NOT DEFINED STM32_BOARD)
    set(STM32_BOARD "" CACHE STRING "STM32 Board Type. Must be set to one of ${STM32_SUPPORTED_BOARDS}")
endif()

if(NOT("${STM32_BOARD}" STREQUAL ""))
    set(BOARD_CMAKE_FILE "${CMAKE_CURRENT_SOURCE_DIR}/boards/${STM32_BOARD}/stm32-board.cmake")
    if (NOT EXISTS ${BOARD_CMAKE_FILE})
      message(FATAL_ERROR "STM32_BOARD is set to \"${STM32_BOARD}\" but file "
        "\'${BOARD_CMAKE_FILE}\' does not exist!")
    else()
        message(STATUS "STM32 Board: ${STM32_BOARD}")
        # Configure Board-specific Options
        include(${BOARD_CMAKE_FILE})

        add_definitions(-D${STM32_CPU_FAMILY})
        add_definitions(-D${STM32_CPU_TYPE})
    endif()
endif()

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
    add_definitions("-O2")
    add_definitions("-Og")
    add_definitions("-g")
elseif("${CMAKE_BUILD_TYPE}" STREQUAL "MinSizeRel")
    add_definitions("-Os")
    add_definitions("-g")
else()
    add_definitions("-O0")
    add_definitions("-g")
endif()

if ((NOT "${CMAKE_BUILD_TYPE}" STREQUAL "")
  AND NOT ("${CMAKE_BUILD_TYPE}" STREQUAL "Debug"))
    # For some reason, -Os and -O3 optimizes things in a way that cause the
    # compiler to throw a bunch of warnings about unused parameters and/or
    # variables.  Degrade those to a warning here so we can still compile.
    add_definitions("-Wno-error=unused-variable")
endif()

###############################################################################
# Set up the environment for the Phisch.Org libraries
###############################################################################
add_definitions("-DGPIO_ENV=STM32")

if ("${CMAKE_SYSTEM_NAME}" STREQUAL "Generic")
    add_definitions("-DHAVE_STM32")
    add_definitions("-DSTM32_CPU_FAMILY=${STM32_CPU_FAMILY}")
    add_definitions("-D${STM32_CPU_FAMILY}")

    add_definitions("-DUART_ENV=STM32")
else()
    add_definitions("-DHOSTBUILD")

    #    add_definitions("-DUART_ENV=${CMAKE_SYSTEM_NAME}")
endif()

###############################################################################
# Include Search Paths for project
###############################################################################
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

###############################################################################
# Actually build the target binary
###############################################################################
set(TARGET_ELF ${TARGET_NAME}.elf)
set(TARGET_BIN ${TARGET_NAME}.bin)
set(TARGET_SRC
    ${TARGET_SRC}
    CompatStdCpp.cpp
    FreeRTOSCallback.cpp
    Infrastructure.cpp
)
add_executable(${TARGET_ELF}
    ${TARGET_SRC}
)
target_link_libraries(${TARGET_ELF}
    build_version
    cmsis
    compatc
    FreeRTOS
    startup
    stm32
    tasks
    uart
)

###############################################################################
# Include Library subdirectories
###############################################################################
add_subdirectory(contrib)
add_subdirectory(lib)
add_subdirectory(stm32)
if(NOT("${STM32_BOARD}" STREQUAL ""))
    add_subdirectory(tasks)
endif()

###############################################################################
# Include Build Version Support
###############################################################################
include(STM32_Archives.cmake)
include(STM32_BuildVersion.cmake)
include(STM32_OpenOCD.cmake)
include(STM32_Startup.cmake)

if ("${CMAKE_SYSTEM_NAME}" STREQUAL "Generic")
    target_link_options(${TARGET_ELF} PRIVATE
        -T ${CMAKE_CURRENT_SOURCE_DIR}/${STM32_LDSCRIPT}
    )
endif()

include(Doxygen.cmake)

###############################################################################
# If we're on an embedded system, include a CMake File that defines how the
# flashable binary image is built
###############################################################################
if ("${CMAKE_SYSTEM_NAME}" STREQUAL "Generic")
    set(GDBINIT_FILE ${CMAKE_CURRENT_SOURCE_DIR}/gdbinit-stm32.tpl)
else()
    find_package(Threads REQUIRED)
    target_link_libraries(${TARGET_ELF}
        ${CMAKE_THREAD_LIBS_INIT}
    )
    set(GDBINIT_FILE ${CMAKE_CURRENT_SOURCE_DIR}/gdbinit-native.tpl)
endif()
configure_file(${GDBINIT_FILE} ${CMAKE_CURRENT_BINARY_DIR}/.gdbinit)

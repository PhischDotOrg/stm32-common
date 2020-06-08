#-
# $Copyright$
#

###############################################################################
# Enable C++17 Support according to:
#
# https://crascit.com/2015/03/28/enabling-cxx11-in-cmake/
#
# Google Test requires > C++11.
###############################################################################
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

###############################################################################
# Unit Test Support, defaults to Off.
###############################################################################
set(UNITTEST FALSE CACHE BOOL "Build Unit Tests, not the target code")

if(NOT ${UNITTEST})
    return()
endif()

message(STATUS "Building with Unit Tests")

set(COVERAGE ${UNITTEST} CACHE BOOL "Code Coverage Support")

###############################################################################
# Unit Test specific configuration
###############################################################################

# Enable CTest / "make test" Target
enable_testing()

# Pre-processor definition used in code to indicate that we're building with
# the host compiler.
add_definitions("-DHOSTBUILD")

if (${COVERAGE})
    message(STATUS "Building with Code Coverage Support")
    # Compiler / Linker Arguments to emit Code Coverage information
    set(COVERAGE_OPTIONS
        "-fprofile-arcs"
        "-ftest-coverage"
    )

    foreach(COVERAGE_OPTION IN LISTS COVERAGE_OPTIONS)
        message(STATUS "Adding Compile and Link Option \"${COVERAGE_OPTION}\"")
        add_definitions(${COVERAGE_OPTION})
        add_link_options(${COVERAGE_OPTION})   
    endforeach()
endif()

# The common/CMakeLists.txt file requires a "TARGET_NAME" variable to be set
# and wants to build some executable with that name.
#
# Satisfy that need.
set(TARGET_NAME     test-firmware)
set(TARGET_SRC
    ${CMAKE_CURRENT_SOURCE_DIR}/common/gtest-main.cpp
)   

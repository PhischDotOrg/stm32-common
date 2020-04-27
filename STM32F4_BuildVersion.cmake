#-
# $Copyright$
#

###############################################################################
# Generate Build Time Stamp and Build ID based on Git ID.
#
# Follows suggestion from https://stackoverflow.com/questions/24292898/compile-date-and-time-using-cmake
#
###############################################################################
set(BUILD_VERSION_CMAKE_FILE  ${CMAKE_CURRENT_BINARY_DIR}/build_version.cmake)
set(BUILD_VERSION_HEADER_FILE  ${CMAKE_CURRENT_BINARY_DIR}/build_version.h)

FILE(WRITE ${BUILD_VERSION_CMAKE_FILE}
 "STRING(TIMESTAMP BUILD_TIME UTC)\n"
)

FILE(APPEND ${BUILD_VERSION_CMAKE_FILE}
    "execute_process(COMMAND git describe --always --dirty=+\n"
    "   WORKING_DIRECTORY " ${CMAKE_CURRENT_SOURCE_DIR}/.. "\n"
    "   TIMEOUT 1\n"
    "   RESULT_VARIABLE   GIT_RESULT\n"
    "   OUTPUT_VARIABLE   GIT_OUTPUT\n"
    "   ERROR_VARIABLE    GIT_ERROR\n"
    "   OUTPUT_STRIP_TRAILING_WHITESPACE\n"
    ")\n"
    "\n"
    "if(NOT(\$\{GIT_RESULT\} EQUAL 0))\n"
    "   message(FATAL_ERROR \"Git Command returned rc=\$\{GIT_RESULT\}. Output: '\$\{GIT_OUTPUT\}'. Error: '\$\{GIT_ERROR\}'\")\n"
    "else()\n"
    "   set(GIT_VERSION \$\{GIT_OUTPUT\})\n"
    "   message(STATUS \"Git Build ID: \$\{GIT_VERSION\}\")\n"
    "endif()\n"
    "\n"
)

FILE (APPEND ${BUILD_VERSION_CMAKE_FILE}
    "FILE(WRITE ${BUILD_VERSION_HEADER_FILE}
        \"#ifndef BUILD_VERSION_H_6314F76B_6D2B_4792_995E_2AFB9AC31A03\\n\"
        \"#define BUILD_VERSION_H_6314F76B_6D2B_4792_995E_2AFB9AC31A03\\n\\n\"
        \"#define BUILD_TIME \\\"\${BUILD_TIME}\\\"\\n\\n\"
        \"#define BUILD_ID \\\"\${GIT_VERSION}\\\"\\n\\n\"
        \"#endif // BUILD_VERSION_H_6314F76B_6D2B_4792_995E_2AFB9AC31A03 \\n\"
    )\n"
)

ADD_CUSTOM_TARGET (
    build_version_files
    COMMAND ${CMAKE_COMMAND} -P ${BUILD_VERSION_CMAKE_FILE}
    ADD_DEPENDENCIES ${BUILD_VERSION_CMAKE_FILE})

SET(TARGET_NAME build_version)

ADD_LIBRARY(${TARGET_NAME} STATIC
    version.c
)
TARGET_INCLUDE_DIRECTORIES(${TARGET_NAME} BEFORE PRIVATE ${CMAKE_CURRENT_BINARY_DIR})
TARGET_INCLUDE_DIRECTORIES(${TARGET_NAME} INTERFACE ${CMAKE_CURRENT_SOURCE_DIR})
ADD_DEPENDENCIES(${TARGET_NAME} build_version_files)

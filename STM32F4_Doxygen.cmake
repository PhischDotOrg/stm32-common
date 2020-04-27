###############################################################################
#
###############################################################################
execute_process(COMMAND git describe --always --dirty=+
   WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/..
   TIMEOUT 5
   RESULT_VARIABLE   GIT_RESULT
   OUTPUT_VARIABLE   GIT_OUTPUT
   ERROR_VARIABLE    GIT_ERROR
   OUTPUT_STRIP_TRAILING_WHITESPACE
)

if(NOT(${GIT_RESULT} EQUAL 0))
   message(FATAL_ERROR "Git Command returned rc=${GIT_RESULT}. Output: '${GIT_OUTPUT}'. Error: '${GIT_ERROR}'")
else()
   set(GIT_VERSION ${GIT_OUTPUT})
   message(STATUS "Git Build ID: ${GIT_VERSION}")
endif()

###############################################################################
#
###############################################################################
set(DOYGEN_OUTPUT_DIRECTORY         ${CMAKE_CURRENT_BINARY_DIR}/doxygen)
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/../CMakeLists.txt)
    set(DOYGEN_TOPLEVEL_DIRECTORY   ${CMAKE_CURRENT_SOURCE_DIR}/..)
else()
    set(DOYGEN_TOPLEVEL_DIRECTORY   ${CMAKE_CURRENT_SOURCE_DIR})
endif()

set(DOXYGEN_EXECUTABLE doxygen)
set(DOXYGEN_SEARCH_PATH
		/usr
		/usr/local
		/opt
)

find_program(
	DOXYGEN_BIN
	${DOXYGEN_EXECUTABLE}
	PATHS ${DOXYGEN_SEARCH_PATH}
	PATH_SUFFIXES /bin /sbin
	DOC "Doxygen Executable Path"
)

if ((NOT DEFINED DOXYGEN_BIN) OR ("${DOXYGEN_BIN}" STREQUAL "DOXYGEN_BIN-NOTFOUND"))
    message(WARNING "Doxygen not found in \"${DOXYGEN_SEARCH_PATH}\". Doxygen will not work.")
else()
	message("Doxygen found at ${DOXYGEN_BIN}")

	message("Doxygen: Encoding Git Version: ${GIT_VERSION}")
	message("Doxygen: Top-level Directory: ${DOYGEN_TOPLEVEL_DIRECTORY}")
endif()

###############################################################################
#
###############################################################################
set(DOXYGEN_CONFIG_FILE STM32F4.doxyfile)

configure_file(${CMAKE_CURRENT_SOURCE_DIR}/${DOXYGEN_CONFIG_FILE} ${CMAKE_CURRENT_BINARY_DIR}/${DOXYGEN_CONFIG_FILE})

add_custom_target(doxygen
    COMMAND ${DOXYGEN_BIN} ${CMAKE_CURRENT_BINARY_DIR}/${DOXYGEN_CONFIG_FILE}
    WORKING_DIRECTORY ${DOYGEN_TOPLEVEL_DIRECTORY}
    COMMENT "Generating Doxygen Doxumentation"
)

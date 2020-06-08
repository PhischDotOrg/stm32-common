#-
# $Copyright$
#

if(NOT ${COVERAGE})
    return()
endif()

###############################################################################
# Find lcov
###############################################################################
set(LCOV_EXECUTABLE lcov)
set(LCOV_SEARCH_PATH
		/usr
		/usr/local
		/opt
)

find_program(
	LCOV_BIN
	${LCOV_EXECUTABLE}
	PATHS ${LCOV_SEARCH_PATH}
	PATH_SUFFIXES /bin /sbin
	DOC "lcov Executable Path"
)

if ((NOT DEFINED LCOV_BIN) OR ("${LCOV_BIN}" STREQUAL "LCOV_BIN-NOTFOUND"))
    message(WARNING "lcov not found in \"${LCOV_BIN}\". Coverage Report will not work.")

    return()
endif()

message("lcov found at ${LCOV_BIN}")

###############################################################################
# Find genhtml
###############################################################################
set(GENHTML_EXECUTABLE genhtml)
set(GENHTML_SEARCH_PATH
		/usr
		/usr/local
		/opt
)

find_program(
	GENHTML_BIN
	${GENHTML_EXECUTABLE}
	PATHS ${GENHTML_SEARCH_PATH}
	PATH_SUFFIXES /bin /sbin
	DOC "genhtml Executable Path"
)

if ((NOT DEFINED GENHTML_BIN) OR ("${GENHTML_BIN}" STREQUAL "GENHTML_BIN-NOTFOUND"))
    message(WARNING "genhtml not found in \"${GENHTML_BIN}\". Coverage Report will not work.")

    return()
endif()

message("genhtml found at ${LCOV_BIN}")

###############################################################################
# Targets to collect coverage info and generate HTML report
###############################################################################
set(COVERAGE_INFO "lcov.info")

add_custom_command(OUTPUT ${COVERAGE_INFO}
    COMMAND ${LCOV_BIN} ARGS
      --directory ${CMAKE_CURRENT_BINARY_DIR}
      --base-directory ${CMAKE_CURRENT_BINARY_DIR}
	  --capture
	  -o ${COVERAGE_INFO}
    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
    COMMENT "Collecting Coverage Info"
)


set(COVERAGE_REPORT_DIR "${CMAKE_CURREN_BINARY_DIR}/coverage")

add_custom_target(coverage
  COMMAND ${GENHTML_BIN} ${CMAKE_CURRENT_BINARY_DIR}/${COVERAGE_INFO} -o ${CMAKE_CURRENT_BINARY_DIR}/${COVERAGE_REPORT_DIR}
  DEPENDS ${COVERAGE_INFO}
  WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
  COMMENT "Generating Coverage Report"
)

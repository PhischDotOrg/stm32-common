#-
# $Copyright$
#

###############################################################################
# Determine Git Version at Configure Time, i.e. when CMake is run to generate
# the build tree.
###############################################################################
execute_process(COMMAND git describe --always --dirty=+
   WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
   TIMEOUT 5
   RESULT_VARIABLE   GIT_RESULT
   OUTPUT_VARIABLE   GIT_OUTPUT
   ERROR_VARIABLE    GIT_ERROR
   OUTPUT_STRIP_TRAILING_WHITESPACE
)

if(NOT(${GIT_RESULT} EQUAL 0))
   message(WARNING "Git Command returned rc=${GIT_RESULT}. Output: '${GIT_OUTPUT}'. Error: '${GIT_ERROR}'")
   set(GIT_VERSION "unknown")
else()
   set(GIT_VERSION ${GIT_OUTPUT})
   message(STATUS "Git Build ID (when CMake was run): ${GIT_VERSION}")
endif()
###############################################################################

set(VERSION_STRING ${CMAKE_PROJECT_VERSION_MAJOR})
if ("${CMAKE_PROJECT_VERSION_MINOR}")
   set(VERSION_STRING "${VERSION_STRING}.${CMAKE_PROJECT_VERSION_MINOR}")
endif()
if ("${CMAKE_PROJECT_VERSION_PATCH}")
   set(VERSION_STRING "${VERSION_STRING}.${CMAKE_PROJECT_VERSION_PATCH}")
endif()
set(VERSION_STRING "${VERSION_STRING}.${GIT_VERSION}")

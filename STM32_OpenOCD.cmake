###############################################################################
# Configuration Options for OpenOCD
###############################################################################
set(OPENOCD_EXECUTABLE openocd)
set(OPENOCD_SEARCH_PATH
		/usr
		/usr/local
		/opt
)

find_program(
	OPENOCD_BIN
	${OPENOCD_EXECUTABLE}
	PATHS ${OPENOCD_SEARCH_PATH}
	PATH_SUFFIXES /bin /sbin
	DOC "OpenOCD Executable Path"
)

if ((NOT DEFINED OPENOCD_BIN) OR ("${OPENOCD_BIN}" STREQUAL "OPENOCD_BIN-NOTFOUND"))
	message(WARNING "OpenOCD not found in \"${OPENOCD_SEARCH_PATH}\". OpenOCD will not work.")
    return()
endif()

message("OpenOCD found at ${OPENOCD_BIN}")

get_filename_component(OPENOCD_BINDIR ${OPENOCD_BIN} DIRECTORY)

set(OPENOCD_PREFIX "${OPENOCD_BINDIR}/.." CACHE PATH "OpenOCD Installation Prefix")

if((NOT DEFINED OPENOCD_BOARD_CFG_DIR) OR ("${OPENOCD_BOARD_CFG_DIR}" STREQUAL ""))
	if (${CMAKE_HOST_WIN32})
		set(OPENOCD_BOARD_CFG_DIR "${OPENOCD_PREFIX}/scripts/board"
			CACHE PATH "OpenOCD Board Configuration Directory")
	else()
		set(OPENOCD_BOARD_CFG_DIR "${OPENOCD_PREFIX}/share/openocd/scripts/board"
			CACHE PATH "OpenOCD Board Configuration Directory")
	endif()
endif()
if (NOT EXISTS ${OPENOCD_BOARD_CFG_DIR})
	message(WARNING "OpenOCD Board Configuration Directory OPENOCD_BOARD_CFG_DIR=\"${OPENOCD_BOARD_CFG_DIR}\" "
	"does not exist! OpenOCD may not work.")
endif()

if (IS_ABSOLUTE ${OPENOCD_BOARD_CFG})
	set(OPENOCD_CFG ${OPENOCD_BOARD_CFG})
else()
	set(OPENOCD_CFG ${OPENOCD_BOARD_CFG_DIR}/${OPENOCD_BOARD_CFG})
endif()

if (NOT EXISTS ${OPENOCD_CFG})
	message(WARNING "OpenOCD Board Configuration File \"${OPENOCD_CFG}\" does not exist! OpenOCD may not work.")
else()
	message("Using OpenOCD Configuration File ${OPENOCD_CFG}")
endif()

###############################################################################
# Add two pseudo-targets that start OpenOCD.
#  - flash: Will download the ELF file onto the target system
#  - gdbserver: Will start the OpenOCD GDB Server at Port 4242
###############################################################################
add_custom_target(flash
    ${OPENOCD_BIN} -f ${OPENOCD_CFG} -c "program ${TARGET_ELF} verify reset exit"
    DEPENDS ${TARGET_ELF}
    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
    COMMENT "Flashing ${TARGET_ELF} via OpenOCD using ${OPENOCD_BOARD_CFG}"
)

add_custom_target(gdbserver
    ${OPENOCD_BIN} -f ${OPENOCD_CFG} -c "gdb_port 4242" -c "telnet_port 4444"
    DEPENDS ${TARGET_ELF}
    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
    COMMENT "Starting OpenOCD GDB Server using ${OPENOCD_BOARD_CFG}"
)

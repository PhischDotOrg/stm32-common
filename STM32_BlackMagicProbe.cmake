set(BMP_DEVICE_FILE "/dev/cu.usbmodem4D81905D1")

###############################################################################
# Configuration Options for ARM GDB
###############################################################################
set(GDB_EXECUTABLE arm-none-eabi-gdb)
set(GDB_SEARCH_PATH
		/usr
		/usr/local
		/opt
)

find_program(
	GDB_BIN
	${GDB_EXECUTABLE}
	PATHS ${GDB_SEARCH_PATH}
	PATH_SUFFIXES /bin /sbin
	DOC "ARM GDB Executable Path"
)

if ((NOT DEFINED GDB_BIN) OR ("${GDB_BIN}" STREQUAL "GDB_BIN-NOTFOUND"))
	message(STATUS "ARM GDB not found in \"${GDB_SEARCH_PATH}\". Flashing with GDB will not work.")
	return()
endif()

message("ARM GDB found at ${GDB_BIN}")

###############################################################################
# Check if BlackMagicProbe UART Device exists.
###############################################################################
if (NOT EXISTS ${BMP_DEVICE_FILE})
	message(WARNING "BlackMagicProbe UART Device not found at \"${BMP_DEVICE_FILE}\". Flashing with GDB will not work.")
	return()
endif()

message("Using BlackMagicProbe UART Device at \"${BMP_DEVICE_FILE}\"")

###############################################################################
# Pseudo-targets that start interface with the BlackMagicProbe Device.
#  - flash: Will download the ELF file onto the target system
###############################################################################
add_custom_target(flash
	${GDB_BIN} -nx --batch -ex "target extended-remote ${BMP_DEVICE_FILE}" -ex "monitor swdp_scan" -ex "attach 1" -ex "load" -ex "compare-sections" -ex "kill" ${TARGET_ELF}
	DEPENDS ${TARGET_ELF}
	WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
	COMMENT "Flashing ${TARGET_ELF} via ARM GDB using ${BMP_DEVICE_FILE}"
)

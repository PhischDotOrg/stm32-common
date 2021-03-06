###############################################################################
# Build Binaries (.hex, .s19, .bin) for archiving purposes
###############################################################################
set(TARGET_HEX		${TARGET_NAME}.hex)
set(TARGET_S19		${TARGET_NAME}.s19)
set(TARGET_BIN		${TARGET_NAME}.bin)
set(TARGET_ASCII	${TARGET_NAME}.txt)
set(TARGET_STRIPPED	${TARGET_NAME}-stripped.elf)

add_custom_command(OUTPUT ${TARGET_STRIPPED}
	COMMAND cp ARGS ${TARGET_ELF} ${TARGET_STRIPPED}
	COMMAND arm-none-eabi-strip ARGS ${TARGET_STRIPPED}
	DEPENDS ${TARGET_ELF}
	WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
	COMMENT "Building stripped .elf File"
)

add_custom_command(OUTPUT ${TARGET_BIN}
	COMMAND arm-none-eabi-objcopy ARGS -O binary -S ${CMAKE_CURRENT_BINARY_DIR}/${TARGET_ELF} ${TARGET_BIN}
	DEPENDS ${TARGET_ELF}
	WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
	COMMENT "Building .bin File"
)

add_custom_command(OUTPUT ${TARGET_S19}
	COMMAND arm-none-eabi-objcopy ARGS -O srec -S ${CMAKE_CURRENT_BINARY_DIR}/${TARGET_ELF} ${TARGET_S19}
	DEPENDS ${TARGET_ELF}
	WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
	COMMENT "Building .s19 File"
)

add_custom_command(OUTPUT ${TARGET_HEX}
	COMMAND arm-none-eabi-objcopy ARGS -O ihex -S ${CMAKE_CURRENT_BINARY_DIR}/${TARGET_ELF} ${TARGET_HEX}
	DEPENDS ${TARGET_ELF}
	WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
	COMMENT "Building .hex File"
)

add_custom_command(OUTPUT ${TARGET_ASCII}
	COMMAND hexdump ARGS -C ${CMAKE_CURRENT_BINARY_DIR}/${TARGET_BIN} > ${CMAKE_CURRENT_BINARY_DIR}/${TARGET_ASCII}
	DEPENDS ${TARGET_BIN}
	WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
	COMMENT "Building ASCII Dump from .bin File"
	VERBATIM
)

add_custom_target(bin
		true
		DEPENDS ${TARGET_BIN} ${TARGET_S19} ${TARGET_HEX} ${TARGET_ASCII}
		WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
		COMMENT "Building Binary Files for archiving."
	)

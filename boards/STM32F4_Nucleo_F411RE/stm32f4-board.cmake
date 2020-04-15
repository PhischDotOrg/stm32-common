set(OPENOCD_BOARD_CFG       "st_nucleo_f4.cfg")
set(STM32F4_STARTUP_CODE    "boards/STM32F4_Nucleo_F411RE/startup_stm32f411xe.s")
set(STM32F4_LDSCRIPT        "boards/STM32F4_Nucleo_F411RE/STM32F411RE_FLASH.ld")

# Used to select include directory for CPU-/Board-specific Headers in common/contrib/STM32F4.cmake
set(STM32F4_CPU_TYPE        "STM32F411xE")

add_definitions("-DSTM32F411xE")
add_definitions("-DHSE_VALUE=((uint32_t)8000000)")
add_definitions("-DHSI_VALUE=((uint32_t)16000000)")

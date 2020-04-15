set(OPENOCD_BOARD_CFG       "st_nucleo_f4.cfg")
set(STM32F4_STARTUP_CODE    "startup_stm32f411xe.s")
set(STM32F4_LDSCRIPT        "STM32F411RE_FLASH.ld")

add_definitions("-DSTM32F411xE")
add_definitions("-DHSE_VALUE=((uint32_t)8000000)")
add_definitions("-DHSI_VALUE=((uint32_t)16000000)")


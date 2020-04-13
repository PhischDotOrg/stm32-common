set(OPENOCD_BOARD_CFG       "stm32f4discovery.cfg")
set(STM32F4_STARTUP_CODE    "startup_stm32f407xx.s")
set(STM32F4_LDSCRIPT        "STM32F407VGTx_FLASH.ld")

add_definitions("-DSTM32F407xx")
add_definitions("-DHSE_VALUE=((uint32_t)8000000)")
add_definitions("-DHSI_VALUE=((uint32_t)16000000)")

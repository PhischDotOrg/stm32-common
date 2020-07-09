set(OPENOCD_BOARD_CFG       "st_nucleo_f4.cfg")
set(STM32F4_STARTUP_CODE    "boards/STM32F4_Nucleo_F411RE/startup_stm32f411xe.s")
set(STM32F4_LDSCRIPT        "boards/STM32F4_Nucleo_F411RE/STM32F411RE_FLASH.ld")
set(STM32F4_CPU_TYPE        "STM32F411xE")

set(STM32F4_HAVE_USB_OTG    True)
add_definitions(-DSTM32F4_HAVE_USB_OTG)

add_definitions(-D${STM32F4_CPU_TYPE})

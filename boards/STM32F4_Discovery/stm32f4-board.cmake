set(OPENOCD_BOARD_CFG       "stm32f4discovery.cfg")
set(STM32F4_STARTUP_CODE    "boards/STM32F4_Discovery/startup_stm32f407xx.S")
set(STM32F4_LDSCRIPT        "boards/STM32F4_Discovery/STM32F407VGTx_FLASH.ld")
set(STM32F4_CPU_TYPE        "STM32F407xx")

add_definitions(-D${STM32F4_CPU_TYPE})

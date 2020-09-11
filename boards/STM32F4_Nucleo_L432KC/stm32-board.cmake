# The Nucleo STM32L432KC Board uses an ST Link V2.1 and an STM32L4 processor. According to
# the comment in the OpenOCD Config File, the stm32l4discovery.cfg file is generic enough
# to work for that configuration, too.
set(OPENOCD_BOARD_CFG       "stm32l4discovery.cfg")
set(STM32_STARTUP_CODE      "boards/STM32F4_Nucleo_L432KC/startup_stm32l432xx.S")
set(STM32_LDSCRIPT          "boards/STM32F4_Nucleo_L432KC/STM32L432XX.ld")
set(STM32_CPU_FAMILY        "STM32L4")
set(STM32_CPU_TYPE          "STM32L432xx")

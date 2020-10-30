file firmware.elf
target extended-remote /dev/cu.usbmodem4D81905D1
monitor swdp_scan
attach 1
set mem inaccessible-by-default off
# load firmware.elf


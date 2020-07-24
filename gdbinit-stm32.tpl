# Re-define "run". Remote STM32F4 Target does not support "run" but wants
# "continue" instead; however Netbeans absolutely wants to send "run" when the
# debugger starts.
define run
continue
end

# Start the ST-Util tool which provides the remote GDB server. The tool exits
# automaticall when the connection is dropped, i.e. GDB exits.
# shell /opt/bin/st-util &

file firmware.elf
target remote localhost:4242
load firmware.elf


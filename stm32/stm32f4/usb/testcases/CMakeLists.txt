#-
# $Copyright$
#

#*******************************************************************************
# Compile Environment
#*******************************************************************************
add_definitions("-DUNITTEST")

#*******************************************************************************
# Testcases
#*******************************************************************************
set(TC_SOURCES
    main.cpp
    test_InEndpointViaSTM32F4.cpp
    test_OutEndpointViaSTM32F4.cpp
    test_UsbCoreViaSTM32F4.cpp
    test_UsbDeviceViaSTM32F4.cpp
)

set(TARGET_EXE test_usb_stm32f4)

if(${HAVE_USB_OTG})
    add_executable(${TARGET_EXE} ${TC_SOURCES})
    target_link_libraries(${TARGET_EXE}
        gmock
        gmock_main
        usb-stm32f4
    )

    add_test(NAME usb-stm32f4 COMMAND ${TARGET_EXE})
endif()
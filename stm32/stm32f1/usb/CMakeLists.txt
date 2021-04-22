###############################################################################
# STM32F1 USB Device Library
###############################################################################
set(TARGET_NAME usb-stm32f1)
set(TARGET_SRC
    Device.cpp
    Endpoint.cpp
    InEndpoint.cpp
    OutEndpoint.cpp
)

if(${HAVE_USB})
    message(STATUS "STM32F1 CPU ${STM32_CPU_TYPE} supports USB Full-Speed Macro")

    add_library(${TARGET_NAME} ${TARGET_SRC})
    target_link_libraries(${TARGET_NAME}
        stm32f1
        usb
    )
    target_include_directories(${TARGET_NAME} PRIVATE
    )
    target_include_directories(${TARGET_NAME} PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
    )
else()
    add_library(${TARGET_NAME} INTERFACE)
    target_include_directories(${TARGET_NAME} INTERFACE
        ${CMAKE_CURRENT_SOURCE_DIR}/include
    )
    target_link_libraries(${TARGET_NAME} INTERFACE
        stm32f1
        usb
    )
endif()

#*******************************************************************************
# Testcases
#*******************************************************************************
if (UNITTEST)
    MESSAGE(STATUS "Building ${TARGET_NAME} Testcases")
    # add_subdirectory(testcases)
endif(UNITTEST)

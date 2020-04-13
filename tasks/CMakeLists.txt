#-
# $Copyright$
#

set(TARGET_NAME tasks)
set(TARGET_SRC
    Task.cpp
)
add_library(${TARGET_NAME} ${TARGET_SRC})
target_include_directories(${TARGET_NAME} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
)
target_include_directories(${TARGET_NAME} PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/..
)
target_link_libraries(${TARGET_NAME}
    FreeRTOS
    gpio
    uart
)

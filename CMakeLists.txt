
include("${PROJECT_SOURCE_DIR}/pico_sdk_import.cmake")

set(SOURCES VEML770.cpp)

add_library(veml7700 ${SOURCES})

target_link_libraries(veml7700 pico_stdlib hardware_i2c)

cmake_minimum_required(VERSION 3.22)

add_library(light_sensor STATIC light_sensor.h light_sensor.cpp)
target_include_directories(light_sensor PUBLIC .)
target_link_libraries(light_sensor mbed-os)

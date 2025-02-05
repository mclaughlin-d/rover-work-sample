cmake_minimum_required(VERSION 3.22)

add_library(light_sensor_thread STATIC
    light_sensor_thread.h
)
target_include_directories(light_sensor_thread PUBLIC .)
target_link_libraries(light_sensor_thread mbed-os rovercan light_sensor)


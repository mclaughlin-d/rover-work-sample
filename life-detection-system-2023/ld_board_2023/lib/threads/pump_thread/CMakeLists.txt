cmake_minimum_required(VERSION 3.22)

add_library(pump_thread STATIC
    pump_thread.h
)
target_include_directories(pump_thread PUBLIC .)
target_link_libraries(pump_thread mbed-os rovercan)

cmake_minimum_required(VERSION 3.22)

add_library(actuator_thread STATIC
    actuator_thread.h
)
target_include_directories(actuator_thread PUBLIC .)
target_link_libraries(actuator_thread mbed-os rovercan linearActuator)


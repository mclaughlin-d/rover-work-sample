cmake_minimum_required(VERSION 3.22)

add_library(linearActuator STATIC linearActuator.h linearActuator.cpp)
target_include_directories(linearActuator PUBLIC .)
target_link_libraries(linearActuator mbed-os)

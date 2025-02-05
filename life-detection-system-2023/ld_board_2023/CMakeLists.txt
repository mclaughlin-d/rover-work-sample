cmake_minimum_required(VERSION 3.19.0 FATAL_ERROR)

set(SHARED_LIB_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../lib)
set(APP_TARGET ld_board_2023)

set(UPLOAD_METHOD_DEFAULT NONE)
set(NONE_UPLOAD_ENABLED TRUE)

include("${SHARED_LIB_DIR}/mbed-cmake/mbed-cmake/mbed-cmake.cmake")

project(${APP_TARGET})

##### LIBRARIES #####
# Add lib subdirectory for libraries
# Note that:
# - libraries must also be added to the target_link_libraries invocation
# - the `${SHARED_LIB_DIR}` directory refers to `rover/firmware/lib`
# - the `lib` directory refers to `./lib` (`rover/firmware/src/<project name>/lib`)
# - refer to this Notion page (link here) for how to create new libraries
#TODO: add notion page referenced above
# -------------------------------------------------------------
add_subdirectory(${SHARED_LIB_DIR}/rovercan ${CMAKE_CURRENT_BINARY_DIR}/rovercan)   # <-- Uncomment this line to enable CAN
add_subdirectory(${SHARED_LIB_DIR}/roverserial/unbuffered_serial_cpp ${SHARED_LIB_DIR}/roverserial/unbuffered_serial_cpp/build) # <-- Uncomment this line to enable UnbufferedRoverSerial
add_subdirectory(lib/linearActuator)
add_subdirectory(lib/light_sensor)
add_subdirectory(lib/threads/actuator_thread)
add_subdirectory(lib/threads/pump_thread)
add_subdirectory(lib/threads/light_sensor_thread)


add_mbed_executable(${APP_TARGET} main.cpp)

target_link_libraries(${APP_TARGET}
        rovercan
        roverserial
        linearActuator
        light_sensor
        actuator_thread
        pump_thread
        light_sensor_thread
)

mbed_cmake_finalize()

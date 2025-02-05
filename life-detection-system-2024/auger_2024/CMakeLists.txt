cmake_minimum_required(VERSION 3.19.0 FATAL_ERROR)

set(SHARED_LIB_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../lib)
set(APP_TARGET auger_2024)

# load project upload methods (must be before mbed-cmake is loaded)
# -------------------------------------------------------------
set(UPLOAD_METHOD_DEFAULT NONE)
set(NONE_UPLOAD_ENABLED TRUE)

include("${SHARED_LIB_DIR}/mbed-cmake/mbed-cmake/mbed-cmake.cmake")

project(${APP_TARGET})


###### LIBRARIES #####
# Add lib subdirectory for libraries
# Note that:
# - libraries must also be added to the target_link_libraries invocation
# - the `${SHARED_LIB_DIR}` directory refers to `rover/firmware/lib`
# - the `lib` directory refers to `./lib` (`rover/firmware/src/<project name>/lib`)
# - refer to this Notion page (link here) for how to create new libraries
#TODO: add notion page referenced above
# -------------------------------------------------------------
add_subdirectory(${SHARED_LIB_DIR}/rovercan ${SHARED_LIB_DIR}/rovercan/build)   # <-- Uncomment this line to enable CAN
# add_subdirectory(${SHARED_LIB_DIR}/roverserial/unbuffered_serial_cpp ${SHARED_LIB_DIR}/roverserial/unbuffered_serial_cpp/build) # <-- Uncomment this line to enable UnbufferedRoverSerial
# add_subdirectory(${SHARED_LIB_DIR}/servo ${SHARED_LIB_DIR}/servo/build) # <-- Uncomment this line to use the servo library
# add_subdirectory(threads) # <-- Uncomment this line to include other threads (used for RoverCAN)

add_mbed_executable(${APP_TARGET} main.cpp)

target_link_libraries(${APP_TARGET}
    # PRIVATE
        rovercan
        roverserial
        # servo
        # threads
)

mbed_cmake_finalize()


cmake_minimum_required(VERSION 2.6)

include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rosbuild_init()


set(AVR_BRIDGE_CONFIG config.yaml)
set(ARDUINO_TYPE "arduino_uno")
set(ADUINO_PORT "/dev/ttyACM0")

rosbuild_find_ros_package(avr_bridge)
include(${avr_bridge_PACKAGE_PATH}/arduino_cmake/avrbuild.cmake)

add_subdirectory(src/Wire)


set(PROJECT_SRC src/main.cpp)
add_executable(firmware ${PROJECT_SRC} ${AVR_ROS_SRC} ${ARDUINO_SRC})

target_link_libraries(firmware Wire)

#to compile, use make
#to program arduino on /dev/ttyUSB0, do make flash
#to check the program size use make size
